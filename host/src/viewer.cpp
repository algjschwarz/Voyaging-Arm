// Interactive viewer for the Voyaging Arm.
//
// Draws the arm from the kinematics constants and lets you drive it two ways:
//   TARGET mode: sliders set the tip position + wrist pitch, inverse() solves the joints.
//   JOINTS mode: sliders set the joint angles directly, forward() reports the tip.
// Each slider has a text box next to its label: click it, type a value, press Enter (or click
// away) to apply. Typed values may go past the slider's range.
// Right-drag orbits the camera, mouse wheel zooms. F11 toggles fullscreen.
//
// The sliders set a target pose (drawn as a faint ghost). The solid arm glides toward it at a
// fixed joint speed, and that gliding pose is handed to the caller every frame (see viewer.hpp).
// The viewer itself knows nothing about the real arm.

#include <raylib.h>
#define RAYGUI_IMPLEMENTATION
#include <raygui.h>

#include "arm/kinematics.hpp"
#include "arm/motion.hpp"
#include "arm/presets.hpp"
#include "arm/viewer.hpp"

#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <numbers>

namespace {

// Initial window size; the window is resizable and the layout follows GetScreenWidth/Height.
constexpr int kInitialWidth = 1120;
constexpr int kInitialHeight = 720;
constexpr int kPanelWidth = 320;

// Kinematics work in mm; 1 scene unit = 100 mm, so the ~400 mm reach is ~4 units.
constexpr float kScale = 0.01f;

constexpr double deg2rad(double d) { return d * std::numbers::pi / 180.0; }
constexpr double rad2deg(double r) { return r * 180.0 / std::numbers::pi; }

// How fast every joint moves toward the target. Must stay below what the servos can do
// (MG996R is ~60 deg per 0.17 s unloaded), or the real arm lags behind the simulation.
constexpr double kJointSpeed = deg2rad(90.0);  // rad/s

// Arm frame is z-up. raylib is y-up. x stays, arm z becomes scene y, arm y becomes scene -z
// (that keeps the frame right-handed).
Vector3 toScene(const arm::position& p) {
    return { float(p.x) * kScale, float(p.z) * kScale, -float(p.y) * kScale };
}

bool isValid(const arm::arm_angles& a) {
    return std::isfinite(a.base) && std::isfinite(a.shoulder) &&
           std::isfinite(a.elbow) && std::isfinite(a.wrist);
}

double distance(const arm::position& a, const arm::position& b) {
    return std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) + (a.z - b.z) * (a.z - b.z));
}

// Every joint along the chain, in arm coordinates (mm). Same math as arm::forward(),
// just keeping the intermediate points so we can draw the links.
//   0 base origin, 1 shoulder axis, 2 elbow axis, 3 wrist pitch axis,
//   4 start of the roll axis (wrist_offset away from 3), 5 tip
struct Chain { std::array<arm::position, 6> p; };

Chain jointPoints(const arm::arm_angles& a, const arm::arm_constants& c) {
    const double se = a.shoulder + a.elbow;
    const double wa = se + a.wrist;

    // in the vertical plane through the arm: (r = reach, z = height)
    struct RZ { double r, z; };
    std::array<RZ, 6> rz{};
    rz[0] = {0.0, 0.0};
    rz[1] = {c.shoulder_offset, c.shoulder_height};
    rz[2] = {rz[1].r + c.length1 * std::cos(a.shoulder), rz[1].z + c.length1 * std::sin(a.shoulder)};
    rz[3] = {rz[2].r + c.length2 * std::cos(se),         rz[2].z + c.length2 * std::sin(se)};
    rz[4] = {rz[3].r - c.wrist_offset * std::sin(wa),    rz[3].z + c.wrist_offset * std::cos(wa)};
    rz[5] = {rz[4].r + c.length3 * std::cos(wa),         rz[4].z + c.length3 * std::sin(wa)};

    Chain out{};
    const double cb = std::cos(a.base), sb = std::sin(a.base);
    for (std::size_t i = 0; i < rz.size(); ++i) {
        out.p[i] = { rz[i].r * cb, rz[i].r * sb, rz[i].z };
    }
    return out;
}

void drawArm(const arm::arm_angles& angles, const arm::arm_constants& c) {
    const Chain ch = jointPoints(angles, c);
    std::array<Vector3, 6> s{};
    for (std::size_t i = 0; i < s.size(); ++i) s[i] = toScene(ch.p[i]);

    // base disc + vertical pedestal up to shoulder height, then the small shoulder offset stub
    DrawCylinder({0, 0, 0}, 0.35f, 0.35f, 0.04f, 24, Color{90, 90, 100, 255});
    const Vector3 pedestalTop{0.0f, float(c.shoulder_height) * kScale, 0.0f};
    DrawCylinderEx({0, 0.04f, 0}, pedestalTop, 0.12f, 0.12f, 16, Color{120, 120, 130, 255});
    DrawCylinderEx(pedestalTop, s[1], 0.05f, 0.05f, 12, Color{120, 120, 130, 255});

    // links
    DrawCylinderEx(s[1], s[2], 0.06f, 0.06f, 12, Color{230, 100, 60, 255});   // link 1
    DrawCylinderEx(s[2], s[3], 0.06f, 0.06f, 12, Color{60, 140, 230, 255});   // link 2
    DrawCylinderEx(s[3], s[4], 0.03f, 0.03f, 12, Color{140, 140, 140, 255});  // wrist offset stub
    DrawCylinderEx(s[4], s[5], 0.045f, 0.03f, 12, Color{70, 180, 110, 255});  // link 3 / claw

    // joints
    DrawSphere(s[1], 0.08f, DARKGRAY);
    DrawSphere(s[2], 0.08f, DARKGRAY);
    DrawSphere(s[3], 0.08f, DARKGRAY);
    DrawSphere(s[5], 0.05f, Color{255, 200, 40, 255});  // tip
}

// Thin outline of a pose, used for the target the solid arm is moving toward.
void drawGhost(const arm::arm_angles& angles, const arm::arm_constants& c) {
    const Chain ch = jointPoints(angles, c);
    const Color ghost{120, 120, 140, 120};
    for (std::size_t i = 1; i + 1 < ch.p.size(); ++i) {
        DrawCylinderEx(toScene(ch.p[i]), toScene(ch.p[i + 1]), 0.015f, 0.015f, 6, ghost);
    }
}

void drawAxes() {
    // arm x (red), arm y (green), arm z (blue), 10 cm long
    DrawLine3D({0, 0, 0}, {1.0f, 0, 0}, RED);
    DrawLine3D({0, 0, 0}, {0, 0, -1.0f}, GREEN);
    DrawLine3D({0, 0, 0}, {0, 1.0f, 0}, BLUE);
}

struct UiState {
    int mode = 0;  // 0 = TARGET (IK), 1 = JOINTS (FK)
    // TARGET mode: tip position in mm, wrist pitch in degrees (absolute, 0 = horizontal)
    float tx = 250.0f, ty = 0.0f, tz = 150.0f, tpitch = 0.0f;
    // JOINTS mode: joint angles in degrees
    float jbase = 0.0f, jshoulder = 0.0f, jelbow = 0.0f, jwrist = 0.0f;
};

UiState defaultUi() { return UiState{}; }

arm::arm_angles jointsFromUi(const UiState& ui) {
    arm::arm_angles a;
    a.base = deg2rad(ui.jbase);
    a.shoulder = deg2rad(ui.jshoulder);
    a.elbow = deg2rad(ui.jelbow);
    a.wrist = deg2rad(ui.jwrist);
    return a;
}

void seedJointsFrom(UiState& ui, const arm::arm_angles& a) {
    ui.jbase = float(rad2deg(a.base));
    ui.jshoulder = float(rad2deg(a.shoulder));
    ui.jelbow = float(rad2deg(a.elbow));
    ui.jwrist = float(rad2deg(a.wrist));
}

void seedTargetFrom(UiState& ui, const arm::arm_angles& a, const arm::arm_constants& c) {
    const arm::position p = arm::forward(a, c);
    ui.tx = float(p.x);
    ui.ty = float(p.y);
    ui.tz = float(p.z);
    ui.tpitch = float(rad2deg(a.shoulder + a.elbow + a.wrist));
}

// Edit state for one text box. The buffer mirrors the slider value until you click into it.
struct Field {
    char buf[24] = {};
    bool editing = false;
};

constexpr int kFieldWidth = 96;

// Label + text box on one line, slider on the next. Advances y.
// Returns true if the value was changed by typing.
bool sliderRow(int x, int& y, int w, const char* name, const char* unit, float* v, float lo, float hi, Field& f) {
    DrawText(TextFormat("%s (%s)", name, unit), x, y + 3, 20, DARKGRAY);

    if (!f.editing) std::snprintf(f.buf, sizeof f.buf, "%.1f", *v);
    bool changed = false;
    const Rectangle box{float(x + w - kFieldWidth), float(y), float(kFieldWidth), 24.0f};
    if (GuiTextBox(box, f.buf, sizeof f.buf, f.editing)) {
        if (f.editing) {
            char* end = nullptr;
            const double parsed = std::strtod(f.buf, &end);
            if (end != f.buf && std::isfinite(parsed)) {
                *v = float(parsed);
                changed = true;
            }
        }
        f.editing = !f.editing;
    }
    y += 28;

    GuiSlider(Rectangle{float(x), float(y), float(w), 20.0f}, "", "", v, lo, hi);
    y += 30;
    return changed;
}

void readoutRow(int x, int& y, const char* text, Color color = DARKGRAY) {
    DrawText(text, x, y, 20, color);
    y += 24;
}

}  // namespace

void arm::run_viewer(const pose_sink& on_pose) {
    const arm::arm_constants c = arm::VOYAGING_ARM;

    SetConfigFlags(FLAG_WINDOW_RESIZABLE);
    InitWindow(kInitialWidth, kInitialHeight, "Voyaging Arm");
    SetWindowMinSize(kPanelWidth + 320, 480);
    SetTargetFPS(60);
    GuiSetStyle(DEFAULT, TEXT_SIZE, 16);

    Camera3D cam{};
    cam.position = {3.0f, 3.2f, 5.0f};
    cam.target = {1.4f, 1.0f, 0.0f};  // a bit out along the reach so the arm is centred
    cam.up = {0.0f, 1.0f, 0.0f};
    cam.fovy = 45.0f;
    cam.projection = CAMERA_PERSPECTIVE;

    UiState ui = defaultUi();
    std::array<Field, 4> fields{};  // one per slider row; only one mode's rows show at a time
    arm::arm_angles lastValid = arm::inverse({ui.tx, ui.ty, ui.tz}, c, deg2rad(ui.tpitch));
    if (!isValid(lastValid)) lastValid = arm::arm_angles{};
    arm::arm_angles commanded = lastValid;  // where the arm is right now, gliding toward the target

    // Debug aid: ARM_VIEWER_SCREENSHOT=/path/out.png saves a frame after startup and quits.
    const char* screenshotPath = std::getenv("ARM_VIEWER_SCREENSHOT");
    int frame = 0;

    while (!WindowShouldClose()) {
        if (IsKeyPressed(KEY_F11)) ToggleBorderlessWindowed();

        // ---- layout follows the current window size
        const int screenW = GetScreenWidth();
        const int screenH = GetScreenHeight();
        const int viewW = screenW - kPanelWidth;

        // ---- camera: only when interacting over the 3D view, never while over the panel
        const bool overPanel = GetMouseX() >= viewW;
        if (!overPanel && (IsMouseButtonDown(MOUSE_BUTTON_RIGHT) || GetMouseWheelMove() != 0.0f)) {
            UpdateCamera(&cam, CAMERA_THIRD_PERSON);
        }

        BeginDrawing();
        ClearBackground(RAYWHITE);

        // ---- control panel (updates ui, so it runs before we solve/draw the arm)
        const int px = viewW + 16;
        const int pw = kPanelWidth - 32;
        int y = 16;
        DrawRectangle(viewW, 0, kPanelWidth, screenH, Color{240, 240, 244, 255});
        DrawLine(viewW, 0, viewW, screenH, LIGHTGRAY);

        DrawText("Voyaging Arm", px, y, 20, BLACK);
        y += 32;

        const int prevMode = ui.mode;
        GuiToggleGroup(Rectangle{float(px), float(y), float(pw / 2 - 2), 28.0f}, "TARGET (IK);JOINTS (FK)", &ui.mode);
        y += 40;
        if (ui.mode != prevMode) {
            if (ui.mode == 1) seedJointsFrom(ui, lastValid);
            else seedTargetFrom(ui, jointsFromUi(ui), c);
            for (Field& f : fields) f.editing = false;
        }

        arm::arm_angles shown;
        bool reachable = true;
        arm::position target{ui.tx, ui.ty, ui.tz};

        if (ui.mode == 0) {
            // full reach along x is shoulder_offset + l1 + l2 + l3 = ~409 mm, so give the sliders headroom
            sliderRow(px, y, pw, "x", "mm", &ui.tx, -450.0f, 450.0f, fields[0]);
            sliderRow(px, y, pw, "y", "mm", &ui.ty, -450.0f, 450.0f, fields[1]);
            sliderRow(px, y, pw, "z", "mm", &ui.tz, -100.0f, 500.0f, fields[2]);
            sliderRow(px, y, pw, "wrist pitch", "deg", &ui.tpitch, -180.0f, 180.0f, fields[3]);

            target = {ui.tx, ui.ty, ui.tz};
            const arm::arm_angles sol = arm::inverse(target, c, deg2rad(ui.tpitch));
            reachable = isValid(sol);
            if (reachable) lastValid = sol;
            shown = lastValid;

            y += 8;
            readoutRow(px, y, reachable ? "reachable" : "UNREACHABLE (showing last pose)", reachable ? DARKGREEN : RED);
            readoutRow(px, y, TextFormat("base      %7.1f deg", rad2deg(shown.base)));
            readoutRow(px, y, TextFormat("shoulder  %7.1f deg", rad2deg(shown.shoulder)));
            readoutRow(px, y, TextFormat("elbow     %7.1f deg", rad2deg(shown.elbow)));
            readoutRow(px, y, TextFormat("wrist     %7.1f deg", rad2deg(shown.wrist)));
            if (reachable) {
                const double err = distance(arm::forward(shown, c), target);
                readoutRow(px, y, TextFormat("IK error  %.3f mm", err), err < 0.01 ? DARKGREEN : RED);
            }
        } else {
            sliderRow(px, y, pw, "base", "deg", &ui.jbase, -180.0f, 180.0f, fields[0]);
            sliderRow(px, y, pw, "shoulder", "deg", &ui.jshoulder, -90.0f, 180.0f, fields[1]);
            sliderRow(px, y, pw, "elbow", "deg", &ui.jelbow, -180.0f, 180.0f, fields[2]);  // inverse() now returns negative (elbow-up)
            sliderRow(px, y, pw, "wrist", "deg", &ui.jwrist, -180.0f, 180.0f, fields[3]);

            shown = jointsFromUi(ui);
            lastValid = shown;
            target = arm::forward(shown, c);

            y += 8;
            readoutRow(px, y, TextFormat("tip x     %7.1f mm", target.x));
            readoutRow(px, y, TextFormat("tip y     %7.1f mm", target.y));
            readoutRow(px, y, TextFormat("tip z     %7.1f mm", target.z));
            readoutRow(px, y, TextFormat("pitch     %7.1f deg", rad2deg(shown.shoulder + shown.elbow + shown.wrist)));
        }

        y += 12;
        if (GuiButton(Rectangle{float(px), float(y), float(pw), 30.0f}, "RESET")) {
            const int keepMode = ui.mode;
            ui = defaultUi();
            ui.mode = keepMode;
            lastValid = arm::inverse({ui.tx, ui.ty, ui.tz}, c, deg2rad(ui.tpitch));
            if (!isValid(lastValid)) lastValid = arm::arm_angles{};
            if (ui.mode == 1) seedJointsFrom(ui, lastValid);
            for (Field& f : fields) f.editing = false;
        }

        // ---- glide toward the target, and hand that pose to whoever is listening
        const double dt = GetFrameTime();
        commanded = arm::step_toward(commanded, shown, kJointSpeed, dt);
        const viewer_status status = on_pose ? on_pose(commanded, dt) : viewer_status{"simulation only", true};

        y += 44;
        readoutRow(px, y, status.text.c_str(), status.ok ? DARKGREEN : RED);

        DrawText("right-drag: orbit   wheel: zoom   F11: fullscreen", px, screenH - 40, 10, GRAY);
        DrawText("click a box to type a value, Enter applies", px, screenH - 28, 10, GRAY);
        DrawText("axes: x red, y green, z blue", px, screenH - 16, 10, GRAY);

        // ---- 3D view, clipped so it never paints over the panel
        BeginScissorMode(0, 0, viewW, screenH);
        BeginMode3D(cam);
        DrawGrid(10, 1.0f);  // 1 m square, 10 cm cells
        drawAxes();
        drawGhost(shown, c);
        drawArm(commanded, c);
        if (ui.mode == 0) {
            DrawSphereWires(toScene(target), 0.09f, 8, 8, reachable ? Color{0, 170, 60, 255} : RED);
        }
        EndMode3D();
        EndScissorMode();

        EndDrawing();

        if (screenshotPath && ++frame == 30) {
            TakeScreenshot(screenshotPath);
            break;
        }
    }

    CloseWindow();
}
