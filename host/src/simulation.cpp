// Interactive simulation of the Voyaging Arm.
//
// Draws the arm from the kinematics constants and lets you drive it two ways:
//   TARGET mode: sliders set the tip position + wrist pitch, inverse() solves the joints
//                (elbow up if that's within the limits, else elbow down).
//   JOINTS mode: sliders set the joint angles directly, forward() reports the tip.
// Wrist roll and gripper have their own sliders in both modes; they don't move the tip.
// Each slider has a text box next to its label: click it, type a value, press Enter (or click
// away) to apply. Typed values may go past the slider's range.
// Right-drag orbits the camera, mouse wheel zooms. F11 toggles fullscreen.
//
// The sliders set a target pose (drawn as a faint ghost). The solid arm glides toward it at a
// fixed joint speed, and that gliding pose is handed to the caller every frame (see simulation.hpp).
// A target outside the limits (a joint too far, or too close to the table) is refused,
// and the glide stops rather than take a step that would break them.
// The simulation itself knows nothing about the real arm.

#include <raylib.h>
#define RAYGUI_IMPLEMENTATION
#include <raygui.h>

#include "arm/kinematics.hpp"
#include "arm/limits.hpp"
#include "arm/motion.hpp"
#include "arm/presets.hpp"
#include "arm/simulation.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <numbers>

namespace {

// Initial window size; the window is resizable and the layout follows GetScreenWidth/Height.
constexpr int kInitialWidth = 1120;
constexpr int kInitialHeight = 860;
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

Vector3 add(Vector3 a, Vector3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
Vector3 scale(Vector3 a, float k) { return {a.x * k, a.y * k, a.z * k}; }
Vector3 cross(Vector3 a, Vector3 b) { return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x}; }
Vector3 normalized(Vector3 a) {
    const float n = std::sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
    return n > 0.0f ? scale(a, 1.0f / n) : Vector3{1, 0, 0};
}

// Jaw pivots sit this far back from the tips (ClawParams/Params.jaw_reach in Arm.FCStd), and this far
// either side of the roll axis. Only for drawing.
constexpr float kJawReach = 66.0f * kScale;
constexpr float kJawPivotGap = 9.0f * kScale;

// The two jaws, from their pivots to their tips. `tip` is the point where the closed tips meet,
// `along` the roll axis (unit, pointing out of the claw).
void drawJaws(Vector3 tip, Vector3 along, double base, double spin, double gripper, Color color) {
    // Wrist roll 0 = jaws open side to side, i.e. across the arm's vertical plane. Positive roll turns
    // them counter-clockwise as seen from behind the claw (see arm_calibrate).
    const Vector3 side0 = toScene({-std::sin(base), std::cos(base), 0.0});  // the arm's left, flat
    const Vector3 up0 = normalized(cross(along, side0));
    const Vector3 side = add(scale(side0, float(std::cos(spin))), scale(up0, -float(std::sin(spin))));

    const Vector3 pivotCentre = add(tip, scale(along, -kJawReach));
    const float closed = std::asin(kJawPivotGap / kJawReach);  // each jaw leans in this much to meet at the tip
    const float open = closed - float(gripper) / 2.0f;          // gripper = angle between the jaws
    for (float sideSign : {1.0f, -1.0f}) {
        const Vector3 pivot = add(pivotCentre, scale(side, sideSign * kJawPivotGap));
        const Vector3 dir = add(scale(along, std::cos(open)), scale(side, -sideSign * std::sin(open)));
        DrawCylinderEx(pivot, add(pivot, scale(dir, kJawReach)), 0.02f, 0.012f, 8, color);
    }
}

void drawArm(const arm::arm_angles& angles, const arm::arm_constants& c) {
    const arm::joint_chain ch = arm::joint_points(angles, c);
    std::array<Vector3, 6> s{};
    for (std::size_t i = 0; i < s.size(); ++i) s[i] = toScene(ch[i]);
    const Vector3 along = normalized(add(s[5], scale(s[4], -1.0f)));
    const Vector3 jawPivots = add(s[5], scale(along, -kJawReach));

    // base disc + vertical pedestal up to shoulder height, then the small shoulder offset stub
    DrawCylinder({0, 0, 0}, 0.35f, 0.35f, 0.04f, 24, Color{90, 90, 100, 255});
    const Vector3 pedestalTop{0.0f, float(c.shoulder_height) * kScale, 0.0f};
    DrawCylinderEx({0, 0.04f, 0}, pedestalTop, 0.12f, 0.12f, 16, Color{120, 120, 130, 255});
    DrawCylinderEx(pedestalTop, s[1], 0.05f, 0.05f, 12, Color{120, 120, 130, 255});

    // links
    DrawCylinderEx(s[1], s[2], 0.06f, 0.06f, 12, Color{230, 100, 60, 255});   // link 1
    DrawCylinderEx(s[2], s[3], 0.06f, 0.06f, 12, Color{60, 140, 230, 255});   // link 2
    DrawCylinderEx(s[3], s[4], 0.03f, 0.03f, 12, Color{140, 140, 140, 255});  // wrist offset stub
    DrawCylinderEx(s[4], jawPivots, 0.045f, 0.04f, 12, Color{70, 180, 110, 255});  // link 3 / claw body
    drawJaws(s[5], along, angles.base, angles.wrist_spin, angles.gripper, Color{40, 130, 75, 255});

    // joints
    DrawSphere(s[1], 0.08f, DARKGRAY);
    DrawSphere(s[2], 0.08f, DARKGRAY);
    DrawSphere(s[3], 0.08f, DARKGRAY);
    DrawSphere(s[5], 0.05f, Color{255, 200, 40, 255});  // tip
}

// Thin outline of a pose, used for the target the solid arm is moving toward.
void drawGhost(const arm::arm_angles& angles, const arm::arm_constants& c) {
    const arm::joint_chain ch = arm::joint_points(angles, c);
    const Color ghost{120, 120, 140, 120};
    for (std::size_t i = 1; i + 1 < ch.size(); ++i) {
        DrawCylinderEx(toScene(ch[i]), toScene(ch[i + 1]), 0.015f, 0.015f, 6, ghost);
    }
    const Vector3 tip = toScene(ch[5]);
    const Vector3 along = normalized(add(tip, scale(toScene(ch[4]), -1.0f)));
    drawJaws(tip, along, angles.base, angles.wrist_spin, angles.gripper, ghost);
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
    float tx = 250.0f, ty = 0.0f, tz = 150.0f, tpitch = -30.0f;  // should be inside the limits (else the arm starts at all zeros)
    // JOINTS mode: joint angles in degrees
    float jbase = 0.0f, jshoulder = 0.0f, jelbow = 0.0f, jwrist = 0.0f;
    // both modes, degrees: wrist roll (0 = jaws open side to side) and gripper (0 = closed)
    float jspin = 0.0f, jgrip = 0.0f;
};

UiState defaultUi() { return UiState{}; }

arm::arm_angles jointsFromUi(const UiState& ui) {
    arm::arm_angles a;
    a.base = deg2rad(ui.jbase);
    a.shoulder = deg2rad(ui.jshoulder);
    a.elbow = deg2rad(ui.jelbow);
    a.wrist = deg2rad(ui.jwrist);
    a.wrist_spin = deg2rad(ui.jspin);
    a.gripper = deg2rad(ui.jgrip);
    return a;
}

// Joints for a tip position + wrist pitch: elbow up if that's safe, else elbow down if that is.
// When neither is, the elbow-up one, so the panel can say why.
arm::arm_angles solve(const arm::position& p, double pitch, const arm::arm_constants& c, const arm::arm_limits& limits) {
    const arm::arm_angles up = arm::inverse(p, c, pitch, true);
    if (isValid(up) && arm::why_unsafe(up, c, limits).empty()) return up;
    const arm::arm_angles down = arm::inverse(p, c, pitch, false);
    if (isValid(down) && arm::why_unsafe(down, c, limits).empty()) return down;
    return up;
}

// The pose RESET and startup go to: the default target if it's safe, else all zeros.
arm::arm_angles startPose(const UiState& ui, const arm::arm_constants& c, const arm::arm_limits& limits) {
    arm::arm_angles a = solve({ui.tx, ui.ty, ui.tz}, deg2rad(ui.tpitch), c, limits);
    a.wrist_spin = deg2rad(ui.jspin);
    a.gripper = deg2rad(ui.jgrip);
    if (!isValid(a) || !arm::why_unsafe(a, c, limits).empty()) {
        a = arm::arm_angles{};
        a.wrist_spin = deg2rad(ui.jspin);
        a.gripper = deg2rad(ui.jgrip);
    }
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

// sliderRow for a joint angle in degrees, held to `range` whether dragged or typed.
void limitedSliderRow(int x, int& y, int w, const char* name, float* v, const arm::joint_range& range, Field& f) {
    const float lo = float(rad2deg(range.min));
    const float hi = float(rad2deg(range.max));
    sliderRow(x, y, w, name, "deg", v, lo, hi, f);
    *v = std::clamp(*v, lo, hi);
}

void readoutRow(int x, int& y, const char* text, Color color = DARKGRAY) {
    DrawText(text, x, y, 20, color);
    y += 24;
}

}  // namespace

void arm::run_simulation(const pose_sink& on_pose, const arm_limits& limits) {
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
    ui.jspin = std::clamp(ui.jspin, float(rad2deg(limits.wrist_spin.min)), float(rad2deg(limits.wrist_spin.max)));
    ui.jgrip = std::clamp(ui.jgrip, float(rad2deg(limits.gripper.min)), float(rad2deg(limits.gripper.max)));
    const UiState initialUi = ui;
    // 0..3: one per mode-specific slider row (only one mode's show at a time); 4, 5: wrist roll, gripper
    std::array<Field, 6> fields{};
    arm::arm_angles lastValid = startPose(ui, c, limits);
    if (ui.mode == 1) seedJointsFrom(ui, lastValid);
    arm::arm_angles commanded = lastValid;  // where the arm is right now, gliding toward the target

    // Debug aid: ARM_SIMULATION_SCREENSHOT=/path/out.png saves a frame after startup and quits.
    const char* screenshotPath = std::getenv("ARM_SIMULATION_SCREENSHOT");
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
            else seedTargetFrom(ui, lastValid, c);
            for (Field& f : fields) f.editing = false;
        }

        arm::arm_angles shown;
        bool reachable = true;
        std::string blocked;  // why the target would hurt the arm; empty when it's safe
        arm::position target{ui.tx, ui.ty, ui.tz};

        if (ui.mode == 0) {
            // full reach along x is shoulder_offset + l1 + l2 + l3 = ~409 mm, so give the sliders headroom
            sliderRow(px, y, pw, "x", "mm", &ui.tx, -450.0f, 450.0f, fields[0]);
            sliderRow(px, y, pw, "y", "mm", &ui.ty, -450.0f, 450.0f, fields[1]);
            sliderRow(px, y, pw, "z", "mm", &ui.tz, -100.0f, 500.0f, fields[2]);
            sliderRow(px, y, pw, "wrist pitch", "deg", &ui.tpitch, -180.0f, 180.0f, fields[3]);

            limitedSliderRow(px, y, pw, "wrist roll", &ui.jspin, limits.wrist_spin, fields[4]);
            limitedSliderRow(px, y, pw, "gripper", &ui.jgrip, limits.gripper, fields[5]);

            target = {ui.tx, ui.ty, ui.tz};
            arm::arm_angles sol = solve(target, deg2rad(ui.tpitch), c, limits);
            sol.wrist_spin = deg2rad(ui.jspin);
            sol.gripper = deg2rad(ui.jgrip);
            reachable = isValid(sol);
            if (reachable) blocked = arm::why_unsafe(sol, c, limits);
            if (reachable && blocked.empty()) {
                lastValid = sol;
            } else {
                // the target is out, but roll and gripper don't depend on it: still move them
                arm::arm_angles held = lastValid;
                held.wrist_spin = sol.wrist_spin;
                held.gripper = sol.gripper;
                if (arm::why_unsafe(held, c, limits).empty()) lastValid = held;
            }
            shown = lastValid;

            y += 8;
            if (!reachable) readoutRow(px, y, "UNREACHABLE (showing last pose)", RED);
            else if (!blocked.empty()) readoutRow(px, y, TextFormat("BLOCKED: %s", blocked.c_str()), RED);
            else readoutRow(px, y, "reachable", DARKGREEN);
            readoutRow(px, y, TextFormat("base      %7.1f deg", rad2deg(shown.base)));
            readoutRow(px, y, TextFormat("shoulder  %7.1f deg", rad2deg(shown.shoulder)));
            readoutRow(px, y, TextFormat("elbow     %7.1f deg", rad2deg(shown.elbow)));
            readoutRow(px, y, TextFormat("wrist     %7.1f deg", rad2deg(shown.wrist)));
            if (reachable && blocked.empty()) {
                const double err = distance(arm::forward(shown, c), target);
                readoutRow(px, y, TextFormat("IK error  %.3f mm", err), err < 0.01 ? DARKGREEN : RED);
            }
        } else {
            // each slider spans exactly its joint's limits; typed values are held to them too
            limitedSliderRow(px, y, pw, "base", &ui.jbase, limits.base, fields[0]);
            limitedSliderRow(px, y, pw, "shoulder", &ui.jshoulder, limits.shoulder, fields[1]);
            limitedSliderRow(px, y, pw, "elbow", &ui.jelbow, limits.elbow, fields[2]);  // negative = elbow up
            limitedSliderRow(px, y, pw, "wrist", &ui.jwrist, limits.wrist, fields[3]);
            limitedSliderRow(px, y, pw, "wrist roll", &ui.jspin, limits.wrist_spin, fields[4]);
            limitedSliderRow(px, y, pw, "gripper", &ui.jgrip, limits.gripper, fields[5]);

            const arm::arm_angles wanted = jointsFromUi(ui);
            blocked = arm::why_unsafe(wanted, c, limits);
            if (blocked.empty()) lastValid = wanted;
            shown = lastValid;
            target = arm::forward(shown, c);

            y += 8;
            if (!blocked.empty()) readoutRow(px, y, TextFormat("BLOCKED: %s", blocked.c_str()), RED);
            else readoutRow(px, y, "safe", DARKGREEN);
            readoutRow(px, y, TextFormat("tip x     %7.1f mm", target.x));
            readoutRow(px, y, TextFormat("tip y     %7.1f mm", target.y));
            readoutRow(px, y, TextFormat("tip z     %7.1f mm", target.z));
            readoutRow(px, y, TextFormat("pitch     %7.1f deg", rad2deg(shown.shoulder + shown.elbow + shown.wrist)));
        }

        y += 12;
        if (GuiButton(Rectangle{float(px), float(y), float(pw), 30.0f}, "RESET")) {
            const int keepMode = ui.mode;
            ui = initialUi;
            ui.mode = keepMode;
            lastValid = startPose(ui, c, limits);
            if (ui.mode == 1) seedJointsFrom(ui, lastValid);
            for (Field& f : fields) f.editing = false;
        }

        // ---- glide toward the target, and hand that pose to whoever is listening
        const double dt = GetFrameTime();
        // Two safe poses can still have an unsafe one between them (e.g. swinging the claw through
        // the table), so every step is checked too; the arm holds still rather than take a bad one.
        const arm::arm_angles next = arm::step_toward(commanded, shown, kJointSpeed, dt);
        const std::string pathBlocked = arm::why_unsafe(next, c, limits);
        if (pathBlocked.empty()) commanded = next;
        const simulation_status status = on_pose ? on_pose(commanded, dt) : simulation_status{"simulation only", true};

        y += 44;
        if (!pathBlocked.empty()) readoutRow(px, y, TextFormat("path blocked: %s", pathBlocked.c_str()), RED);
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
            DrawSphereWires(toScene(target), 0.09f, 8, 8, reachable && blocked.empty() ? Color{0, 170, 60, 255} : RED);
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
