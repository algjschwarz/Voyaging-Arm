// Servo calibration for the Voyaging Arm, run from a terminal:
//   arm_calibrate /dev/ttyACM0 [calibration file]
//
// The servos have no position feedback, so a human does the looking. Each joint is three steps:
//   1. horn   the servo sits at command 90; push the horn on where you want servo 90 to be
//   2. 0°     nudge with the arrows until the arm is at 0° relative to the table, Enter
//   3. 90°    same for 90°. Tab picks another angle for either step if one can't be reached.
// The line through those two (angle, command) points converts any angle to a command and back.
//
// Every angle you see is relative to the table. The kinematics measure the elbow from the upper arm
// and the wrist from the forearm, so for those the program holds the joints before them in a known
// pose (shoulder straight up, forearm flat) and converts when it saves.
//
// From the joint list you can also drive any servo directly, without calibrating it.
// Each finished joint is saved straight away; joints already in the file are kept.

#include "arm/serial_link.hpp"
#include "arm/servo_map.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <exception>
#include <iomanip>
#include <optional>
#include <poll.h>
#include <sstream>
#include <string>
#include <termios.h>
#include <unistd.h>
#include <utility>
#include <vector>

#ifndef ARM_CALIBRATION_FILE
#define ARM_CALIBRATION_FILE "servos.txt"
#endif

namespace {

constexpr int kJoints = 6;
constexpr double kSmallStep = 1.0;
constexpr double kBigStep = 5.0;
constexpr double kMinPoseGap = 10.0;  // commands closer than this can't be two different poses

// The angle the kinematics use = the angle you mark (relative to the table) + this.
// Base: you mark 90 = forward, the kinematics use 0 = forward.
// Elbow: marked with the shoulder held straight up (90), and the kinematics measure it from the upper arm.
// Wrist: marked with the forearm held flat (0), so the table angle already is the angle from the forearm.
constexpr std::array<double, kJoints> kKinematicsOffset{-90, 0, -90, 0, 0, 0};

// The joints held still while calibrating `joint`, and where, in kinematics angles.
std::vector<std::pair<int, double>> holdsFor(int joint) {
    if (joint == 2) return {{1, 90}};            // shoulder straight up
    if (joint == 3) return {{1, 90}, {2, -90}};  // shoulder straight up, forearm flat
    return {};
}

// What the held joints look like, for the screen.
const char* holdingText(int joint) {
    if (joint == 2) return "shoulder held straight up";
    if (joint == 3) return "shoulder held straight up, forearm held flat";
    return "";
}

// Through the two calibration points: angle -> command, and command -> angle. Both in the file's angles.
double commandFor(const arm::servo_cal& c, double angle) {
    return c.cmd_a + (angle - c.joint_a) * (c.cmd_b - c.cmd_a) / (c.joint_b - c.joint_a);
}
double angleOf(const arm::servo_cal& c, double cmd) {
    return c.joint_a + (cmd - c.cmd_a) * (c.joint_b - c.joint_a) / (c.cmd_b - c.cmd_a);
}

// Puts the terminal into key-at-a-time mode and restores it on exit.
class RawTerminal {
public:
    RawTerminal() {
        tcgetattr(STDIN_FILENO, &saved_);
        termios t = saved_;
        t.c_lflag &= ~(ICANON | ECHO | ISIG);  // ISIG off: Ctrl-C arrives as a key, so we can release servos first
        t.c_cc[VMIN] = 0;
        t.c_cc[VTIME] = 0;
        tcsetattr(STDIN_FILENO, TCSANOW, &t);
    }
    ~RawTerminal() { tcsetattr(STDIN_FILENO, TCSANOW, &saved_); }

private:
    termios saved_{};
};

enum Key { kUp = 1000, kDown, kLeft, kRight, kEscape };

bool isEnter(int key) { return key == '\n' || key == '\r'; }
bool isBackspace(int key) { return key == 127 || key == 8; }

// Steps of the walk-through, in order. List is the joint picker.
enum class Step { List, Horn, First, Second, Done };

// Angles (relative to the table) a pose can be marked at, per joint. The first two are the
// defaults for steps 2 and 3; Tab goes through the rest.
const std::vector<double>& poseAngles(int joint) {
    static const std::vector<double> tilt{0, 90, 180, 45, 135, -45, -90};
    static const std::vector<double> wrist{0, -90, 90, -45, 45};
    static const std::vector<double> spin{0, 90, -90, 180};
    static const std::vector<double> gripper{0, 90, 45, 135, 180};
    switch (joint) {
        case 3: return wrist;
        case 4: return spin;
        case 5: return gripper;
        default: return tilt;
    }
}

// Where to suggest putting servo 90 (the middle of its travel), relative to the table.
constexpr std::array<double, kJoints> kSuggestedMiddle{90, 90, 0, 0, 0, 45};

std::string fmt(double v) { return std::to_string(int(std::lround(v))); }

// One line saying where a part points at a table angle, e.g. "upper arm straight up".
std::string tiltText(const std::string& part, int a) {
    switch (a) {
        case 0: return part + " flat, pointing forward";
        case 45: return part + " pointing up and forward, halfway";
        case 90: return part + " straight up";
        case 135: return part + " pointing up and backward, halfway";
        case 180: return part + " flat, pointing backward";
        case -45: return part + " pointing down and forward, halfway";
        case -90: return part + " straight down";
    }
    return part + " at " + std::to_string(a) + " degrees";
}

// What the arm looks like with `joint` at `angle` (relative to the table), in one line.
std::string looksLike(int joint, double angle) {
    const int a = int(std::lround(angle));
    switch (joint) {
        case 0: switch (a) {  // "right" and "left" as seen standing behind the arm
            case 0: return "arm pointing to its right";
            case 45: return "arm pointing halfway between right and forward";
            case 90: return "arm pointing straight forward";
            case 135: return "arm pointing halfway between forward and left";
            case 180: return "arm pointing to its left";
            case -45: return "arm pointing halfway between right and backward";
            case -90: return "arm pointing straight backward";
        } break;
        case 1: return tiltText("upper arm", a);
        case 2: return tiltText("forearm", a);
        case 3: return tiltText("gripper", a);
        case 4: switch (a) {  // seen from behind the gripper
            case 0: return "jaws open side to side";
            case 90: return "jaws open up/down (a quarter turn counter-clockwise)";
            case -90: return "jaws open up/down (a quarter turn clockwise)";
            case 180: return "jaws open side to side, turned a half turn";
        } break;
        case 5:
            if (a == 0) return "jaws closed, just touching";
            return "jaws open " + std::to_string(a) + " degrees";
    }
    return std::to_string(a) + " degrees";
}

// The pose angles to offer after `first`, best partner first: 90 away, then 45 or 135, then 180.
// Angles already skipped as out of reach go last.
std::vector<double> partnersOf(int joint, double first, const std::vector<double>& skipped) {
    std::vector<double> out;
    for (double a : poseAngles(joint)) if (a != first) out.push_back(a);
    const auto rank = [&](double a) {
        const bool wasSkipped = std::find(skipped.begin(), skipped.end(), a) != skipped.end();
        return std::abs(std::abs(a - first) - 90) + (wasSkipped ? 1000 : 0);
    };
    std::stable_sort(out.begin(), out.end(), [&](double x, double y) { return rank(x) < rank(y); });
    return out;
}

// The entry after `current` in `order`, going round.
double nextIn(const std::vector<double>& order, double current) {
    const auto it = std::find(order.begin(), order.end(), current);
    return it == order.end() || it + 1 == order.end() ? order.front() : *(it + 1);
}

struct Joint {
    int id = 0;  // which joint, firmware order
    double cmd = 90;
    bool driven = false;
    double firstAngle = 0, secondAngle = 90;  // the two poses, relative to the table
    std::optional<double> firstCmd, secondCmd;
    std::vector<double> skipped;  // angles Tabbed past, so the servo can't reach them

    // Tab: the next angle to try for a pose the servo can't reach.
    void nextFirstAngle() {
        skipped.push_back(firstAngle);
        firstAngle = nextIn(poseAngles(id), firstAngle);
    }
    void nextSecondAngle() {
        skipped.push_back(secondAngle);
        secondAngle = nextIn(partnersOf(id, firstAngle, skipped), secondAngle);
    }
    double bestSecondAngle() const { return partnersOf(id, firstAngle, skipped).front(); }

    bool complete() const { return firstCmd && secondCmd; }
};

// "0=8": 0 degrees is at servo 8.
std::string pose(double angle, const std::optional<double>& cmd) {
    return cmd ? fmt(angle) + "=" + fmt(*cmd) : std::string("--");
}

std::string upper(std::string s) {
    for (char& c : s) c = char(std::toupper((unsigned char)c));
    return s;
}

struct App {
    arm::serial_link& link;
    std::string path;
    arm::partial_calibration saved{};
    std::array<Joint, kJoints> joints{};
    int sel = 0;
    Step step = Step::List;
    bool quitArmed = false;
    std::string message;
    std::string arduino;

    Joint& joint() { return joints[std::size_t(sel)]; }

    // Moves every servo that's on, e.g. "P - 95.0 120.0 - - -"; "-" tells the Arduino to leave that servo alone.
    void send() {
        std::ostringstream line;
        line << "P" << std::fixed << std::setprecision(1);
        for (const Joint& j : joints) {
            if (j.driven) line << ' ' << j.cmd;
            else line << " -";
        }
        link.write_line(line.str());
    }

    void moveTo(double cmd) {
        joint().cmd = std::clamp(cmd, 0.0, 180.0);
        joint().driven = true;
        send();
    }

    void release(int i) {
        link.write_line("R " + std::to_string(i));
        joints[std::size_t(i)].driven = false;
    }

    void releaseAll() {
        link.write_line("R");
        for (Joint& j : joints) j.driven = false;
    }

    // Where joint `i` is now, relative to the table, if it's calibrated.
    std::optional<double> tableAngleNow(int i) const {
        const auto& c = saved[std::size_t(i)];
        if (!c) return std::nullopt;
        return angleOf(*c, joints[std::size_t(i)].cmd) - kKinematicsOffset[std::size_t(i)];
    }

    arm::servo_cal calOf(const Joint& j) const {
        const double off = kKinematicsOffset[std::size_t(j.id)];
        return {.joint_a = j.firstAngle + off, .cmd_a = *j.firstCmd,
                .joint_b = j.secondAngle + off, .cmd_b = *j.secondCmd,
                .min_cmd = 0, .max_cmd = 180};
    }

    void save() {
        saved[std::size_t(sel)] = calOf(joint());
        arm::save_calibration(path, saved);
    }

    void enter(Step s) {
        step = s;
        message.clear();
        Joint& j = joint();
        switch (s) {
            case Step::List:
                break;
            case Step::Horn: {  // the horn may move, so anything measured before is stale
                for (const auto& [h, angle] : holdsFor(sel)) {
                    if (!saved[std::size_t(h)]) {
                        step = Step::List;
                        message = "do " + std::string(arm::kJointNames[std::size_t(h)]) + " first";
                        return;
                    }
                    Joint& held = joints[std::size_t(h)];
                    held.cmd = std::clamp(commandFor(*saved[std::size_t(h)], angle), 0.0, 180.0);
                    held.driven = true;
                }
                j.firstAngle = poseAngles(sel)[0];
                j.secondAngle = poseAngles(sel)[1];
                j.skipped.clear();
                j.firstCmd.reset();
                j.secondCmd.reset();
                moveTo(90);
                break;
            }
            case Step::First:
                moveTo(j.firstCmd.value_or(j.cmd));
                break;
            case Step::Second:
                moveTo(j.secondCmd.value_or(j.cmd));
                break;
            case Step::Done:
                save();
                break;
        }
    }

    void accept() {
        Joint& j = joint();
        switch (step) {
            case Step::List: enter(Step::Horn); break;
            case Step::Horn: enter(Step::First); break;
            case Step::First:
                j.firstCmd = j.cmd;
                if (j.secondAngle == j.firstAngle) j.secondAngle = j.bestSecondAngle();
                enter(Step::Second);
                break;
            case Step::Second:
                if (std::abs(j.cmd - *j.firstCmd) < kMinPoseGap) { message = "that's hardly moved from the 2nd step; keep going"; break; }
                j.secondCmd = j.cmd;
                enter(Step::Done);
                break;
            case Step::Done:
                for (int k = 1; k <= kJoints; ++k) {
                    if (!saved[std::size_t((sel + k) % kJoints)]) { sel = (sel + k) % kJoints; break; }
                }
                enter(Step::List);
                break;
        }
    }

    void back() {
        switch (step) {
            case Step::List: case Step::Horn: enter(Step::List); break;
            case Step::First: enter(Step::Horn); break;
            case Step::Second: enter(Step::First); break;
            case Step::Done: enter(Step::Second); break;
        }
    }

    // Returns false to quit.
    bool handle(int key) {
        const bool quitKey = key == 'q' || key == 3;
        if (!quitKey) quitArmed = false;

        if (quitKey) {
            const bool midJoint = step != Step::List && step != Step::Done;
            if (midJoint && !quitArmed) { quitArmed = true; message = "this joint isn't finished: q again to quit anyway"; return true; }
            return false;
        }

        if (key == kEscape) {
            release(sel);
            enter(Step::List);
            message = "released it (limp)";
            return true;
        }

        if (step == Step::List) {
            message.clear();
            if (key >= '1' && key <= '6') sel = key - '1';
            else if (key == kUp) sel = (sel + kJoints - 1) % kJoints;
            else if (key == kDown) sel = (sel + 1) % kJoints;
            else if (key == kRight) moveTo(joint().cmd + kBigStep);
            else if (key == kLeft) moveTo(joint().cmd - kBigStep);
            else if (key == ' ') { if (joint().driven) release(sel); else moveTo(joint().cmd); }
            else if (isEnter(key)) accept();
            return true;
        }

        Joint& j = joint();
        const bool canNudge = step == Step::First || step == Step::Second;
        switch (key) {
            case kRight: if (canNudge) { moveTo(j.cmd + kSmallStep); message.clear(); } break;
            case kLeft:  if (canNudge) { moveTo(j.cmd - kSmallStep); message.clear(); } break;
            case kUp:    if (canNudge) { moveTo(j.cmd + kBigStep); message.clear(); } break;
            case kDown:  if (canNudge) { moveTo(j.cmd - kBigStep); message.clear(); } break;
            case '\t':
                if (step == Step::First) j.nextFirstAngle();
                else if (step == Step::Second) j.nextSecondAngle();
                message.clear();
                break;
            default:
                if (isEnter(key)) accept();
                else if (isBackspace(key)) back();
        }
        return true;
    }

    std::string instructions() const {
        const Joint& j = joints[std::size_t(sel)];
        const std::string name = upper(arm::kJointNames[std::size_t(sel)]);
        const std::string holding = holdingText(sel);
        const char* nudge = "  <- ->  move 1      up/down  move 5      Backspace  go back\r\n";
        switch (step) {
            case Step::List:
                return "  up/down  pick joint      <- ->  move it 5      Space  on/limp\r\n"
                       "  Enter  calibrate it (its servo turns on at 90: hold the arm)      q  quit\r\n";
            case Step::Horn:
                return "  " + name + "   1 of 3: HORN\r\n\r\n"
                       "  Servo is at 90." + (holding.empty() ? "" : "  (" + holding + ")") + "\r\n"
                       "  Put the horn on where you want servo 90 to be.\r\n"
                       "  Suggested: " + looksLike(sel, kSuggestedMiddle[std::size_t(sel)]) + "\r\n"
                       "  Close is fine: the next two steps measure it exactly.\r\n\r\n"
                       "  Enter  done      Backspace  back to the list\r\n";
            case Step::First:
            case Step::Second: {
                const bool first = step == Step::First;
                const double angle = first ? j.firstAngle : j.secondAngle;
                Joint next = j;
                if (first) next.nextFirstAngle();
                else next.nextSecondAngle();
                const double nextAngle = first ? next.firstAngle : next.secondAngle;
                return "  " + name + "   " + (first ? "2" : "3") + " of 3: " + fmt(angle) + " DEGREES\r\n\r\n" +
                       (holding.empty() ? "" : "  (" + holding + ")\r\n") +
                       "  Arrows until:  " + looksLike(sel, angle) + "\r\n"
                       "  Enter  that's it                servo now: " + fmt(j.cmd) + "\r\n\r\n"
                       "  Can't reach it? Tab  use " + fmt(nextAngle) + " degrees instead\r\n\r\n" + nudge;
            }
            case Step::Done: {
                const arm::servo_cal c = calOf(j);
                const double off = kKinematicsOffset[std::size_t(sel)];
                double lo = angleOf(c, 0) - off, hi = angleOf(c, 180) - off;
                if (lo > hi) std::swap(lo, hi);
                return "  " + name + " saved:   " + fmt(j.firstAngle) + " degrees = servo " + fmt(*j.firstCmd) +
                       "     " + fmt(j.secondAngle) + " degrees = servo " + fmt(*j.secondCmd) + "\r\n"
                       "  Servo 0..180 reaches " + fmt(lo) + " .. " + fmt(hi) + " degrees.\r\n\r\n"
                       "  Enter  next joint      Backspace  redo step 3\r\n";
            }
        }
        return "";
    }

    void draw() const {
        std::string s = "\x1b[H\x1b[2J";
        s += "Voyaging Arm servo calibration    file: " + path + "\r\n\r\n";
        s += "      joint        servo  state   now       poses (degrees=servo)\r\n";
        for (int i = 0; i < kJoints; ++i) {
            const Joint& j = joints[std::size_t(i)];
            const auto now = tableAngleNow(i);
            char row[160];
            std::snprintf(row, sizeof row, " %s %d  %-11s %5.0f  %-6s  %-8s  %-8s %-8s %s\r\n",
                          i == sel ? ">" : " ", i + 1, arm::kJointNames[std::size_t(i)], j.cmd,
                          j.driven ? "on" : "limp", now ? (fmt(*now) + " deg").c_str() : "",
                          pose(j.firstAngle, j.firstCmd).c_str(), pose(j.secondAngle, j.secondCmd).c_str(),
                          saved[std::size_t(i)] ? "saved" : "");
            s += row;
        }
        s += "\r\n" + instructions() + "\r\n";
        s += "  Esc  emergency: this servo goes limp (it may drop!)\r\n\r\n";
        if (!message.empty()) s += "  " + message + "\r\n";
        if (!arduino.empty()) s += "  arduino: " + arduino + "\r\n";
        std::fwrite(s.data(), 1, s.size(), stdout);
        std::fflush(stdout);
    }
};

// Splits raw terminal bytes into keys, turning arrow escape sequences into Key values.
// An arrow can arrive split across reads, so an unfinished sequence stays in `pending` for the next
// call; with `flush` (no more input came) a leftover lone ESC is the Esc key.
template <class F>
void forEachKey(std::string& pending, bool flush, F&& f) {
    std::size_t i = 0;
    for (; i < pending.size(); ++i) {
        if (pending[i] != '\x1b') { f(int((unsigned char)pending[i])); continue; }
        if (i + 1 == pending.size() || (pending[i + 1] == '[' && i + 2 == pending.size())) {
            if (!flush) break;
            f(kEscape);
            i = pending.size();
            break;
        }
        if (pending[i + 1] != '[') { f(kEscape); continue; }
        switch (pending[i + 2]) {
            case 'A': f(kUp); break;
            case 'B': f(kDown); break;
            case 'C': f(kRight); break;
            case 'D': f(kLeft); break;
        }
        i += 2;
    }
    pending.erase(0, i);
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        std::fprintf(stderr, "usage: %s <serial port> [calibration file]\n  e.g. %s /dev/ttyACM0\n", argv[0], argv[0]);
        return 1;
    }
    const std::string path = argc > 2 ? argv[2] : ARM_CALIBRATION_FILE;

    try {
        const arm::partial_calibration existing = arm::load_partial_calibration(path);
        std::printf("connecting to %s...\n", argv[1]);
        arm::serial_link link(argv[1]);

        App app{.link = link, .path = path, .saved = existing};
        for (int i = 0; i < kJoints; ++i) {
            Joint& j = app.joints[std::size_t(i)];
            j.id = i;
            j.firstAngle = poseAngles(i)[0];
            j.secondAngle = poseAngles(i)[1];
            if (!existing[std::size_t(i)]) continue;
            const arm::servo_cal& c = *existing[std::size_t(i)];
            j.cmd = c.cmd_a;
            j.firstAngle = c.joint_a - kKinematicsOffset[std::size_t(i)];
            j.secondAngle = c.joint_b - kKinematicsOffset[std::size_t(i)];
            j.firstCmd = c.cmd_a;
            j.secondCmd = c.cmd_b;
        }
        while (app.sel < kJoints - 1 && existing[std::size_t(app.sel)]) ++app.sel;

        RawTerminal raw;
        bool running = true;
        bool redraw = true;
        std::string keys;  // read but not handled yet
        while (running) {
            if (redraw) app.draw();
            redraw = false;
            pollfd p{STDIN_FILENO, POLLIN, 0};
            const bool gotInput = ::poll(&p, 1, 20) > 0;
            if (gotInput) {
                char buf[256];
                const int n = int(::read(STDIN_FILENO, buf, sizeof buf));
                if (n > 0) keys.append(buf, std::size_t(n));
            }
            if (!keys.empty()) {
                forEachKey(keys, !gotInput, [&](int key) { if (running) running = app.handle(key); });
                redraw = true;
            }
            for (const std::string& l : link.poll_lines()) { app.arduino = l; redraw = true; }
        }
        app.releaseAll();
        std::printf("\x1b[H\x1b[2Jreleased all servos\n");
    } catch (const std::exception& e) {
        std::fprintf(stderr, "\n%s\n", e.what());
        return 1;
    }
    return 0;
}
