// Servo calibration for the Voyaging Arm, run from a terminal:
//   arm_calibrate /dev/ttyACM0 [calibration file]
//
// The servos have no position feedback, so a human does the looking. The program walks one joint
// through four steps:
//   1. 0 degrees      nudge with the arrows until it's there, Enter
//   2. 90 degrees     same. If the servo runs out of travel first (a horn a spline off), Tab picks
//                     another angle instead (180, -90, then 45s): any two different ones work.
// The limits need no angle: they're stored as commands, and the line through the two poses
// says what angle any command is.
//   3. one limit      the servo creeps on its own; press any key the moment it stops or buzzes,
//   4. other limit    it backs off, you nudge until it's just clear of the edge, Enter
// Each finished joint is saved straight away; joints already in the file are kept.

#include "arm/serial_link.hpp"
#include "arm/servo_map.hpp"

#include <algorithm>
#include <chrono>
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
#include <vector>

#ifndef ARM_CALIBRATION_FILE
#define ARM_CALIBRATION_FILE "servos.txt"
#endif

namespace {

using Clock = std::chrono::steady_clock;

constexpr int kJoints = 6;
constexpr double kSmallStep = 1.0;
constexpr double kBigStep = 5.0;
constexpr double kBackOff = 3.0;                           // after you stop a creep, to get off the edge
constexpr auto kCreepEvery = std::chrono::milliseconds(120);  // 1 deg per tick, ~8 deg/s
constexpr double kMinPoseGap = 10.0;                       // commands closer than this can't be two different poses

// You mark where each joint reads two right angles, in your own terms (base: 90 = forward).
// This turns your angle into the one the kinematics use (base: 0 = forward), so the two can differ.
constexpr std::array<double, kJoints> kKinematicsOffset{-90, 0, 0, 0, 0, 0};

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
enum class Step { List, PowerOn, First, Second, LimitA, LimitB, Done };

// Angles a pose can be marked at, easiest to line up by eye first. Any two different ones work.
constexpr std::array<double, 7> kPoseAngles{0, 90, 180, -90, 45, 135, -45};

// The pose angles to offer after `first`, best partner first: 90 away, then 45 or 135, then 180.
// Angles already skipped as out of reach go last.
std::vector<double> partnersOf(double first, const std::vector<double>& skipped) {
    std::vector<double> out;
    for (double a : kPoseAngles) if (a != first) out.push_back(a);
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
    double cmd = 90;
    bool driven = false;
    double firstAngle = 0, secondAngle = 90;  // the angles the two poses are at, your terms
    std::optional<double> firstCmd, secondCmd, limitA, limitB;
    std::vector<double> skipped;  // angles Tabbed past, so the servo can't reach them

    // Tab: the next angle to try for a pose the servo can't reach.
    void nextFirstAngle() {
        skipped.push_back(firstAngle);
        firstAngle = nextIn({kPoseAngles.begin(), kPoseAngles.end()}, firstAngle);
    }
    void nextSecondAngle() {
        skipped.push_back(secondAngle);
        secondAngle = nextIn(partnersOf(firstAngle, skipped), secondAngle);
    }
    double bestSecondAngle() const { return partnersOf(firstAngle, skipped).front(); }

    bool complete() const { return firstCmd && secondCmd && limitA && limitB; }
};

std::string fmt(const std::optional<double>& v) {
    return v ? std::to_string(int(std::lround(*v))) : std::string("--");
}

// "90@85": 90 degrees is at command 85.
std::string pose(double angle, const std::optional<double>& cmd) {
    return cmd ? fmt(angle) + "@" + fmt(cmd) : std::string("--");
}

struct App {
    arm::serial_link& link;
    std::string path;
    arm::partial_calibration saved{};
    std::array<Joint, kJoints> joints{};
    int sel = 0;
    Step step = Step::List;
    int creepDir = 0;  // non-zero while the servo is creeping toward a limit
    Clock::time_point nextCreep{};
    bool quitArmed = false;
    std::string message;
    std::string arduino;

    Joint& joint() { return joints[std::size_t(sel)]; }

    // Moves just the selected servo, e.g. "P - 95.0 - - - -"; "-" tells the Arduino to leave that servo alone.
    void send() {
        std::ostringstream line;
        line << "P" << std::fixed << std::setprecision(1);
        for (int i = 0; i < kJoints; ++i) {
            if (i == sel) line << ' ' << joint().cmd;
            else line << " -";
        }
        link.write_line(line.str());
        joint().driven = true;
    }

    void moveTo(double cmd) {
        joint().cmd = std::clamp(cmd, 0.0, 180.0);
        send();
    }

    void releaseAll() {
        link.write_line("R");
        for (Joint& j : joints) j.driven = false;
    }

    void save() {
        const Joint& j = joint();
        saved[std::size_t(sel)] = arm::servo_cal{
            .joint_a = j.firstAngle + kKinematicsOffset[std::size_t(sel)], .cmd_a = *j.firstCmd,
            .joint_b = j.secondAngle + kKinematicsOffset[std::size_t(sel)], .cmd_b = *j.secondCmd,
            .min_cmd = std::min(*j.limitA, *j.limitB), .max_cmd = std::max(*j.limitA, *j.limitB)};
        arm::save_calibration(path, saved);
    }

    void enter(Step s) {
        step = s;
        creepDir = 0;
        message.clear();
        Joint& j = joint();
        switch (s) {
            case Step::List:
                break;
            case Step::PowerOn:
                if (j.driven) enter(Step::First);
                break;
            case Step::First:
                moveTo(j.firstCmd.value_or(j.cmd));
                break;
            case Step::Second:
                moveTo(j.secondCmd.value_or(*j.firstCmd));
                break;
            case Step::LimitA:
            case Step::LimitB:
                moveTo(*j.firstCmd);
                creepDir = s == Step::LimitA ? -1 : 1;
                nextCreep = Clock::now() + std::chrono::milliseconds(600);  // let it get back to zero first
                break;
            case Step::Done:
                save();
                moveTo(*j.firstCmd);
                break;
        }
    }

    void stopCreep() {
        moveTo(joint().cmd - creepDir * kBackOff);
        creepDir = 0;
        message = "stopped and backed off a little";
    }

    // Called every loop; returns true if it moved the servo.
    bool tick() {
        if (!creepDir || Clock::now() < nextCreep) return false;
        nextCreep += kCreepEvery;
        const double next = joint().cmd + creepDir;
        if (next < 0 || next > 180) {
            creepDir = 0;
            message = "reached the end of the servo's own range without hitting anything";
            return true;
        }
        moveTo(next);
        return true;
    }

    void accept() {
        Joint& j = joint();
        switch (step) {
            case Step::List: enter(Step::PowerOn); break;
            case Step::PowerOn: enter(Step::First); break;
            case Step::First:
                j.firstCmd = j.cmd;
                if (!j.secondCmd || j.secondAngle == j.firstAngle) j.secondAngle = j.bestSecondAngle();
                enter(Step::Second);
                break;
            case Step::Second:
                if (std::abs(j.cmd - *j.firstCmd) < kMinPoseGap) { message = "that's hardly moved from the first pose; keep going"; break; }
                j.secondCmd = j.cmd;
                enter(Step::LimitA);
                break;
            // It already reached both poses, so the edges can't be between them.
            case Step::LimitA:
                if (j.cmd > std::min(*j.firstCmd, *j.secondCmd)) { message = "it got further than this during the poses; the edge is past it"; break; }
                j.limitA = j.cmd;
                enter(Step::LimitB);
                break;
            case Step::LimitB:
                if (j.cmd < std::max(*j.firstCmd, *j.secondCmd)) { message = "it got further than this during the poses; the edge is past it"; break; }
                j.limitB = j.cmd;
                enter(Step::Done);
                break;
            case Step::Done:
                for (int k = 1; k <= kJoints; ++k) {
                    if (!joints[std::size_t((sel + k) % kJoints)].complete()) { sel = (sel + k) % kJoints; break; }
                }
                enter(Step::List);
                break;
        }
    }

    void back() {
        switch (step) {
            case Step::List: case Step::PowerOn: case Step::First: enter(Step::List); break;
            case Step::Second: enter(Step::First); break;
            case Step::LimitA: enter(Step::Second); break;
            case Step::LimitB: enter(Step::LimitA); break;
            case Step::Done: enter(Step::LimitB); break;
        }
    }

    // Returns false to quit.
    bool handle(int key) {
        const bool quitKey = key == 'q' || key == 3;
        if (!quitKey) quitArmed = false;

        if (creepDir) {  // any key stops a creep, before it does anything else
            stopCreep();
            if (!quitKey && key != kEscape) return true;
        }

        if (quitKey) {
            const bool midJoint = step != Step::List && step != Step::Done;
            if (midJoint && !quitArmed) { quitArmed = true; message = "this joint isn't finished: q again to quit anyway"; return true; }
            return false;
        }

        if (step == Step::List) {
            if (key >= '1' && key <= '6') sel = key - '1';
            else if (key == kUp) sel = (sel + kJoints - 1) % kJoints;
            else if (key == kDown) sel = (sel + 1) % kJoints;
            else if (isEnter(key)) accept();
            return true;
        }

        Joint& j = joint();
        const bool canNudge = step == Step::First || step == Step::Second || step == Step::LimitA || step == Step::LimitB;
        switch (key) {
            case kRight: if (canNudge) { moveTo(joint().cmd + kSmallStep); message.clear(); } break;
            case kLeft:  if (canNudge) { moveTo(joint().cmd - kSmallStep); message.clear(); } break;
            case kUp:    if (canNudge) { moveTo(joint().cmd + kBigStep); message.clear(); } break;
            case kDown:  if (canNudge) { moveTo(joint().cmd - kBigStep); message.clear(); } break;
            case '\t':
                if (step == Step::First) j.nextFirstAngle();
                else if (step == Step::Second) j.nextSecondAngle();
                message.clear();
                break;
            case kEscape:
                link.write_line("R " + std::to_string(sel));
                joint().driven = false;
                enter(Step::List);
                message = "released it (limp)";
                break;
            default:
                if (isEnter(key)) accept();
                else if (isBackspace(key)) back();
        }
        return true;
    }

    std::string instructions() const {
        const Joint& j = joints[std::size_t(sel)];
        const std::string name = arm::kJointNames[std::size_t(sel)];
        const char* nudge = "  <- ->  nudge 1      up/down  nudge 5      Enter  that's it      Backspace  go back a step\r\n";
        switch (step) {
            case Step::List:
                return "  Pick a joint with up/down or 1-6, then press Enter.\r\n\r\n"
                       "  Enter  start      q  quit\r\n";
            case Step::PowerOn:
                return "  " + name + ": the servo is off right now.\r\n"
                       "  Press Enter to turn it on. It will move to command " + fmt(j.cmd) +
                       ", so hold the arm if it might swing.\r\n\r\n"
                       "  Enter  turn on      Backspace  back to the list\r\n";
            case Step::First:
            case Step::Second: {
                const bool first = step == Step::First;
                const std::string angle = fmt(first ? j.firstAngle : j.secondAngle);
                Joint next = j;
                if (first) next.nextFirstAngle();
                else next.nextSecondAngle();
                return "  " + name + "  step " + (first ? "1" : "2") + " of 4: " + angle + " DEGREES\r\n\r\n"
                       "  Nudge until it's at " + angle + " degrees.\r\n"
                       "  Can't get there? Tab  use " + fmt(first ? next.firstAngle : next.secondAngle) +
                       " degrees instead.\r\n\r\n" + nudge;
            }
            case Step::LimitA:
            case Step::LimitB: {
                std::string s = "  " + name + "  step " + (step == Step::LimitA ? "3" : "4") + " of 4: " +
                                (step == Step::LimitA ? "FIRST" : "OTHER") + " LIMIT\r\n\r\n";
                if (creepDir) {
                    s += "  It's moving slowly on its own. Watch it.\r\n"
                         "  Press ANY KEY the moment it hits something, stops moving, or buzzes.\r\n";
                } else {
                    s += "  Nudge until it's right at the edge but NOT buzzing or pushing, then Enter.\r\n\r\n" +
                         std::string(nudge);
                }
                return s;
            }
            case Step::Done:
                return "  " + name + " done and saved to " + path + ".\r\n\r\n"
                       "  Enter  next joint      Backspace  redo the last limit\r\n";
        }
        return "";
    }

    void draw() const {
        std::string s = "\x1b[H\x1b[2J";
        s += "Voyaging Arm servo calibration    file: " + path + "\r\n";
        s += "Angles count counter-clockwise (base: seen from above; arm joints: seen from the arm's right side).\r\n\r\n";
        s += "      joint        cmd   state     pose 1    pose 2     limits\r\n";
        for (int i = 0; i < kJoints; ++i) {
            const Joint& j = joints[std::size_t(i)];
            char row[160];
            std::snprintf(row, sizeof row, " %s %d  %-11s %5.0f   %-7s  %-8s  %-8s   %4s..%-4s %s\r\n",
                          i == sel ? ">" : " ", i + 1, arm::kJointNames[std::size_t(i)], j.cmd,
                          j.driven ? "on" : "limp", pose(j.firstAngle, j.firstCmd).c_str(), pose(j.secondAngle, j.secondCmd).c_str(),
                          fmt(j.limitA).c_str(), fmt(j.limitB).c_str(),
                          saved[std::size_t(i)] ? "saved" : "");
            s += row;
        }
        s += "\r\n" + instructions() + "\r\n";
        s += "  Esc  emergency: this servo goes limp (it may drop!)      q  quit\r\n\r\n";
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
            if (!existing[std::size_t(i)]) continue;
            const arm::servo_cal& c = *existing[std::size_t(i)];
            Joint& j = app.joints[std::size_t(i)];
            j.cmd = c.cmd_a;
            j.firstAngle = c.joint_a - kKinematicsOffset[std::size_t(i)];
            j.secondAngle = c.joint_b - kKinematicsOffset[std::size_t(i)];
            j.firstCmd = c.cmd_a;
            j.secondCmd = c.cmd_b;
            j.limitA = c.min_cmd;
            j.limitB = c.max_cmd;
        }
        while (app.sel < kJoints - 1 && app.joints[std::size_t(app.sel)].complete()) ++app.sel;

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
            if (app.tick()) redraw = true;
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
