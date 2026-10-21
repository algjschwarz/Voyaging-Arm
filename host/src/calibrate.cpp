// Servo calibration for the Voyaging Arm, run from a terminal:
//   arm_calibrate /dev/ttyACM0 [calibration file]
//
// The servos have no position feedback, so a human does the measuring: step one servo with the
// arrow keys, and when the joint is where you want it, mark that command. Per joint you mark
// a zero pose, a second pose at a known angle, and the two limits. Save writes the calibration
// file the viewer reads. You can calibrate a joint at a time while building the arm; joints
// already in the file are kept.
//
// If a servo buzzes against a stop: SPACE releases it (goes limp), b steps back the way it came.

#include "arm/serial_link.hpp"
#include "arm/servo_map.hpp"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <iomanip>
#include <optional>
#include <poll.h>
#include <sstream>
#include <string>
#include <termios.h>
#include <unistd.h>

#ifndef ARM_CALIBRATION_FILE
#define ARM_CALIBRATION_FILE "servos.txt"
#endif

namespace {

constexpr int kJoints = 6;
constexpr double kSmallStep = 1.0;
constexpr double kBigStep = 5.0;
constexpr double kBackOff = 5.0;

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

// Everything marked so far for one joint. A joint is calibrated once all of it is set.
struct Joint {
    double cmd = 90;
    bool driven = false;
    int lastDir = 1;
    std::optional<double> zeroCmd;
    std::optional<double> secondAngle, secondCmd;
    std::optional<double> limitLow, limitHigh;

    bool complete() const { return zeroCmd && secondCmd && limitLow && limitHigh; }

    arm::servo_cal toCal() const {
        return {.joint_a = 0, .cmd_a = *zeroCmd, .joint_b = *secondAngle, .cmd_b = *secondCmd,
                .min_cmd = std::min(*limitLow, *limitHigh), .max_cmd = std::max(*limitLow, *limitHigh)};
    }
};

std::string fmt(const std::optional<double>& v) {
    return v ? std::to_string(int(*v + (*v < 0 ? -0.5 : 0.5))) : std::string("--");
}

struct App {
    arm::serial_link& link;
    std::string path;
    std::array<Joint, kJoints> joints{};
    int sel = 0;
    bool dirty = false;
    bool quitArmed = false;
    std::string message = "pick a joint with 1-6";
    std::string arduino;
    // typing a number after g or x
    char promptFor = 0;
    std::string promptText;

    // Moves just the selected servo, e.g. "P - 95.0 - - - -"; "-" tells the Arduino to leave that servo alone.
    void sendSelected() {
        std::ostringstream line;
        line << "P" << std::fixed << std::setprecision(1);
        for (int i = 0; i < kJoints; ++i) {
            if (i == sel) line << ' ' << joints[std::size_t(i)].cmd;
            else line << " -";
        }
        link.write_line(line.str());
        joints[std::size_t(sel)].driven = true;
    }

    void moveBy(double delta, bool rememberDirection = true) {
        Joint& j = joints[std::size_t(sel)];
        j.cmd = std::clamp(j.cmd + delta, 0.0, 180.0);
        if (rememberDirection) j.lastDir = delta > 0 ? 1 : -1;
        sendSelected();
        message.clear();
    }

    void release(int i) {
        link.write_line("R " + std::to_string(i));
        joints[std::size_t(i)].driven = false;
    }

    void releaseAll() {
        link.write_line("R");
        for (Joint& j : joints) j.driven = false;
    }

    void save() {
        arm::partial_calibration cal{};
        std::string incomplete;
        for (int i = 0; i < kJoints; ++i) {
            const Joint& j = joints[std::size_t(i)];
            if (j.complete()) cal[std::size_t(i)] = j.toCal();
            else if (j.zeroCmd || j.secondCmd || j.limitLow || j.limitHigh) incomplete += std::string(" ") + arm::kJointNames[std::size_t(i)];
        }
        arm::save_calibration(path, cal);
        dirty = false;
        message = "saved to " + path;
        if (!incomplete.empty()) message += "  (not saved, still incomplete:" + incomplete + ")";
    }

    void finishPrompt() {
        Joint& j = joints[std::size_t(sel)];
        char* end = nullptr;
        const double v = std::strtod(promptText.c_str(), &end);
        const bool empty = promptText.empty();
        const bool ok = empty || (end && *end == '\0');
        if (promptFor == 'g') {
            if (!empty && ok) moveBy(v - j.cmd);
        } else if (promptFor == 'x') {
            const double angle = empty ? 90.0 : v;
            if (!ok) message = "not a number";
            else if (angle == 0) message = "the second pose needs a non-zero angle (z marks 0)";
            else if (j.zeroCmd && *j.zeroCmd == j.cmd) message = "that's the zero command; move the joint to the second pose first";
            else {
                j.secondAngle = angle;
                j.secondCmd = j.cmd;
                dirty = true;
                message = "marked " + fmt(angle) + " deg at command " + fmt(j.cmd);
            }
        }
        promptFor = 0;
        promptText.clear();
    }

    // Returns false to quit.
    bool handle(int key) {
        if (promptFor) {
            if (key == '\n' || key == '\r') finishPrompt();
            else if (key == kEscape) { promptFor = 0; promptText.clear(); }
            else if (key == 127 || key == 8) { if (!promptText.empty()) promptText.pop_back(); }
            else if ((key >= '0' && key <= '9') || key == '.' || key == '-') promptText += char(key);
            return true;
        }

        if (key != 'q' && key != 3) quitArmed = false;
        Joint& j = joints[std::size_t(sel)];
        switch (key) {
            case '1': case '2': case '3': case '4': case '5': case '6':
                sel = key - '1';
                message = std::string("selected ") + arm::kJointNames[std::size_t(sel)] +
                          (joints[std::size_t(sel)].driven ? "" : "; first move snaps it to the shown command");
                break;
            case kRight: moveBy(kSmallStep); break;
            case kLeft:  moveBy(-kSmallStep); break;
            case kUp:    moveBy(kBigStep); break;
            case kDown:  moveBy(-kBigStep); break;
            case 'b': moveBy(-j.lastDir * kBackOff, false); message = "backed off"; break;
            case ' ': release(sel); message = "released (limp)"; break;
            case 'r': releaseAll(); message = "released all"; break;
            case 'g': promptFor = 'g'; break;
            case 'x': promptFor = 'x'; break;
            case 'z':
                if (j.secondCmd && *j.secondCmd == j.cmd) { message = "that's the second pose's command; move to zero first"; break; }
                j.zeroCmd = j.cmd; dirty = true; message = "marked 0 deg at command " + fmt(j.cmd); break;
            case '[': j.limitLow = j.cmd; dirty = true; message = "marked low limit at " + fmt(j.cmd); break;
            case ']': j.limitHigh = j.cmd; dirty = true; message = "marked high limit at " + fmt(j.cmd); break;
            case 's': save(); break;
            case 'q': case 3:
                if (dirty && !quitArmed) { quitArmed = true; message = "unsaved marks: s to save, q again to quit anyway"; break; }
                return false;
        }
        return true;
    }

    void draw() const {
        std::string s = "\x1b[H\x1b[2J";
        s += "Voyaging Arm servo calibration    file: " + path + (dirty ? "  (unsaved)" : "") + "\r\n\r\n";
        s += "      joint        cmd   state      zero   second pose    limits     \r\n";
        for (int i = 0; i < kJoints; ++i) {
            const Joint& j = joints[std::size_t(i)];
            char row[160];
            std::snprintf(row, sizeof row, " %s %d  %-11s %5.0f   %-9s  %4s   %4s @ %-4s    %4s..%-4s  %s\r\n",
                          i == sel ? ">" : " ", i + 1, arm::kJointNames[std::size_t(i)], j.cmd,
                          j.driven ? "DRIVEN" : "limp", fmt(j.zeroCmd).c_str(),
                          fmt(j.secondAngle).c_str(), fmt(j.secondCmd).c_str(),
                          fmt(j.limitLow).c_str(), fmt(j.limitHigh).c_str(), j.complete() ? "ok" : "");
            s += row;
        }
        s += "\r\n";
        s += "  1-6        pick joint            <- ->         step 1      up down         step 5\r\n";
        s += "  SPACE      RELEASE (limp)        b             back off 5  r               release all\r\n";
        s += "  g          go to a command       z             this pose is 0 deg\r\n";
        s += "  x          this pose is __ deg   [ ]           this is the low / high limit\r\n";
        s += "  s          save                  q             quit (releases everything)\r\n\r\n";
        s += "  Tip: find a limit by stepping until it stops moving or buzzes, back off, then mark it.\r\n";
        s += "  Released joints may drop under gravity, so hold the arm or use b instead.\r\n\r\n";
        if (promptFor == 'g') s += "  go to command: " + promptText + "_   (Enter, Esc cancels)\r\n";
        else if (promptFor == 'x') s += "  this pose is how many degrees? [90] " + promptText + "_   (Enter, Esc cancels)\r\n";
        else s += "  " + message + "\r\n";
        if (!arduino.empty()) s += "  arduino: " + arduino + "\r\n";
        std::fwrite(s.data(), 1, s.size(), stdout);
        std::fflush(stdout);
    }
};

// Splits raw terminal bytes into keys, turning arrow escape sequences into Key values.
template <class F>
void forEachKey(const char* buf, int n, F&& f) {
    for (int i = 0; i < n; ++i) {
        if (buf[i] == '\x1b') {
            if (i + 2 < n && buf[i + 1] == '[') {
                switch (buf[i + 2]) {
                    case 'A': f(kUp); break;
                    case 'B': f(kDown); break;
                    case 'C': f(kRight); break;
                    case 'D': f(kLeft); break;
                }
                i += 2;
            } else {
                f(kEscape);
            }
        } else {
            f(int((unsigned char)buf[i]));
        }
    }
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

        App app{.link = link, .path = path};
        for (int i = 0; i < kJoints; ++i) {
            if (!existing[std::size_t(i)]) continue;
            const arm::servo_cal& c = *existing[std::size_t(i)];
            Joint& j = app.joints[std::size_t(i)];
            j.cmd = c.cmd_a;
            j.zeroCmd = c.cmd_a;
            j.secondAngle = c.joint_b;
            j.secondCmd = c.cmd_b;
            j.limitLow = c.min_cmd;
            j.limitHigh = c.max_cmd;
        }

        RawTerminal raw;
        bool running = true;
        bool redraw = true;
        while (running) {
            if (redraw) app.draw();
            redraw = false;
            pollfd p{STDIN_FILENO, POLLIN, 0};
            if (::poll(&p, 1, 50) > 0) {
                char buf[32];
                const int n = int(::read(STDIN_FILENO, buf, sizeof buf));
                forEachKey(buf, n, [&](int key) { if (running) running = app.handle(key); });
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
