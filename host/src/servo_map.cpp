#include "arm/servo_map.hpp"
#include <algorithm>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <numbers>
#include <sstream>
#include <stdexcept>

namespace arm
{
    namespace
    {
        double to_servo(double radians, const servo_cal& c)
        {
            const double joint_deg = radians * 180.0 / std::numbers::pi;
            const double cmd = c.cmd_a + (joint_deg - c.joint_a) * (c.cmd_b - c.cmd_a) / (c.joint_b - c.joint_a);
            return std::clamp(cmd, c.min_cmd, c.max_cmd);
        }
    }

    std::array<double, 6> to_servo_commands(const arm_angles& a, const servo_calibration& cal)
    {
        return {
            to_servo(a.base, cal[0]),
            to_servo(a.shoulder, cal[1]),
            to_servo(a.elbow, cal[2]),
            to_servo(a.wrist, cal[3]),
            to_servo(a.wrist_spin, cal[4]),
            to_servo(a.gripper, cal[5]),
        };
    }

    std::string to_arduino_line(const std::array<double, 6>& servo_commands)
    {
        std::ostringstream line;
        line << "P" << std::fixed << std::setprecision(1);  // one decimal place, e.g. 45.3
        for (double command : servo_commands) line << ' ' << command;
        return line.str();
    }

    servo_calibration load_calibration(const std::string& path)
    {
        const partial_calibration partial = load_partial_calibration(path);
        servo_calibration cal;
        std::string missing;
        for (std::size_t i = 0; i < cal.size(); ++i) {
            if (partial[i]) cal[i] = *partial[i];
            else missing += std::string(" ") + kJointNames[i];
        }
        if (!missing.empty()) {
            throw std::runtime_error("not calibrated:" + missing + "\nrun arm_calibrate first");
        }
        return cal;
    }

    partial_calibration load_partial_calibration(const std::string& path)
    {
        partial_calibration cal{};
        std::ifstream in(path);
        if (!in) return cal;

        std::string line;
        for (int n = 1; std::getline(in, line); ++n) {
            line = line.substr(0, line.find('#'));
            std::istringstream ss(line);
            std::string name;
            if (!(ss >> name)) continue;  // blank or comment

            const auto it = std::find_if(kJointNames.begin(), kJointNames.end(),
                                         [&](const char* j) { return name == j; });
            if (it == kJointNames.end()) {
                throw std::runtime_error(path + ":" + std::to_string(n) + ": unknown joint '" + name + "'");
            }
            servo_cal c;
            if (!(ss >> c.joint_a >> c.cmd_a >> c.joint_b >> c.cmd_b >> c.min_cmd >> c.max_cmd)) {
                throw std::runtime_error(path + ":" + std::to_string(n) + ": expected 6 numbers after '" + name + "'");
            }
            if (c.joint_a == c.joint_b) {
                throw std::runtime_error(path + ":" + std::to_string(n) + ": the two poses need different joint angles");
            }
            cal[std::size_t(it - kJointNames.begin())] = c;
        }
        return cal;
    }

    void save_calibration(const std::string& path, const partial_calibration& cal)
    {
        std::ofstream out(path);
        if (!out) throw std::runtime_error("can't write " + path);
        out << "# Servo calibration, written by arm_calibrate. Joints not listed are never driven.\n"
               "# name        joint_a  cmd_a  joint_b  cmd_b  min_cmd  max_cmd\n";
        for (std::size_t i = 0; i < cal.size(); ++i) {
            if (!cal[i]) continue;
            const servo_cal& c = *cal[i];
            char buf[128];
            std::snprintf(buf, sizeof buf, "%-12s %8.1f %6.1f %8.1f %6.1f %8.1f %8.1f\n",
                          kJointNames[i], c.joint_a, c.cmd_a, c.joint_b, c.cmd_b, c.min_cmd, c.max_cmd);
            out << buf;
        }
    }
}
