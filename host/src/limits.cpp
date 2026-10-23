#include "arm/limits.hpp"
#include <array>
#include <cstdio>
#include <utility>

namespace arm
{
    namespace
    {
        double to_deg(double radians) { return radians * 180.0 / std::numbers::pi; }

        std::string past(const char* name, double limit)
        {
            char buf[64];
            std::snprintf(buf, sizeof buf, "%s past %.0f deg", name, to_deg(limit));
            return buf;
        }
    }

    std::string why_unsafe(const arm_angles& pose, const arm_constants& constants, const arm_limits& limits)
    {
        struct joint { const char* name; double angle; joint_range range; };
        const std::array<joint, 4> joints{{
            {"base", pose.base, limits.base},
            {"shoulder", pose.shoulder, limits.shoulder},
            {"elbow", pose.elbow, limits.elbow},
            {"wrist", pose.wrist, limits.wrist},
        }};
        for (const auto& [name, angle, range] : joints) {
            if (angle < range.min) return past(name, range.min);
            if (angle > range.max) return past(name, range.max);
        }

        // The shoulder axis never moves, so only the points after it can reach the table.
        const joint_chain chain = joint_points(pose, constants);
        const std::array<std::pair<const char*, std::size_t>, 4> points{{
            {"elbow", 2}, {"wrist", 3}, {"wrist", 4}, {"tip", 5},
        }};
        for (const auto& [name, i] : points) {
            if (chain[i].z < limits.ground_clearance) {
                char buf[64];
                std::snprintf(buf, sizeof buf, "%s %.0f mm above the table", name, chain[i].z);
                return buf;
            }
        }
        return {};
    }
}
