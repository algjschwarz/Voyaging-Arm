#pragma once
#include <string>
#include <string_view>
#include <vector>

namespace arm {
    // A text-line connection to the Arduino over a serial port (POSIX only).
    // Knows nothing about the arm: it just sends and receives lines.
    // Nothing here blocks except the constructor, so it is safe to call from a render loop.
    class serial_link
    {
    public:
        // Opens the port and waits (up to timeout_ms) for the board to print "READY".
        // Opening the port resets an Uno, so commands sent before that would be lost.
        // Throws std::runtime_error if the port can't be opened or the board never answers.
        explicit serial_link(const std::string& path, int baud = 115200, int timeout_ms = 3000);
        ~serial_link();

        serial_link(const serial_link&) = delete;
        serial_link& operator=(const serial_link&) = delete;

        // Sends `line` followed by '\n'. Throws std::runtime_error if the port has gone away.
        void write_line(std::string_view line);

        // Returns every complete line received since the last call (without the '\n').
        // Returns immediately; a half-received line stays buffered until the rest arrives.
        std::vector<std::string> poll_lines();

    private:
        int fd_ = -1;
        std::string rx_;
    };
}
