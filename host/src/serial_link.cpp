#include "arm/serial_link.hpp"
#include <cerrno>
#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <stdexcept>
#include <termios.h>
#include <thread>
#include <unistd.h>

namespace arm
{
    namespace
    {
        [[noreturn]] void fail(const std::string& what)
        {
            throw std::runtime_error(what + ": " + std::strerror(errno));
        }

        speed_t to_speed(int baud)
        {
            switch (baud) {
                case 9600: return B9600;
                case 19200: return B19200;
                case 38400: return B38400;
                case 57600: return B57600;
                case 115200: return B115200;
                case 230400: return B230400;
                default: throw std::runtime_error("unsupported baud rate " + std::to_string(baud));
            }
        }
    }

    serial_link::serial_link(const std::string& path, int baud, int timeout_ms)
    {
        const speed_t speed = to_speed(baud);
        fd_ = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd_ < 0) fail("open " + path);

        termios tty{};
        if (tcgetattr(fd_, &tty) != 0) { ::close(fd_); fail("tcgetattr " + path); }
        cfmakeraw(&tty);                  // 8 data bits, no parity, no echo, no line editing
        tty.c_cflag |= CLOCAL | CREAD;    // ignore modem lines, enable the receiver
        tty.c_cflag &= ~CSTOPB;           // 1 stop bit
        cfsetispeed(&tty, speed);
        cfsetospeed(&tty, speed);
        if (tcsetattr(fd_, TCSANOW, &tty) != 0) { ::close(fd_); fail("tcsetattr " + path); }
        tcflush(fd_, TCIOFLUSH);

        // wait for the board to come out of reset
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
        while (std::chrono::steady_clock::now() < deadline) {
            for (const std::string& l : poll_lines()) {
                if (l == "READY") return;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        ::close(fd_);
        throw std::runtime_error("no READY from " + path + " (is the arm firmware flashed?)");
    }

    serial_link::~serial_link()
    {
        if (fd_ >= 0) ::close(fd_);
    }

    void serial_link::write_line(std::string_view line)
    {
        std::string out(line);
        out += '\n';
        std::size_t sent = 0;
        while (sent < out.size()) {
            const ssize_t n = ::write(fd_, out.data() + sent, out.size() - sent);
            if (n > 0) { sent += std::size_t(n); continue; }
            if (n < 0 && errno != EAGAIN && errno != EINTR) fail("serial write");
            // kernel buffer is full (only happens if the board stopped reading); wait briefly for room
            pollfd p{fd_, POLLOUT, 0};
            ::poll(&p, 1, 10);
        }
    }

    std::vector<std::string> serial_link::poll_lines()
    {
        char buf[256];
        while (true) {
            const ssize_t n = ::read(fd_, buf, sizeof buf);
            if (n > 0) { rx_.append(buf, std::size_t(n)); continue; }
            if (n < 0 && errno == EINTR) continue;
            if (n < 0 && errno != EAGAIN) fail("serial read");
            break;  // nothing more available right now
        }

        std::vector<std::string> lines;
        std::size_t start = 0, nl;
        while ((nl = rx_.find('\n', start)) != std::string::npos) {
            std::string l = rx_.substr(start, nl - start);
            if (!l.empty() && l.back() == '\r') l.pop_back();  // Serial.println sends "\r\n"
            lines.push_back(std::move(l));
            start = nl + 1;
        }
        rx_.erase(0, start);
        return lines;
    }
}
