#include <rclcpp/rclcpp.hpp>
#include <encoders_package/msg/encoder_state.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <array>
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <stdexcept>
#include <string>
#include <thread>

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

class EncodersDriver : public rclcpp::Node
{
public:
  EncodersDriver()
  : Node("encoders_driver")
  {
    const auto port = declare_parameter<std::string>(
      "port",
      "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5970065826-if00");

    // Mantiene solo l'ultimo stato in attesa di consegna.
    publisher_ =
      create_publisher<encoders_package::msg::EncoderState>(
        "encoders_state",
        rclcpp::SensorDataQoS().keep_last(1));

    resetService_ = create_service<std_srvs::srv::Trigger>(
      "reset_encoders",
      [this](
        const std_srvs::srv::Trigger::Request::SharedPtr,
        std_srvs::srv::Trigger::Response::SharedPtr response)
      {
        const char command = 'Z';
        ssize_t written;
        do {
          written = ::write(fd_, &command, 1);
        } while (written < 0 && errno == EINTR && rclcpp::ok());

        response->success = (written == 1);
        if (response->success) {
          response->message =
            "Comando Z accettato dal driver seriale; "
            "azzeramento non confermato dal firmware.";
        } else if (written < 0) {
          const int error = errno;
          response->message =
            std::string("Invio Z fallito: ") + std::strerror(error);
        } else {
          response->message = "Invio Z fallito: nessun byte scritto.";
        }
      });

    openSerial(port);

    RCLCPP_INFO(
      get_logger(),
      "Seriale: %s, 921600 baud. Topic: encoders_state",
      port.c_str());
  }

  ~EncodersDriver() override
  {
    if (fd_ >= 0) {
      ::close(fd_);
    }
  }

  void run()
  {
    std::array<char, 4096> buffer{};

    while (rclcpp::ok()) {
      pollfd descriptor{};
      descriptor.fd = fd_;
      descriptor.events = POLLIN;

      const int result = ::poll(&descriptor, 1, 100);

      if (result < 0) {
        if (errno == EINTR) {
          continue;
        }
        throwSystemError("poll");
      }

      if (result == 0) {
        continue;
      }

      if (descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) {
        throw std::runtime_error("Seriale disconnessa o non disponibile");
      }

      if (!(descriptor.revents & POLLIN)) {
        continue;
      }

      // Svuota i dati disponibili senza aspettare altri byte.
      while (rclcpp::ok()) {
        const ssize_t received =
          ::read(fd_, buffer.data(), buffer.size());

        if (received > 0) {
          consume(buffer.data(), static_cast<size_t>(received));
          continue;
        }

        if (received == 0) {
          break;
        }

        if (errno == EINTR) {
          continue;
        }

        if (errno == EAGAIN || errno == EWOULDBLOCK) {
          break;
        }

        throwSystemError("read");
      }
    }
  }

private:
  [[noreturn]] static void throwSystemError(const char * operation)
  {
    const int error = errno;
    throw std::runtime_error(
      std::string(operation) + ": " + std::strerror(error));
  }

  void openSerial(const std::string & port)
  {
    fd_ = ::open(
      port.c_str(),
      O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);

    if (fd_ < 0) {
      throwSystemError("open");
    }

    try {
      termios config{};

      if (::tcgetattr(fd_, &config) != 0) {
        throwSystemError("tcgetattr");
      }

      ::cfmakeraw(&config);

      if (::cfsetispeed(&config, B921600) != 0 ||
          ::cfsetospeed(&config, B921600) != 0) {
        throwSystemError("cfsetspeed");
      }

      config.c_cflag &= ~(CSIZE | PARENB | CSTOPB | CRTSCTS);
      config.c_cflag |= CS8 | CLOCAL | CREAD;
      config.c_iflag &= ~(IXON | IXOFF | IXANY);

      config.c_cc[VMIN] = 0;
      config.c_cc[VTIME] = 0;

      if (::tcsetattr(fd_, TCSANOW, &config) != 0) {
        throwSystemError("tcsetattr");
      }

      if (::tcflush(fd_, TCIFLUSH) != 0) {
        throwSystemError("tcflush");
      }
    } catch (...) {
      ::close(fd_);
      fd_ = -1;
      throw;
    }
  }

  void consume(const char * data, size_t length)
  {
    for (size_t i = 0; i < length; ++i) {
      const char character = data[i];

      // All'avvio, oppure dopo una riga troppo lunga,
      // riallinea il parser al prossimo terminatore.
      if (discardUntilNewline_) {
        if (character == '\n') {
          discardUntilNewline_ = false;
        }
        continue;
      }

      if (character == '\n') {
        const auto receptionStamp = now();

        // Accetta anche righe terminate con CRLF.
        if (!line_.empty() && line_.back() == '\r') {
          line_.pop_back();
        }

        publishLine(line_, receptionStamp);
        line_.clear();
        continue;
      }

      if (line_.size() >= MAX_LINE_LENGTH) {
        line_.clear();
        discardUntilNewline_ = true;
        continue;
      }

      line_.push_back(character);
    }
  }

  static bool parseLine(
    const std::string & line,
    std::array<double, 6> & values)
  {
    if (line.empty()) {
      return false;
    }

    const char * cursor = line.c_str();
    const char * limit = cursor + line.size();

    for (size_t i = 0; i < values.size(); ++i) {
      if (cursor >= limit) {
        return false;
      }

      errno = 0;
      char * end = nullptr;
      const double value = std::strtod(cursor, &end);

      if (end == cursor || errno == ERANGE || !std::isfinite(value)) {
        return false;
      }

      values[i] = value;

      if (i + 1 < values.size()) {
        if (end >= limit || *end != ',') {
          return false;
        }
        cursor = end + 1;
      } else {
        // Esattamente sei valori, nessun contenuto aggiuntivo.
        return end == limit;
      }
    }

    return false;
  }

  void publishLine(
    const std::string & line,
    const rclcpp::Time & receptionStamp)
  {
    std::array<double, 6> values{};

    // Scarta righe incomplete, diagnostiche o malformate.
    if (!parseLine(line, values)) {
      return;
    }

    encoders_package::msg::EncoderState message;
    message.stamp = receptionStamp;

    message.bottom.position = values[0];
    message.bottom.velocity = values[1];

    message.middle.position = values[2];
    message.middle.velocity = values[3];

    message.top.position = values[4];
    message.top.velocity = values[5];

    publisher_->publish(message);
  }

  static constexpr size_t MAX_LINE_LENGTH = 256;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr resetService_;

  int fd_ = -1;
  bool discardUntilNewline_ = true;
  std::string line_;

  rclcpp::Publisher<
    encoders_package::msg::EncoderState>::SharedPtr publisher_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int exitCode = 0;

  try {
    auto node = std::make_shared<EncodersDriver>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    // Le callback dei servizi non interrompono il ciclo di lettura seriale.
    std::thread serviceThread([&executor]() {
      try {
        executor.spin();
      } catch (const std::exception & error) {
        RCLCPP_ERROR(
          rclcpp::get_logger("encoders_driver"),
          "Executor: %s", error.what());
        rclcpp::shutdown();
      }
    });

    try {
      node->run();
    } catch (...) {
      executor.cancel();
      serviceThread.join();
      throw;
    }

    executor.cancel();
    serviceThread.join();
  } catch (const std::exception & error) {
    RCLCPP_ERROR(
      rclcpp::get_logger("encoders_driver"),
      "%s", error.what());
    exitCode = 1;
  }

  rclcpp::shutdown();
  return exitCode;
}