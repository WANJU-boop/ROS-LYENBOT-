// 串口读取 + ROS2 发布（读线程阻塞，主线程 spin）
// 文件：serial_topic_pub.cpp

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <iostream>
#include <iomanip>
#include <string>
#include <vector>
#include <thread>
#include <atomic>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>

static speed_t baud_to_flag(int b) {
  switch (b) {
    case 9600:   return B9600;
    case 19200:  return B19200;
    case 38400:  return B38400;
    case 57600:  return B57600;
    case 115200: return B115200;
    default:     return B9600;
  }
}

class SerialTopicPub : public rclcpp::Node {
public:
  SerialTopicPub()
  : rclcpp::Node("serial_topic_pub"), fd_(-1), stop_(false)
  {
    // —— 参数（可通过 --ros-args -p 覆盖）——
    port_  = this->declare_parameter<std::string>("port",  "/dev/ttyACM0");
    baud_  = this->declare_parameter<int>("baud", 9600);
    topic_ = this->declare_parameter<std::string>("topic", "RS485");
    // eol 为空字符串：按“块”发布；比如设 "\r\n" 可按行分割发布
    eol_   = this->declare_parameter<std::string>("eol",   "");

    pub_ = this->create_publisher<std_msgs::msg::String>(topic_, 10);

    if (!open_and_config()) {
      RCLCPP_FATAL(this->get_logger(), "Open %s @ %d failed: %s",
                   port_.c_str(), baud_, std::strerror(errno));
      throw std::runtime_error("serial open failed");
    }

    // 启动读线程（阻塞式 read）
    th_ = std::thread([this]{ this->read_loop(); });

    RCLCPP_INFO(this->get_logger(),
      "Started. port=%s baud=%d topic=%s eol='%s'",
      port_.c_str(), baud_, topic_.c_str(), eol_.c_str());
  }

  ~SerialTopicPub() override {
    // 通知线程退出，并用“关闭 fd”来打断阻塞的 read()
    stop_ = true;
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
    if (th_.joinable()) th_.join();
  }

private:
  bool open_and_config() {
    fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY); // 阻塞模式
    if (fd_ < 0) {
      RCLCPP_ERROR(this->get_logger(), "open: %s", std::strerror(errno));
      return false;
    }

    struct termios tio{};
    if (tcgetattr(fd_, &tio) != 0) {
      RCLCPP_ERROR(this->get_logger(), "tcgetattr: %s", std::strerror(errno));
      return false;
    }

    cfmakeraw(&tio);                 // 原始模式
    tio.c_cflag |= (CLOCAL | CREAD); // 本地模式 + 允许接收
    tio.c_cflag &= ~PARENB;          // 无校验
    tio.c_cflag &= ~CSTOPB;          // 1 停止位
    tio.c_cflag &= ~CRTSCTS;         // 无硬件流控
    tio.c_cflag &= ~CSIZE;           // 设置 8 数据位
    tio.c_cflag |= CS8;

    // 阻塞读取：至少等 1 个字节（VMIN=1, VTIME=0）
    tio.c_cc[VMIN]  = 1;
    tio.c_cc[VTIME] = 0;

    speed_t sp = baud_to_flag(baud_);
    cfsetispeed(&tio, sp);
    cfsetospeed(&tio, sp);

    if (tcsetattr(fd_, TCSANOW, &tio) != 0) {
      RCLCPP_ERROR(this->get_logger(), "tcsetattr: %s", std::strerror(errno));
      return false;
    }
    tcflush(fd_, TCIFLUSH);
    return true;
  }

  void read_loop() {
    std::vector<unsigned char> buf(256);
    std::string line_buf;
    const bool line_mode = !eol_.empty();
    const std::size_t eol_len = eol_.size();

    while (!stop_) {
      ssize_t n = ::read(fd_, buf.data(), buf.size()); // 阻塞直到至少1字节
      if (n > 0) {
        if (!line_mode) {
          // —— 按“块”发布 —— //
          std_msgs::msg::String msg;
          msg.data.assign(reinterpret_cast<char*>(buf.data()),
                          static_cast<std::size_t>(n));
          pub_->publish(msg);
          RCLCPP_INFO(this->get_logger(), "[RX %ldB] %s",
                      static_cast<long>(n), msg.data.c_str());

          // 终端可见/不可见字符混合打印（可选）
          for (ssize_t i = 0; i < n; ++i) {
            if (buf[i] >= 32 && buf[i] <= 126) std::cout << static_cast<char>(buf[i]);
            else std::cout << "[0x" << std::hex << std::setw(2)
                           << std::setfill('0') << static_cast<int>(buf[i])
                           << std::dec << "]";
          }
          std::cout << std::flush;
        } else {
          // —— 按“行”发布（以 eol_ 分隔）—— //
          line_buf.append(reinterpret_cast<char*>(buf.data()),
                          static_cast<std::size_t>(n));
          for (;;) {
            std::size_t pos = line_buf.find(eol_);
            if (pos == std::string::npos) break;
            std::string one = line_buf.substr(0, pos);
            line_buf.erase(0, pos + eol_len);

            std_msgs::msg::String msg;
            msg.data = one;
            pub_->publish(msg);
            RCLCPP_INFO(this->get_logger(), "[RX] %s", one.c_str());

            // 同步打印
            std::cout << one << std::endl;
          }
        }
      } else if (n == 0) {
        // 阻塞模式几乎不会返回 0，这里留作兼容
        continue;
      } else {
        // n < 0：如果是我们主动 close(fd_) 触发的，直接退出
        if (stop_) break;
        if (errno == EAGAIN || errno == EWOULDBLOCK) continue;
        RCLCPP_WARN(this->get_logger(), "read error: %s", std::strerror(errno));
        // 小睡一下避免刷屏
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    }
    RCLCPP_INFO(this->get_logger(), "Reader thread exit.");
  }

private:
  // 参数
  std::string port_;
  int baud_;
  std::string topic_;
  std::string eol_;

  // 串口/线程
  int fd_;
  std::atomic<bool> stop_;
  std::thread th_;

  // ROS
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<SerialTopicPub>();
    rclcpp::spin(node);          // 主线程只负责 ROS 回调
  } catch (const std::exception& e) {
    std::fprintf(stderr, "Fatal: %s\n", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
