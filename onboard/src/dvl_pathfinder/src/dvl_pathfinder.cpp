#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <filesystem>
#include <memory>
#include <regex>
#include <string>
#include <thread>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fmt/core.h>
#include <rclcpp/rclcpp.hpp>
#include <rcpputils/asserts.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
#include <yaml-cpp/yaml.h>

extern "C" {
  #include "PDD_Include.h"
}

namespace fs = std::filesystem;

const std::string DVL_PATHFINDER_PACKAGE_PATH = ament_index_cpp::get_package_share_directory("dvl_pathfinder");

// Path to robot config file
inline const char* ROBOT_NAME = std::getenv("ROBOT_NAME");
const std::string ROBOT_CONFIG_FILE_PATH =
  DVL_PATHFINDER_PACKAGE_PATH + "/config/" + std::string(ROBOT_NAME ? ROBOT_NAME : "") + ".yaml";

/**
 * @brief ROS 2 Jazzy node that reads PD0 ensembles from a Teledyne ADCP over a serial port and publishes
 *        vessel‑frame velocity and range‑to‑bottom.
 *
 * Topics
 * -------
 *   adcp/velocity (geometry_msgs/Vector3Stamped) – Vessel velocity in m/s (x = surge, y = sway, z = heave).
 *   adcp/range    (std_msgs/Float64)            – Range to bottom in metres.
 *
 * Parameters
 * ----------
 *   device    (string, default: "/dev/ttyUSB4")  – Serial device for the ADCP interface.
 *   baud      (int,    default: 115200)          – Baud rate. 9 600‑115 200 supported.
 *   frame_id  (string, default: "adcp_link")     – Frame ID stamped into outgoing messages.
 */
class AdcpReader : public rclcpp::Node
{
public:
  static constexpr speed_t DEFAULT_BAUD = B115200;
  static constexpr std::size_t BUF_SZ = 10'000;

  AdcpReader() : Node("adcp_reader")
  {
    // ── Parameters ───────────────────────────────────────────────────────────
    // Get the FTDI string from the robot config file
    std::string ftdi;
    read_robot_config(ftdi);

    // Find the serial port by FTDI string
    device_ = findSerialPortByFtdiString(ftdi);
    if (device_.empty()) {
      RCLCPP_FATAL(get_logger(), "Failed to find serial port with FTDI string '%s'", ftdi.c_str());
      throw std::runtime_error("Serial port not found");
    }

    // ── Publishers ───────────────────────────────────────────────────────────
    vel_pub_   = create_publisher<geometry_msgs::msg::Vector3Stamped>("adcp/velocity", 10);
    range_pub_ = create_publisher<std_msgs::msg::Float64>("adcp/range", 10);

    // ── Serial initialisation ────────────────────────────────────────────────
    if (!openSerial()) {
      RCLCPP_FATAL(get_logger(), "Failed to open serial device '%s'", device_.c_str());
      throw std::runtime_error("Serial open failed");
    }

    RCLCPP_INFO(get_logger(), "Connected to DVL Pathfinder at %s.", device_.c_str());

    // ── Teledyne PD0 decoder ─────────────────────────────────────────────────
    decoder_ = std::make_unique<tdym::PDD_Decoder>();
    tdym::PDD_InitializeDecoder(decoder_.get());
    tdym::PDD_SetInvalidValue(0);

    // ── Background read thread ───────────────────────────────────────────────
    read_thread_ = std::thread(&AdcpReader::readLoop, this);
  }

  ~AdcpReader() override
  {
    running_.store(false);
    if (read_thread_.joinable()) {
      read_thread_.join();
    }
    if (fd_ >= 0) {
      close(fd_);
    }
  }

private:
  void read_robot_config(std::string &ftdi) {
    try {
      YAML::Node config = YAML::LoadFile(ROBOT_CONFIG_FILE_PATH);
      ftdi   = config["ftdi"].as<std::string>();
    } catch (const std::exception &e) {
      RCLCPP_ERROR(get_logger(), "Exception: %s", e.what());
      rcpputils::check_true(
        false, fmt::format("Could not read robot config file. Make sure it is in the correct format. '%s'",
                           ROBOT_CONFIG_FILE_PATH));
    }
  }

  /* Find the serial port by FTDI string. */
  std::string findSerialPortByFtdiString(const std::string& ftdi) {
    const std::string by_id_path = "/dev/serial/by-id";

    if (!fs::exists(by_id_path) || !fs::is_directory(by_id_path)) {
        RCLCPP_ERROR(get_logger(), "Directory %s does not exist.", by_id_path.c_str());
        return "";
    }

    for (const auto& entry : fs::directory_iterator(by_id_path)) {
        std::string filename = entry.path().filename().string();

        if (filename.find(ftdi) != std::string::npos) {
            // Resolve symlink to get actual device path
            std::error_code ec;
            std::string resolved_path = fs::read_symlink(entry.path(), ec).string();

            if (ec) {
                RCLCPP_ERROR(this->get_logger(), "Failed to resolve symlink: %s", ec.message().c_str());
                continue;
            }

            // Join with /dev/serial/by-id to get full device path
            std::string full_path = fs::canonical(entry.path(), ec).string();
            if (!ec) {
                return full_path;
            }
        }
    }

    return ""; // No match found
  }

  /* Open and configure the serial port. */
  bool openSerial()
  {
    fd_ = open(device_.c_str(), O_RDONLY | O_NOCTTY);
    if (fd_ < 0) {
      perror("open");
      return false;
    }

    termios tio{};
    tcgetattr(fd_, &tio);
    cfmakeraw(&tio);

    cfsetspeed(&tio, DEFAULT_BAUD);

    tio.c_cflag |= CLOCAL | CREAD; // ignore modem lines, enable RX
    tio.c_cc[VMIN]  = 0;           // non‑blocking read
    tio.c_cc[VTIME] = 10;          // 1 s timeout

    if (tcsetattr(fd_, TCSANOW, &tio) < 0) {
      perror("tcsetattr");
      return false;
    }
    return true;
  }

  /* Thread function that continuously reads, decodes and publishes data. */
  void readLoop()
  {
    unsigned char buf[BUF_SZ];
    tdym::PDD_PD0Ensemble ens{};
    unsigned int ens_num = 0;

    while (running_.load() && rclcpp::ok()) {
      ssize_t n = read(fd_, buf, BUF_SZ);
      if (n < 0) {
        perror("read");
        continue;
      }
      if (n == 0) {
        continue; // timeout – no data yet
      }

      tdym::PDD_AddDecoderData(decoder_.get(), buf, static_cast<int>(n));

      while (tdym::PDD_GetPD0Ensemble(decoder_.get(), &ens)) {
        ens_num = tdym::PDD_GetEnsembleNumber(&ens, tdym::PDBB);

        double vv[FOUR_BEAMS];
        tdym::PDD_GetVesselVelocities(&ens, vv);
        double range = tdym::PDD_GetRangeToBottom(&ens, vv);

        geometry_msgs::msg::Vector3Stamped vel_msg;
        vel_msg.header.stamp = now();
        vel_msg.header.frame_id = frame_id_;
        vel_msg.vector.x = vv[0];
        vel_msg.vector.y = vv[1];
        vel_msg.vector.z = vv[2];
        vel_pub_->publish(vel_msg);

        std_msgs::msg::Float64 range_msg;
        range_msg.data = range;
        range_pub_->publish(range_msg);

        RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000,
          "Ensemble %u  Vel: [%.3f %.3f %.3f]  Range: %.3f m",
          ens_num, vv[0], vv[1], vv[2], range);
      }
    }
  }

  // ── Member variables ──────────────────────────────────────────────────────
  std::string device_;
  std::string frame_id_ = "dvl";
  int         fd_{-1};
  std::atomic<bool> running_{true};
  std::unique_ptr<tdym::PDD_Decoder> decoder_;
  std::thread read_thread_;

  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr             range_pub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AdcpReader>();

  // Use a Multi‑threaded executor – helps when callbacks become CPU‑bound.
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();

  rclcpp::shutdown();
  return 0;
}
