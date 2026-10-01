#include <fcntl.h>
#include <fmt/core.h>
#include <termios.h>
#include <unistd.h>
#include <yaml-cpp/yaml.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <atomic>
#include <chrono>
#include <custom_msgs/msg/dvl_raw.hpp>
#include <filesystem>
#include <memory>
#include <cmath>
#include <limits>
#include <rclcpp/rclcpp.hpp>
#include <rcpputils/asserts.hpp>
#include <regex>
#include <string>
#include <thread>
#include <vector>

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
 *   /sensors/dvl/raw (custom_msgs/msg/DVLRaw)      – Raw DVL data.
 *
 */
class DVLPathfinder : public rclcpp::Node {
   public:
    static constexpr speed_t DEFAULT_BAUD = B115200;
    static constexpr std::size_t BUF_SZ = 10'000;

    DVLPathfinder() : Node("dvl_pathfinder") {
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
        dvl_raw_pub_ = create_publisher<custom_msgs::msg::DVLRaw>("/sensors/dvl/raw", 10);

        // ── Serial initialisation ────────────────────────────────────────────────
        if (!openSerial()) {
            RCLCPP_FATAL(get_logger(), "Failed to open serial device '%s'", device_.c_str());
            throw std::runtime_error("Serial open failed");
        }

        RCLCPP_INFO(get_logger(), "Connected to DVL Pathfinder at %s.", device_.c_str());

        // ── Teledyne PD0 decoder ─────────────────────────────────────────────────
        decoder_ = std::make_unique<tdym::PDD_Decoder>();
        tdym::PDD_InitializeDecoder(decoder_.get());
        // Keep invalid values distinguishable from a real zero-velocity bottom
        // track. The fusion node uses this to reject invalid DVL fixes.
        tdym::PDD_SetInvalidValue(std::numeric_limits<double>::quiet_NaN());

        // ── Background read thread ───────────────────────────────────────────────
        read_thread_ = std::thread(&DVLPathfinder::readLoop, this);
    }

    ~DVLPathfinder() override {
        running_.store(false);
        if (read_thread_.joinable()) {
            read_thread_.join();
        }
        if (fd_ >= 0) {
            close(fd_);
        }
    }

   private:
    void read_robot_config(std::string& ftdi) {
        try {
            YAML::Node config = YAML::LoadFile(ROBOT_CONFIG_FILE_PATH);
            ftdi = config["ftdi"].as<std::string>();
        } catch (const std::exception& e) {
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

        return "";  // No match found
    }

    /* Open and configure the serial port. */
    bool openSerial() {
        fd_ = open(device_.c_str(), O_RDONLY | O_NOCTTY);
        if (fd_ < 0) {
            perror("open");
            return false;
        }

        termios tio{};
        tcgetattr(fd_, &tio);
        cfmakeraw(&tio);

        cfsetspeed(&tio, DEFAULT_BAUD);

        tio.c_cflag |= CLOCAL | CREAD;  // ignore modem lines, enable RX
        tio.c_cc[VMIN] = 0;             // non‑blocking read
        tio.c_cc[VTIME] = 10;           // 1 s timeout

        if (tcsetattr(fd_, TCSANOW, &tio) < 0) {
            perror("tcsetattr");
            return false;
        }
        return true;
    }

    /* Thread function that continuously reads, decodes and publishes data. */
    void readLoop() {
        unsigned char buf[BUF_SZ];
        tdym::PDD_PD0Ensemble ens{};

        while (running_.load() && rclcpp::ok()) {
            ssize_t n = read(fd_, buf, BUF_SZ);
            if (n < 0) {
                perror("read");
                continue;
            }
            if (n == 0) {
                continue;  // timeout – no data yet
            }

            tdym::PDD_AddDecoderData(decoder_.get(), buf, static_cast<int>(n));

            while (tdym::PDD_GetPD0Ensemble(decoder_.get(), &ens)) {
                double vv[FOUR_BEAMS];
                double beam_ranges[FOUR_BEAMS];
                const bool has_velocity = tdym::PDD_GetVesselVelocities(&ens, vv) != 0;
                const double range = tdym::PDD_GetRangeToBottom(&ens, beam_ranges);

                const bool bottom_track_valid = has_velocity && range > 0.0 &&
                                                std::isfinite(vv[0]) && std::isfinite(vv[1]) &&
                                                std::isfinite(vv[2]);
                tdym::PDD_HPRSensor hpr;
                const bool has_attitude = tdym::PDD_GetHPRSensor(&ens, &hpr, tdym::PDBT) != 0 &&
                                          std::isfinite(hpr.roll) && std::isfinite(hpr.pitch) &&
                                          std::isfinite(hpr.heading);

                custom_msgs::msg::DVLRaw dvl_raw_msg;
                dvl_raw_msg.header.stamp = now();
                dvl_raw_msg.header.frame_id = frame_id_;
                // DVLRaw uses the Pathfinder/Wayfinder wire convention of
                // millimetres per second; dvl_to_odom converts it to SI.
                dvl_raw_msg.bs_transverse = vv[0] * 1e3;
                dvl_raw_msg.bs_longitudinal = vv[1] * 1e3;
                dvl_raw_msg.bs_normal = vv[2] * 1e3;
                dvl_raw_msg.bd_range = range;
                // The fourth bottom-track velocity is the DVL's error velocity.
                // It is consumed by dvl_to_odom to set a per-sample covariance.
                dvl_raw_msg.bi_error = vv[3] * 1e3;
                dvl_raw_msg.bs_status = bottom_track_valid ? "A" : "V";
                dvl_raw_msg.sa_valid = has_attitude;
                if (has_attitude) {
                    dvl_raw_msg.sa_roll = hpr.roll;
                    dvl_raw_msg.sa_pitch = hpr.pitch;
                    dvl_raw_msg.sa_heading = hpr.heading;
                }

                if (ens.bottomTrack != nullptr) {
                    dvl_raw_msg.bt_quality_valid = true;
                    for (std::size_t i = 0; i < FOUR_BEAMS; ++i) {
                        dvl_raw_msg.bt_beam_ranges[i] = beam_ranges[i];
                        dvl_raw_msg.bt_correlation[i] = ens.bottomTrack->correlation[i];
                        dvl_raw_msg.bt_intensity[i] = ens.bottomTrack->intensity[i];
                        dvl_raw_msg.bt_percent_good[i] = ens.bottomTrack->percGood[i];
                        dvl_raw_msg.bt_rssi[i] = ens.bottomTrack->rssi[i];
                    }
                }
                dvl_raw_pub_->publish(dvl_raw_msg);
            }
        }
    }

    // ── Member variables ──────────────────────────────────────────────────────
    std::string device_;
    std::string frame_id_ = "dvl";
    int fd_{-1};
    std::atomic<bool> running_{true};
    std::unique_ptr<tdym::PDD_Decoder> decoder_;
    std::thread read_thread_;

    rclcpp::Publisher<custom_msgs::msg::DVLRaw>::SharedPtr dvl_raw_pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DVLPathfinder>();

    // Use a Multi‑threaded executor – helps when callbacks become CPU‑bound.
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();

    rclcpp::shutdown();
    return 0;
}
