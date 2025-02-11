/**
 * @file tcp_ver.cpp
 * @brief Viewlink gimbal & image capture module using TCP interface
 * @author Seungwook Lee, Sungjun Park
 * @date 2025-02-12
 * @note Saves detection images to ${HOME}/saved_images/<date>
 */

#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Image.h>
#include <vision_msgs/Detection2DArray.h>
#include "ViewLink.h"
#include "kari_dronecop_rd_payload_mgmt/payload_mc_gimbal_ctrl_cmd.h"

#include <boost/format.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgcodecs.hpp> 

#include <filesystem>
#include <ctime>
#include <iomanip>
#include <sys/stat.h>
#include <pwd.h>

namespace fs = std::filesystem;

class Gimbal {
public:
    Gimbal(ros::NodeHandle& nh, ros::NodeHandle& nhp) 
    : is_first_loop(true), 
    ir_pip_disable_count(0),
    detection_image_count(0)
    {
        // Get home directory
        const char* home_dir = getenv("HOME");
        if (!home_dir) {
            struct passwd* pwd = getpwuid(getuid());
            if (pwd)
                home_dir = pwd->pw_dir;
        }

        // Get current date for folder name
        auto now = std::chrono::system_clock::now();
        auto in_time_t = std::chrono::system_clock::to_time_t(now);
        std::stringstream date_ss;
        date_ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d");
        
        // Create default save path
        if (home_dir) {
            save_dir = std::string(home_dir) + "/saved_images/" + date_ss.str() + "/";
        } else {
            save_dir = "/tmp/saved_images/" + date_ss.str() + "/";
            ROS_WARN("Could not determine HOME directory, using /tmp instead");
        }

        // Create directories if they don't exist
        try {
            fs::create_directories(save_dir);
        } catch (const std::exception& e) {
            ROS_ERROR("Failed to create save directory: %s", e.what());
            ros::shutdown();
        }

        // ROS Publisher & Subscriber 설정
        pose_pub = nh.advertise<geometry_msgs::Vector3>("gimbal_pose_deg", 10);

        cmd_sub = nh.subscribe("gimbal_cmd_deg", 1, &Gimbal::gimbalCmdCallback, this);
        gcs_cmd_sub = nh.subscribe("gcs_cmd", 1, &Gimbal::gcsCmdCallback, this);
        detection_sub = nh.subscribe("detection", 1, &Gimbal::detectionCallback, this);
        test_image_sub = nh.subscribe("/usb_cam/image_raw", 1, &Gimbal::testImageCallback, this);

        rate_ms = nhp.param<int>("rate_ms", 200);
        rate_ms = std::min(std::max(rate_ms, 100), 5000);
        cmd_multiple = nhp.param<double>("cmd_multiple", 10000);
        pan_ang_ref = nhp.param<double>("netgun_pan_ang_ref", 0.0);      // 기본값: 0.0도
        tilt_ang_ref = nhp.param<double>("netgun_tilt_ang_ref", -15.0);  // 기본값: -20.0도

        std::string format_string;
        nhp.param<std::string>("filename_format", format_string, std::string("detection%04i.%s"));
        nhp.param<std::string>("encoding", encoding, std::string("bgr8"));
        g_format.parse(format_string);

        // ViewLink SDK 초기화
        if (VLK_Init() != VLK_ERROR_NO_ERROR) {
            ROS_ERROR("Failed to initialize ViewLink SDK");
            ros::shutdown();
        }

        // Register device status callback
        VLK_RegisterDevStatusCB(Gimbal::DeviceStatusCallback, this);
        VLK_SetKeepAliveInterval(rate_ms);

        // TCP 연결 설정
        VLK_CONN_PARAM connParam;
        connParam.emType = VLK_CONN_TYPE_TCP;
        strcpy(connParam.ConnParam.IPAddr.szIPV4, nhp.param<std::string>("gimbal_ip", "10.10.10.102").c_str());  // 짐벌의 IP 주소
        connParam.ConnParam.IPAddr.iPort = 2000;  // 짐벌의 TCP 포트

        // 연결 시도
        VLK_Connect(&connParam, nullptr, nullptr);
        // if ( != VLK_ERROR_NO_ERROR) {
        //     ROS_ERROR("Failed to connect to gimbal via TCP");
        //     ros::shutdown();
        // }
        // @TODO Fix detecting ill connection
        if (!VLK_IsTCPConnected()) {
            ROS_WARN_THROTTLE(1, "Gimbal not connected");
            ros::shutdown();
        }

        last_capture_time_sec = ros::Time::now().toSec();
        ROS_INFO("Gimbal connected successfully");
    }

    ~Gimbal() {
        VLK_Disconnect();
        VLK_UnInit();
    }

    // Callback function to receive device status
    static int DeviceStatusCallback(int iType, const char* szBuffer, int iBufLen, void* pUserParam) {
        Gimbal* gimbal = static_cast<Gimbal*>(pUserParam);
        if (iType == VLK_DEV_STATUS_TYPE_TELEMETRY && iBufLen == sizeof(VLK_DEV_TELEMETRY)) {
            const VLK_DEV_TELEMETRY* telemetry = reinterpret_cast<const VLK_DEV_TELEMETRY*>(szBuffer);
            gimbal->updateTelemetry(*telemetry);
        }
        return 0;
    }
    // Update telemetry data (called from callback)
    void updateTelemetry(const VLK_DEV_TELEMETRY& newTelemetry) {
        std::lock_guard<std::mutex> lock(telemetry_mutex);
        // std::cout << "updateTelemetry" <<std::endl;
        current_telemetry = newTelemetry;
        telemetry_updated = true;
        getGimbalPose();
    }

    void gimbalCmdCallback(const geometry_msgs::Vector3::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(telemetry_mutex);
        // std::cout << "gimbalCmdCB" <<std::endl;

        double roll = msg->x;   // 롤 값
        double pitch = msg->y;  // 피치 값
        double yaw = msg->z;    // 요 값

        ROS_INFO("Setting Gimbal Pose - Pitch: %.2f, Yaw: %.2f", pitch, yaw);

        // ViewLink SDK를 사용하여 짐벌 움직이기
        VLK_TurnTo(yaw, pitch);  // 짐벌은 Roll을 직접 지원하지 않으므로 Pitch, Yaw만 사용
    }

    void gcsCmdCallback(const kari_dronecop_rd_payload_mgmt::payload_mc_gimbal_ctrl_cmd::ConstPtr msg) {
        std::lock_guard<std::mutex> lock(telemetry_mutex);
        // std::cout << "gcsCmdCallback" <<std::endl;
        double pan_rate_cmd = msg->pan_rate_cmd;
        double tilt_rate_cmd = msg->tilt_rate_cmd;
        double zoom_pos_cmd = msg->zoom_pos_cmd;
        int homing_mode_cmd = msg->homing_mode_cmd;
        int netgun_mode_cmd = msg->netgun_mode_cmd;
        int img_capture_cmd = msg->img_capture_cmd;
        save_detection_image = img_capture_cmd > 0 ? true : false;

        ROS_INFO("Setting Gimbal Rate - pan rate: %.2f, tilt rate: %.2f", pan_rate_cmd, tilt_rate_cmd);
        VLK_Move(pan_rate_cmd * cmd_multiple, tilt_rate_cmd * cmd_multiple);

        //zoom 기능
        if (zoom_pos_cmd > 0.0) {
            ROS_INFO("Setting Gimbal Zoom: %.2f", zoom_pos_cmd);
            VLK_ZoomTo(zoom_pos_cmd);
        }

        if (img_capture_cmd) {
            captureImage();
        }

        // 홈 기능 - zoom 다시 1배율로 set
        else if (homing_mode_cmd == 1) {
            ROS_INFO("Returning Gimbal to Home Position");
            VLK_TurnTo(0.0, 0.0);  // Not using VLK_Home() due to unwanted behavior
            // VLK_Home();  // 짐벌 초기 위치로 복귀
            VLK_ZoomTo(1.0); //reset zoom to (X 1.0)
        }
        //assume no zoom & netgun at same time
        else if(netgun_mode_cmd == 1){
            ROS_INFO("Activating Netgun Mode - Aligning Gimbal");
            // Netgun 정렬을 위한 Pan, Tilt 값 - launch file
            VLK_TurnTo(pan_ang_ref, tilt_ang_ref);  // Netgun 정렬을 위한 Pan, Tilt 값 설정
        }
    }

    void detectionCallback(const vision_msgs::Detection2DArray::ConstPtr msg) {
        if (msg->detections.size() > 0) {
            if (ros::Time::now().toSec() - last_capture_time_sec > 1.0 && save_detection_image) {
                saveDetectionImages(msg->detections[0].source_img); // @TODO Check if the detection image is actually included
            }
        }
    }

    void testImageCallback(const sensor_msgs::Image& msg) {
        if (ros::Time::now().toSec() - last_capture_time_sec > 1.0 && save_detection_image) {
            saveDetectionImages(msg); // @TODO Check if the detection image is actually included
        }
    }

    bool saveDetectionImages(const sensor_msgs::Image& image_msg) {
        // capture detection image for any detection
        // ref: https://github.com/ros-perception/image_pipeline/blob/noetic/image_view/src/nodes/image_saver.cpp#L145
        cv::Mat image;
        std::string filename;
        try
        {
            image = cv_bridge::toCvCopy(image_msg, encoding)->image;
        } catch(cv_bridge::Exception)
        {
            ROS_ERROR("Unable to convert %s image to %s", image_msg.encoding.c_str(), encoding.c_str());
            return false;
        }

        if (!image.empty()) {
            try {
                filename = (g_format).str();
            } catch (...) { g_format.clear(); }
            try {
                filename = (g_format % detection_image_count).str();
            } catch (...) { g_format.clear(); }
            try { 
                filename = (g_format % detection_image_count % "jpg").str();
            } catch (...) { g_format.clear(); }

        
            if (save_detection_image) {
                // Create timestamp string in YYYY-MM-DD_HH-mm-ss format
                auto now = std::chrono::system_clock::now();
                auto in_time_t = std::chrono::system_clock::to_time_t(now);
                std::stringstream timestamp_ss;
                timestamp_ss << std::put_time(std::localtime(&in_time_t), "%H-%M-%S");
                // timestamp_ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d_%H-%M-%S");
                
                // Construct final filename with timestamp
                std::string final_filename = timestamp_ss.str() + "_" + filename;
                std::string full_path = fs::path(save_dir) / final_filename;

                try {
                    cv::imwrite(full_path, image);
                    ROS_INFO("Saved image %s", full_path.c_str());
                    save_detection_image = false;
                    detection_image_count++;
                } catch (const cv::Exception& e) {
                    ROS_ERROR("Failed to save image: %s", e.what());
                    return false;
                }
            }
        } else {
            ROS_WARN("Couldn't save image, no data!");
            return false;
        }
        return true;
    }

    void getGimbalPose() {
        std::cout << "getGimbalPose" <<std::endl;
        if (!VLK_IsTCPConnected()) {
            ROS_WARN_THROTTLE(1, "Gimbal not connected");
            return;
        }
        if (is_first_loop) {
            VLK_Home();
            VLK_ZoomTo(1.0); //reset zoom to (X 1.0)
            is_first_loop = false;
        }
        if (ir_pip_disable_count++ < 50)
        {
            int iEnablePIP = 0;
            VLK_SetImageColor(VLK_IMAGE_TYPE::VLK_IMAGE_TYPE_VISIBLE1, iEnablePIP, VLK_IR_COLOR::VLK_IR_COLOR_WHITEHOT);
        }
        // Query device configuration to trigger a telemetry update
        VLK_QueryDevConfiguration();

        // Get latest telemetry data
        VLK_DEV_TELEMETRY telemetry;
        {
            if (!telemetry_updated) {
                ROS_WARN_THROTTLE(1, "No telemetry data available");
                return;
            }
            telemetry = current_telemetry;
            ROS_INFO("telem pitch: %.2f", telemetry.dPitch);
            ROS_INFO("telem yaw  : %.2f", telemetry.dYaw);
        }

        // Publish pose data
        geometry_msgs::Vector3 pose_msg;
        pose_msg.x = 0.0;                // Roll (짐벌에서 지원 X)
        pose_msg.y = telemetry.dPitch;   // Pitch
        pose_msg.z = telemetry.dYaw;     // Yaw

        pose_pub.publish(pose_msg);
    }

    void captureImage() {
        double time_now_sec = ros::Time::now().toSec();
        if (time_now_sec - last_capture_time_sec > 1.0)
        {
            VLK_Photograph();
            last_capture_time_sec = time_now_sec;
        }
    }

    // parameters
    int rate_ms;
    double cmd_multiple;
    double pan_ang_ref, tilt_ang_ref;

    // loop control
    bool is_first_loop;
    int ir_pip_disable_count;

    // last variables
    double last_capture_time_sec;

    // image saver
    boost::format g_format;
    bool save_detection_image;
    int detection_image_count;
    std::string encoding;
    std::string save_dir;

private:
    ros::Subscriber cmd_sub, gcs_cmd_sub;
    ros::Subscriber detection_sub, test_image_sub;
    ros::Publisher pose_pub;
    
    // Telemetry data storage
    std::mutex telemetry_mutex;
    VLK_DEV_TELEMETRY current_telemetry;
    bool telemetry_updated = false;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "viewlink_ros_light");
    ros::NodeHandle nh, nhp("~");

    Gimbal gimbal(nh, nhp);
    ros::Rate rate(gimbal.rate_ms);

    ros::spin();

    return 0;
}