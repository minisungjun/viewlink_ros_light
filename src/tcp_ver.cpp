#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include "ViewLink.h"
#include "kari_dronecop_rd_payload_mgmt/payload_mc_gimbal_ctrl_cmd.h"

class Gimbal {
public:
    Gimbal(ros::NodeHandle& nh, ros::NodeHandle& nhp) {
        // ROS Publisher & Subscriber 설정
        cmd_sub = nh.subscribe("gimbal_cmd_deg", 1, &Gimbal::gimbalCmdCallback, this);
        pose_pub = nh.advertise<std_msgs::Float32MultiArray>("gimbal_pose_deg", 10);
        gcs_cmd_sub = nh.subscribe("gcs_cmd", 1, &Gimbal::gcsCmdCallback, this);

        rate_hz = nhp.param<double>("rate", 30);
        ros::Rate rate(rate_hz);  // 30Hz
        cmd_multiple = nhp.param<double>("cmd_multiple", 10000);

        // ViewLink SDK 초기화
        if (VLK_Init() != VLK_ERROR_NO_ERROR) {
            ROS_ERROR("Failed to initialize ViewLink SDK");
            ros::shutdown();
        }

        // Register device status callback
        VLK_RegisterDevStatusCB(Gimbal::DeviceStatusCallback, this);

        // TCP 연결 설정
        VLK_CONN_PARAM connParam;
        connParam.emType = VLK_CONN_TYPE_TCP;
        strcpy(connParam.ConnParam.IPAddr.szIPV4, "10.10.10.102");  // 짐벌의 IP 주소
        connParam.ConnParam.IPAddr.iPort = 2000;  // 짐벌의 TCP 포트

        // 연결 시도
        if (VLK_Connect(&connParam, nullptr, nullptr) != VLK_ERROR_NO_ERROR) {
            ROS_ERROR("Failed to connect to gimbal via TCP");
            ros::shutdown();
        }

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
        current_telemetry = newTelemetry;
        telemetry_updated = true;
    }

    void gimbalCmdCallback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
        if (msg->data.size() < 3) {
            ROS_WARN("Received invalid gimbal command");
            return;
        }

        double roll = msg->data[0];   // 롤 값
        double pitch = msg->data[1];  // 피치 값
        double yaw = msg->data[2];    // 요 값

        ROS_INFO("Setting Gimbal Pose - Roll: %.2f, Pitch: %.2f, Yaw: %.2f", roll, pitch, yaw);

        // ViewLink SDK를 사용하여 짐벌 움직이기
        VLK_TurnTo(yaw, pitch);  // 짐벌은 Roll을 직접 지원하지 않으므로 Pitch, Yaw만 사용
    }

    void gcsCmdCallback(const kari_dronecop_rd_payload_mgmt::payload_mc_gimbal_ctrl_cmd::ConstPtr msg) {
        double pan_rate_cmd = msg->pan_rate_cmd;
        double tilt_rate_cmd = msg->tilt_rate_cmd;
        double zoom_pos_cmd = msg->zoom_pos_cmd;
        int homing_mode_cmd = msg->homing_mode_cmd;

        ROS_INFO("Setting Gimbal Rate - pan rate: %.2f, tilt rate: %.2f", pan_rate_cmd, tilt_rate_cmd);
        VLK_Move(pan_rate_cmd * cmd_multiple, tilt_rate_cmd * cmd_multiple);

        //zoom 기능
        if (zoom_pos_cmd > 0.0) {
            ROS_INFO("Setting Gimbal Zoom: %.2f", zoom_pos_cmd);
            VLK_ZoomTo(zoom_pos_cmd);
        }

        // 홈 기능 - zoom 다시 1배율로 set
        if (homing_mode_cmd == 1) {
            ROS_INFO("Returning Gimbal to Home Position");
            VLK_Home();  // 짐벌 초기 위치로 복귀
            VLK_ZoomTo(1.0); //reset zoom to (X 1.0)
        }
    }

    void getGimbalPose() {
        if (!VLK_IsTCPConnected()) {
            ROS_WARN_THROTTLE(1, "Gimbal not connected");
            return;
        }

        // Query device configuration to trigger a telemetry update
        VLK_QueryDevConfiguration();

        // Get latest telemetry data
        VLK_DEV_TELEMETRY telemetry;
        {
            std::lock_guard<std::mutex> lock(telemetry_mutex);
            if (!telemetry_updated) {
                ROS_WARN_THROTTLE(1, "No telemetry data available");
                return;
            }
            telemetry = current_telemetry;
        }

        // Publish pose data
        std_msgs::Float32MultiArray pose_msg;
        pose_msg.data.resize(3);
        pose_msg.data[0] = 0.0;                // Roll (짐벌에서 지원 X)
        pose_msg.data[1] = telemetry.dPitch;   // Pitch
        pose_msg.data[2] = telemetry.dYaw;     // Yaw

        pose_pub.publish(pose_msg);
    }

    double rate_hz;
    double cmd_multiple;

private:
    ros::Subscriber cmd_sub, gcs_cmd_sub;
    ros::Publisher pose_pub;
    
    // Telemetry data storage
    std::mutex telemetry_mutex;
    VLK_DEV_TELEMETRY current_telemetry;
    bool telemetry_updated = false;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "gimbal_control_node");
    ros::NodeHandle nh, nhp("~");

    Gimbal gimbal(nh, nhp);
    ros::Rate rate(gimbal.rate_hz);

    while (ros::ok()) {
        gimbal.getGimbalPose();  // Now we can safely call this
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}