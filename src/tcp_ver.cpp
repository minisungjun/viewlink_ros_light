#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include "ViewLink.h"
#include "kari_dronecop_rd_payload_mgmt/payload_mc_gimbal_ctrl_cmd.h"

class Gimbal {
public:
    Gimbal(ros::NodeHandle& nh, ros::NodeHandle& nhp) {
        // ROS Publisher & Subscriber 설정
        cmd_sub = nh.subscribe("gimbal_cmd_deg", 1, &Gimbal::gimbalCmdCallback, this);
        pose_pub = nh.advertise<geometry_msgs::Vector3>("gimbal_pose_deg", 10);
        gcs_cmd_sub = nh.subscribe("gcs_cmd", 1, &Gimbal::gcsCmdCallback, this);

        rate_ms = nhp.param<int>("rate_ms", 200);
        rate_ms = std::min(std::max(rate_ms, 100), 5000);
        cmd_multiple = nhp.param<double>("cmd_multiple", 10000);
        pan_ang_ref = nhp.param<double>("netgun_pan_ang_ref", 0.0);      // 기본값: 0.0도
        tilt_ang_ref = nhp.param<double>("netgun_tilt_ang_ref", -15.0);  // 기본값: -20.0도


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

        ROS_INFO("Setting Gimbal Rate - pan rate: %.2f, tilt rate: %.2f", pan_rate_cmd, tilt_rate_cmd);
        VLK_Move(pan_rate_cmd * cmd_multiple, tilt_rate_cmd * cmd_multiple);

        //zoom 기능
        if (zoom_pos_cmd > 0.0) {
            ROS_INFO("Setting Gimbal Zoom: %.2f", zoom_pos_cmd);
            VLK_ZoomTo(zoom_pos_cmd);
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

    void getGimbalPose() {
        std::cout << "getGimbalPose" <<std::endl;
        if (!VLK_IsTCPConnected()) {
            ROS_WARN_THROTTLE(1, "Gimbal not connected");
            return;
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

    int rate_ms;
    double cmd_multiple;
    double pan_ang_ref, tilt_ang_ref;

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
    ros::Rate rate(gimbal.rate_ms);

    ros::spin();

    return 0;
}