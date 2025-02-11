#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include "ViewLink.h"

class Gimbal {
public:
    Gimbal(ros::NodeHandle& nh) {
        // ROS Publisher & Subscriber 설정
        cmd_sub = nh.subscribe("gimbal_cmd_deg", 10, &Gimbal::gimbalCmdCallback, this);
        pose_pub = nh.advertise<std_msgs::Float32MultiArray>("gimbal_pose_deg", 10);

        // ViewLink SDK 초기화
        if (VLK_Init() != VLK_ERROR_NO_ERROR) {
            ROS_ERROR("Failed to initialize ViewLink SDK");
            ros::shutdown();
        }

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

    void getGimbalPose() {
        VLK_DEV_TELEMETRY telemetry;
        if (VLK_IsTCPConnected()) {
            VLK_QueryDevConfiguration();  // 짐벌 상태 요청
        }

        // Pose 데이터 발행
        std_msgs::Float32MultiArray pose_msg;
        pose_msg.data.resize(3);
        pose_msg.data[0] = 0.0;                // Roll (짐벌에서 지원 X)
        pose_msg.data[1] = telemetry.dPitch;   // Pitch
        pose_msg.data[2] = telemetry.dYaw;     // Yaw

        pose_pub.publish(pose_msg);
    }

private:
    ros::Subscriber cmd_sub;
    ros::Publisher pose_pub;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "gimbal_control_node");
    ros::NodeHandle nh;

    Gimbal gimbal(nh);
    ros::Rate rate(30);  // 30Hz

    while (ros::ok()) {
        gimbal.getGimbalPose();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
