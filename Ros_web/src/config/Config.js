const Config = {
    ROSBRIDGE_SERVER_IP: "localhost",
    ROSBRIDGE_SERVER_PORT: "9090",
    RECONNECTION_TIMER: 3500,
    CMD_VEL_TOPIC: "cmd_vel",
    ODOM_TOPIC: "/odom",
    POSE_TOPIC: "amcl_pose",
    TARGET_TOPIC: "/robot_target_topic",
    FEEDBACK_TOPIC: "/robot_feedback_topic",
}
export default Config;