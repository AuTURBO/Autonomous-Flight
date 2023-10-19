#include <ros/ros.h>
#include "CtrlFSM.h"
#include <signal.h>
#include "std_msgs/Float32.h"

CtrlFSM* pFSM; //# Finite State Machine 객체

void mySigintHandler(int sig) {
    ROS_INFO("[Ctrl] Exiting...");
    ros::shutdown();
}

int main(int argc, char* argv[]) {
    //# ROS 노드 초기화
    ros::init(argc, argv, "Ctrl");
    ros::NodeHandle nh("~"); // 파라미터 접근을 위한 노드 핸들러
    signal(SIGINT, mySigintHandler); // Ctrl+C를 처리하기 위한 시그널 핸들러

    //# 파라미터 초기화
    Parameter_t param;
    param.config_from_ros_handle(nh);
    param.init();

    // 컨트롤러 및 관련 객체 초기화
    Controller controller(param);
    HovThrKF hov_thr_kf(param);
    CtrlFSM fsm(param, controller, hov_thr_kf);
    pFSM = &fsm;

    //# 호버 스로틀 칼만 필터 초기화
    fsm.hov_thr_kf.init();
    fsm.hov_thr_kf.set_hov_thr(param.hov_percent);

    ROS_INFO("Initial value for hov_thr set to %.2f/%.2f",
             fsm.hov_thr_kf.get_hov_thr(),
             param.mass * param.gra / param.full_thrust);
    ROS_INFO("Hovering thrust kalman filter is %s.",
             param.hover.use_hov_percent_kf ? "used" : "NOT used");

    //# 컨트롤러 설정
    fsm.controller.config();

    //# ROS 구독자 초기화
    ros::Subscriber odom_sub =
        nh.subscribe<nav_msgs::Odometry>("odom",
                                         100,
                                         boost::bind(&Odom_Data_t::feed, &fsm.odom_data, _1),
                                         ros::VoidConstPtr(),
                                         ros::TransportHints().tcpNoDelay());

    ros::Subscriber cmd_sub = 
        nh.subscribe<quadrotor_msgs::PositionCommand>("cmd",
                                                      100,
                                                      boost::bind(&Command_Data_t::feed, &fsm.cmd_data, _1),
                                                      ros::VoidConstPtr(),
                                                      ros::TransportHints().tcpNoDelay());

    ros::Subscriber imu_sub =
        nh.subscribe<sensor_msgs::Imu>("imu",
                                         100,
                                         boost::bind(&Imu_Data_t::feed, &fsm.imu_data, _1),
                                         ros::VoidConstPtr(),
                                         ros::TransportHints().tcpNoDelay());

    //# ROS 퍼블리셔 초기화
    fsm.controller.ctrl_FCU_pub = nh.advertise<mavros_msgs::AttitudeTarget>("/setpoint_raw/attitude", 10);
    fsm.controller.debug_roll_pub = nh.advertise<std_msgs::Float32>("/debug_roll", 10);
    fsm.controller.debug_pitch_pub = nh.advertise<std_msgs::Float32>("/debug_pitch", 10);
    fsm.traj_start_trigger_pub = nh.advertise<geometry_msgs::PoseStamped>("/traj_start_trigger", 10);

    //# 제어 주기 설정
    ros::Rate r(param.ctrl_rate);

    //# 주요 루프
    while (ros::ok()) {
        r.sleep();
        ros::spinOnce();

        if (fsm.px4_init()) {
            fsm.process();
        }
    }

    return 0;
}
