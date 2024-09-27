#include "CtrlFSM.h"
#include <uav_utils/converters.h>

using namespace Eigen;
using std::cout;
using std::endl;
using namespace uav_utils;

//# CtrlFSM 클래스 초기화 시 메뉴얼조종을 설정, 호버링포즈 4차원벡터의 모든 계수를 0으로 설정
CtrlFSM::CtrlFSM(Parameter_t& param_, Controller& controller_, HovThrKF& hov_thr_kf_)
    : param(param_), controller(controller_), hov_thr_kf(hov_thr_kf_)
{
    state = MANUAL_CTRL;
    hover_pose.setZero();
}

//# 드론 자세 컨트롤 프로세스
void CtrlFSM::process()
{
    ros::Time now_time = ros::Time::now();
    Controller_Output_t u;
    SO3_Controller_Output_t u_so3; //# SO3 알고리즘

    controller.config_gain(param.track_gain);
    process_cmd_control(u, u_so3);
    controller.publish_ctrl(u, now_time);
    hov_thr_kf.simple_update(u.des_v_real, odom_data.v);
    param.config_full_thrust(hov_thr_kf.get_hov_thr());
}

//# 다양한 메시지 수신 여부 확인 메서드
bool CtrlFSM::rc_is_received(const ros::Time& now_time)
{
    return (now_time - rc_data.rcv_stamp).toSec() < param.msg_timeout.rc;
}

bool CtrlFSM::cmd_is_received(const ros::Time& now_time)
{
    return (now_time - cmd_data.rcv_stamp).toSec() < param.msg_timeout.cmd;
}

bool CtrlFSM::odom_is_received(const ros::Time& now_time)
{
    return (now_time - odom_data.rcv_stamp).toSec() < param.msg_timeout.odom;
}

bool CtrlFSM::imu_is_received(const ros::Time& now_time)
{
    return (now_time - imu_data.rcv_stamp).toSec() < param.msg_timeout.imu;
}

//# 현재 오도메트리에서 yaw 값을 추출
double CtrlFSM::get_yaw_from_odom()
{
    return get_yaw_from_quaternion(odom_data.q);
}

//# 호버 상태에서 제어 수행
void CtrlFSM::process_hover_control(Controller_Output_t& u, SO3_Controller_Output_t& u_so3)
{
    Desired_State_t des;
    des.p = hover_pose.head<3>();
    des.v = Vector3d::Zero();
    des.yaw = hover_pose(3);
    des.a = Vector3d::Zero();
    des.jerk = Vector3d::Zero();
    controller.update(des, odom_data, u, u_so3);
}

//# 명령을 기반으로 제어 수행
void CtrlFSM::process_cmd_control(Controller_Output_t& u, SO3_Controller_Output_t& u_so3)
{
    Desired_State_t des;
    des.p = cmd_data.p;
    des.v = cmd_data.v;
    des.yaw = cmd_data.yaw;
    des.a = cmd_data.a;
    des.jerk = cmd_data.jerk;
    des.head_rate = cmd_data.head_rate;

    controller.update(des, odom_data, u, u_so3);
}

//# IMU와 오도메트리를 사용하여 자세 조정
void CtrlFSM::align_with_imu(Controller_Output_t& u)
{
    double imu_yaw = get_yaw_from_quaternion(imu_data.q);
    double odom_yaw = get_yaw_from_odom();
    double des_yaw = u.yaw;
    u.yaw = yaw_add(yaw_add(des_yaw, -odom_yaw), imu_yaw);
}

//# 현재 오도메트리 값을 기반으로 호버 위치 설정
void CtrlFSM::set_hov_with_odom()
{
    hover_pose.head<3>() = odom_data.p;
    hover_pose(3) = get_yaw_from_odom();
}

//# OFFBOARD 모드를 토글하는 메서드
void CtrlFSM::toggle_offboard_mode(bool on_off)
{
    mavros_msgs::SetMode offb_set_mode;
    ros::Time last_request = ros::Time::now();

    if (on_off)
    {
        offb_set_mode.request.custom_mode = "OFFBOARD";
        controller.set_FCU_mode.call(offb_set_mode);
    }
    else
    {
        offb_set_mode.request.custom_mode = "ALTCTL";
        controller.set_FCU_mode.call(offb_set_mode);
    }
}

//# PX4 초기화 여부 확인
bool CtrlFSM::px4_init()
{
    return (odom_data.odom_init && imu_data.imu_init && cmd_data.cmd_init);
}
