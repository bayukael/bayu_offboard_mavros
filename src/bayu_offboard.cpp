#include "libcontroller/controller.h"
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/Altitude.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/Thrust.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <ros/ros.h>
#include <string>
#include <vector>
#include <cmath>
#include <algorithm>
#include <utility>
#include <limits> // "infinity" for float, int, etc a.k.a numeric limits

#define CONSTANT_G 9.80665
#define THRUST_HOVER 0.7

template <typename T1>
class RosMsgsWrapper{
    ros::Subscriber m_sub;
    public:
    T1 m_msg;
    RosMsgsWrapper(std::string, int);
    void msg_cb(const typename T1::ConstPtr&);
};

template <typename T1>
RosMsgsWrapper<T1>::RosMsgsWrapper(std::string topic, int queue_size) :
        m_sub(ros::NodeHandle().subscribe(topic, queue_size, &RosMsgsWrapper<T1>::msg_cb, this)){
    ;
}

template <typename T1>
void RosMsgsWrapper<T1>::msg_cb(const typename T1::ConstPtr& msg){
    m_msg = *msg;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "bayu_fly");
    ros::NodeHandle nh;

    RosMsgsWrapper<sensor_msgs::NavSatFix> globalpos_global("mavros/global_position/global", 100);
    RosMsgsWrapper<mavros_msgs::Altitude> mav_alt_amsl("mavros/altitude", 100);
    RosMsgsWrapper<mavros_msgs::State> mav_state("mavros/state", 100);
    RosMsgsWrapper<geometry_msgs::PoseStamped> mav_local_pose("mavros/local_position/pose",100);
    RosMsgsWrapper<geometry_msgs::TwistStamped> mav_local_vel("mavros/local_position/velocity_local",100);
    RosMsgsWrapper<sensor_msgs::Imu> mav_imu_data("mavros/imu/data", 100);
    RosMsgsWrapper<geometry_msgs::Vector3> mav_local_pos_sp("offboard/setpoint_position/local",10);

    ros::Publisher setpoint_thrust_pub = nh.advertise<mavros_msgs::Thrust>("mavros/setpoint_attitude/thrust",100);
    ros::Publisher setpoint_attitude_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_attitude/attitude",100);
    ros::Publisher setpoint_velocity_pub = nh.advertise<geometry_msgs::Twist>("mavros/setpoint_velocity/cmd_vel_unstamped",10);

    ros::ServiceClient arm_srv_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_srv_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    arm_srv_client.waitForExistence();
    set_mode_srv_client.waitForExistence();

    mavros_msgs::SetMode mode;
    mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    PID_Controller mav_pos_control(tf2::Vector3(1.0, 1.0, 1.0), tf2::Vector3(0.0, 0.0, 0.0), tf2::Vector3(0.0, 0.0, 0.0), tf2::Vector3(-1.0, -1.0, 1.0));
    PID_Controller mav_vel_control(tf2::Vector3(1.8, 1.8, 4.0), tf2::Vector3(0.0, 0.0, 0.2), tf2::Vector3(0.2, 0.2, 0.0), tf2::Vector3(2.0, 2.0, 2.0));
    
    std::vector<std::pair<tf2::Vector3, double> > mav_position_sps;
    mav_position_sps.push_back(std::pair<tf2::Vector3, double>(tf2::Vector3(0.0, 0.0, 10.0),0.0));
    mav_position_sps.push_back(std::pair<tf2::Vector3, double>(tf2::Vector3(10.0, -20.0,15.0),M_PI/2));
    mav_position_sps.push_back(std::pair<tf2::Vector3, double>(tf2::Vector3(-20.0,-25.0, 10.0),-M_PI/2));
    int i = 0;
    int flight_example_done = 0;
    ros::Time time_last_send_info = ros::Time::now();
    ros::Time time_last_arm_rec = ros::Time::now();
    ros::Time time_last_mission_req = ros::Time::now();
    ros::Rate loop_rate(50);
    while(ros::ok() && i<mav_position_sps.size()){
        if(ros::Time::now() - time_last_arm_rec > ros::Duration(1.0) && !mav_state.m_msg.armed){
            ROS_INFO("___### SENDING ARM COMMAND ###___");
            arm_srv_client.call(arm_cmd);
            time_last_arm_rec = ros::Time::now();
        }
        if(ros::Time::now() - time_last_mission_req > ros::Duration(1.0) && mav_state.m_msg.mode != mode.request.custom_mode && mav_state.m_msg.armed){
            ROS_INFO("___### SENDING FLIGHT MODE ###___");
            set_mode_srv_client.call(mode);
            time_last_mission_req = ros::Time::now();
        }

        // Prepare position measurement data in tf2::Vector3
        tf2::Vector3 mav_position(mav_local_pose.m_msg.pose.position.x, mav_local_pose.m_msg.pose.position.y, mav_local_pose.m_msg.pose.position.z);
        tf2::Vector3 mav_position_sp(0.0, 0.0, 0.0);
        if(mav_state.m_msg.mode == "OFFBOARD"){
            mav_position_sp = mav_position_sps[i].first;
        }
        

        // Compute control input with P Controller
        tf2::Vector3 mav_vel_sp = mav_pos_control.compute_control_input(mav_position_sp, mav_position);
        tf2::Vector3 mav_vel_xy_sp = mav_vel_sp;
        mav_vel_xy_sp.setZ(0.0);
        if(mav_vel_xy_sp.length() > 0.0){
            mav_vel_xy_sp = mav_vel_xy_sp.normalized() * std::min(mav_vel_xy_sp.length(), 3.0);
            mav_vel_sp.setX(mav_vel_xy_sp.x());
            mav_vel_sp.setY(mav_vel_xy_sp.y());
        }
        else{
            mav_vel_sp.setX(0.0);
            mav_vel_sp.setY(0.0);
        }
        // ROS_INFO("P Value = [%.4f, %.4f, %.4f]", mav_pos_control.P().x(), mav_pos_control.P().y(), mav_pos_control.P().z());
        // ROS_INFO("I Value = [%.4f, %.4f, %.4f]", mav_pos_control.I().x(), mav_pos_control.I().y(), mav_pos_control.I().z());
        // ROS_INFO("D Value = [%.4f, %.4f, %.4f]", mav_pos_control.D().x(), mav_pos_control.D().y(), mav_pos_control.D().z());
        

        // Prepare velocity measurement data in tf2::Vector3
        tf2::Vector3 mav_vel(mav_local_vel.m_msg.twist.linear.x, mav_local_vel.m_msg.twist.linear.y, mav_local_vel.m_msg.twist.linear.z);

        // Compute control input with PD Controller
        tf2::Vector3 mav_acc_sp = mav_vel_control.compute_control_input(mav_vel_sp, mav_vel);
        tf2::Vector3 mav_acc_sp_g = mav_acc_sp + tf2::Vector3(0.0,0.0, CONSTANT_G);

        // ROS_INFO("Vel_error.x = %.4f - %.4f = %.4f", mav_vel_sp.x(), mav_vel.x(), mav_vel_sp.x() - mav_vel.x());
        // ROS_INFO("Vel_error.y = %.4f - %.4f = %.4f", mav_vel_sp.y(), mav_vel.y(), mav_vel_sp.y() - mav_vel.y());
        // ROS_INFO("Vel_error.z = %.4f - %.4f = %.4f", mav_vel_sp.z(), mav_vel.z(), mav_vel_sp.z() - mav_vel.z());
        // ROS_INFO("acc_sp.x = %f", mav_acc_sp_g.getX());
        // ROS_INFO("acc_sp.y = %f", mav_acc_sp_g.getY());
        // ROS_INFO("acc_sp.z = %f", mav_acc_sp_g.getZ());

        tf2::Vector3 axis_of_rotation = tf2::Vector3(0.0, 0.0, 1.0).cross(mav_acc_sp_g);
        double angle = tf2::Vector3(0.0, 0.0, 1.0).angle(mav_acc_sp_g);

        tf2::Quaternion mav_yaw_sp(tf2::Vector3(0.0, 0.0, 1.0), mav_position_sps[i].second);
        tf2::Quaternion q_rotate(axis_of_rotation, angle);
        tf2::Quaternion mav_attitude_sp = q_rotate * mav_yaw_sp;
        mav_attitude_sp.normalize();
        // mav_attitude_sp.setRotation(axis_of_rotation, angle);
        double thrust_sp = mav_acc_sp_g.length() / CONSTANT_G * THRUST_HOVER;
        thrust_sp = std::max(std::min(0.9, thrust_sp), 0.1);
        // ROS_INFO("Thrust sp: %f", thrust_sp);

        mavros_msgs::Thrust thr_msg;
        thr_msg.header.stamp = ros::Time::now();
        thr_msg.thrust = thrust_sp;
        setpoint_thrust_pub.publish(thr_msg);

        geometry_msgs::PoseStamped attitude_msg;
        attitude_msg.header.stamp = ros::Time::now();
        attitude_msg.pose.position.x = 0;
        attitude_msg.pose.position.y = 0;
        attitude_msg.pose.position.z = 0;
        attitude_msg.pose.orientation.x = mav_attitude_sp.getX();
        attitude_msg.pose.orientation.y = mav_attitude_sp.getY();
        attitude_msg.pose.orientation.z = mav_attitude_sp.getZ();
        attitude_msg.pose.orientation.w = mav_attitude_sp.getW();
        setpoint_attitude_pub.publish(attitude_msg);
        if((mav_position_sp - mav_position).length() < 0.4 && mav_state.m_msg.mode == "OFFBOARD"){
            ROS_INFO("Setpoint pos: [%.4f, %.4f, %.4f] has been reached", mav_position_sp.x(),mav_position_sp.y(),mav_position_sp.z());
            if(i+1 < mav_position_sps.size()){
                i++;
            }
            else{
                mode.request.custom_mode = "AUTO.LOITER";
                flight_example_done = 1;
            }
        }

        if(flight_example_done && mav_state.m_msg.mode == "AUTO.LOITER"){
            break;
        }

        ros::spinOnce();
        // if(ros::Time::now() - time_last_send_info > ros::Duration(0.5)){
        //     // ROS_INFO("## mavros/global_position/global");
        //     // ROS_INFO("Latitude\t: %.8f", globalpos_global.m_msg.latitude);
        //     // ROS_INFO("Longitude\t: %.8f", globalpos_global.m_msg.longitude);
        //     // ROS_INFO("Altitude\t: %.8f", globalpos_global.m_msg.altitude);
        //     ROS_INFO("## mavros/local_position/pose");
        //     ROS_INFO("Pos.x\t\t: %.8f", mav_local_pose.m_msg.pose.position.x);
        //     ROS_INFO("Pos.y\t\t: %.8f", mav_local_pose.m_msg.pose.position.y);
        //     ROS_INFO("Pos.z\t\t: %.8f", mav_local_pose.m_msg.pose.position.z);
        //     ROS_INFO("Attitude.x\t: %.8f", mav_local_pose.m_msg.pose.orientation.x);
        //     ROS_INFO("Attitude.y\t: %.8f", mav_local_pose.m_msg.pose.orientation.y);
        //     ROS_INFO("Attitude.z\t: %.8f", mav_local_pose.m_msg.pose.orientation.z);
        //     ROS_INFO("Attitude.w\t: %.8f", mav_local_pose.m_msg.pose.orientation.w);
        //     // ROS_INFO("## mavros/local_position/velocity");
        //     // ROS_INFO("lin.x\t\t: %.8f", mav_local_vel.m_msg.twist.linear.x);
        //     // ROS_INFO("lin.y\t\t: %.8f", mav_local_vel.m_msg.twist.linear.y);
        //     // ROS_INFO("lin.z\t\t: %.8f", mav_local_vel.m_msg.twist.linear.z);
        //     // ROS_INFO("ang.x\t: %.8f", mav_local_vel.m_msg.twist.angular.x);
        //     // ROS_INFO("ang.y\t: %.8f", mav_local_vel.m_msg.twist.angular.y);
        //     // ROS_INFO("ang.z\t: %.8f", mav_local_vel.m_msg.twist.angular.z);
        //     ROS_INFO("## mavros/altitude");
        //     ROS_INFO("Altitude\t: %.8f", mav_alt_amsl.m_msg.amsl);

        //     // ROS_INFO("## mavros/state");
        //     // ROS_INFO("Connected\t: %d", mav_state.m_msg.connected);
        //     // ROS_INFO("Armed\t\t: %d", mav_state.m_msg.armed);
        //     // ROS_INFO("Guided\t\t: %d", mav_state.m_msg.guided);
        //     // ROS_INFO("Manual Input\t: %d", mav_state.m_msg.manual_input);
        //     // ROS_INFO("Mode\t\t: %s", mav_state.m_msg.mode.c_str());
        //     ROS_INFO(" ");
        //     time_last_send_info = ros::Time::now();
        // }
        
        loop_rate.sleep();
    }

    return 0;
}