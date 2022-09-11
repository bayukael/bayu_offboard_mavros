#ifndef CONTROLLER_H_INCLUDED
#define CONTROLLER_H_INCLUDED

#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Vector3.h>

class PID_Controller{
    tf2::Vector3 m_kp;
    tf2::Vector3 m_ki;
    tf2::Vector3 m_kd;
    tf2::Vector3 m_P;
    tf2::Vector3 m_I;
    tf2::Vector3 m_D;
    tf2::Vector3 m_last_v;
    tf2::Vector3 m_control_input_limit;
    public:
    PID_Controller(tf2::Vector3 kp, tf2::Vector3 ki, tf2::Vector3 kd, tf2::Vector3 limit);
    tf2::Vector3 compute_control_input(tf2::Vector3 v_sp, tf2::Vector3 v);
    tf2::Vector3 P();
    tf2::Vector3 I();
    tf2::Vector3 D();
    
};

#endif