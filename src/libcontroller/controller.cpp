#include "libcontroller/controller.h"
#include <tf2/LinearMath/Vector3.h>
#include <algorithm>
#include <limits>

PID_Controller::PID_Controller(tf2::Vector3 kp, tf2::Vector3 ki, tf2::Vector3 kd, tf2::Vector3 limit) :
        m_kp(kp),
        m_ki(ki),
        m_kd(kd),
        m_control_input_limit(limit),
        m_P(tf2::Vector3(0.0,0.0,0.0)),
        m_I(tf2::Vector3(0.0,0.0,0.0)),
        m_D(tf2::Vector3(0.0,0.0,0.0)),
        m_last_v(tf2::Vector3(0.0,0.0,0.0)){
    if(m_control_input_limit.x() < 0.0){
        m_control_input_limit.setX(std::numeric_limits<double>::max());
    }
    if(m_control_input_limit.y() < 0.0){
        m_control_input_limit.setY(std::numeric_limits<double>::max());
    }
    if(m_control_input_limit.z() < 0.0){
        m_control_input_limit.setZ(std::numeric_limits<double>::max());
    }
}

tf2::Vector3 PID_Controller::compute_control_input(tf2::Vector3 v_sp, tf2::Vector3 v){
    tf2::Vector3 error_v = v_sp - v;
    tf2::Vector3 delta_v = m_last_v - v;

    m_P = error_v * m_kp;
    m_I += error_v * m_ki;
    m_D = delta_v * m_kd;
    tf2::Vector3 raw_control_input = m_P + m_I + m_D;
    
    tf2::Vector3 control_input;
    control_input.setX(std::max(std::min(raw_control_input.x(),m_control_input_limit.x()), -m_control_input_limit.x()));
    control_input.setY(std::max(std::min(raw_control_input.y(),m_control_input_limit.y()), -m_control_input_limit.y()));
    control_input.setZ(std::max(std::min(raw_control_input.z(),m_control_input_limit.z()), -m_control_input_limit.z()));
    if(m_ki.x() != 0.0){
        m_I.setX(control_input.x() - (m_P.x() + m_D.x()));
    }
    if(m_ki.y() != 0.0){
        m_I.setY(control_input.y() - (m_P.y() + m_D.y()));
    }
    if(m_ki.z() != 0.0){
        m_I.setZ(control_input.z() - (m_P.z() + m_D.z()));
    }
    // m_I = (control_input - (m_P + m_D));
    m_last_v = v;
    return control_input;
}

tf2::Vector3 PID_Controller::P(){return m_P;}
tf2::Vector3 PID_Controller::I(){return m_I;}
tf2::Vector3 PID_Controller::D(){return m_D;}