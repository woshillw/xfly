#include "xfly_pid_controller_node.h"

void OdometryCallback(const nav_msgs::Odometry::ConstPtr &odometry);

Xlfy_Geometry::Xlfy_Geometry(std::string pubstr, std::string substr,  ros::NodeHandle &n1, const ros::NodeHandle &_private_nh)
    : cmd_Q(1.0f, 0.0f, 0.0f, 0.0f),
      cmd_Angular(0.0f, 0.0f, 0.0f),
      expect_position(0.0f, 0.0f, 0.0f),
      position_gain(0.0f, 1.0f, 0.5f),
      attitude_gain(1.0f, 1.0f, 0.7f),
      output_angular_accel(0.0f, 0.0f, 0.0f),
      dt(0.01),
      n1(n1),
      _private_nh(_private_nh)
{
    motor_pub = n1.advertise<mav_msgs::Actuators>(pubstr, 10);

    odemetry_sub = n1.subscribe(substr, 10, &Xlfy_Geometry::OdometryCallback, this);

    command_pose_sub = n1.subscribe("command/pose", 1, &Xlfy_Geometry::CommandPosCallback, this);

    _state_r_pub = n1.advertise<std_msgs::Float32>("state/roll", 1);

    _state_p_pub = n1.advertise<std_msgs::Float32>("state/pitch", 1);

    _state_y_pub = n1.advertise<std_msgs::Float32>("state/yaw", 1);

    _state_r_der_pub = n1.advertise<std_msgs::Float32>("state/roll_der", 1);

    _state_p_der_pub = n1.advertise<std_msgs::Float32>("state/pitch_der", 1);

    _state_y_der_pub = n1.advertise<std_msgs::Float32>("state/yaw_der", 1);

    _state_u_pub = n1.advertise<std_msgs::Float32>("state/xpos", 1);

    _state_v_pub = n1.advertise<std_msgs::Float32>("state/ypos", 1);

    _state_w_pub = n1.advertise<std_msgs::Float32>("state/zpos", 1);

    _state_u_der_pub = n1.advertise<std_msgs::Float32>("state/xvel", 1);

    _state_v_der_pub = n1.advertise<std_msgs::Float32>("state/yvel", 1);

    _state_w_der_pub = n1.advertise<std_msgs::Float32>("state/zvel", 1);

    Eigen::Matrix<double, 3, 4> angular_to_m;

    angular_to_m << -1, 1, 1, -1,
        -1, 1, -1, 1,
        -1, -1, 1, 1;

    _allocationMatrix = Pinv(angular_to_m);

    InitParam();
}

Eigen::MatrixXd Xlfy_Geometry::Pinv(Eigen::MatrixXd A)
{
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV); // M=USV*
    double Pinvtoler = 1.e-8;                                                            // tolerance
    int row = A.rows();
    int col = A.cols();
    int k = std::min(row, col);
    Eigen::MatrixXd X = Eigen::MatrixXd::Zero(col, row);
    Eigen::MatrixXd singularValues_inv = svd.singularValues(); //奇异值
    Eigen::MatrixXd singularValues_inv_mat = Eigen::MatrixXd::Zero(col, row);
    for (long i = 0; i < k; ++i)
    {
        if (singularValues_inv(i) > Pinvtoler)
            singularValues_inv(i) = 1.0 / singularValues_inv(i);
        else
            singularValues_inv(i) = 0;
    }
    for (long i = 0; i < k; ++i)
    {
        singularValues_inv_mat(i, i) = singularValues_inv(i);
    }
    X = (svd.matrixV()) * (singularValues_inv_mat) * (svd.matrixU().transpose()); // X=VS+U*

    return X;
}

void Xlfy_Geometry::showParam()
{
    std::cout << "FM_b_to_actuator allocation:\n"
              << _allocationMatrix << std::endl;
}

void Xlfy_Geometry::CommandPosCallback(xfly::xfly_pose pose)
{
    expect_position(0) = pose.x;
    expect_position(1) = pose.y;
    expect_position(2) = pose.z;
}

void Xlfy_Geometry::OdometryCallback(nav_msgs::Odometry odometry)
{
    // ROS_INFO("Get!");

    state_Q.w() = odometry.pose.pose.orientation.w;
    state_Q.x() = odometry.pose.pose.orientation.x;
    state_Q.y() = odometry.pose.pose.orientation.y;
    state_Q.z() = odometry.pose.pose.orientation.z;
    state_Q.normalize();

    state_pos(0) = odometry.pose.pose.position.x;
    state_pos(1) = odometry.pose.pose.position.y;
    state_pos(2) = odometry.pose.pose.position.z;

    state_vel(0) = odometry.twist.twist.linear.x;
    state_vel(1) = odometry.twist.twist.linear.y;
    state_vel(2) = odometry.twist.twist.linear.z;
    state_vel = state_Q.toRotationMatrix() * state_vel; //从机体系旋转到导航系

    state_angle_rate(0) = odometry.twist.twist.angular.x;
    state_angle_rate(1) = odometry.twist.twist.angular.y;
    state_angle_rate(2) = odometry.twist.twist.angular.z;
}

void Xlfy_Geometry::GetRosParameter(const ros::NodeHandle &nh, const std::string &key,
                                    const float &default_value, float *value)
{
    ROS_ASSERT(value != nullptr);
    bool have_parameter = nh.getParam(key, *value);
    if (!have_parameter)
    {
        ROS_WARN_STREAM("[rosparam]: could not find parameter " << nh.getNamespace() << "/" << key
                                                                << ", setting to default: " << default_value);
        *value = default_value;
    }
}

void Xlfy_Geometry::InitParam()
{
    float gain, kp, ki, kd, integral_limit, output_limit,dt;
    GetRosParameter(_private_nh, "pid_dt", 0.01, &dt);

    GetRosParameter(_private_nh, "position_gain/x", 0.0, &gain);
    position_gain(0) = gain;
    GetRosParameter(_private_nh, "position_gain/y", 0.0, &gain);
    position_gain(1) = gain;
    GetRosParameter(_private_nh, "position_gain/z", 0.0, &gain);
    position_gain(2) = gain;

    GetRosParameter(_private_nh, "pid_u/kp", 1, &kp);
    GetRosParameter(_private_nh, "pid_u/ki", 0.01, &ki);
    GetRosParameter(_private_nh, "pid_u/kd", 0.55, &kd);
    GetRosParameter(_private_nh, "pid_u/integral_limit", 2, &integral_limit);
    GetRosParameter(_private_nh, "pid_u/output_limit", 1, &output_limit);
    PID_Init(&_pid_u, kp, ki, kd, dt, output_limit, integral_limit);
    
    GetRosParameter(_private_nh, "pid_v/kp", 1, &kp);
    GetRosParameter(_private_nh, "pid_v/ki", 0.01, &ki);
    GetRosParameter(_private_nh, "pid_v/kd", 0.55, &kd);
    GetRosParameter(_private_nh, "pid_v/integral_limit", 2, &integral_limit);
    GetRosParameter(_private_nh, "pid_v/output_limit", 1, &output_limit);
    PID_Init(&_pid_v, kp, ki, kd, dt, output_limit, integral_limit);
    
    GetRosParameter(_private_nh, "pid_w/kp", 1, &kp);
    GetRosParameter(_private_nh, "pid_w/ki", 0.01, &ki);
    GetRosParameter(_private_nh, "pid_w/kd", 0.55, &kd);
    GetRosParameter(_private_nh, "pid_w/integral_limit", 2, &integral_limit);
    GetRosParameter(_private_nh, "pid_w/output_limit", 1, &output_limit);
    PID_Init(&_pid_w, kp, ki, kd, dt, output_limit, integral_limit);
    
    GetRosParameter(_private_nh, "pid_p/kp", 1.8, &kp);
    GetRosParameter(_private_nh, "pid_p/ki", 0.025, &ki);
    GetRosParameter(_private_nh, "pid_p/kd", 0.88, &kd);
    GetRosParameter(_private_nh, "pid_p/integral_limit", 2, &integral_limit);
    GetRosParameter(_private_nh, "pid_p/output_limit", 1, &output_limit);
    PID_Init(&_pid_p, kp, ki, kd, dt, output_limit, integral_limit);
    
    GetRosParameter(_private_nh, "pid_q/kp", 1.8, &kp);
    GetRosParameter(_private_nh, "pid_q/ki", 0.025, &ki);
    GetRosParameter(_private_nh, "pid_q/kd", 0.88, &kd);
    GetRosParameter(_private_nh, "pid_q/integral_limit", 2, &integral_limit);
    GetRosParameter(_private_nh, "pid_q/output_limit", 1, &output_limit);
    PID_Init(&_pid_q, kp, ki, kd, dt, output_limit, integral_limit);
    
    GetRosParameter(_private_nh, "pid_r/kp", 1.8, &kp);
    GetRosParameter(_private_nh, "pid_r/ki", 0.025, &ki);
    GetRosParameter(_private_nh, "pid_r/kd", 0.88, &kd);
    GetRosParameter(_private_nh, "pid_r/integral_limit", 2, &integral_limit);
    GetRosParameter(_private_nh, "pid_r/output_limit", 1, &output_limit);
    PID_Init(&_pid_r, kp, ki, kd, dt, output_limit, integral_limit);
    
    float kf, km, l;
    ////从 rosparam 读取飞行器参数 初始化控制分配矩阵

    GetRosParameter(n1, "vehicle/kf", 1.7088e-05, &kf);
    GetRosParameter(n1, "vehicle/km", 1.7088e-05 * 0.016, &km);
    GetRosParameter(n1, "vehicle/l", 0.355, &l);
    GetRosParameter(n1, "vehicle/mass", 2.274, &_mass);
    GetRosParameter(n1, "vehicle/Ix", 0.021968, &_Ix);
    GetRosParameter(n1, "vehicle/Iy", 0.021968, &_Iy);
    GetRosParameter(n1, "vehicle/Iz", 0.042117, &_Iz);
}

void Xlfy_Geometry::EulerBasedControl(double dt)
{
    Eigen::Vector3d expect_velocity(0, 0, 0);
    Eigen::Vector3d expect_angular(0, 0, 0);
    Eigen::Vector3d expect_rpy(0, 0, 0);
    Eigen::Vector3d expect_accel(0, 0, 0);

    ////////////位置控制 双环PID///////////////////
    expect_velocity << position_gain(0) * (expect_position(0) - state_pos(0)),
        position_gain(1) * (expect_position(1) - state_pos(1)),
        position_gain(2) * (expect_position(2) - state_pos(2));

    expect_accel << PID_calculate(&_pid_u, expect_velocity(0), state_vel(0)),
        PID_calculate(&_pid_v, expect_velocity(1), state_vel(1)),
        PID_calculate(&_pid_w, expect_velocity(2), state_vel(2));

    _F_e = expect_accel;

    ////////////attitude控制 双环PID///////////////////
    expect_rpy(0) = -expect_accel(1);
    expect_rpy(1) = expect_accel(0);

    expect_angular << attitude_gain(0) * (expect_rpy(0) - state_rpy(0)*57.3),
        attitude_gain(1) * (expect_rpy(1) - state_rpy(1)*57.3),
        0;
    expect_angular = expect_angular / 57.3;

    output_angular_accel << PID_calculate(&_pid_p, expect_angular(0), state_angle_rate(0)),
        PID_calculate(&_pid_q, expect_angular(1), state_angle_rate(1)),
        PID_calculate(&_pid_r, expect_angular(2), state_angle_rate(2));

    // if (output_angular_accel.norm() > 1)
    // {
    //     output_angular_accel /= output_angular_accel.norm();
    // }
}

void Xlfy_Geometry::QuaternionBasedControl(double dt)
{
    // err_Q = state_Q.conjugate() * cmd_Q;
    // cmd_Angular = 2.0f / 0.225 * Sgn(err_Q.w()) * err_Q.vec();

    // output_angular_accel << pid_calculate(&_pid_p, cmd_Angular(0), state_angle_rate(0), 0, dt),
    //     pid_calculate(&_pid_q, cmd_Angular(1), state_angle_rate(1), 0, dt),
    //     pid_calculate(&_pid_r, cmd_Angular(2), state_angle_rate(2), 0, dt);

    // if (output_angular_accel.norm() > 1)
    // {
    //     output_angular_accel /= output_angular_accel.norm();
    // }
}

void Xlfy_Geometry::ControlAllocation()
{
    Eigen::Matrix<double, 4, 1> MotorControl;
    MotorControl = _allocationMatrix * (output_angular_accel);

    // printf("z=%f\n", _F_e(2));
    // printf("vx=%f,vy=%f,vz=%f\n", state_vel(0), state_vel(1), state_vel(2));
    // printf("px=%f,py=%f,pz=%f\n", state_pos(0), state_pos(1), state_pos(2));

    double motor[4];
    // motor[0] = _F_e(2) * 800; //+ MotorControl[0] * 400;
    // motor[1] = _F_e(2) * 800; //+ MotorControl[1] * 400;
    // motor[2] = _F_e(2) * 800; //+ MotorControl[2] * 400;
    // motor[3] = _F_e(2) * 800; //+ MotorControl[3] * 400;

    motor[0] = _F_e(2) * 1000 + MotorControl[0] * 500;
    motor[1] = _F_e(2) * 1000 + MotorControl[1] * 500;
    motor[2] = _F_e(2) * 1000 + MotorControl[2] * 500;
    motor[3] = _F_e(2) * 1000 + MotorControl[3] * 500;

    // motor[0] = _F_e(2) * 800;
    // motor[1] = _F_e(2) * 800;
    // motor[2] = _F_e(2) * 800;
    // motor[3] = _F_e(2) * 800;
    // printf("m1=%f,m2=%f,m3=%f,m4=%f\n", motor[0], motor[1], motor[2], motor[3]);
    SetMotor(motor);
}

double Xlfy_Geometry::Sgn(double val)
{
    if (val >= 0.0f)
        return 1.0f;
    else
        return -1.0f;
}

Eigen::Vector3d &Xlfy_Geometry::getRPY()
{
    return state_rpy;
}
Eigen::Quaterniond &Xlfy_Geometry::getQuaterniond()
{
    return state_Q;
}

void Xlfy_Geometry::QtoEuler(Eigen::Vector3d &rpy, const Eigen::Quaterniond &Q)
{
    rpy(0) = atan2(2 * (Q.w() * Q.x() + Q.y() * Q.z()), 1 - 2 * (Q.x() * Q.x() + Q.y() * Q.y()));
    rpy(1) = asin(2 * (Q.w() * Q.y() - Q.x() * Q.z()));
    rpy(2) = atan2(2 * (Q.w() * Q.z() + Q.y() * Q.x()), 1 - 2 * (Q.z() * Q.z() + Q.y() * Q.y()));
}

void Xlfy_Geometry::SendRPY()
{
    std_msgs::Float32 msg;
    msg.data = state_rpy(0) / M_PI * 180;
    _state_r_pub.publish(msg);
    msg.data = state_rpy(1) / M_PI * 180;
    _state_p_pub.publish(msg);
    msg.data = state_rpy(2) / M_PI * 180;
    _state_y_pub.publish(msg);

    msg.data = state_angle_rate(0) / M_PI * 180;
    _state_r_der_pub.publish(msg);
    msg.data = state_angle_rate(1) / M_PI * 180;
    _state_p_der_pub.publish(msg);
    msg.data = state_angle_rate(2) / M_PI * 180;
    _state_y_der_pub.publish(msg);

    msg.data = state_pos(0);
    _state_u_pub.publish(msg);
    msg.data = state_pos(1);
    _state_v_pub.publish(msg);
    msg.data = state_pos(2);
    _state_w_pub.publish(msg);

    msg.data = state_vel(0);
    _state_u_der_pub.publish(msg);
    msg.data = state_vel(1);
    _state_v_der_pub.publish(msg);
    msg.data = state_vel(2);
    _state_w_der_pub.publish(msg);
}

void Xlfy_Geometry::SetMotor(double motor[])
{
    motor_msg.angular_velocities.clear();

    motor_msg.angular_velocities.push_back(motor[0]);
    motor_msg.angular_velocities.push_back(motor[1]);
    motor_msg.angular_velocities.push_back(motor[2]);
    motor_msg.angular_velocities.push_back(motor[3]);

    motor_pub.publish(motor_msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "xfly_pid_controller_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    Xlfy_Geometry xlfy_Geometry("/xfly/gazebo/command/motor_speed", "/xfly/ground_truth/odometry", nh, private_nh);

    ros::Rate loop_rate(100);

    xlfy_Geometry.showParam();

    double testmotor[4] = {520, 520, 520, 520};
    while (ros::ok())
    {
        xlfy_Geometry.EulerBasedControl(0.001);
        xlfy_Geometry.ControlAllocation();

        ros::spinOnce();

        xlfy_Geometry.QtoEuler(xlfy_Geometry.getRPY(), xlfy_Geometry.getQuaterniond());
        xlfy_Geometry.SendRPY();

        loop_rate.sleep();
    }

    return 0;
}