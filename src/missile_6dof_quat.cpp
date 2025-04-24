#include "missile_6dof_quat.hpp"
#include <Eigen/Geometry>

Missile6DOFQuat::Missile6DOFQuat() {
    state = Eigen::VectorXd::Zero(13);
    state(6) = 1.0; // unit quaternion

    g = 9.81;
    mass = 2513.74;

    J = Eigen::Matrix3d::Zero();
    J.diagonal() << 451000.61, 171000.0, 171000.0; // Ix, Iy, Iz
}

Eigen::VectorXd Missile6DOFQuat::dynamics(const Eigen::VectorXd& x, const Eigen::VectorXd& u) {
    Eigen::VectorXd dx(13);

    // Extract state
    Eigen::Vector3d pos = x.segment<3>(0);
    Eigen::Vector3d vel = x.segment<3>(3);
    Eigen::Quaterniond q(x(6), x(7), x(8), x(9)); // [w, x, y, z]
    Eigen::Vector3d omega = x.segment<3>(10);

    q.normalize();

    // Extract control
    Eigen::Vector3d force_body = u.segment<3>(0); // [Fx, Fy, Fz]
    Eigen::Vector3d torque_body = u.segment<3>(3); // [Tx, Ty, Tz]

    // Rotation: body → world
    Eigen::Matrix3d R = q.toRotationMatrix();

    // Translational dynamics
    Eigen::Vector3d accel = (R * force_body) / mass;
    accel[2] -= g; // gravity along Z in inertial

    dx.segment<3>(0) = vel;
    dx.segment<3>(3) = accel;

    // Quaternion derivative
    Eigen::Quaterniond omega_q(0, omega(0), omega(1), omega(2));
    Eigen::Quaterniond q_dot = q * omega_q;
    q_dot.coeffs() *= 0.5;

    dx(6) = q_dot.w();
    dx(7) = q_dot.x();
    dx(8) = q_dot.y();
    dx(9) = q_dot.z();

    // Angular velocity derivative
    Eigen::Vector3d omega_dot = J.inverse() * (torque_body - omega.cross(J * omega));
    dx.segment<3>(10) = omega_dot;

    return dx;
}

void Missile6DOFQuat::reset_state() {
    state = Eigen::VectorXd::Zero(13);
    state(6) = 1.0; // reset quaternion
}

Eigen::VectorXd Missile6DOFQuat::get_state() const {
    return state;
}

void Missile6DOFQuat::set_state(const Eigen::VectorXd& new_state) {
    state = new_state;
}
