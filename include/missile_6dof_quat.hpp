#pragma once

#include <Eigen/Dense>

class Missile6DOFQuat {
public:
    Missile6DOFQuat();
    
    Eigen::VectorXd dynamics(const Eigen::VectorXd& state, const Eigen::VectorXd& control);
    void reset_state();
    Eigen::VectorXd get_state() const;
    void set_state(const Eigen::VectorXd& new_state);

private:
    Eigen::VectorXd state; // 13D state
    double mass, g;
    Eigen::Matrix3d J; // Inertia matrix
};
