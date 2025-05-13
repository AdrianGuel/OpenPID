#include <iostream>
#include <Eigen/Dense>
#include "pid.hpp"
#include "generic_system.hpp"

Eigen::VectorXf msd_dynamics(const Eigen::VectorXf& x, const Eigen::VectorXf& u) {
    float m = 1.0f;   // mass
    float c = 0.5f;   // damping
    float k = 5.0f;   // stiffness

    Eigen::VectorXf dx(2);
    dx(0) = x(1);  // dx = v
    dx(1) = (u(0) - c * x(1) - k * x(0)) / m;  // dv = (F - cv - kx)/m

    return dx;
}

int main() {
    using namespace std;

    PID<float> pid(100.0f, 1.0f, 20.0f);  // PID gains

    // Initial state: x = 0, v = 0
    Eigen::VectorXf x0(2);
    x0 << 0.0f, 0.0f;

    // Create GenericSystem with user-defined dynamics
    GenericSystem<float> msd(msd_dynamics, x0);

    float dt = 0.01f;
    float setpoint = 1.0f;

    for (int i = 0; i < 1000; ++i) {
        Eigen::VectorXf state = msd.get_state();  // x and v
        float force = pid.compute(setpoint, state(0), dt);

        Eigen::VectorXf u(1);
        u << force;

        msd.update(u, dt);

        cout << "t=" << i * dt << "s | Position: " << state(0)
             << " | Velocity: " << state(1)
             << " | Force: " << force << "\n";
    }

    return 0;
}
