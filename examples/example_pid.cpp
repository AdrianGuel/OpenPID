#include <iostream>
#include "pid.hpp"
// #include "msd_system.hpp"

int main() {
    PID<float> pid(100.0f, 1.0f, 20.0f);  // Adjust gains as needed
    // MassSpringDamper<float> msd(1.0f, 0.5f, 5.0f);  // mass=1kg, damping=0.5, stiffness=5

    // float dt = 0.01f;
    // float setpoint = 1.0f;  // target position

    // for (int i = 0; i < 1000; ++i) {
    //     float position = msd.get_position();
    //     float force = pid.compute(setpoint, position, dt);
    //     msd.update(force, dt);

    //     std::cout << "t=" << i*dt << "s | Position: " << position
    //               << " | Velocity: " << msd.get_velocity()
    //               << " | Force: " << force << "\n";
    // }

    return 0;
}
