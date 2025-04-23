#ifndef MSD_SYSTEM_HPP
#define MSD_SYSTEM_HPP

#include <cmath>

template <typename T>
class MassSpringDamper {
public:
    MassSpringDamper(T mass, T damping, T stiffness)
        : m_(mass), c_(damping), k_(stiffness), x_(0), v_(0) {}

    void reset(T x0 = 0, T v0 = 0) {
        x_ = x0;
        v_ = v0;
    }

    // Simulates one time step with given force input
    void update(T force, T dt) {
        T a = (force - c_ * v_ - k_ * x_) / m_;
        v_ += a * dt;
        x_ += v_ * dt;
    }

    T get_position() const { return x_; }
    T get_velocity() const { return v_; }

private:
    T m_, c_, k_;  // mass, damping, stiffness
    T x_, v_;      // position, velocity
};

#endif // MSD_SYSTEM_HPP
