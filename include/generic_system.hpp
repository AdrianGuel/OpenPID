#ifndef GENERIC_SYSTEM_HPP
#define GENERIC_SYSTEM_HPP

#include <Eigen/Dense>
#include <functional>

template <typename T>
class GenericSystem {
public:
    using Vector = Eigen::Matrix<T, Eigen::Dynamic, 1>;
    using DynamicsFunc = std::function<Vector(const Vector&, const Vector&)>;

    GenericSystem(DynamicsFunc dynamics, const Vector& x0)
        : dynamics_fn(dynamics), state(x0) {}

    void reset(const Vector& x0) {
        state = x0;
    }

    void update(const Vector& input, T dt) {
        Vector dx = dynamics_fn(state, input);
        state += dt * dx;
    }

    Vector get_state() const {
        return state;
    }

private:
    DynamicsFunc dynamics_fn;
    Vector state;
};

#endif // GENERIC_SYSTEM_HPP
