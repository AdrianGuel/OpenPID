#ifndef STATE_FEEDBACK_HPP
#define STATE_FEEDBACK_HPP

#include <array>

/**
 * Full-State Feedback controller class for 4-state systems (e.g. inverted pendulum on cart).
 * Control law: F = -(k1*x0 + k2*x1 + k3*(x2 - r) + k4*x3)
 */
template <typename T>
class StateFeedback {
public:
    StateFeedback(T k1, T k2, T k3, T k4)
        : k1_(k1), k2_(k2), k3_(k3), k4_(k4) {}

    void set_reference(T r) {
        r_ = r;
    }

    T compute(const std::array<T, 4>& state) const {
        return -(k1_ * state[0] + k2_ * state[1] + k3_ * (state[2] - r_) + k4_ * state[3]);
    }

private:
    T k1_, k2_, k3_, k4_;
    T r_ = static_cast<T>(0);
};

#endif // STATE_FEEDBACK_HPP