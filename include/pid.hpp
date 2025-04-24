#ifndef PID_HPP
#define PID_HPP

template <typename T>
class PID {
public:
    PID(T kp, T ki, T kd)
        : kp_(kp), ki_(ki), kd_(kd),
          prev_error_(0), integral_(0),
          ek1_(0), ek2_(0), uk1_(0) {}

    void reset() {
        prev_error_ = static_cast<T>(0);
        integral_ = static_cast<T>(0);
        ek1_ = ek2_ = uk1_ = static_cast<T>(0);
    }

    // Classic PID implementation
    T compute(T setpoint, T measurement, T dt) {
        T error = setpoint - measurement;
        integral_ += error * dt;
        T derivative = (error - prev_error_) / dt;
        T output = (kp_ * error) + (ki_ * integral_) + (kd_ * derivative);
        prev_error_ = error;
        return output;
    }

    // Recursive PID
    T compute_recursive(T setpoint, T measurement, T dt) {
        T ek = setpoint - measurement;
        T output = uk1_ + (kp_ + ki_ * dt + kd_ / dt) * ek
                         + (-2 * kd_ / dt - kp_) * ek1_
                         + (kd_ / dt) * ek2_;

        // Update internal state
        uk1_ = output;
        ek2_ = ek1_;
        ek1_ = ek;

        return output;
    }

private:
    T kp_, ki_, kd_;
    T prev_error_;
    T integral_;

    // For recursive PID
    T ek1_, ek2_, uk1_;
};

#endif // PID_HPP

