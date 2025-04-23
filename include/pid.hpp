#ifndef PID_HPP
#define PID_HPP

template <typename T>
class PID {
public:
    PID(T kp, T ki, T kd) 
        : kp_(kp), ki_(ki), kd_(kd), prev_error_(0), integral_(0) {}

    void reset() {
        prev_error_ = static_cast<T>(0);
        integral_ = static_cast<T>(0);
    }

    T compute(T setpoint, T measurement, T dt) {
        T error = setpoint - measurement;
        integral_ += error * dt;
        T derivative = (error - prev_error_) / dt;
        T output = (kp_ * error) + (ki_ * integral_) + (kd_ * derivative);
        prev_error_ = error;
        return output;
    }

private:
    T kp_, ki_, kd_;
    T prev_error_;
    T integral_;
};

#endif // PID_HPP
