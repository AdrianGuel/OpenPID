/* pid.h - PID Controller Header */
#ifndef PID_H
#define PID_H

typedef struct {
    float kp;  // Proportional gain
    float ki;  // Integral gain
    float kd;  // Derivative gain
    float prev_error;  // Previous error
    float integral;  // Integral term
} PIDController;

void pid_init(PIDController *pid, float kp, float ki, float kd);
float pid_compute(PIDController *pid, float setpoint, float measurement, float dt);

#endif /* PID_H */
