/* example_pid.c - Example usage of PID Controller */
#include <stdio.h>
#include "pid.h"

int main() {
    PIDController pid;
    pid_init(&pid, 1.0f, 0.1f, 0.05f);

    float setpoint = 10.0f;
    float measurement = 0.0f;
    float dt = 0.1f;

    for (int i = 0; i < 20; i++) {
        float control_signal = pid_compute(&pid, setpoint, measurement, dt);
        measurement += control_signal * dt;  // Simulated system response
        printf("Step %d: Control Signal = %.2f, Measurement = %.2f\n", i, control_signal, measurement);
    }
    
    return 0;
}
