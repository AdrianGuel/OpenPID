import sys
import os
import math
import plotly.graph_objects as go
from plotly.subplots import make_subplots

# Add path to compiled openpid module
build_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'build'))
sys.path.append(build_path)

import openpid

# Parameters from your Processing sketch
g = 9.81
M = 0.5     # cart mass
m = 0.2     # pendulum mass
b = 0.1     # friction
I = 0.006   # inertia
l = 0.3     # length to center of mass
dt = 0.05   # timestep

# PID recursive coefficients
kp = 10
ki = 10
kd = 2

r = math.pi  # reference angle (inverted)
ek = 0.0
ek1 = 0.0
ek2 = 0.0
uk1 = 0.0

# Create the system
pend = openpid.PendulumOnCart(M, m, l, b, I, g)
pend.reset(x0=0, v0=0.0, theta0=math.pi + 0.01, omega0=0.2)
# Create PID controller
pid = openpid.PID(kp, ki, kd)
pid.reset()

# Logs
steps = 1000
times, angles, positions, forces = [], [], [], []

for i in range(steps):
    t = i * dt
    theta = pend.get_angle()
    
    # Use recursive PID control
    F = pid.compute_recursive(r, theta, dt)

    # Update system
    pend.update(F, dt)

    # Log
    times.append(t)
    angles.append(theta)
    positions.append(pend.get_position())
    forces.append(F)

# Animate angle and force
fig = make_subplots(rows=2, cols=1, shared_xaxes=True,
                    subplot_titles=("Pendulum Angle (rad)", "Control Force (N)"))

fig.add_trace(go.Scatter(x=[times[0]], y=[angles[0]], mode="lines", name="Angle"), row=1, col=1)
fig.add_trace(go.Scatter(x=[times[0]], y=[forces[0]], mode="lines", name="Force"), row=2, col=1)

frames = [
    go.Frame(
        data=[
            go.Scatter(x=times[:i], y=angles[:i]),
            go.Scatter(x=times[:i], y=forces[:i]),
        ],
        name=str(i)
    )
    for i in range(2, len(times))
]

fig.frames = frames

fig.update_layout(
    updatemenus=[{
        "type": "buttons",
        "buttons": [{
            "label": "Play",
            "method": "animate",
            "args": [None, {
                "frame": {"duration": 10, "redraw": True},
                "fromcurrent": True,
                "transition": {"duration": 0}
            }]
        }]
    }],
    title="Recursive PID Control of Inverted Pendulum",
    height=600
)

fig.update_yaxes(title_text="Angle (rad)", row=1, col=1)
fig.update_yaxes(title_text="Force (N)", row=2, col=1)
fig.update_xaxes(title_text="Time (s)", row=2, col=1)

fig.show()
