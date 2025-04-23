import sys
import os
import plotly.graph_objects as go
from plotly.subplots import make_subplots

# Add compiled module to path
build_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'build'))
sys.path.append(build_path)

import openpid

# System setup
pid = openpid.PID(100.0, 1.0, 20.0)
msd = openpid.MassSpringDamper(1.0, 0.5, 5.0)

dt = 0.01
setpoint = 1.0
steps = 100

positions, forces, times = [], [], []

for i in range(steps):
    t = i * dt
    pos = msd.get_position()
    force = pid.compute(setpoint, pos, dt)
    msd.update(force, dt)

    times.append(t)
    positions.append(pos)
    forces.append(force)

# Create animated figure with subplots
fig = make_subplots(rows=2, cols=1, shared_xaxes=True,
                    subplot_titles=("Position vs Time", "Control Force vs Time"))

# Initial empty traces
fig.add_trace(go.Scatter(x=[], y=[], name="Position", mode='lines'), row=1, col=1)
fig.add_trace(go.Scatter(x=[], y=[], name="Force", mode='lines'), row=2, col=1)

# Animation frames
frames = []
for i in range(1, len(times)):
    frames.append(go.Frame(
        data=[
            go.Scatter(x=times[:i], y=positions[:i]),
            go.Scatter(x=times[:i], y=forces[:i]),
        ],
        name=str(i)
    ))

fig.frames = frames

# Animation layout
fig.update_layout(
    updatemenus=[dict(
        type="buttons",
        showactive=False,
        buttons=[dict(label="Play",
                      method="animate",
                      args=[None, {"frame": {"duration": 10, "redraw": True},
                                   "fromcurrent": True, "transition": {"duration": 0}}])],
    )],
    title="Animated PID Control of Mass-Spring-Damper System",
    xaxis_title="Time (s)",
    yaxis_title="Position",
    height=600
)

fig.update_yaxes(title_text="Position", row=1, col=1)
fig.update_yaxes(title_text="Force", row=2, col=1)
fig.update_xaxes(title_text="Time (s)", row=2, col=1)

fig.show()
