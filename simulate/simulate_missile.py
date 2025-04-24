import sys
import os
import numpy as np
import plotly.graph_objects as go
from plotly.subplots import make_subplots
import imageio

# Use absolute path from script location
build_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'build'))
sys.path.append(build_path)

import openpid

# Simulation parameters
dt = 0.05
steps = 300

# Initialize missile system
missile = openpid.Missile6DOFQuat()
missile.reset_state()

# Initial thrust in body frame: [Fx, Fy, Fz, Mx, My, Mz]
u = np.array([22000, 0, 0, 0, 0, 0], dtype=np.float64)

# Logs
states = []
positions = []
heights = []
times = []

for i in range(steps):
    t = i * dt
    state = missile.get_state()
    dx = missile.dynamics(state, u)
    new_state = state + dx * dt
    missile.set_state(new_state)

    states.append(state)
    positions.append((state[0], state[1]))
    heights.append(state[2])
    times.append(t)

states = np.array(states)

# Plotting
fig = make_subplots(
    rows=3, cols=1,
    specs=[
        [{"type": "xy"}],
        [{"type": "xy"}],
        [{"type": "scene"}]  # 👈 enables 3D plotting in this subplot
    ],
    subplot_titles=("X-Y Position", "Z Height", "3D Trajectory")
)

# Position X-Y
fig.add_trace(go.Scatter(x=[p[0] for p in positions], y=[p[1] for p in positions],
                         mode="lines", name="XY Path"), row=1, col=1)

# Height
fig.add_trace(go.Scatter(x=times, y=heights, mode="lines", name="Z Height"), row=2, col=1)

# 3D
fig.add_trace(go.Scatter3d(
    x=states[:, 0], y=states[:, 1], z=states[:, 2],
    mode="lines", line=dict(width=4),
    name="3D Trajectory"
), row=3, col=1)

fig.update_layout(height=900, title="Missile 6DOF Simulation Without Control")
fig.update_yaxes(title="Y (m)", row=1, col=1)
fig.update_yaxes(title="Height Z (m)", row=2, col=1)
fig.update_xaxes(title="Time (s)", row=2, col=1)

fig.update_layout(scene=dict(
    xaxis_title='X (m)',
    yaxis_title='Y (m)',
    zaxis_title='Z (m)'
), scene_aspectmode='cube')

fig.show()
