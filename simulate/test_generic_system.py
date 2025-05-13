import sys
import os
import math
import plotly.graph_objects as go
from plotly.subplots import make_subplots
import imageio
import numpy as np

# Add path to compiled openpid module
build_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'build'))
sys.path.append(build_path)
import openpid

# Define a simple second-order system: double integrator
def dynamics(x, u):
    dx = np.zeros_like(x)
    dx[0] = x[1]
    dx[1] = u[0]
    return dx

x0 = np.array([1.0, 0.0], dtype=np.float32)
sys = openpid.GenericSystem(dynamics, x0)

for t in range(100):
    u = np.array([1.0], dtype=np.float32)  # No input force
    sys.update(u, 0.05)
    print(sys.get_state())
