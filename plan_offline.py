import casadi as ca
import numpy as np
import matplotlib.pyplot as plt

# Parameters
mu = 0.8           # friction coefficient
g = 9.81           # gravity
amax = 2.0         # max acceleration [m/s^2]
L = 10.0           # total arc length of the closed loop [m]
N = 100            # number of discretization points
ds = L / (N - 1)   # distance between points

# Curvature profile: alternating straight and curves (R=inf for straight, R=2 for curves)
R_profile = np.ones(N) * np.inf
for i in range(20, 40):
    R_profile[i] = 2.0
for i in range(60, 80):
    R_profile[i] = 2.0

# Convert curvature to max velocity constraint (v^2/R <= mu * g)
vmax_profile = np.sqrt(np.minimum(mu * g * R_profile, np.inf))

# CasADi optimization variables
v = ca.MX.sym("v", N)
a = ca.MX.sym("a", N - 1)

# Objective: minimize total time T ≈ sum(ds / v_i)
T = 0
for i in range(N - 1):
    T += ds / v[i]

# Constraints list
g = []
lbg = []
ubg = []

# Initial and final velocity (closed loop)
g += [v[0], v[-1]]
lbg += [0.0, 0.0]
ubg += [0.0, 0.0]

# Velocity bounds from curvature
for i in range(N):
    g += [v[i]]
    lbg += [0.0]
    ubg += [vmax_profile[i]]

# Acceleration constraints: a_i = (v_{i+1}^2 - v_i^2)/(2*ds)
for i in range(N - 1):
    ai = (v[i + 1]**2 - v[i]**2) / (2 * ds)
    g += [ai]
    lbg += [-amax]
    ubg += [amax]

# Define NLP
opt_variables = v
nlp = {"x": opt_variables, "f": T, "g": ca.vertcat(*g)}

# Solver options
opts = {"ipopt.print_level": 0, "print_time": 0}
solver = ca.nlpsol("solver", "ipopt", nlp, opts)

# Initial guess
v0 = np.minimum(vmax_profile, 1.0)

# Solve
solution = solver(x0=v0, lbg=lbg, ubg=ubg)
v_opt = np.array(solution["x"]).flatten()

# Plot results
print(v_opt)


