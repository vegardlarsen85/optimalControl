import matplotlib.pyplot as plt
import numpy as np
from Git.OptimalControl.DirectCollocation import dircol

# Double Integrator Example
# x[0] = position
# x[1] = velocity
# u[0] = control effort


def L(x,u):
    return x[0]**2 + x[1]**2

def phi(x,u):
    return 10*x[0]**2 + 10*x[1]**2

def f(x,u,d):
    dx = np.empty(shape=(len(x)))
    dx[0] = x[1]
    dx[1] = u[0] + d[0]
    return dx

t = np.arange(0,10,0.5)
controller = dircol(xdim=2,                       # Dimension of State Vector
                    udim=1,                       # Dimension of Control Vector
                    ddim=1,                       # Dimension of Disturbance Vector
                    L=L,                          # Running Cost
                    phi=phi,                      # Terminal Cost
                    f=f,                          # System Dynamics
                    g=[],                         # Point Inequality Constraints
                    h=[],                         # Point Equality Constraints
                    umax = [5],                   # Control Upper Bound
                    umin = [-5],                  # Control Lower Bound
                    xmax=[None,None],             # State Upper Bound
                    xmin=[None,None],             # State Lower Bound
                    t = t)                        # Discrete Time Steps. Shape = (N)


x,u = controller.path(x0=[-20,0],               # Initial State. Shape = (xdim)
                      u_guess = None,           # Initial Guess for Control Trajectory. Shape = (N,udim)
                      x_guess = None,           # Initial Guess for State Trajectory. Shape = (N,xdim)
                      d_guess = None,           # Initial Guess for Disturbance Trajectory. Shape = (N,ddim)
                      maxIter=10,               # Max iterations done by scipy's SLSQP Optimizer.
                      minCost=0,                # Min cost-tolerance in Optimizer before termination
                      xN=[None,None])           # Hard Equality-constraint on Terminal State. Shape = (xdim)


# Plotting discrete solution
plt.scatter(t,x[:,0],s=15)
plt.scatter(t,u[:,0],s=15)

# Interpolation into "Continuous" Solution
t = np.arange(0,10,0.01)
x,u = controller.interpolate(x=x,u=u,d=None,t=t)
plt.plot(t,u[:,0],label='control')
plt.plot(t,x[:,0],label='position')
plt.grid(True)
plt.legend()
plt.show()






