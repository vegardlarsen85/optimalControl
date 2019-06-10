import numpy as np
from scipy.optimize import minimize
from scipy.optimize import Bounds
import itertools


class dircol:

    def __init__(self,xdim,udim,ddim,L,phi,f,g,h,umax,umin,xmax,xmin,t):
        self.xdim = xdim                        # Dimension of State-Vector
        self.udim = udim                        # Dimension of Control Vector
        self.ddim = ddim                        # Dimension of Disturbance Vector
        self.zdim = self.xdim + self.udim       # Dimension of Augmented State-Control Vector
        self.L = L                              # Running Cost
        self.phi = phi                          # Terminal Cost
        self.f = f                              # System Dynamics
        self.g = g                              # Point Inequalities g(x,u) <= 0
        self.h = h                              # Point Equalities h(x,u) = 0
        self.umax = umax                        # Upper Bounds on Control Effort
        self.umin = umin                        # Lower Bounds on Control Effort
        self.xmax = xmax                        # Upper Bounds on State
        self.xmin = xmin                        # Lower Bounds on State
        self.t = t
        self.dt = self.t[1::] - self.t[:len(t) - 1:]  # dt[k] = t[k+1] - t[k]
        self.N = len(t)


    def pack(self,x,u):
        z = []
        for i, j in zip(x, u):
            z.append(i)
            z.append(j)
        z = list(itertools.chain(*z))
        return z


    def unpack(self,z):
        x = []
        for i in range(self.xdim):
            x.append(z[i::self.zdim])
        x = list(zip(*x))
        for i in range(len(x)):
            x[i] = list(x[i])
        u = []
        for i in range(self.udim):
            u.append(z[i + self.xdim::self.zdim])
        u = list(zip(*u))
        for i in range(len(u)):
            u[i] = list(u[i])
        return x,u


    def cost(self,z):
        x, u = self.unpack(z)
        L = 0
        if self.L is not None:
            for t in range(self.N-1):
                L = L + self.dt[t]*self.L(x=x[t],u=u[t])
        phi = 0
        if self.phi is not None:
            phi = self.phi(x=x[self.N-1],u=u[self.N-1])
        J = L + phi
        return J


    def equalities(self,z):
        x,u = self.unpack(z)
        constraints = []

        for i in range(self.xdim):
            # Initial Condition
            if self.x0[i] is not None:
                constraint = x[0][i] - self.x0[i]
                constraints.append(constraint)

            # Final Boundary Condition
            if self.xN[i] is not None:
                constraint = x[len(x)-1][i] - self.xN[i]
                constraints.append(constraint)

        # System Dynamics
        f = []
        for t in range(self.N):
            f.append(self.f(x=x[t],u=u[t],d=self.d_guess[t]))
        for t in range(self.N - 1):
            for i in range(self.xdim):
                constraint = x[t+1][i] - x[t][i] - 0.5*self.dt[t]*(f[t+1][i]+f[t][i])
                constraints.append(constraint)

        # Point equality constraints
        for el in self.h:
            for t in range(self.N):
                constraint = - el(x=x[t],u=u[t])
                constraints.append(constraint)

        # Return List of all Equality-Constraints
        return constraints

    def inequalities(self,z):
        x, u = self.unpack(z)
        constraints = []

        # Point inequality constraints
        for el in self.g:
            for t in range(self.N):
                constraint = - el(x=x[t],u=u[t])
                constraints.append(constraint)

        # Return List of all Inequality-Constraints
        return constraints


    def path(self,x0,xN,u_guess,x_guess,d_guess,maxIter,minCost):
        self.x0 = list(x0)
        self.xN = list(xN)

        # Initial Guess
        if x_guess is None:
            self.x_guess = np.zeros(shape=(self.N,self.xdim))
        else:
            self.x_guess = x_guess
        if u_guess is None:
            self.u_guess = np.zeros(shape=(self.N,self.udim))
        else:
            self.u_guess = u_guess
        if d_guess is None:
            self.d_guess = np.zeros(shape=(self.N,self.ddim))
        else:
            self.d_guess = d_guess
        z = self.pack(x=list(self.x_guess), u=list(self.u_guess))

        # Add equality and inequality Constraints
        constraints = []
        constraints.append({'type': 'eq', 'fun': self.equalities})
        constraints.append({'type': 'ineq', 'fun': self.inequalities})

        # Add Upper and Lower Bounds on Control and State
        self.umin = [el if (el is not None) else -np.inf for el in self.umin]
        self.umax = [el if (el is not None) else np.inf for el in self.umax]
        self.xmin = [el if (el is not None) else -np.inf for el in self.xmin]
        self.xmax = [el if (el is not None) else np.inf for el in self.xmax]
        lowerBounds = [self.xmin + self.umin][0] * self.N
        upperBounds = [self.xmax + self.umax][0] * self.N
        bounds = Bounds(lowerBounds,upperBounds)

        # Optimizer
        # Scipy's Sequential Least Squares Program
        minimizer = minimize(fun=self.cost,                 # Cost Function
                             x0=z,                          # Initial Trajectory Guess
                             method='SLSQP',                # Optimization Method
                             constraints=constraints,       # Inequality & Equality Constraints
                             bounds=bounds,                 # Upper and Lower Bounds
                             options={'maxiter':maxIter},   # Maximum Iteration before Termination
                             tol=minCost)                   # Minimum Cost before Termination

        # Output from Optimizer - Path that minimize cost
        z = minimizer.x
        x, u = self.unpack(z)
        x = np.array(x,dtype=np.float32)
        u = np.array(u,dtype=np.float32)
        return x,u

    def interpolate(self,x,u,d,t):

        uc = np.empty(shape=(0, self.udim))
        xc = np.empty(shape=(0, self.xdim))

        if d is None:
            d = np.zeros(shape=(self.N,self.ddim))

        for i in range(len(t)):
            k = self.floor(timearray=self.t,t=t[i])
            tau = t[i] - self.t[k]
            if k < self.N-1:
                U = u[k,:] + (tau/self.dt[k])*(u[k+1,:]-u[k,:])
                X = x[k,:] + tau*self.f(x=x[k],u=u[k],d=d[k]) + tau**2/(2*self.dt[k])*(self.f(x=x[k+1],u=u[k+1],d=d[k+1]) - self.f(x=x[k],u=u[k],d=d[k]))
            else:
                U = u[self.N-1]
                X = x[self.N-1]

            uc = np.vstack((uc, U))
            xc = np.vstack((xc, X))

        return xc, uc

    @staticmethod
    def floor(timearray, t):
        k = np.argwhere(timearray < t)
        if len(k) < 1:
            k = 0
        else:
            k = np.argmax(k, axis=0)[0]
        return int(k)





