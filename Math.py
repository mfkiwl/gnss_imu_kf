import numpy as np


class GHKFilter(object):
    """
    Implements the g-h-k filter.
    """
    def __init__(self, x, dx, ddx, dt, g, h, k):
        self.x = x
        self.dx = dx
        self.ddx = ddx
        self.x_prediction = self.x
        self.dx_prediction = self.dx
        self.ddx_prediction = self.ddx

        self.dt = dt
        self.g = g
        self.h = h
        self.k = k

        if np.ndim(x) == 0:
            self.y = 0.  # residual
            self.z = 0.
        else:
            self.y = np.zeros(len(x))
            self.z = np.zeros(len(x))


    def update(self, z, g=None, h=None, k=None):
        """
        Performs the g-h filter predict and update step on the
        measurement z.
        """
        if g is None:
            g = self.g
        if h is None:
            h = self.h
        if k is None:
            k = self.k

        dt = self.dt
        dt_sqr = dt**2
        #prediction step
        self.ddx_prediction = self.ddx
        self.dx_prediction  = self.dx + self.ddx*dt
        self.x_prediction   = self.x  + self.dx*dt + .5*self.ddx*(dt_sqr)

        # update step
        self.y = z - self.x_prediction

        self.ddx = self.ddx_prediction + 2*k*self.y / dt_sqr
        self.dx  = self.dx_prediction  + h * self.y / dt
        self.x   = self.x_prediction   + g * self.y

        return (self.x, self.dx)

