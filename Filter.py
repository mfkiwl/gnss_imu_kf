import numpy as np


class GHFilter(object):
    def __init__(self, x, dx, dt, g, h):
        self.x = x
        self.dx = dx
        self.dt = dt
        self.g = g
        self.h = h
        self.dx_prediction = self.dx
        self.x_prediction  = self.x

        if np.ndim(x) == 0:
            self.y = 0.   # residual
            self.z = 0.
        else:
            self.y = np.zeros(len(x))
            self.z = np.zeros(len(x))

    def update(self, z, dx_pred=None, x_pred=None, g=None, h=None):
        if g is None:
            g = self.g
        if h is None:
            h = self.h
        if self.g >= 0.95:
            self.g , self.h = 0.9, 0.9

        #prediction step
        if dx_pred is None and x_pred is None:
            self.dx_prediction = self.dx
            self.x_prediction[0] = self.x[0] + (self.dx[0]*self.dt)
            self.x_prediction[1] = self.x[1] + (self.dx[1]*self.dt)
        else:
            try:
                self.dx_prediction = dx_pred
                self.x_prediction = x_pred
            except:
                print('Incorrect value')

        # update step
        self.y[0] = z[0] - self.x_prediction[0]
        self.y[1] = z[1] - self.x_prediction[1]
        self.dx[0] = self.dx_prediction[0] + h * self.y[0] * self.dt
        self.dx[1] = self.dx_prediction[1] + h * self.y[1] * self.dt
        self.x[0]  = self.x_prediction[0]  + g * self.y[0]
        self.x[1]  = self.x_prediction[1]  + g * self.y[1]

        return self.x, self.dx