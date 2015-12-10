import numpy as np
import scipy.integrate as integrate
import csv
import os
import trajectorylibrary

class TrajectoryFollowerPlant(object):

    def __init__(self, trajectoryLibDir):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        rad = np.pi/180.0

        self.state = np.array([self.x, self.y, self.theta*rad])

        self.LoadLibrary(trajectoryLibDir)


    def dynamics(self, state, t, controlInput):

        dqdt = np.zeros_like(state)

        u = controlInput

        dqdt[0] = 12
        dqdt[1] = 0
        dqdt[2] = 0

        return dqdt

    def setFrame(self, frame):
        self.frame = frame

    def setStateFromFrame(self, frame):
         # get roll, pitch, yaw from the frame, set the state to that . . .
         pass

    def setPlantState(self, x, y, theta):
        self.state = np.array([x, y, theta])

    def simulate(self, dt=0.05):
        t = np.arange(0.0, 10, dt)
        newState = integrate.odeint(self.dynamics, self.state, t)
        print "Finished simulation:", newState
        print "Shape is", np.shape(newState)
        return newState

    def simulateOneStep(self, startTime=0.0, dt=0.05, controlInput=None):
        t = np.linspace(startTime, startTime+dt, 2)
        newState = integrate.odeint(self.dynamics, self.state, t, args=(controlInput,))
        self.state = newState[-1,:]
        return self.state
