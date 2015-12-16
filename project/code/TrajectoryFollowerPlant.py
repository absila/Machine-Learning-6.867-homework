import numpy as np
import scipy.integrate as integrate
import csv
import os
from TrajectoryLibrary import TrajectoryLibrary

class TrajectoryFollowerPlant(object):

    def __init__(self, trajectoryLibDir):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.current_trajnum = 0
        self.current_traj_t = -1

        rad = np.pi/180.0

        self.state = np.array([self.x, self.y, self.theta*rad])

        self.lib = TrajectoryLibrary(trajectoryLibDir, False)

    def GetTrajlib(self):
        return self.lib

    def GetCurrentTraj(self):
        return self.lib.GetTrajectoryByNumber(self.current_trajnum)

    def GetCurrentTrajTime(self):
        return self.current_traj_t

    def dynamics(self, state, t, controlInput):

        u = controlInput

        if u is None:
            # no change in trajectory
            delta_t = t[1] - t[0]
            #print 'delta t = ' + str(delta_t)

            self.current_traj_t = self.current_traj_t + delta_t

        else:
            # new trajectory
            self.current_trajnum = u

            self.current_traj_t = 0

        # get state
        traj = self.lib.GetTrajectoryByNumber(self.current_trajnum)
        #print 'current traj t = ' + str(self.current_traj_t)
        state = traj.GetState(self.current_traj_t)

        x = state[0]
        y = state[1]
        theta = state[5]

        return np.array([x, y, theta])

    def setFrame(self, frame):
        self.frame = frame

    def setStateFromFrame(self, frame):
         # get roll, pitch, yaw from the frame, set the state to that . . .
         pass

    def setPlantState(self, x, y, theta):
        self.state = np.array([x, y, theta])

    #def simulate(self, dt=0.05):
        #t = np.arange(0.0, 10, dt)
        #newState = integrate.odeint(self.dynamics, self.state, t)
        #print "Finished simulation:", newState
        #print "Shape is", np.shape(newState)
        #return newState

    def simulateOneStep(self, startTime=0.0, dt=0.05, controlInput=None):
        #print 'start time = ' + str(startTime)
        #print 'dt = ' + str(dt)
        t = np.linspace(startTime, startTime+dt, 2)
        #print 't = ' + str(t)
        newState = self.dynamics(self.state, t, None)
        #print newState
        self.state = newState
        return self.state
