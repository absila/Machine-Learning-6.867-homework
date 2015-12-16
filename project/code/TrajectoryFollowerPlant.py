import numpy as np
import scipy.integrate as integrate
import csv
import os
from TrajectoryLibrary import TrajectoryLibrary
import math

class TrajectoryFollowerPlant(object):

    def __init__(self, trajectoryLibDir):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.current_trajnum = 0
        self.current_traj_t = -1
        self.traj_init_state = np.array([0, 0, 0])

        rad = np.pi/180.0

        self.state = np.array([self.x, self.y, self.theta*rad])

        self.lib = TrajectoryLibrary(trajectoryLibDir, False)

    def GetTrajlib(self):
        return self.lib

    def GetCurrentTraj(self):
        return self.lib.GetTrajectoryByNumber(self.current_trajnum)

    def SetTrajectory(self, number):
        self.current_trajnum = number
        self.current_traj_t = -1
        print 'SET TRAJECTORY = ' + str(number)

    def GetCurrentTrajTime(self):
        return self.current_traj_t

    def RollPitchYawToQuat(self, rpy):
        roll = rpy[0]
        pitch = rpy[1]
        yaw = rpy[2]

        halfroll = roll / 2
        halfpitch = pitch / 2
        halfyaw = yaw / 2

        sin_r2 = math.sin (halfroll)
        sin_p2 = math.sin (halfpitch)
        sin_y2 = math.sin (halfyaw)

        cos_r2 = math.cos (halfroll)
        cos_p2 = math.cos (halfpitch)
        cos_y2 = math.cos (halfyaw)

        q = [1, 0, 0, 0]

        q[0] = cos_r2 * cos_p2 * cos_y2 + sin_r2 * sin_p2 * sin_y2
        q[1] = sin_r2 * cos_p2 * cos_y2 - cos_r2 * sin_p2 * sin_y2
        q[2] = cos_r2 * sin_p2 * cos_y2 + sin_r2 * cos_p2 * sin_y2
        q[3] = cos_r2 * cos_p2 * sin_y2 - sin_r2 * sin_p2 * cos_y2

        return q

    def dynamics(self, state, t):

        if self.current_traj_t < 0:
            # init new traj
            self.traj_init_state = state
            self.current_traj_t = 0
            print 'init state = ' + str(state)

        # update time
        delta_t = t[1] - t[0]
        #print 'delta t = ' + str(delta_t)

        # get trajectory value
        traj = self.lib.GetTrajectoryByNumber(self.current_trajnum)

        self.current_traj_t = self.current_traj_t + delta_t
        traj_state = traj.GetState(self.current_traj_t)

        trans = dict()
        trans["trans_vec"] = [state[0], state[1], 0]
        trans["quat"] = self.RollPitchYawToQuat([0, 0, self.traj_init_state[2]])

        transformed_state = traj.GetXyzYawTransformedPoint(self.current_traj_t, trans)

        print 'traj_state[5] = ' + str(traj_state[5])

        x = transformed_state[0]
        y = transformed_state[1]
        theta = self.traj_init_state[2] + traj_state[5]

        print 'theta = ' + str(theta)

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

    def simulateOneStep(self, startTime=0.0, dt=0.05):
        #print 'start time = ' + str(startTime)
        #print 'dt = ' + str(dt)
        t = np.linspace(startTime, startTime+dt, 2)
        #print 't = ' + str(t)
        newState = self.dynamics(self.state, t)
        #print newState
        self.state = newState
        return self.state
