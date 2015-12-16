import numpy as np
import scipy.integrate as integrate
import ddapp.objectmodel as om
from TrajectoryFollowerPlant import TrajectoryFollowerPlant

class OnlineTrajectoryPlanner(object):

    def __init__(self, sensor, plant):
        self.Sensor = sensor
        self.currentTraj = 0
        self.plant = plant

        # I need a 2D map of the previously seen points
        # assume perfect state estimate
        # so just record all previously seen points in a huge list
        self.map2d = list()

    def AddToMap(self, raycast):
        self.map2d.append(raycastResult)

        # todo: remove old entries


    def computeControlInput(self, state, t, frame, raycastResult):
        # add the raycast result to the map
        self.AddToMap(raycastResult)

        # now we have a fully up-to-date map
        # make a control decision

        # controller:
        #   1. are we in collision?
        #if self.GetDistance(self.map2d, self.plant.GetCurrentTraj(), self.plant.GetCurrentTrajTime()) < todo:
            #pass


        #u = self.countStuffController()
        u, actionIdx = self.countInverseDistancesController()

        if randomize:
            if np.random.uniform(0,1,1)[0] < self.epsilonRand:
                # otherActionIdx = np.setdiff1d(self.actionSetIdx, np.array([actionIdx]))
                # randActionIdx = np.random.choice(otherActionIdx)
                actionIdx = np.random.choice(self.actionSetIdx)
                u = self.actionSet[actionIdx]

        return u, actionIdx


    def countStuffController(self):
        firstHalf = self.distances[0:self.numRays/2]
        secondHalf = self.distances[self.numRays/2:]
        tol = 1e-3;

        numLeft = np.size(np.where(firstHalf < self.Sensor.rayLength - tol))
        numRight = np.size(np.where(secondHalf < self.Sensor.rayLength - tol))

        if numLeft == numRight:
            actionIdx = 1
        elif numLeft > numRight:
            actionIdx = 2
        else:
            actionIdx = 0

        u = self.actionSet[actionIdx]
        return u, actionIdx

    def countInverseDistancesController(self):
        midpoint = np.floor(self.numRays/2.0)
        leftHalf = np.array((self.distances[0:midpoint]))
        rightHalf = np.array((self.distances[midpoint:]))
        tol = 1e-3;

        inverseLeftHalf = (1.0/leftHalf)**2
        inverseRightHalf = (1.0/rightHalf)**2

        numLeft = np.sum(inverseLeftHalf)
        numRight = np.sum(inverseRightHalf)


        if numLeft == numRight:
            actionIdx = 1
        elif numLeft > numRight:
            actionIdx = 2
        else:
            actionIdx = 0


        # print "leftHalf ", leftHalf
        # print "rightHalf", rightHalf
        # print "inverseLeftHalf", inverseLeftHalf
        # print "inverserRightHalf", inverseRightHalf
        # print "numLeft", numLeft
        # print "numRight", numRight

        u = self.actionSet[actionIdx]
        return u, actionIdx


    def computeControlInputFromFrame(self):
        carState = 0
        t = 0
        frame = om.findObjectByName('robot frame')
        return self.computeControlInput(carState, t, frame)

