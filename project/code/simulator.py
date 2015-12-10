import ddapp.vtkAll as vtk
import ddapp.visualization as vis
import ddapp.objectmodel as om
from ddapp.debugVis import DebugData
from ddapp.consoleapp import ConsoleApp
from ddapp.timercallback import TimerCallback
from ddapp import applogic
from ddapp import transformUtils
import numpy as np
import time
import scipy.integrate as integrate
import argparse
import matplotlib.pyplot as plt
import shelve

from PythonQt import QtCore, QtGui

from world import World
from car import CarPlant
from trajectoryfollowerplant import TrajectoryFollowerPlant
from sensor import SensorObj
from controller import ControllerObj


class Simulator(object):


    def __init__(self, percentObsDensity=20, endTime=40, randomizeControl=False, nonRandomWorld=False,
                 circleRadius=0.7, worldScale=1.0, supervisedTrainingTime=500, autoInitialize=True, verbose=True):
        self.verbose = verbose
        self.randomizeControl = randomizeControl
        self.startSimTime = time.time()
        self.collisionThreshold = 1.3
        self.randomSeed = 5
        self.Sensor_rayLength = 8

        self.percentObsDensity = percentObsDensity
        self.supervisedTrainingTime = 10
        self.learningRandomTime = 10
        self.learningEvalTime = 10
        self.defaultControllerTime = 10
        self.nonRandomWorld = nonRandomWorld
        self.circleRadius = circleRadius
        self.worldScale = worldScale
        # create the visualizer object
        self.app = ConsoleApp()
        # view = app.createView(useGrid=False)
        self.view = self.app.createView(useGrid=False)

        self.options = dict()
        self.setDefaultOptions()
        self.initializeColorMap()
        if autoInitialize:
            self.initialize()

    def setDefaultOptions(self):

        defaultOptions = dict()

        defaultOptions['World'] = dict()
        defaultOptions['World']['obstaclesInnerFraction'] = 0.85
        defaultOptions['World']['randomSeed'] = 40
        defaultOptions['World']['percentObsDensity'] = 7.5
        defaultOptions['World']['nonRandomWorld'] = True
        defaultOptions['World']['circleRadius'] = 1.75
        defaultOptions['World']['scale'] = 1.0


        defaultOptions['Sensor'] = dict()
        defaultOptions['Sensor']['rayLength'] = 10
        defaultOptions['Sensor']['rayMinToHit'] = 9.5
        defaultOptions['Sensor']['numRays'] = 20


        defaultOptions['Car'] = dict()
        defaultOptions['Car']['velocity'] = 12

        defaultOptions['TrajectoryLibrary'] = dict()
        defaultOptions['TrajectoryLibrary']['dir'] = ''

        defaultOptions['dt'] = 0.05


        defaultOptions['runTime'] = dict()
        defaultOptions['runTime']['simTime'] = 5


        for k in defaultOptions:
            self.options.setdefault(k, defaultOptions[k])


        for k in defaultOptions:
            if not isinstance(defaultOptions[k], dict):
                continue

            for j in defaultOptions[k]:
                self.options[k].setdefault(j, defaultOptions[k][j])


    def initializeColorMap(self):
        self.colorMap = dict()
        self.colorMap['defaultRandom'] = [0,0,1]
        self.colorMap['learnedRandom'] = [1.0,  0.54901961,  0.] # this is orange
        self.colorMap['learned'] = [ 0.58039216,  0.0,  0.82745098] # this is yellow
        self.colorMap['default'] = [0,1,0]

    def initialize(self):

        self.dt = self.options['dt']
        self.setDefaultOptions()

        self.Sensor = SensorObj(rayLength=self.options['Sensor']['rayLength'],
                                numRays=self.options['Sensor']['numRays'],
                                rayMinToHit=self.options['Sensor']['rayMinToHit'])
        self.Controller = ControllerObj(self.Sensor)
        self.Plant = TrajectoryFollowerPlant(self.options['TrajectoryLibrary']['dir'])

        # create the things needed for simulation
        om.removeFromObjectModel(om.findObjectByName('world'))
        self.world = World.buildCircleWorld(percentObsDensity=self.options['World']['percentObsDensity'],
                                            circleRadius=self.options['World']['circleRadius'],
                                            nonRandom=self.options['World']['nonRandomWorld'],
                                            scale=self.options['World']['scale'],
                                            randomSeed=self.options['World']['randomSeed'],
                                            obstaclesInnerFraction=self.options['World']['obstaclesInnerFraction'])

        om.removeFromObjectModel(om.findObjectByName('robot'))
        self.robot, self.frame = World.buildRobot()
        self.locator = World.buildCellLocator(self.world.visObj.polyData)
        self.Sensor.setLocator(self.locator)
        self.frame = self.robot.getChildFrame()
        self.frame.setProperty('Scale', 3)
        self.frame.setProperty('Edit', True)
        self.frame.widget.HandleRotationEnabledOff()
        rep = self.frame.widget.GetRepresentation()
        rep.SetTranslateAxisEnabled(2, False)
        rep.SetRotateAxisEnabled(0, False)
        rep.SetRotateAxisEnabled(1, False)

        self.Plant.setFrame(self.frame)
        print "Finished initialization"


    def runSingleSimulation(self, updateQValues=True, controllerType='default', simulationCutoff=None):

        if self.verbose: print "using QValue based controller = ", useQValueController

        self.setCollisionFreeInitialState()

        currentPlantState = np.copy(self.Plant.state)
        nextPlantState = np.copy(self.Plant.state)
        self.setRobotFrameState(currentPlantState[0], currentPlantState[1], currentPlantState[2])
        currentRaycast = self.Sensor.raycastAll(self.frame)
        nextRaycast = np.zeros(self.Sensor.numRays)

        # record the reward data
        runData = dict()
        reward = 0
        discountedReward = 0
        avgReward = 0
        startIdx = self.counter


        while (self.counter < self.numTimesteps - 1):
            idx = self.counter
            currentTime = self.t[idx]
            self.stateOverTime[idx,:] = currentPlantState
            x = self.stateOverTime[idx,0]
            y = self.stateOverTime[idx,1]
            theta = self.stateOverTime[idx,2]
            self.setRobotFrameState(x,y,theta)
            # self.setRobotState(currentPlantState[0], currentPlantState[1], currentPlantState[2])
            currentRaycast = self.Sensor.raycastAll(self.frame)
            self.raycastData[idx,:] = currentRaycast
            S_current = (currentPlantState, currentRaycast)


            controlInput, controlInputIdx = self.Controller.computeControlInput(currentPlantState,
                                                                        currentTime, self.frame,
                                                                        raycastDistance=currentRaycast,
                                                                        randomize=False)

            self.controlInputData[idx] = controlInput

            nextPlantState = self.Plant.simulateOneStep(controlInput=controlInput, dt=self.dt)

            x = nextPlantState[0]
            print x
            y = nextPlantState[1]
            theta = nextPlantState[2]

            self.setRobotFrameState(x,y,theta)

            nextRaycast = self.Sensor.raycastAll(self.frame)

            self.counter+=1
            # break if we are in collision
            if self.checkInCollision(currentRaycast):
                if self.verbose: print "Had a collision, terminating simulation"
                break

            if self.counter >= simulationCutoff:
                break

            currentPlantState = nextPlantState
            currentRaycast = nextRaycast

        # fill in the last state by hand
        self.stateOverTime[self.counter,:] = currentPlantState
        print currentPlantState
        self.raycastData[self.counter,:] = currentRaycast


        # this just makes sure we don't get stuck in an infinite loop.
        if startIdx == self.counter:
            self.counter += 1

        return runData

    def runBatchSimulation(self, endTime=None, dt=0.05):
        print 'in run batch'
        # for use in playback
        self.dt = self.options['dt']

        self.endTime = self.options['runTime']['simTime']

        self.t = np.arange(0.0, self.endTime, dt)
        maxNumTimesteps = np.size(self.t)
        self.stateOverTime = np.zeros((maxNumTimesteps+1, 3))
        self.raycastData = np.zeros((maxNumTimesteps+1, self.Sensor.numRays))
        self.controlInputData = np.zeros(maxNumTimesteps+1)
        self.rewardData = np.zeros(maxNumTimesteps+1)
        self.emptyQValue = np.zeros(maxNumTimesteps+1, dtype='bool')
        self.numTimesteps = maxNumTimesteps

        self.counter = 0
        self.simulationData = []

        self.initializeStatusBar()
        print 'in run batch2'
        self.idxDict = dict()
        numRunsCounter = 0


        self.idxDict['defaultRandom'] = self.counter
        loopStartIdx = self.counter
        while (self.counter < self.numTimesteps):
            self.printStatusBar()
            useQValueController = False
            startIdx = self.counter
            runData = self.runSingleSimulation(updateQValues=True, controllerType='defaultRandom',
                                               simulationCutoff=self.numTimesteps)

            runData['startIdx'] = startIdx
            runData['controllerType'] = "defaultRandom"
            runData['duration'] = self.counter - runData['startIdx']
            runData['endIdx'] = self.counter
            runData['runNumber'] = numRunsCounter
            numRunsCounter+=1
            self.simulationData.append(runData)

        print 'in run batch3'
        self.idxDict['learnedRandom'] = self.counter
        loopStartIdx = self.counter


        # BOOKKEEPING
        # truncate stateOverTime, raycastData, controlInputs to be the correct size
        self.numTimesteps = self.counter + 1
        self.stateOverTime = self.stateOverTime[0:self.counter+1, :]
        self.raycastData = self.raycastData[0:self.counter+1, :]
        self.controlInputData = self.controlInputData[0:self.counter+1]
        self.endTime = 1.0*self.counter/self.numTimesteps*self.endTime

    def initializeStatusBar(self):
        self.numTicks = 10
        self.nextTickComplete = 1.0 / float(self.numTicks)
        self.nextTickIdx = 1
        print "Simulation percentage complete: (", self.numTicks, " # is complete)"

    def printStatusBar(self):
        fractionDone = float(self.counter) / float(self.numTimesteps)
        if fractionDone > self.nextTickComplete:

            self.nextTickIdx += 1
            self.nextTickComplete += 1.0 / self.numTicks

            timeSoFar = time.time() - self.startSimTime
            estimatedTimeLeft_sec = (1 - fractionDone) * timeSoFar / fractionDone
            estimatedTimeLeft_minutes = estimatedTimeLeft_sec / 60.0

            print "#" * self.nextTickIdx, "-" * (self.numTicks - self.nextTickIdx), "estimated", estimatedTimeLeft_minutes, "minutes left"



    def setCollisionFreeInitialState(self):
        tol = 5

        while True:
            x = np.random.uniform(self.world.Xmin+tol, self.world.Xmax-tol, 1)[0]
            y = np.random.uniform(self.world.Ymin+tol, self.world.Ymax-tol, 1)[0]
            theta = np.random.uniform(0,2*np.pi,1)[0]
            self.Plant.setPlantState(x,y,theta)
            self.setRobotFrameState(x,y,theta)

            if not self.checkInCollision():
                break

        # if self.checkInCollision():
        #     print "IN COLLISION"
        # else:
        #     print "COLLISION FREE"

        return x,y,theta

    def setupPlayback(self):

        self.timer = TimerCallback(targetFps=30)
        self.timer.callback = self.tick

        playButtonFps = 1.0/self.dt
        print "playButtonFPS", playButtonFps
        self.playTimer = TimerCallback(targetFps=playButtonFps)
        self.playTimer.callback = self.playTimerCallback
        self.sliderMovedByPlayTimer = False

        panel = QtGui.QWidget()
        l = QtGui.QHBoxLayout(panel)

        playButton = QtGui.QPushButton('Play/Pause')
        playButton.connect('clicked()', self.onPlayButton)

        slider = QtGui.QSlider(QtCore.Qt.Horizontal)
        slider.connect('valueChanged(int)', self.onSliderChanged)
        self.sliderMax = self.numTimesteps
        slider.setMaximum(self.sliderMax)
        self.slider = slider

        l.addWidget(playButton)
        l.addWidget(slider)

        w = QtGui.QWidget()
        l = QtGui.QVBoxLayout(w)
        l.addWidget(self.view)
        l.addWidget(panel)
        w.showMaximized()



        self.frame.connectFrameModified(self.updateDrawIntersection)
        self.updateDrawIntersection(self.frame)

        applogic.resetCamera(viewDirection=[0.2,0,-1])
        self.view.showMaximized()
        self.view.raise_()

        elapsed = time.time() - self.startSimTime
        simRate = self.counter/elapsed
        print "Total run time", elapsed
        print "Ticks (Hz)", simRate
        print "Number of steps taken", self.counter
        self.app.start()

    def run(self, launchApp=True):
        self.counter = 1
        self.runBatchSimulation()

        if launchApp:
            self.setupPlayback()

    def updateDrawIntersection(self, frame):

        origin = np.array(frame.transform.GetPosition())
        #print "origin is now at", origin
        d = DebugData()

        sliderIdx = self.slider.value

        colorMaxRange = [1,1,0]

        for i in xrange(self.Sensor.numRays):
            ray = self.Sensor.rays[:,i]
            rayTransformed = np.array(frame.transform.TransformNormal(ray))
            #print "rayTransformed is", rayTransformed
            intersection,wasOccluded = self.Sensor.raycast(self.locator, origin, origin + rayTransformed*self.Sensor.rayLength)


            if intersection is not None:
                d.addLine(origin+rayTransformed*self.Sensor.rayMinToHit, intersection, color=[1,0,0])
            else:

                if wasOccluded is not None:
                    # occlusion!
                    d.addLine(origin, wasOccluded, color=[1,1,0])
                else:
                    d.addLine(origin+rayTransformed*self.Sensor.rayMinToHit, origin+rayTransformed*self.Sensor.rayLength, color=[0,1,0])

            #if intersection is not None:
                #d.addLine(origin, intersection, color=[1,0,0])
            #else:
                #d.addLine(origin, origin+rayTransformed*self.Sensor.rayLength, color=colorMaxRange)

        vis.updatePolyData(d.getPolyData(), 'rays', colorByName='RGB255')

        #camera = self.view.camera()
        #camera.SetFocalPoint(frame.transform.GetPosition())
        #camera.SetPosition(frame.transform.TransformPoint((-30,0,10)))

    def setRobotFrameState(self, x, y, theta):
        t = vtk.vtkTransform()
        t.Translate(x,y,0.0)
        t.RotateZ(np.degrees(theta))
        self.robot.getChildFrame().copyFrame(t)

    # returns true if we are in collision
    def checkInCollision(self, raycastDistance=None):
        if raycastDistance is None:
            self.setRobotFrameState(self.Plant.state[0],self.Plant.state[1],self.Plant.state[2])
            raycastDistance = self.Sensor.raycastAll(self.frame)

        if np.min(raycastDistance) < self.collisionThreshold:
            return True
        else:
            return False

    def tick(self):
        #print timer.elapsed
        #simulate(t.elapsed)
        x = np.sin(time.time())
        y = np.cos(time.time())
        self.setRobotFrameState(x,y,0.0)
        if (time.time() - self.playTime) > self.endTime:
            self.playTimer.stop()

    def tick2(self):
        newtime = time.time() - self.playTime
        print time.time() - self.playTime
        x = np.sin(newtime)
        y = np.cos(newtime)
        self.setRobotFrameState(x,y,0.0)

    # just increment the slider, stop the timer if we get to the end
    def playTimerCallback(self):
        self.sliderMovedByPlayTimer = True
        currentIdx = self.slider.value
        nextIdx = currentIdx + 1
        self.slider.setSliderPosition(nextIdx)
        if currentIdx >= self.sliderMax:
            print "reached end of tape, stopping playTime"
            self.playTimer.stop()

    def onSliderChanged(self, value):
        if not self.sliderMovedByPlayTimer:
            self.playTimer.stop()
        numSteps = len(self.stateOverTime)
        idx = int(np.floor(numSteps*(1.0*value/self.sliderMax)))
        idx = min(idx, numSteps-1)
        x,y,theta = self.stateOverTime[idx]
        self.setRobotFrameState(x,y,theta)
        self.sliderMovedByPlayTimer = False

    def onPlayButton(self):

        if self.playTimer.isActive():
            self.onPauseButton()
            return

        print 'play'
        self.playTimer.start()
        self.playTime = time.time()

    def onPauseButton(self):
        print 'pause'
        self.playTimer.stop()

    def saveToFile(self, filename):

        # should also save the run data if it is available, i.e. stateOverTime, rewardOverTime

        filename = 'data/' + filename + ".out"
        my_shelf = shelve.open(filename,'n')

        my_shelf['options'] = self.options


        my_shelf['simulationData'] = self.simulationData
        my_shelf['stateOverTime'] = self.stateOverTime
        my_shelf['raycastData'] = self.raycastData
        my_shelf['controlInputData'] = self.controlInputData
        my_shelf['emptyQValue'] = self.emptyQValue
        my_shelf['numTimesteps'] = self.numTimesteps
        my_shelf['idxDict'] = self.idxDict
        my_shelf['counter'] = self.counter
        my_shelf.close()


    @staticmethod
    def loadFromFile(filename):
        filename = 'data/' + filename + ".out"
        sim = Simulator(autoInitialize=False, verbose=False)

        my_shelf = shelve.open(filename)
        sim.options = my_shelf['options']
        sim.initialize()

        sim.simulationData = my_shelf['simulationData']
        # sim.runStatistics = my_shelf['runStatistics']
        sim.stateOverTime = np.array(my_shelf['stateOverTime'])
        sim.raycastData = np.array( my_shelf['raycastData'])
        sim.controlInputData = np.array(my_shelf['controlInputData'])
        sim.emptyQValue = np.array(my_shelf['emptyQValue'])
        sim.numTimesteps = my_shelf['numTimesteps']
        sim.idxDict = my_shelf['idxDict']
        sim.counter = my_shelf['counter']

        my_shelf.close()

        return sim




if __name__ == "__main__":
    # main(sys.argv[1:])
    parser = argparse.ArgumentParser(description='interpret simulation parameters')
    parser.add_argument('--percentObsDensity', type=float, nargs=1, default=[10])
    parser.add_argument('--endTime', type=int, nargs=1, default=[40])
    parser.add_argument('--randomizeControl', action='store_true', default=False)
    parser.add_argument('--nonRandomWorld', action='store_true', default=False)
    parser.add_argument('--circleRadius', type=float, nargs=1, default=0.7)
    parser.add_argument('--worldScale', type=float, nargs=1, default=1.0)
    parser.add_argument('--supervisedTrainingTime', type=float, nargs=1, default=500)
    argNamespace = parser.parse_args()
    percentObsDensity = argNamespace.percentObsDensity[0]
    endTime = argNamespace.endTime[0]
    randomizeControl = argNamespace.randomizeControl
    nonRandomWorld = argNamespace.nonRandomWorld
    circleRadius = argNamespace.circleRadius[0]
    worldScale = argNamespace.worldScale[0]
    supervisedTrainingTime = argNamespace.supervisedTrainingTime[0]
    sim = Simulator(percentObsDensity=percentObsDensity, endTime=endTime, randomizeControl=randomizeControl,
                    nonRandomWorld=nonRandomWorld, circleRadius=circleRadius, worldScale=worldScale,
                    supervisedTrainingTime=supervisedTrainingTime)
    sim.run()


