__author__ = 'abarry'
from simulator import Simulator

sim = Simulator(autoInitialize=False, verbose=False)

#
# # small world
# sim.randomSeed = 11
# sim.randomizeControl       = True
# sim.percentObsDensity      = 3.5
# sim.nonRandomWorld         = True
# sim.circleRadius           = 2.0
# sim.worldScale             = 0.5
#
#
# big world
# sim.randomSeed = 10
# sim.randomizeControl       = True
# sim.percentObsDensity      = 3
# sim.nonRandomWorld         = True
# sim.circleRadius           = 2.5
# sim.worldScale             = 1


# # big dense
# sim.randomSeed = 12
# sim.randomizeControl       = True
# sim.percentObsDensity      = 7
# sim.nonRandomWorld         = True
# sim.circleRadius           = 1.5
# sim.worldScale             = 1

sim.options['Sensor']['rayMinToHit'] = 9.5
sim.options['runTime']['simTime'] = 5

# sim.options['Reward']['collisionPenalty'] = 200


sim.randomSeed = 8
sim.randomizeControl       = True
sim.percentObsDensity      = 5
sim.nonRandomWorld         = True
sim.circleRadius           = 2.5
sim.worldScale             = 1
sim.options['World']['obstaclesInnerFraction'] = 0.8


## For testing
# sim.supervisedTrainingTime = 10
# sim.learningRandomTime = 10
# sim.learningEvalTime = 10
# sim.defaultControllerTime = 10


sim.initialize()
sim.run()
