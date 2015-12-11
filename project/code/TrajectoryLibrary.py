import csv
import os
from Trajectory import Trajectory

class TrajectoryLibrary(object):

    def __init__(self, libDir, quiet):

        # open the file
        self.LoadLibrary(libDir, quiet)


    def LoadLibrary(self, libDir, quiet=False):
        # load a trajectory library from a directory

        if quiet == False:
            print 'Loading library from ' + libDir

        # enumerate files in the directory
        if (libDir[-1] != '/'):
            libDir = libDir + "/"


        temp_trajs = list()
        count = 0

        # open the directory and find all the files that end in .csv
        for filename in os.listdir(libDir):
            if os.path.isfile(libDir + '/' + filename):

                if len(filename) > 4 and filename[-6:] == "-x.csv":
                    # found a .csv file
                    # load a trajectory

                    this_traj = Trajectory(libDir + '/' + filename[0:-6], quiet);

                    temp_trajs.append(this_traj)
                    count = count + 1;
            else:
                print filename + ' is not a file.'
        # now we have loaded everything into memory, so sort

        self.traj_vec = []
        for i in range(0, len(temp_trajs)):

            flag = False

            for traj in temp_trajs:
                if traj.GetTrajectoryNumber() == i:
                    self.traj_vec.append(traj)
                    flag = True
                    break

            if flag == False:
                print 'ERROR: missing trajectory #' + i
                return False

        if not quiet:
            print 'Loaded ' + str(len(self.traj_vec)) + ' trajectorie(s)'

        if len(self.traj_vec) > 0:
            return True

        return False

    def GetTrajectoryByNumber(self, num):
        return self.traj_vec[num]
