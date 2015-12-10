import csv
import os

class Trajectory(object):

    def __init__(self, filename_prefix, quiet):

        # open the file
        if quiet == False:
            print 'Loading trajectory: \n\t' + filename_prefix

        traj_number_str = filename_prefix[-5:]

        self.trajectory_number = int(traj_number_str);

        self.xpoints = LoadMatrixFromCSV(filename_prefix + '-x.csv', quiet);
        self.upoints = LoadMatrixFromCSV(filename_prefix + '-u.csv', quiet);
        self.kpoints = LoadMatrixFromCSV(filename_prefix + '-controller.csv', quiet);
        self.affine_points = LoadMatrixFromCSV(filename_prefix + '-affine.csv', quiet);




    def LoadMatrixFromCSV(self, filename, quiet):

        if quiet == False:
            print 'Loading ' + filename

        with open(filename, 'rb') as csvfile:
            reader = csv.readre(csvfile, delimiter=',')
            for row in reader:
                print ', '.join(row)

