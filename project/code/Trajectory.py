import csv
import os

class Trajectory(object):

    def __init__(self, filename_prefix, quiet):

        # open the file
        if quiet == False:
            print 'Loading trajectory: \n\t' + filename_prefix

        traj_number_str = filename_prefix[-5:]

        self.trajectory_number = int(traj_number_str);

        self.xpoints = self.LoadMatrixFromCSV(filename_prefix + '-x.csv', quiet);
        self.upoints = self.LoadMatrixFromCSV(filename_prefix + '-u.csv', quiet);
        self.kpoints = self.LoadMatrixFromCSV(filename_prefix + '-controller.csv', quiet);
        self.affine_points = self.LoadMatrixFromCSV(filename_prefix + '-affine.csv', quiet);

    def LoadMatrixFromCSV(self, filename, quiet):

        if quiet == False:
            print 'Loading ' + filename

        matrix = []

        counter = 0

        with open(filename, 'rb') as csvfile:
            reader = csv.reader(csvfile, delimiter=',')
            for row in reader:
                if counter == 0:
                    pass
                else:
                    numrow = []
                    for cell in row:
                        num = float(cell)
                        numrow.append(num)
                    #print row
                    #print '--------------'
                    matrix.append(numrow)


                if counter == 2:
                    self.dt = matrix[1][0] - matrix[0][0];

                counter = counter + 1

        return matrix

    def GetState(self, t):

        index = self.GetIndexAtTime(t);
        row_vec = self.xpoints[index];

        return row_vec[1:] # remove time

    def GetIndexAtTime(self, t):
        # round t to the nearest dt_

        t0 = self.xpoints[0][0];
        tf = self.xpoints[-1][0];

        print 'tf = ' + str(tf)

        if t <= t0:
           return 0;
        elif t >= tf:
            print 'yep'
            return len(self.xpoints) - 1;

        # otherwise, we are somewhere in the bounds of the trajectory
        num_dts = round(t / self.dt);
        remainder = t % self.dt;
        print 't = ' + str(t) + " remainder = " + str(remainder) + ' 0.5*self.dt = ' + str(0.5*self.dt) + " num_dts = " + str(num_dts)

        if remainder > 0.5*self.dt:
            print 'yep2'
            num_dts = num_dts + 1

        starting_dts = int(t0 / self.dt);
        index = int(num_dts + starting_dts);

        if index > len(self.xpoints) - 1:
            index = len(self.xpoints) - 1

        print 'index of ' + str(t) + ' = ' + str(index)
        return index

    def GetTrajectoryNumber(self):
        return self.trajectory_number
