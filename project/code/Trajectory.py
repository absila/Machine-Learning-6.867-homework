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

        if t <= t0:
           return 0;
        elif t >= tf:
            return len(self.xpoints) - 1;

        # otherwise, we are somewhere in the bounds of the trajectory
        num_dts = round(t / self.dt);
        remainder = t % self.dt;
        #print 't = ' + str(t) + " remainder = " + str(remainder) + ' 0.5*self.dt = ' + str(0.5*self.dt) + " num_dts = " + str(num_dts)

        if remainder > 0.5*self.dt:
            num_dts = num_dts + 1

        starting_dts = int(t0 / self.dt);
        index = int(num_dts + starting_dts);

        if index > len(self.xpoints) - 1:
            index = len(self.xpoints) - 1

        return index

    def GetTimeAtIndex(self, idx):
        row_vec = self.xpoints[idx];
        return row_vec[0]

    def GetTrajectoryNumber(self):
        return self.trajectory_number

    def GetNumberOfPoints(self):
        return len(self.xpoints)

    def GetXyzYawTransformedPoint(self, t, trans):
        # apply the transformation from the global frame: origin = (0,0,0)
        # to the local frame point

        # transform is in the form:
        #   trans["trans_vec"]
        #   trans["quat"]
        original_point = self.GetState(t);

        # (from libbot)
        #bot_trans_apply_vec(&trans_xyz_yaw, original_point, xyz);

        rot = trans["quat"]
        trans_vec = trans["trans_vec"]
        v = []
        v.append(original_point[0])
        v.append(original_point[1])
        v.append(original_point[2])

        #bot_quat_rotate_to(btrans->rot_quat, src, dst);
        ab  =  rot[0]*rot[1]
        ac = rot[0]*rot[2]
        ad  =  rot[0]*rot[3]

        nbb = -rot[1]*rot[1]
        bc = rot[1]*rot[2]
        bd  =  rot[1]*rot[3]
        ncc = -rot[2]*rot[2]
        cd = rot[2]*rot[3]
        ndd = -rot[3]*rot[3]

        r = [0, 0, 0]
        r[0] = 2*((ncc + ndd)*v[0] + (bc - ad)*v[1] + (ac + bd)*v[2]) + v[0]
        r[1] = 2*((ad + bc)*v[0] + (nbb + ndd)*v[1] + (cd - ab)*v[2]) + v[1]
        r[2] = 2*((bd - ac)*v[0] + (ab + cd)*v[1] + (nbb + ncc)*v[2]) + v[2]

        dst = [0, 0, 0]
        dst[0] = r[0] + trans_vec[0];
        dst[1] = r[1] + trans_vec[1];
        dst[2] = r[2] + trans_vec[2];

        return dst

    #def ClosestObstacleInRemainderOfTrajectory(self, map2d, body_to_local, current_t):
        ## for each point remaining in the trajectory
        #number_of_points = self.GetNumberOfPoints();

        #starting_index = self.GetIndexAtTime(current_t);
        #point_distances = list()
        #closest_obstacle_distance = -1;

        #for i in range(starting_index, number_of_points):
            ## for each point in the trajectory

            ## subtract the current position (ie move the trajectory to where we are)
            #transformed_point[3];

            #this_t = GetTimeAtIndex(i);

            #self.GetXyzYawTransformedPoint(this_t, body_to_local, transformed_point);

            #// check if there is an obstacle nearby
            #point_distances.at(i) = octomap.NearestNeighbor(transformed_point);

        #}

        #for (int i = starting_index; i < number_of_points; i++) {
            #double distance_to_point = point_distances.at(i);
            #if (distance_to_point >= 0) {
                #if (distance_to_point < closest_obstacle_distance || closest_obstacle_distance < 0) {
                    #closest_obstacle_distance = distance_to_point;
                #}
            #}
        #}

         #// check minumum altitude
        #double min_altitude_traj = GetMinimumAltitude() + body_to_local.trans_vec[2];
        #if (min_altitude_traj < min_altitude_allowed) {
            #// this trajectory would impact the ground
            #closest_obstacle_distance = 0;
        #}// else if (min_altitude_traj < closest_obstacle_distance || closest_obstacle_distance < 0) {
         #//   closest_obstacle_distance = min_altitude_traj;
        #//}

        #return closest_obstacle_distance;
