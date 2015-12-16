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

    #def GetXyzYawTransformedPoint(self, t, transform):
        ## apply the transformation from the global frame: orgin = (0,0,0)
        ## to the local frame point

        #state = self.GetState(t);

        #double original_point[3];
        #original_point[0] = state[0];
        #original_point[1] = state[1];
        #original_point[2] = state[2];

        ##// remove roll and pitch from the transform
        #BotTrans trans_xyz_yaw;

        #bot_trans_copy(&trans_xyz_yaw, &transform);

        #double rpy[3];
        #bot_quat_to_roll_pitch_yaw(trans_xyz_yaw.rot_quat, rpy);

        #rpy[0] = 0;
        #rpy[1] = 0;

        #bot_roll_pitch_yaw_to_quat(rpy, trans_xyz_yaw.rot_quat);


        #bot_trans_apply_vec(&trans_xyz_yaw, original_point, xyz);

    #}

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
