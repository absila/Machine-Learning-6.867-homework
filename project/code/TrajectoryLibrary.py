import csv
import os
import trajectory

class TrajectoryLibrary(object):

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




    def LoadLibrary(self, libDir, quiet=False):
        # load a trajectory library from a directory

        # enumerate files in the directory
        if (libDir[-1] != '/'):
            libDir = libDir + "/"


        temp_trajs = list()
        count = 0

        # open the directory and find all the files that end in .csv
        for filename in os.listdir(libDir):
            if os.path.isfile(filename):
                print (filename)

                if filename.length() > 4 and filename[-6:] == "-x.csv":
                    # found a .csv file
                    # load a trajectory

                    # todo
                    this_traj = Trajectory(dirname + filename[0:-6], quiet);

                    temp_trajs.append(this_traj)
                    count = count + 1;

        #// now we have loaded everything into memory, so sort
        #for (int i = 0; i < (int)temp_traj.size(); i++) {

            #bool flag = false;
            #for (auto traj : temp_traj) {
                #if (traj.GetTrajectoryNumber() == i) {
                    #traj_vec_.push_back(traj);
                    #flag = true;
                    #break;
                #}
            #}
            #if (flag == false) {
                #std::cerr << "ERROR: missing trajectory #" << i << std::endl;

                #return false;
            #}
        #}

        #//(void)closedir(dirp);

        #if (!quiet) {
            #std::cout << "Loaded " << traj_vec_.size() << " trajectorie(s)" << std::endl;
        #}

        #if (traj_vec_.size() > 0) {
            #return true;
        #}
        #return false;
