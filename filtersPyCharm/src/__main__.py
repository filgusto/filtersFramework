# Main program to initialize the filters class
from CustomLibraries.BibVrepMethods.BibVrepManipulation import BibVrepManipulation
from CustomLibraries.ClassKalmanFilter import KalmanFilter

import numpy as np
import time





if __name__ == '__main__':

    # a message to the user
    print("Main code started")

    # invoking the vrep manipulator library
    vrep = BibVrepManipulation()

    # invoking the Kalman Filter library
    kf = KalmanFilter()

    # helpful variables
    flag_firstRead = True
    iteration_number = 1


    # The infinity loop
    while True:

        # small pause for better computations
        time.sleep(0.05)

        # retrieving robot's data from V-REP
        robot_pose = vrep.getrobotpose()
        robot_position = robot_pose[0]
        robot_orientation = robot_pose[1]
        robot_velocity = vrep.getrobotvelocities()
        simTime_actual = vrep.getsimtimelastretrival()

        # treating time
        if flag_firstRead == 1:

            # saves the first timestamp
            simTime_first = simTime_actual
            simTime_basis = [simTime_actual]
        else:

            # saves the actual time in an basis array
            simTime_basis.extend(simTime_actual-simTime_first)

            # computes the latest time step
            simTime_deltaT = simTime_basis[-1] - simTime_basis[-2]

        # assembling the real state vector
        robot_real_x_t = np.array([robot_position[0], robot_position[1],
                        robot_velocity[0], robot_velocity[0]])

        # creating a simulated noisy sensor measurement
        z_t = kf.noisyReading(robot_real_x_t)

        # initializes the kalman filter parameters
        if flag_firstRead:

            # the robot initial position is assumed to be known
            mu_t = robot_real_x_t

            # initializes the covariance matrix as a identity
            Sig_t = np.identity(4)

        # process the filter recursively
        else:

            # calls the filter method
            mu_t, Sig_t = kf.kf(mu_t, Sig_t, z_t, simTime_deltaT)

        # shows the current error
        print('-----')
        print(mu_t)
        print(robot_real_x_t)


