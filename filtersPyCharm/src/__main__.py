# Main program to initialize the filters class
from CustomLibraries.BibVrepMethods.BibVrepManipulation import BibVrepManipulation
from CustomLibraries.ClassKalmanFilter import KalmanFilter
from CustomLibraries.ClassParticleFilter import ParticleFilter
from CustomLibraries.ClassSimPlot import SimPlot

import numpy as np
import time

if __name__ == '__main__':

    # ========== running parameters settings HERE ==========

    # sim type
    # 1 - Kalman filter
    # 2 - Extended Kalman filter
    # 3 - Particle filter
    sim_type = 3

    # -- parameters for the particle filter

    # number of particles
    pf_particlesNumber = 500

    # ========== pre configurations ==========

    # a message to the user
    print("Main code started")

    # invoking the v-rep manipulation library
    vrep = BibVrepManipulation()

    # invoking the plot library
    # parameter 1 indicates kf plot style
    plotHandler = SimPlot(sim_type)

    # invoking the right filters libraries
    if sim_type == 1:
        kf = KalmanFilter()
    elif sim_type == 2:
        # ekf = ExtendedKalmanFilter()
        pass
    elif sim_type == 3:
        pf = ParticleFilter(pf_particlesNumber)

    # helpful variables
    flag_firstRead = True
    iteration_number = 1

    # ========== simulation phase ==========

    # The infinity loop
    while True:

        # ----- arranging simulation variables -----

        # small pause for better computations
        time.sleep(0.05)

        # retrieving robot's data from V-REP
        robot_pose = vrep.getrobotpose()
        robot_position = robot_pose[0]
        robot_orientation = robot_pose[1]
        robot_velocity = vrep.getrobotvelocities()
        simTime_actual = vrep.getsimtimelastretrival()

        # treating time
        if flag_firstRead == True:

            # saves the first timestamp
            simTime_first = simTime_actual
            simTime_basis = np.array(0)

            # saving the real robots real state vector
            robot_real_x_t = np.array([[robot_position[0]], [robot_position[1]],
                                          [robot_velocity[0]], [robot_velocity[1]]])

        else:

            # saves the actual time in an basis array
            simTime_basis = np.vstack([simTime_basis, simTime_actual-simTime_first])

            # computes the latest time step
            simTime_deltaT = simTime_basis[-1] - simTime_basis[-2]

            # Appending the new real state
            robot_real_x_t = np.hstack((robot_real_x_t, np.array([[robot_position[0]], [robot_position[1]],
                                       [robot_velocity[0]], [robot_velocity[1]]])))

        # ----- Kalman filter steps -----
        if sim_type == 1:

            # initializes the kalman filter parameters
            if flag_firstRead:

                # the robot initial position is assumed to be known
                mu_t = robot_real_x_t

                # initializes the covariance matrix as a identity
                Sig_t = np.identity(4)

                # disables the flag
                flag_firstRead = False

            # process the filter recursively
            else:

                # creating a simulated noisy sensor measurement
                z_t = kf.noisyReading(robot_real_x_t[-1, :])

                # adjusts the mu_t shape for inserting it in KF
                if iteration_number < 3:
                    aux_mu_t1 = mu_t
                else:
                    aux_mu_t1 = mu_t[-1]

                # calls the filter method
                aux_mu_t, Sig_t = kf.kf(aux_mu_t1, Sig_t, z_t, simTime_deltaT)

                # Saves the desirable variables into arrays
                mu_t = np.vstack([mu_t, aux_mu_t])

                # plotting
                plotHandler.kf_draw(robot_real_x_t, mu_t, z_t)

        # ----- Extended kalman steps -----
        if sim_type == 2:

            pass

        # ----- Particle filter steps -----
        if sim_type == 3:

            if flag_firstRead:

                # particles initialization
                xCal = pf.particlesInitialization(robot_real_x_t)

                # plots the initial particle set
                plotHandler.pf_draw(robot_real_x_t, xCal, 'black')

                # disables the flag
                flag_firstRead = False

            else:

                # creates a noisy reading
                z_t = pf.noisyReading(robot_real_x_t[:, -1])

                # invokes iterativelly the particle filter algorithm
                xCal = pf.pf(xCal, z_t, simTime_deltaT, plotHandler)

                # plot of the robot real position
                plotHandler.pf_draw(robot_real_x_t, None, None)

        # ----- Post processing -----

        # increments the counter
        iteration_number += 1

        # shows the current error
        # print('-----')
        # print(iteration_number)
        # print(robot_real_x_t[-1])
        # print(mu_t[-1])
        # print(robot_real_x_t[-1])


