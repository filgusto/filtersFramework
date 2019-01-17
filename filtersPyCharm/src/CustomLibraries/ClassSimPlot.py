# This class fournishes methods to plot your simulation relevant data
import matplotlib.pyplot as plt
import time

class SimPlot:

    # Constructor method
    # -- Input
    #   - filterType: 1 for KF, 2 for EKF, 3 for PF
    def __init__(self, filterType):

        # Instantiating useful variables
        self.ax = [None]
        self.fig = [None]

        # Instantiating the correctly filter plot style
        if filterType == 1:
            self.kf_initiate()
        elif filterType == 2:
            self.ekf_initiate()
        elif filterType == 3:
            self.pf_initiate()
        else:
            print('SimPlot constructed with unknown arguments.')

    # ===== Method - Kalman filter plot initiation
    def kf_initiate(self):

        self.fig[0] = plt.figure()

        self.ax[0] = self.fig[0].add_subplot(1, 1, 1)

        # test showing
        self.ax[0].set_xlabel('pos x [m]')
        self.ax[0].set_ylabel('pos y [m]')
        self.ax[0].set_title('Kalman Filter')


    def kf_draw(self, x_real, x_estimated):

        # plots the received arrays
        self.ax[0].plot(x_real, '-r')
        self.ax[0].plot(x_estimated, '-b')

        # displays the plot
        # self.fig[0].show()

# for class testing purposes
if __name__ == '__main__':

    test = SimPlot(1)

    a = input('Press any button to close the figure')