import numpy as np

# class definition
class ParticleFilter:

    # class constructor
    def __init__(self):

        # system covariance
        self.cov_x = np.array([[0.5], [0.5], [0.1], [0.1]])

        # measurement covariance
        self.cov_z = np.array([0.5], [0.5], [0.2]])

        # particles quantity
        self.M = 1000

        # initial state variance
        self.V = 3

        # system measurement covariance matrix
        self.R_t = np.array([[self.cov_z[0], 0, 0],
                             [0, self.cov_z[1], 0],
                             [0, 0, self.cov_z[2]]])

    # ===== Method - Particle Filter recursive
    # -- Input
    #   - xCal_t1: last time step particle set
    #   - u_n: control vector
    #   - z_n: measurement vector
    #   - deltaT: time step
    #   - resampleMethod: resampling method 1- normal 2- small covariance
    def pf(self, xCal_t1, u_t, z_t, deltaT, resampleMethod):

        # starts the new particles set
        xCal_hat = []
        xCal_t = []

        # prediction phase
        for x_tm1 in xCal_t1:

            # propagates the particle considering the motion model
            x_tm_hat = self.motionModel(x_tm1, deltaT)

            # computes the estimated measurement given the particle
            z_tm_hat = self.measurementModel(x_tm_hat)

            # computes the particle weight based on both real and estimated measurements using a gaussian pdf evaluationo
            w_tm = self.gaussianEvaluate(z_t, z_tm_hat, self.R_t)

            # stores the computes values
            xCal_hat.append([[x_tm_hat],[w_tm]])

        # PAREI AQUI

    # ===== Method - motionModel propagation
    def motionModel(self, x_t1, deltaT):

        # computes p(x_t | x_t1, u)
        aux_x_that = np.array([[x_t1[0] + x_t1[2] * deltaT],
                       [x_t1[1] + x_t1[3] * deltaT],
                       [x_t1[2]],
                       [x_t1[3]]])

        # computing the deviation
        epsilon_t = np.multiply(np.sqrt(self.cov_x),
                                (np.random.rand(4,1)-0.5)*2)

        # computes x_that
        x_that = aux_x_that + epsilon_t

        return x_that

    # ===== Method - measurementModel
    def measurementModel(self, x):

        # computes z_t considering x_t
        z = np.array([[np.sqrt(x[0]**2 + x[1]**2)],
                       [(x[0]*x[2] + x[1]*x[3]) / (np.sqrt(x[0]**2 + x[1]**2))],
                        [np.arctan2(x[1], x[0])]])

        return z

    # ===== Method - evaluate an gaussian considering z_t as mean and z_that as the evaluation point
    def gaussianEvaluate(self, z_t, z_that, R_t):

        # computes the gaussian with z_t as mean, R_t as the covariance matrix and z_that as the evaluated point
        res = (1 / np.sqrt(np.linalg.det(R_t) * ((2 * np.pi) ** len(z_t)))) * (
            np.exp(-0.5 * (z_t.T - z_that.T) * np.linalg.inv(R_t) * (z_t - z_that)))

        return res

# for class testing purposes
if __name__ == '__main__':

    test = ParticleFilter()