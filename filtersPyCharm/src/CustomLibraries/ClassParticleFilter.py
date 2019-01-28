import numpy as np

# class definition
class ParticleFilter:

    # class constructor
    def __init__(self, M = None):

        # system covariance
        self.cov_x = np.array([[0.5], [0.5], [0.1], [0.1]])

        # measurement covariance
        self.cov_z = np.array([[0.5], [0.5], [0.2]])

        # particles default quantity
        if M is None:
            self.M = 1000
        else:
            self.M = M

        # initial state variance
        self.V = 3

        # system measurement covariance matrix
        self.R_t = np.array([[self.cov_z[0][0], 0.0, 0.0],
                             [0.0, self.cov_z[1][0], 0.0],
                             [0.0, 0.0, self.cov_z[2][0]]])

    # ===== Method - Particle Filter recursive
    # -- Input
    #   - xCal_t1: last time step particle set
    #   - u_n: control vector
    #   - z_n: measurement vector
    #   - deltaT: time step
    #   - resampleMethod: resampling method 1- normal 2- small covariance
    def pf(self, xCal_t1, z_t, deltaT):

        # starts the new particles set
        xCal_hat = []
        xCal_t = []

        # prediction phas
        for m in range(self.M - 1):

            # extracting the particle from the set
            x_tm1 = xCal_t1[:, m]

            # propagates the particle considering the motion model
            x_tm_hat = self.motionModel(x_tm1, deltaT)

            # computes the estimated measurement given the particle
            z_tm_hat = self.measurementModel(x_tm_hat)

            # computes the particle weight based on both real and estimated measurements using a gaussian pdf evaluationo
            w_tm = self.gaussianEvaluate(z_t, z_tm_hat, self.R_t)

            # stores the computed values
            xCal_hat.append([x_tm_hat, w_tm])

        # PAREI AQUI
        pass

    # ===== Method - motionModel propagation
    def motionModel(self, x_t1, deltaT):

        # computes p(x_t | x_t1, u)
        aux_x_that = np.array([x_t1[0] + x_t1[2] * deltaT,
                       x_t1[1] + x_t1[3] * deltaT,
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
        z = np.array([np.sqrt(x[0] ** 2 + x[1] ** 2),
                      (x[0] * x[2] + x[1] * x[3]) / (np.sqrt(x[0] ** 2 + x[1] ** 2)),
                      np.arctan2(x[1], x[0])]).reshape(3, 1)

        return z

    # ===== Method - evaluate an gaussian considering z_t as mean and z_that as the evaluation point
    def gaussianEvaluate(self, z_t, z_that, R_t):

        # computes the gaussian with z_t as mean, R_t as the covariance matrix and z_that as the evaluated point
        res = (1 / np.sqrt(np.linalg.det(R_t) * ((2 * np.pi) ** len(z_t)))) * (
            np.exp(-0.5 * (z_t - z_that).T.dot(np.linalg.inv(R_t)).dot(z_t - z_that)))

        return res[0][0]

    # ===== Method - initial particles generation
    def particlesInitialization(self, x_t_ini):

        # generates the starting points from a gaussian distribution around the belief
        xCal = x_t_ini + np.sqrt(self.V) * (np.random.rand(x_t_ini.size , self.M) - 0.5)

        return xCal

    # ===== Method - generates a noisy reading
    # This method takes the robot real position and generates a noisy reading
    def noisyReading(self, x):

        # returns a measurement vector based on the measurement model
        z_hat = self.measurementModel(x)

        # creates adds random noise to the reading
        z_noisy = z_hat + (self.R_t.dot((2*(np.random.rand(3, 1)-0.5))))

        return z_noisy

# for class testing purposes
if __name__ == '__main__':

    fp = ParticleFilter()
























