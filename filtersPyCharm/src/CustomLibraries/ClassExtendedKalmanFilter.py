import numpy as np

class ExtendedKalmanFilter:

    # class constructor
    def __init__(self):

        # measurement covariance
        self.cov_z = np.array([[0.1], [0.1], [0.1]])

        # system measurement covariance matrix
        self.Q_t = np.array([[self.cov_z[0][0], 0.0, 0.0],
                             [0.0, self.cov_z[1][0], 0.0],
                             [0.0, 0.0, self.cov_z[2][0]]])

    # ===== Method: Extended Kalman Filter
    def ekf(self, mu_t1, Sig_t1, z_t, deltaT):

        # G jacobian matrix
        G_t = self.jacobian_G(deltaT)

        # H jacobian matrix
        H_t = self.jacobian_H(mu_t1)

        # Q covariance matrix
        R_t = self.matrix_Q(deltaT)

        # prediction step
        m_that = self.motionModel(mu_t1, deltaT)
        Sig_that = G_t.dot(Sig_t1.dot(G_t.T)) + R_t

        # kalman gain computation
        aux_k_t = H_t.dot(Sig_that.dot(H_t.T)) + self.Q_t
        K_t = Sig_that.dot(H_t.T).dot(np.linalg.inv(aux_k_t))

        # update step
        mu_t = mu_t1 + K_t.dot(z_t - self.measurementModel(m_that))
        Sig_t = (np.identity(4) - K_t.dot(H_t)).dot(Sig_that)

        return mu_t, Sig_t

    # ===== Method: Computation of prediction covariance Matrix
    def matrix_Q(self, deltaT):

        # assembling the matrix
        Q_n = np.array([[deltaT**3/3, 0, deltaT**2/2, 0],
                        [0, deltaT**3/3, 0, deltaT**2/2],
                        [deltaT**2/2, 0, deltaT, 0],
                        [0, deltaT**2, 0, deltaT]])

        return Q_n

    # ===== Method: Computation of G jacobian matrix
    def jacobian_G(self, deltaT):

        G_t = np.array([[1, 0, deltaT, 0],
                        [0, 1, 0, deltaT],
                        [0, 0, 1, 0],
                        [0, 0, 0, 1]])

        return G_t

    # ===== Method: Computation of H jacobian matrix
    def jacobian_H(self, m_t1):

        # separating the relevant variables for the sake of comprehension
        x = m_t1[0]
        y = m_t1[1]
        xd = m_t1[2]
        yd = m_t1[3]

        # creating each matrix item
        h11 = x / np.sqrt(x**2 + y**2)
        h12 = y / np.sqrt(x**2 + y**2)
        h21 = (y * (xd*y - x*yd)) / np.sqrt((x**2 + y**2)**3)
        h22 = (x * (x*yd - xd*y)) / np.sqrt((x**2 + y**2)**3)
        h23 = h11
        h24 = h12
        h31 = -y / (x**2 + y**2)
        h32 =  x / (x**2 + y**2)

        # assembling H matrix
        H_t = np.array([[h11[0], h12[0], 0, 0],
                        [h21[0], h22[0], h23[0], h24[0]],
                        [h31[0], h32[0], 0, 0]])

        return H_t

    # ===== Method - measurementModel
    def measurementModel(self, x):
        # avoiding division by zero
        if x[0] == 0 and x[1] == 0:
            x[0] = 0.000001
            x[1] = 0.000001

        # computes z_t considering x_t
        z = np.array([np.sqrt(x[0] ** 2 + x[1] ** 2),
                      (x[0] * x[2] + x[1] * x[3]) / (np.sqrt(x[0] ** 2 + x[1] ** 2)),
                      np.arctan2(x[1], x[0])]).reshape(3, 1)

        return z

    # ===== Method - motionModel propagation
    def motionModel(self, x_t1, deltaT):

        # computes p(x_t | x_t1, u)
        x_that = np.array([x_t1[0] + x_t1[2] * deltaT,
                               x_t1[1] + x_t1[3] * deltaT,
                               [x_t1[2][0]],
                               [x_t1[3][0]]])

        return x_that

    # ===== Method - generates a noisy reading
    # This method takes the robot real position and generates a noisy reading
    def noisyReading(self, x):
        # returns a measurement vector based on the measurement model
        z_hat = self.measurementModel(x)

        # creates adds random noise to the reading
        z_noisy = z_hat + (self.Q_t.dot((2 * (np.random.rand(3, 1) - 0.5))))

        return z_noisy

# ================== Class test
if __name__ == '__main__':

    test = ExtendedKalmanFilter()