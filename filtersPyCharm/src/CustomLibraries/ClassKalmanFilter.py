import numpy as np


# Kalman Filter Class
class KalmanFilter:

    # Class constructor
    def __init__(self):

        # defining kalman filter parameters

        # standard deviation in state x
        self.desv_x = 0.1

        # standard deviation in measurement z
        self.desv_z = 0.3

        # measurement matrix - initiated as an identity
        self.H = np.identity(4, dtype=float)

        # system model covariance matrix
        self.Q = self.desv_x * np.identity(4, dtype=float)

        # measurement covariance matrix
        self.R = self.desv_z * np.identity(4, dtype=float)

    # ===== Method - Kalman Filter
    def kf(self, mu_t1, Sig_t1, z_t, deltaT):

        # A matrix assembly
        A_t = np.array([[1, 0, deltaT, 0],
                        [0, 1, 0, deltaT],
                        [0, 0, 1, 0],
                        [0, 0, 0, 1]])

        # C matrix assembly
        C_t = np.array([[np.power(deltaT, 3)/3, 0, np.power(deltaT, 2)/2, 0],
                        [0, np.power(deltaT, 3)/3, 0, np.power(deltaT, 2)/2],
                        [np.power(deltaT, 2)/2, 0, deltaT, 0],
                        [0, np.power(deltaT, 2)/2, 0, deltaT]])

        # 1- prediction step

        # system prediction
        mu_tHat = np.dot(A_t, mu_t1)

        # measurement prediction
        Sig_tHat = np.linalg.multi_dot(A_t, Sig_t1, A_t.T) + self.R


        # 2- update step

        # kalman gain
        K = np.matmul(Sig_tHat, C_t.T, np.linalg.inv(np.matmul(C_t, Sig_tHat, C_t.T) + self.Q))

        # state estimated mean
        mu_t = mu_tHat + K * (z_t - C_t * mu_tHat)

        # state covariance
        Sig_t = np.matmul((np.identity(4) - np.matmul(K, C_t)), Sig_tHat)

        return mu_t, Sig_t

    # ===== Method - adds noise to a reading
    def noisyReading(self, x):

        return x + self.desv_z * ((np.random.rand(4)-0.5) * 2)

# ===================== For class test only
if __name__ == '__main__':

    test = KalmanFilter()

