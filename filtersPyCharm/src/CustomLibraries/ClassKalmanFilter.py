import numpy as np


# Kalman Filter Class
class KalmanFilter:

    # Class constructor
    def __init__(self):

        # defining kalman filter parameters

        # standard deviation in state x
        self.desv_x = 0.05

        # standard deviation in measurement z
        self.desv_z = 0.05

        # measurement matrix - initiated as an identity
        self.H = np.identity(4, dtype=float)

        # system model covariance matrix
        self.Q = self.desv_x * np.identity(4, dtype=float)

        # measurement covariance matrix
        self.R = self.desv_z * np.identity(4, dtype=float)

    # ===== Method - Kalman Filter
    # -- Input
    #   - mu_t1: last estimation mean
    #   - Sig_t1: last estimation covariance matrix
    #   - z_t: measurement vector
    #   - deltaT: time step
    def kf(self, mu_t1, Sig_t1, z_t, deltaT):

        # A matrix assembly
        A_t = np.array([[1, 0, deltaT, 0],
                        [0, 1, 0, deltaT],
                        [0, 0, 1, 0],
                        [0, 0, 0, 1]])

        # C matrix assembly
        C_t = np.array([[(deltaT)**3/3, 0, (deltaT**2)/2, 0],
                        [0, (deltaT**3)/3, 0, (deltaT**2)/2],
                        [(deltaT**2)/2, 0, deltaT, 0],
                        [0, (deltaT**2)/2, 0, deltaT]])

        # 1- prediction step

        # system prediction
        mu_tHat = A_t.dot(mu_t1)

        # measurement prediction
        Sig_tHat = A_t.dot(Sig_t1).dot(A_t.T) + self.R

        # 2- update step

        # kalman gain computed in two steps
        aux_k = C_t.dot(Sig_tHat).dot(C_t.T) + self.Q
        aux_k = aux_k.astype(float)
        K = Sig_tHat.dot(C_t.T).dot(np.linalg.inv(aux_k))

        # state estimated mean
        mu_t = mu_tHat + K.dot((z_t.T - C_t.dot(mu_tHat.T)))

        # state covariance
        Sig_t = (np.identity(4) - K.dot(C_t)).dot(Sig_tHat)

        return mu_t, Sig_t

    # ===== Method - adds noise to a reading
    def noisyReading(self, x):

        # return x + self.desv_z * ((np.random.rand(len(x))-0.5) * 2)
        return x + self.desv_z * (np.random.rand(len(x)) - 0.5)

# ===================== For class test only
if __name__ == '__main__':

    test = KalmanFilter()

