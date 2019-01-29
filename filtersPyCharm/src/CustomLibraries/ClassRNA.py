import numpy as np


# class definition
class NeuralNetwork:

    # constructor method
    def __init__(self, training_iterations, lamb):

        # lambda parameter
        if lamb is not None:
            self.lamb = lamb
        else:
            self.lamb = 0

        # training iterations parameter
        if training_iterations is not None:
            self.training_iterations = training_iterations
        else:
            self.training_iterations = 100

        # neural network architecture
        layer_input_size = 3
        layer_hidden1_size = 10
        layer_output_size = 2

        # radomly initialize the weights
        self.theta1 = self.randInitializeWeights(layer_input_size, layer_hidden1_size)
        self.theta2 = self.randInitializeWeights(layer_hidden1_size, layer_output_size)

        # -- measurement models parameters

        # measurement covariance
        self.cov_z = np.array([[0.05], [0.05], [0.05]])

        # system measurement covariance matrix
        self.R_t = np.array([[self.cov_z[0][0], 0.0, 0.0],
                             [0.0, self.cov_z[1][0], 0.0],
                             [0.0, 0.0, self.cov_z[2][0]]])

    # ===== Method: Train the network weights
    # Notice that, in neural networks notation, z_t is our x vector, as it is an iput
    # and x_real is the y vector, as we want to estimate it
    def train(self, z_t, x_real):

        # prepares only the robot position to be estimated
        y = x_real[0:2]

        # forward propagates the input
        l0 = np.vstack((1, z_t)).T
        l1 = self.nonlin_commonLayer(l0.dot(self.theta1.T))
        l1 = np.hstack((1, l1[0]))
        l2 = self.nonlin_outputLayer(l1.dot(self.theta2.T))

        # back propagation of the error, considering the chain rule
        l2_error = y - l2

        l2_delta = l2_error * self.nonlin_commonLayer(l2, deriv=True)

        l1_error = l2_delta.dot(self.theta2)
        l1_delta = l1_error[1:11] * self.nonlin_commonLayer(l1, deriv=True)[1:11]

        # update weights with no learning rate term
        self.theta2 += l1.reshape(l1.size, 1).dot(l2_delta.reshape(1, l2_delta.size)).T
        self.theta1 += l0.reshape(l0.size, 1).dot(l1_delta.reshape(1, l1_delta.size)).T

        # prints the actual error to the user
        print('Error: ' + str(np.mean(np.abs(l2_error))) + ' real: ' + str(x_real[0:2]) +  '  predicted: ' + str(l2))

        return self.theta1, self.theta2

    # ===== Method: Predict function
    # -- desc: given a training set, this function predicts the output given the trained weights
    def predict(self, z_t):

        # forward propagates the input to predict, considering the trained weights
        l0 = np.vstack((1, z_t)).T
        l1 = self.nonlin_commonLayer(l0.dot(self.theta1.T))
        l1 = np.hstack((1, l1[0]))
        p = self.nonlin_outputLayer(l1.dot(self.theta2.T))

        return p

    # ===== Method: Weights random initializer
    # -- Inputs
    #   - l_in: number of units in the entrance layer
    #   - l_out: number of units in the exit layer
    def randInitializeWeights(self, l_in, l_out):

        # seed the random number
        np.random.seed(1)

        # epsilon computation
        # this formula is defined by Andrew NG on its Coursera course.
        epsilon_init = np.sqrt(6) / np.sqrt(l_in + l_out)

        # weights computations
        weights_matrix = (np.random.random((l_out, 1 + l_in))) * 2 * epsilon_init - epsilon_init

        return weights_matrix

    # ===== Method: Computes the sigmoid function for a given array
    def sigmoid(self, z):

        # computes the sigmoid function
        g = 1.0 / (1.0 + np.exp(-z))

        return g

    # ===== Method: Sigmoid gradient
    def sigmoidGradient(self, z):

        # computes the sigmoid
        aux = 1 / (1 + np.exp(-z))

        # computes the gradient
        g = aux * (1 - aux)

        return g

    # ===== Method: non linear of the first layer
    def nonlin_commonLayer(self, x, deriv=False):

        if deriv == True:
            return (x*(1-x))

        return 1/(1+np.exp(-x))

    # ===== Method: non linear function of the last layer
    # the sigmoid function is regularized between -5 and 5
    # the common sigmoid is between 0 and 1
    def nonlin_outputLayer(self, x, deriv=False):

        if deriv == True:
            return (x*(1-x))

        return (1/(1+np.exp(-x)) - 0.5) *  10

    # ===== Method - generates a noisy reading
    # This method takes the robot real position and generates a noisy reading
    def noisyReading(self, x):

        # returns a measurement vector based on the measurement model
        z_hat = self.measurementModel(x)

        # creates adds random noise to the reading
        z_noisy = z_hat + (self.R_t.dot((2 * (np.random.rand(3, 1) - 0.5))))

        return z_noisy

    # ===== Method - measurementModel
    def measurementModel(self, x):

        # avoids zero division
        if x[0] == 0 and x[1] == 0:
            x[0] = 0.00000001
            x[1] = 0.00000001

        # computes z_t considering x_t
        z = np.array([np.sqrt(x[0] ** 2 + x[1] ** 2),
            (x[0] * x[2] + x[1] * x[3]) / (np.sqrt(x[0] ** 2 + x[1] ** 2)),
            np.arctan2(x[1], x[0])]).reshape(3, 1)

        return z

# ===== Class test =====
if __name__ == '__main__':

    pass

    # test = NeuralNetwork(0, 100)

    # sigmoid gradient
    # g = test.sigmoidGradient(np.array([-10, 0, 10]))

    # t1 = test.randInitializeWeights(3, 4)
    # t2 = test.randInitializeWeights(4, 3)
    # x_arr = np.array([[1.1, 1.1, 1.1, 1.1],[2, 2, 2, 2],[3, 3, 3, 3]])
    # p = test.predict(t1, t2, x_arr)

