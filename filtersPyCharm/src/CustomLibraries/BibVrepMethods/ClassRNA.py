import numpy as np


# class definition
class RNA:

    # constructor method
    def __init__(self, lamb):

        # lambda parameter
        if lamb is not None:
            self.lamb = lamb
        else:
            self.lamb = 0

        # neural network architecture
            layer_input_size =
            layer_hidden1_size =
            layer_output_size =



    # ===== Method: Weights random initializer
    # -- Inputs
    #   - l_in: number of units in the entrance layer
    #   - l_out: number of units in the exit layer
    def randInitializeWeights(self, l_in, l_out):

        # epsilon computation
        # this formula is defined by Andrew NG on its Coursera course.
        epsilon_init = np.sqrt(6) / np.sqrt(l_in + l_out)

        # weights computations
        weights_matrix = (np.random.rand(l_out, 1 + l_in)) * 2 * epsilon_init - epsilon_init

        return weights_matrix

    # ===== Method: Computes the sigmoid function for a given array
    def sigmoid(self, z):

        # computes the sigmoid function
        g = 1.0 / (1.0 + np.exp(-z))

        return g

    # ===== Methode: Sigmoid gradient
    def sigmoidGradient(self, z):

        # computes the sigmoid
        aux = 1 / (1 + np.exp(-z))

        # computes the gradient
        g = aux * (1 - aux)

        return g


    # ===== Method: Predict function
    # -- desc: given a training set, this function predicts the output given the trained weights
    def predict(self, Theta1, Theta2, X):

        # useful values
        m = X.shape[1]
        labels_number = Theta2.shape[0]

        # performing forward propagation - first layer
        z1 = np.hstack((np.ones(m).reshape(m, 1), X.T))
        h1 = self.sigmoid( z1.dot(Theta1.T))

        # performing forward propagation - second layer
        z2 = np.hstack((np.ones(m).reshape(m, 1), h1.T))
        h2 = self.sigmoid(z2.dot(Theta2.T))

        # converting the probabilities into binaries
        prediction = np.where(h2>0.5, 1, 0)

        return prediction


        i=1
# ===== Class test =====
if __name__ == '__main__':

    test = RNA()

    # sigmoid gradient
    # g = test.sigmoidGradient(np.array([-10, 0, 10]))

    # t1 = test.randInitializeWeights(3, 4)
    # t2 = test.randInitializeWeights(4, 3)
    # x_arr = np.array([[1.1, 1.1, 1.1, 1.1],[2, 2, 2, 2],[3, 3, 3, 3]])
    # p = test.predict(t1, t2, x_arr)
    # print(p)
