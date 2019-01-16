# Vrep Connection Class

# Imports V-REP API
try:
    import vrep
except:
    print('--------------------------------------------------------------')
    print('"vrep.py" could not be imported. This means very probably that')
    print('either "vrep.py" or the remoteApi library could not be found.')
    print('Make sure both are in the same folder as this file,')
    print('or appropriately adjust the file "vrep.py"')
    print('--------------------------------------------------------------')
    print('')

# Other imports
import time

# Class definition
class BibVrepManipulation:

    # ===== Class constructor
    def __init__(self):

        # sends a message to the user
        print("VrepManipulation class invoked successfully")

        # class atributes instatiation
        self.ClientID = []

        # vrep connection
        self.connect(19997)

        # start variables streaming from v-rep
        self.startstreaming()

    # ===== Method: Conection to V-REP
    # -- Input
    #   - port: Port number of V-REP application

    def connect(self, port):

        # closes any existing connection
        vrep.simxFinish(-1)

        # connects to V-REP
        self.ClientID = vrep.simxStart('127.0.0.1', port, True, True, 5000, 5)

        # tests the connection
        if self.ClientID != -1:

            # sends confirmation message both on V-REP and python console
            print("V-REP connected successfully.")
            vrep.simxAddStatusbarMessage(self.ClientID, 'A Python interface has connected to V-REP', vrep.simx_opmode_oneshot)

            # returns 1 for successful connection
            return 1

        # failed connection
        else:

            # sends a message to the user
            print("V-REP connection failed.")
            return -1

    # ===== Method: For begin streaming from V-REP
    def startstreaming(self):

        # receives the robot handler
        _, self.handle_robot = vrep.simxGetObjectHandle(self.ClientID, 'robot', vrep.simx_opmode_blocking)

        # start robot position streaming
        vrep.simxGetObjectPosition(self.ClientID, self.handle_robot, -1, vrep.simx_opmode_streaming)

        # start robot orientation streaming
        vrep.simxGetObjectOrientation(self.ClientID, self.handle_robot, -1, vrep.simx_opmode_streaming)

        # start robot velocities streaming
        vrep.simxGetObjectVelocity(self.ClientID, self.handle_robot, vrep.simx_opmode_streaming)

        # waits a little time to assure streaming
        time.sleep(0.3)

    # ====== Method: get the robot pose
    # disclaimer: this method works properly only if vrep_startstreaming was once called
    def getrobotpose(self):

        # receives a position reading from v-rep
        _, position = vrep.simxGetObjectPosition(self.ClientID, self.handle_robot, -1, vrep.simx_opmode_buffer)

        # receives a orientation reading from v-rep
        _, orientation = vrep.simxGetObjectOrientation(self.ClientID, self.handle_robot, -1, vrep.simx_opmode_buffer)

        return (position, orientation)

    # ====== Method: get the robot linear and angular velocities
    def getrobotvelocities(self):

        # Retrieve robot's velocities
        vel = vrep.simxGetObjectVelocity(self.ClientID, self.handle_robot, vrep.simx_opmode_buffer)

        return vel[1]

    # ====== Method: retrieves the simulation time from last command sent to the simulator
    def getsimtimelastretrival(self):

        # retrieves the simulation time in last call
        return vrep.simxGetLastCmdTime(self.ClientID)/1000.0

# ====================================================
# For class testing purposes
if __name__ == '__main__':

    test = BibVrepManipulation()

    print(test.vrep_getrobotvelocities())