import numpy as np

class Traffic:
    def __init__(self, sumo_file):
        # initial state
        self.N = 100 # number of links and controlling lights
        self.x_0 = np.zeros(self.N)
        self.xdot_0 = np.zeros(self.N)

        # intial control (green time)
        self.g_0 = np.zeros(self.N)

        # traffic cycle conditions
        self.C = 60 # length of a traffic cycle [s]
        self.T = 1 # length of a discrete time step [s]
        self.num_timesteps = 10000 # number of total timesteps in the simulation

        # dynamics constants (extracted from SUMO)
        self.S = np.zeros(self.N) # max saturation ie. max outflow when a light is green
        self.B = 0 # constant matrix describing the relation between green time and the outlow
        self.T = 1
        self.D = 0

        # costs
        self.Q = np.zeros((self.N, self.N))
        self.R = np.zeros((self.N, self.N))

    # function for parsing the SUMO file and creating the dynamics from it
    def parse_sumo(self, sumo_file):
        # TODO: learn the necessary dynamics from the linksToFlows.csv file
        pass

    # Eqn. 5
    def g_k(self, G_k):
        S = self.C
        C = self.C 
        return G_k @ S / C

    # Eqn. 6
    def x_kp1(self, x_k, g_k, d_k):
        B = self.B
        T = self.B
        return x_k + B @ g_k + T * d_k

    # getting the cost of a particular state
    def cost(self, x_k, g_k):
        Q = self.Q
        R = self.R
        return x_k.T @ Q @ x_k + g_k.T @ R @ g_k



