import numpy as np
import pandas as pd

class Traffic:
    def __init__(self, sumo_file):

        # SUMO file
        self.sumo_file = sumo_file
        self.df = pd.read_csv(self.sumo_file)

        # initial state
        self.N = self.get_N() # number of links and controlling lights
        self.x_0 = np.zeros(self.N)
        self.xdot_0 = np.zeros(self.N)

        # intial control (green time)
        self.g_0 = np.zeros(self.N)

        # traffic cycle conditions
        self.C = 60 # length of a traffic cycle [s]
        self.T = 1 # length of a discrete time step [s]
        self.num_timesteps = 10000 # number of total timesteps in the simulation

        # dynamics constants (extracted from SUMO)

        self.A = np.eye(self.N) # identity matrix on queue length
        self.S = np.zeros(self.N) # max saturation ie. max outflow when a light is green
        self.B = np.zeros((self.N, self.N)) # constant matrix describing the relation between green time and the outlow
        self.D = np.zeros((self.N, self.N)) # disturbance matrix
        self.t_z0 = 0 # exit flow rate within a link
        self.d_z = 0 # disturbance flow rate within a link, could base it off the length of a link

        # costs
        self.Q = np.ones((self.N, self.N)) # TODO: pick better costs to queue length
        self.R = 10 * np.ones((self.N, self.N)) # TODO: pick better costs to green time, or switching

        # update the dynamics according to the SUMO file
        self.parse_sumo()

    # function for getting the correction number of lights and links
    def get_N(self):
        # remove the lines with the ":" in them
        self.df = self.df[~self.df.iloc[:,0].str.contains(":")]
        self.df.reset_index(drop=True, inplace=True)

        # getting the number of lights to control
        return len(self.df.index)

    # function for parsing the SUMO file and creating the dynamics from it
    def parse_sumo(self):

        # defining the B matrix
        for z in range(self.N):
            T = self.T
            q_z = self.df['inflow'][z]
            s_z = self.t_z0 * self.df['inflow'][z]
            d_z = self.d_z
            u_z = self.df['outflow'][z]
            self.B[z][z] = T * (q_z - s_z + d_z - u_z) # Eqn. 3

    # Eqn. 5
    def g_k(self, G_k):
        S = self.C
        C = self.C 
        return G_k @ S / C

    # Eqn. 6
    def x_kp1(self, x_k, g_k):
        B = self.B
        T = self.B
        d_k = self.d_z # assuming that the disturbance is time invariant
        return x_k + B @ g_k + T * d_k

    # getting the cost of a particular state
    def cost(self, x_k, g_k):
        Q = self.Q
        R = self.R
        return x_k.T @ Q @ x_k + g_k.T @ R @ g_k


if __name__ == "__main__":
    print("hello")
    traffic = Traffic("linkToFlows.csv")
    traffic.parse_sumo()
    print(traffic.B)

