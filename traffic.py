#!/usr/bin/env python3

import numpy as np
import pandas as pd
from pydrake.systems.controllers import DiscreteTimeLinearQuadraticRegulator
from pydrake.solvers import MathematicalProgram, Solve, OsqpSolver
import pydrake.symbolic as sym

class Traffic:
    def __init__(self, sumo_df=None, sumo_file=None, cycle_time=60, time_horizon=5, yellow_time=5, all_red_time=2):

        # SUMO file
        self.sumo_file = sumo_file
        self.df = pd.read_csv(self.sumo_file) if sumo_file is not None else sumo_df

        if self.df is None:
            raise "Dynamics DF is None"

        # initial state
        self.N = self.get_N() # number of links and controlling lights
        self.x_0 = np.zeros(self.N)
        self.xdot_0 = np.zeros(self.N)

        # traffic cycle and timing conditions
        self.C = cycle_time # length of a traffic cycle [s]
        self.T = time_horizon # number of cycles
        self.num_runs = 100 # number of cycles to simulate
        self.yellow_time = yellow_time # time for yellow light [s]
        self.all_red_time = all_red_time # time for all red light [s]

        # dynamics constants (extracted from SUMO)
        self.A = np.eye(self.N) # identity matrix on queue length
        self.B = np.eye(self.N) # constant matrix describing the relation between green time and the outlow
        self.d_k = np.zeros(self.N) # despawn rate
        self.s_k = np.zeros(self.N) # spawn rate

        # cost matrices
        self.Q = np.eye(self.N) # TODO: pick better costs to queue length (ie. 1 / x_z,max)
        self.R = np.eye(self.N) # TODO: pick better costs to green time, or switching

        # update the dynamics according to the SUMO file
        self.parse_sumo()

    # function for getting the correction number of lights and links
    def get_N(self):
        # remove the lines with the ":" in them
        # self.df = self.df[~self.df.iloc[:,0].str.contains(":")]
        self.df.reset_index(drop=True, inplace=True)

        # getting the number of lights to control
        return len(self.df.index)

    # function for parsing the SUMO file and creating the dynamics and cost from it
    def parse_sumo(self):

        # defining the B matrix based on historical data from the cycle horizon
        for z in range(self.N):
            
            B = np.zeros(self.N)

            # coming into the links
            q_z = self.df['inflow'][z]

            # leaving the link
            u_z = self.df['outflow'][z]

            self.B[z][z] = (q_z - u_z) * self.T / self.C # Eqn. 3

            # Q cost on maximum queue length
            self.Q[z][z] = 100 * 1 / self.df['maxQueueLength'][z]

            # R cost on maximum green time (ie. cycle time)
            self.R[z][z] = 1 / self.C

        # updating the spawn and despawn rates
        self.d_k = np.array(self.df['despawns'])
        self.s_k = np.array(self.df['spawns'])

    # Eqn. 6, dynamics of the system
    def x_kp1(self, x_k, g_k):
        return self.A @ x_k + self.T / self.C * self.B @ g_k + self.T * (self.s_k - self.d_k)
    
    # standard LQR coming from linear dynamics and quadratic cost
    def compute_LQR_feedback(self, x_current):
        A = self.A
        B  = self.B
        R = self.R
        Q = self.Q

        # Solve the discrete-time algebraic Riccati equation and get gains
        K, optimal_cost = DiscreteTimeLinearQuadraticRegulator(A, B, Q, R)

        # getting the greentime
        g_current = np.reshape(-K @ x_current, (self.N,))

        # returns gains along with corresponding link ordering
        lqr_df = self.df.copy()
        lqr_df['green time'] = np.around(g_current, 2)
        return lqr_df # very finnicky, some values are returning negative, need to fix the cost function or the dynamics. Also adjust the scaling with a time cycle
    
    def add_initial_state_constraint(self, prog, x, x_current):
        # initial state constraint
        # print(x_current.shape)
        # print(x[0].shape)
        prog.AddBoundingBoxConstraint(x_current, x_current, x[0])
        # print('intial state constraint added')

    def add_input_saturation_constraint(self, prog, x, g):
        # limits on green time
        for i in range(self.T):
            prog.AddBoundingBoxConstraint(0,  self.C - 2 * (self.all_red_time + self.yellow_time), g[i])
        
        # print('input saturation constraint added')


    def add_dynamics_constraint(self, prog, x, g):
        # adding dynamics constraints
        for k in range(self.T - 1):
            x_e = x[k]
            g_e = g[k]
            x_e_kp1 = x[k+1]
            x_dyn_kp1 = self.x_kp1(x[k], g[k])
            # print(x_dyn_kp1.shape)
            # print(x_e_kp1.shape)

            for i in range(self.N):
                prog.AddLinearEqualityConstraint(x_dyn_kp1[i] == x_e_kp1[i])

        # print('dynamics constraint added')

    def add_parallel_light_constraint(self, prog, x, g):

        # get all possible junctionIds from the dataframe
        junction_ids = self.df['junctionId'].unique()

        # remove nan from the list
        junction_ids = [x for x in junction_ids if str(x) != 'nan']

        for id in junction_ids:

            # get the list of all rows with the id and orientation 0.0 or 1.0
            idx_with_0 = self.df.index[(self.df['junctionId'] == id) & (self.df['orientation'] == 0.0)].tolist()
            idx_with_1 = self.df.index[(self.df['junctionId'] == id) & (self.df['orientation'] == 1.0)].tolist()

            for k in range(self.T):
                # all 0 direction lights must be equal
                prog.AddLinearEqualityConstraint(g[k][idx_with_0[0]] == g[k][idx_with_0[1]])

                # all 1 direction lights must be equal
                prog.AddLinearEqualityConstraint(g[k][idx_with_1[0]] == g[k][idx_with_1[1]])
                            
        # print('parallel light constraint added')

    def add_perpindicular_light_constraint(self, prog, x, g):

        # get all possible junctionIds from the dataframe
        junction_ids = self.df['junctionId'].unique()

        # remove nan from the list
        junction_ids = [x for x in junction_ids if str(x) != 'nan']

        for id in junction_ids:

            # get the list of all rows with the id and orientation 0.0 or 1.0
            idx_with_0 = self.df.index[(self.df['junctionId'] == id) & (self.df['orientation'] == 0.0)].tolist()
            idx_with_1 = self.df.index[(self.df['junctionId'] == id) & (self.df['orientation'] == 1.0)].tolist()

            for k in range(self.T):
                # sum of adjacent lights must be equal to the total possible green time
                prog.AddLinearEqualityConstraint(g[k][idx_with_0[0]] + g[k][idx_with_1[0]] == self.C - 2 * (self.all_red_time + self.yellow_time))
                prog.AddLinearEqualityConstraint(g[k][idx_with_0[1]] + g[k][idx_with_1[1]] == self.C - 2 * (self.all_red_time + self.yellow_time))
                            
        # print('perpindicular light constraint added')

    def add_cost(self, prog, x, g):
        # quadratic cost on the state and input
        for k in range(self.T): 
            x_d = np.zeros(self.N)
            g_d = np.zeros(self.N) #+ self.C / 2 * np.ones(self.N) # don't want to deviate too much from 50/50 split
            x_e = x[k] - x_d
            g_e = g[k] - g_d
            prog.AddQuadraticCost(x_e.T @ self.Q @ x_e)
        # print('cost added')

    def compute_MPC_feedback(self, x_current, use_clf=False):

        # Parameters for the QP
        T = self.T
        n_x = self.N
        n_g = self.N

        # Initialize mathematical program and decalre decision variables
        prog = MathematicalProgram()
        x = np.zeros((T, n_x), dtype="object")
        g = np.zeros((T, n_g), dtype="object")
        for i in range(T):
            x[i] = prog.NewContinuousVariables(n_x, "x_" + str(i))
            g[i] = prog.NewContinuousVariables(n_g, "g_" + str(i))

        # Add constraints and cost
        self.add_initial_state_constraint(prog, x, x_current)
        self.add_input_saturation_constraint(prog, x, g)
        self.add_dynamics_constraint(prog, x, g)
        self.add_parallel_light_constraint(prog, x, g)
        self.add_perpindicular_light_constraint(prog, x, g)
        self.add_cost(prog, x, g)

        # Solve the QP
        solver = OsqpSolver()
        result = solver.Solve(prog)
        g_mpc = result.GetSolution(g[0])

        mpc_df = self.df.copy()
        mpc_df['green time'] = np.around(g_mpc, 2)
        return mpc_df


if __name__ == "__main__":
    traffic = Traffic(sumo_file="linkToFlows.csv")
    traffic.parse_sumo()
    print(traffic.B)
    x_current = np.ones((traffic.N, 1))
    lqr_df = traffic.compute_LQR_feedback(x_current)
    print("LQR RESULT: \n", lqr_df['green time'])
    mpc_df = traffic.compute_MPC_feedback(x_current)
    print("MPC RESULT: \n", mpc_df['green time'])

