import numpy as np
import pandas as pd
from pydrake.systems.controllers import DiscreteTimeLinearQuadraticRegulator
from pydrake.solvers import MathematicalProgram, Solve, OsqpSolver
import pydrake.symbolic as sym

class Traffic:
    def __init__(self, sumo_file):

        # SUMO file
        self.sumo_file = sumo_file
        self.df = pd.read_csv(self.sumo_file)

        # initial state
        self.N = self.get_N() # number of links and controlling lights
        self.x_0 = np.zeros(self.N)
        self.xdot_0 = np.zeros(self.N)

        # traffic cycle and timing conditions
        self.C = 60 # length of a traffic cycle [s]
        self.num_steps = 60 # number of steps in a cycle
        self.T = self.C / self.num_steps # time step [s]
        self.num_runs = 100 # number of cycles to simulate

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
        self.df = self.df[~self.df.iloc[:,0].str.contains(":")]
        self.df.reset_index(drop=True, inplace=True)

        # getting the number of lights to control
        return len(self.df.index)

    # function for parsing the SUMO file and creating the dynamics from it
    def parse_sumo(self):

        # defining the B matrix based on historical data from the cycle horizon
        for z in range(self.N):
            
            B = np.zeros(self.N)

            # coming into the links
            q_z = self.df['inflow'][z]

            # leaving the link
            u_z = self.df['outflow'][z]

            self.B[z][z] = (q_z - u_z) # Eqn. 3

        # updating the spawn and despawn rates
        self.d_k = np.array(self.df['despawns'])
        self.s_k = np.array(self.df['spawns'])

    # Eqn. 6, dynamics of the system
    def x_kp1(self, x_k, g_k):
        return self.A @ x_k + self.B @ g_k + self.T * (self.s_k - self.d_k)
    
    # standard LQR coming from linear dynamics and quadratic cost
    def compute_LQR_feedback(self, x_current):
        A = self.A
        B  = self.B
        R = self.R
        Q = self.Q

        # Solve the discrete-time algebraic Riccati equation and get gains
        K, optimal_cost = DiscreteTimeLinearQuadraticRegulator(A, B, Q, R)

        # getting the greentime
        g_current = -K @ x_current

        # returns gains along with corresponding link ordering
        return g_current, self.df['link'] # very finnicky, some values are returning negative, need to fix the cost function or the dynamics. Also adjust the scaling with a time cycle
    
    def add_initial_state_constraint(self, prog, x, x_current):
        # initial state constraint.
        prog.AddBoundingBoxConstraint(x_current, x_current, x[0])

    def add_input_saturation_constraint(self, prog, x, g):
        # limits on green time
        for i in range(self.num_steps-1):
            prog.AddBoundingBoxConstraint(0, self.C, g[i])


    def add_dynamics_constraint(self, prog, x, g):
        # adding dynamics constraints
        for k in range(self.num_steps-1):
            x_e = x[k]
            g_e = g[k]
            x_e_kp1 = x[k+1]
            x_dyn_kp1 = self.x_kp1(x, g)

            for i in range(self.N):
                prog.AddLinearEqualityConstraint(x_dyn_kp1[i] == x_e_kp1[i])

    def add_cost(self, prog, x, g):
        # quadratic cost on the state and input
        for k in range(self.num_steps - 1): 
            x_e = x[k] 
            g_e = g[k]
            prog.AddQuadraticCost(x_e.T @ self.Q @ x_e + g_e.T @ self.R @ g_e)

    def compute_MPC_feedback(self, x_current, use_clf=False):

        # Parameters for the QP
        N = self.N
        T = self.T

        # Initialize mathematical program and decalre decision variables
        prog = MathematicalProgram()
        x = np.zeros((N, 6), dtype="object")
        for i in range(N):
            x[i] = prog.NewContinuousVariables(6, "x_" + str(i))
            g = np.zeros((N-1, 2), dtype="object")
        for i in range(N-1):
            g[i] = prog.NewContinuousVariables(2, "g_" + str(i))

        # Add constraints and cost
        self.add_initial_state_constraint(prog, x, x_current)
        self.add_input_saturation_constraint(prog, x, g)
        self.add_dynamics_constraint(prog, x, g)
        self.add_cost(prog, x, g)

        # Solve the QP
        solver = OsqpSolver()
        result = solver.Solve(prog)
        g_mpc = result.GetSolution(u[0])

        return g_mpc, self.df['link']


if __name__ == "__main__":
    traffic = Traffic("linkToFlows.csv")
    traffic.parse_sumo()
    print(traffic.B)
    # x_current = np.ones((traffic.N, 1))
    # g_lqr, link_list = traffic.compute_LQR_feedback(x_current)
    # print("LQR RESULT: ", g_lqr)
    g_mpc, link_list = traffic.compute_MPC_feedback(x_current)
    print("MPC RESULT: ", g_mpc)

