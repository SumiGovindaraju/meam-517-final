from traffic_dynamics import Traffic
from scipy.linalg import solve_discrete_are
import osqp
import numpy as np

class TrafficController:
    def __init__(self):
        # Initialize any necessary variables or data structures
        pass

    def LQR(self, traffic_object):
        A = traffic_object.A
        B  = traffic_object.B
        R = traffic_object.R
        Q = traffic_object.Q

        # Solve the discrete-time algebraic Riccati equation
        P = solve_discrete_are(A, B, Q, R)
        P = np.eye(traffic_object.N)

        # Compute the optimal green-time gain G
        G = -np.linalg.inv(R + B.T @ P @ B) @ B.T @ P @ A

        return G # very finnicky, some values are returning negative, need to fix the cost function or the dynamics. Also adjust the scaling with a time cycle
    
    def MPC(self, traffic_object):
        # TODO: use this to add constaints to the above LQR problem in OSQP
        pass  
        
if __name__ == "__main__":
    traffic = Traffic("linkToFlows.csv")
    traffic.parse_sumo()
    controller = TrafficController()
    G = controller.LQR(traffic)
    print(G)