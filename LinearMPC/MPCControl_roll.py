import numpy as np

from .MPCControl_base import MPCControl_base


class MPCControl_roll(MPCControl_base):
    x_ids: np.ndarray = np.array([2, 5])
    u_ids: np.ndarray = np.array([3])

    def _get_cost_matrices(self) -> tuple[np.ndarray, np.ndarray]:
        """
        Define cost matrices for roll controller.
        """
        Q = np.diag([
            1.0,     # w_z 
            10.0     # gamma
        ])
        
        R = np.array([[1.0]])  # P_diff torque effort
        
        return Q, R

    def _get_constraints(self) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """
        Define constraints for roll controller.
        """
        # u in U = { u | Mu <= m } FYI u = Pdiff
        M = np.array([[1], [-1]])
        m = np.array([20, 20])
    
        # x in X = { x | Fx <= f } FYI x = (w_z,gamma) UNCONSTRAINED
        F = np.zeros((0,2))
        f = np.zeros((0,))

        return M, m, F, f
    
    # def _setup_controller(self) -> None:
    #     #################################################
    #     # YOUR CODE HERE
    #     self.ocp = ...
    #     # YOUR CODE HERE
    #     #################################################
    
    # def get_u(
    #     self, x0: np.ndarray, x_target: np.ndarray = None, u_target: np.ndarray = None
    # ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    #     #################################################
    #     # YOUR CODE HERE
    #     u0 = ...
    #     x_traj = ...
    #     u_traj = ...
    #     # YOUR CODE HERE
    #     #################################################
    #     return u0, x_traj, u_traj
