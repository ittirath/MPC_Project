import numpy as np

from .MPCControl_base import MPCControl_base


class MPCControl_xvel(MPCControl_base):
    x_ids: np.ndarray = np.array([1, 4, 6])
    u_ids: np.ndarray = np.array([1])
    
    def _get_cost_matrices(self) -> tuple[np.ndarray, np.ndarray]:
        """
        Define cost matrices for x-velocity controller.
        """
        Q = np.diag([
            1.0,    # w_y
            10.0,   # v_x (main objective)
            1.0     # beta 
        ])
        R = np.array([[0.1]])  # d2 torque effort
        return Q, R

    def _get_constraints(self) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """
        Define constraints for x-velocity controller.
        """
        # u in U = { u | Mu <= m } FYI u = d2
        M = np.array([[1], [-1]])
        m = np.array([np.deg2rad(15), np.deg2rad(15)]) 
    
        # x in X = { x | Fx <= f } FYI x = (w_y,beta,v_x)
        F = np.array([[0, 1, 0], [0, -1, 0]])
        f = np.array([np.deg2rad(10), np.deg2rad(10)]) 

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