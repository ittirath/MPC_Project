import numpy as np

from .MPCControl_base import MPCControl_base


class MPCControl_yvel(MPCControl_base):
    x_ids: np.ndarray = np.array([0, 3, 7])
    u_ids: np.ndarray = np.array([0])
    
    def _get_cost_matrices(self) -> tuple[np.ndarray, np.ndarray]:
        """
        Define cost matrices Q and R for y-velocity controller. 
        These penalize deviations from target state and control effort.
        """
        Q = np.diag([
            1.0,    # w_x
            1.0,    # alpha 
            10.0    # v_y (main objective)
        ])
        
        R = np.array([[0.1]])  # d1 torque effort
        
        return Q, R

    def _get_constraints(self) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """
        Define constraints for y-velocity controller.
        """
        # u in U = { u | Mu <= m } FYI u = d1
        M = np.array([[1], [-1]])
        m = np.array([np.deg2rad(15), np.deg2rad(15)]) 
    
        # x in X = { x | Fx <= f } FYI x = (w_x,alpha,v_y)
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
