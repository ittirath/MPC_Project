import numpy as np

from .MPCControl_base import MPCControl_base


class MPCControl_zvel(MPCControl_base):
    x_ids: np.ndarray = np.array([8])
    u_ids: np.ndarray = np.array([2])

    # only useful for part 5 of the project
    d_estimate: np.ndarray
    d_gain: float
    
    def _get_cost_matrices(self) -> tuple[np.ndarray, np.ndarray]:
        """
        Define cost matrices for z-velocity (altitude) controller.
        """
        Q = np.array([[1.0]])   # v_z (must be 2D for quad_form)
        R = np.array([[1.0]])   # P_avg torque effort
        
        return Q, R

    def _get_constraints(self) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """
        Define constraints for z-velocity controller.
        """
        # u in U = { u | Mu <= m } FYI u = Pavg
        P_hover = 66.6667
        M = np.array([[1], [-1]])
        m = np.array([40 - P_hover, 80 - P_hover])
    
        # x in X = { x | Fx <= f } FYI x = v_z UNCONSTRAINED
        F = np.zeros((0,1)) # columns = number of states = nx
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

    def setup_estimator(self):
        # FOR PART 5 OF THE PROJECT
        ##################################################
        # YOUR CODE HERE

        self.d_estimate = ...
        self.d_gain = ...

        # YOUR CODE HERE
        ##################################################

    def update_estimator(self, x_data: np.ndarray, u_data: np.ndarray) -> None:
        # FOR PART 5 OF THE PROJECT
        ##################################################
        # YOUR CODE HERE
        self.d_estimate = ...
        # YOUR CODE HERE
        ##################################################
