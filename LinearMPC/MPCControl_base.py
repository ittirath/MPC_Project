import cvxpy as cp
import numpy as np
from control import dlqr
from mpt4py import Polyhedron
from scipy.signal import cont2discrete

def max_invariant_set(A_cl, X: Polyhedron, max_iter = 150) -> Polyhedron:
	"""
	Compute invariant set for an autonomous linear time invariant system x^+ = A_cl x
	"""
	O = X
	itr = 1
	converged = False
	while itr < max_iter:
		Oprev = O
		F, f = O.A, O.b
		# Compute the pre-set
		O = Polyhedron.from_Hrep(np.vstack((F, F @ A_cl)), np.vstack((f, f)).reshape((-1,)))
		O.minHrep(True)
		_ = O.Vrep  # TODO: this is a tempary fix since the contains() method is not robust enough when both inner and outer polyhera only has H-rep.
		if O == Oprev:
			converged = True
			break
		#print('Iteration {0}... not yet converged\n'.format(itr))
		itr += 1
	
	if converged:
		print('Maximum invariant set successfully computed after {0} iterations.'.format(itr))
	return O # this is O not 0

class MPCControl_base:
    """Complete states indices"""

    x_ids: np.ndarray
    u_ids: np.ndarray

    """Optimization system"""
    A: np.ndarray
    B: np.ndarray
    xs: np.ndarray
    us: np.ndarray
    nx: int
    nu: int
    Ts: float
    H: float
    N: int

    """Optimization problem"""
    ocp: cp.Problem
    # added these for clarity
    x_var: cp.Variable      
    u_var: cp.Variable
    x0_param: cp.Parameter

    def __init__(
        self,
        A: np.ndarray,
        B: np.ndarray,
        xs: np.ndarray,
        us: np.ndarray,
        Ts: float,
        H: float,
    ) -> None:
        self.Ts = Ts
        self.H = H
        self.N = int(H / Ts)
        self.nx = self.x_ids.shape[0]
        self.nu = self.u_ids.shape[0]

        # System definition
        xids_xi, xids_xj = np.meshgrid(self.x_ids, self.x_ids)
        A_red = A[xids_xi, xids_xj].T
        uids_xi, uids_xj = np.meshgrid(self.x_ids, self.u_ids)
        B_red = B[uids_xi, uids_xj].T

        self.A, self.B = self._discretize(A_red, B_red, Ts)
        self.xs = xs[self.x_ids]
        self.us = us[self.u_ids]

        self._setup_controller()

    def _setup_controller(self) -> None:
        #################################################
        # YOUR CODE HERE
        """
        Generic MPC setup - implemented in base class. 
        Calls subclass-specific methods for costs and constraints.
        """
        # Get cost and constraints matrices from subclass
        Q, R = self._get_cost_matrices()
        M, m, F, f = self._get_constraints()
        
        # Input constraints: U = { u | Mu <= m }
        U = Polyhedron.from_Hrep(M, m)
        # State constraints: X = { x | Fx <= f }
        X = Polyhedron.from_Hrep(F, f)
        
        # Terminal Controller 
        K, Qf, _ = dlqr(self.A, self.B, Q, R) # K = terminal controller, Qf = terminal weight
        K = -K
        A_cl = self.A + self.B @ K

        # Compute maximal invariant set for terminal constraint
        KU = Polyhedron.from_Hrep(U.A @ K, U.b) # BK <= b because Bu = BKx 
        # the feasible terminal set is the feasible set of X intersected with BK
        O_inf = max_invariant_set(A_cl, X.intersect(KU))  

        # Define decision variables
        self.x_var = cp.Variable((self.nx, self.N + 1)) 
        self.u_var = cp.Variable((self. nu, self.N))
        self.x0_param = cp. Parameter(self.nx)
        
        # ---- Costs ----
        cost = 0
        for i in range(self.N):
            cost += cp.quad_form(self.x_var[:,i], Q)
            cost += cp.quad_form(self.u_var[:,i], R)

        # Terminal cost
        cost += cp.quad_form(self.x_var[:, -1], Qf)

        # ---- Constraints ----
        constraints = []

        # Initial condition constraint
        constraints.append(self.x_var[:, 0] == self.x0_param)
        
        # System dynamics
        constraints.append(self.x_var[:,1:] == self.A @ self.x_var[:,:-1] + self.B @ self.u_var)

        # State constraints
        constraints.append(X.A @ self.x_var[:, :-1] <= X.b.reshape(-1, 1))

        # Input constraints
        constraints.append(U.A @ self.u_var <= U.b.reshape(-1, 1))

        # Terminal Constraints
        constraints.append(O_inf.A @ self.x_var[:, -1] <= O_inf.b.reshape(-1, 1))
        
        # Create the optimization problem
        self.ocp = cp.Problem(cp. Minimize(cost), constraints)

        # YOUR CODE HERE
        #################################################
    
    def _get_cost_matrices(self) -> tuple[np.ndarray, np.ndarray]:
        """
        Subclasses MUST override this to provide their specific Q and R matrices.
        """
        raise NotImplementedError("Subclass must implement _get_cost_matrices()")

    def _get_constraints(self) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """
        Subclasses MUST override this to provide their specific constraints.
        Returns dictionary with keys: 'u_min', 'u_max', 'x_min', 'x_max' (optional)
        """
        raise NotImplementedError("Subclass must implement _get_constraints()")
    
    @staticmethod
    def _discretize(A: np.ndarray, B: np.ndarray, Ts: float):
        nx, nu = B.shape
        C = np.zeros((1, nx))
        D = np.zeros((1, nu))
        A_discrete, B_discrete, _, _, _ = cont2discrete(system=(A, B, C, D), dt=Ts)
        return A_discrete, B_discrete

    def get_u(
        self, x0: np.ndarray, x_target: np.ndarray = None, u_target: np.ndarray = None
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        #################################################
        # YOUR CODE HERE
        """
        Solve the MPC problem and return optimal control. 
        Implemented in base class - works for all subclasses! 
        """
        # Update targets if provided (for Todo 3.1 it's None)
        if x_target is not None:
            self.xs = x_target
        if u_target is not None: 
            self.us = u_target
        
        # Set initial condition
        self.x0_param. value = x0 - self.xs  # IMPORTANT: deviation from equilibrium
        
        # Solve the optimization problem (PIQP is better than OSQP for small horizons)
        self.ocp.solve(solver=cp.PIQP, verbose=False)
        
        # Extract results
        if self.ocp.status == 'optimal':
            # The optimizer gives deviations from (xs,us), add back equilibrium (xs,us)
            u0 = self.u_var[:, 0].value + self.us
            x_traj = self.x_var.value + self.xs.reshape(-1, 1) 
            u_traj = self.u_var.value + self.us.reshape(-1, 1)
        else:
            # If solver fails return equilibrium
            u0 = self.us 
            x_traj = np.tile(self.xs.reshape(-1, 1), (1, self.N + 1))
            u_traj = np.tile(self.us.reshape(-1, 1), (1, self.N))
            print(f"Warning: MPC solver failed with status: {self.ocp.status}")

        # YOUR CODE HERE
        #################################################

        return u0, x_traj, u_traj
