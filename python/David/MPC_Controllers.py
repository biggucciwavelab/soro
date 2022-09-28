import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
from scipy.optimize import minimize
from scipy.linalg import block_diag
from MPC_Reference_Generators import *
import cvxopt
from threading import Thread
from scipy.optimize import minimize
from time import time
from scipy.linalg import solve_discrete_are
from scipy.linalg import dft, sqrtm

# minimize u for:  0.5 * u.T @ P @ u + q.T @ u
# subject to:            Aineq @ u <= bineq
#                          Aeq @ u  = beq
def cvxopt_solve_qp(P, q, Aineq=None, bineq=None, Aeq=None, beq=None):
    P = .5 * (P + P.T)  # make sure P is symmetric
    args = [cvxopt.matrix(P), cvxopt.matrix(q)]
    if Aineq is not None:
        args.extend([cvxopt.matrix(Aineq), cvxopt.matrix(bineq)])
        if Aeq is not None:
            args.extend([cvxopt.matrix(Aeq), cvxopt.matrix(beq)])
    cvxopt.solvers.options['show_progress'] = False
    sol = cvxopt.solvers.qp(*args)
    if 'optimal' not in sol['status']:
        return None
    return np.array(sol['x']).reshape((P.shape[1],))


class MPC_Controller:
    numBots = -1
    max_c = -1
    delta_t = 0.01

    positions = []
    velocities = []
    normals = []

    calculating = False

    # Constructor:
    # numBots - number of bots
    # delta_t - time-step duration
    # max_c -   constraint for output value: U within [-max_c, max_c]
    def __init__(self, numBots, botMass, delta_t, max_c=-1):
        self.numBots = numBots
        self.max_c = max_c
        self.delta_t = delta_t
        self.output = np.zeros(self.numBots)
        self.botMass = botMass

    # Returns optimized U for first step
    def calculateControl(self):
        return np.ones(self.numBots)

    def getMPCMatrices(self, A, B, Q, R, P, N):
        m = np.array(B).shape[0] # State size
        n = np.array(B).shape[1] # Control size

        # First the matrices which allow x_bar to be calculated form x0 and u_bar:
        try:
            omega = np.kron(np.eye(N - 1), Q)
            omega = block_diag(omega, P)
        except:
            omega = P


        psi = np.kron(np.eye(N), R)

        # Now the matrices for the QP solver:

        phi = [[]]
        rho = [[]]
        A_prev = np.eye(m)
        tmp = np.zeros((m * N, n))
        for i in range(N):
            tmp = np.vstack((tmp[m:, :], A_prev @ B))

            try:
                rho = np.hstack((tmp, rho))
            except:
                rho = tmp

            A_prev = A_prev @ A

            try:
                phi = np.vstack((phi, A_prev))
            except:
                phi = A_prev

        # test = rho' * omega * rho
        G = 2 * (psi + rho.T @ omega @ rho)
        G = (G + G.T)/2            # To ensure matlab will consider G symmetric
        F = 2 * rho.T @ omega @ phi

        mask = np.hstack((np.eye(n), np.zeros((n, n * (N - 1)))))

        krhc = - mask @ np.linalg.inv(G) @ F

        return krhc, F, G, phi, rho, mask

    def getMPCConstraintMatrices(self, C, phi, rho, N, xmax=None, xmin=None, ymax=None, ymin=None):

        # Create matrix to convert X_bar to Y_bar
        Cmat = np.kron(np.eye(N), C)

        Aineq = np.empty((0, rho.shape[1]), float)
        bineq_c = np.empty((0, N), float)
        bineq_x0 = np.empty((0, phi.shape[1]), float)


        # if xmax != None:
        #     Aineq = np.vstack((Aineq, rho))
        #     bineq_c = np.vstack((bineq_c, np.kron(np.ones(N, 1), xmax)))
        #     bineq_x0 = np.vstack((bineq_x0, - phi))
        #
        # if xmin != None:
        #     Aineq = np.vstack((Aineq, - rho))
        #     bineq_c = np.vstack((bineq_c, - np.kron(np.ones(N, 1), xmin)))
        #     bineq_x0 = np.vstack((bineq_x0, phi))
        #
        # if ymax != None:
        #     Aineq = np.vstack((Aineq, Cmat @ rho))
        #     bineq_c = np.vstack((bineq_c, np.kron(np.ones(N, 1), ymax)))
        #     bineq_x0 = np.vstack((bineq_x0, - Cmat @ phi))
        #
        # if ymin != None:
        #     Aineq = np.vstack((Aineq, - Cmat @ rho))
        #     bineq_c = np.vstack((bineq_c, - np.kron(np.ones(N, 1), ymin)))
        #     bineq_x0 = np.vstack((bineq_x0, Cmat @ phi))

        return Aineq, bineq_c, bineq_x0

    # Usually function called externally, updates state information and reference values as well as calls
    # calculateControl. Returns the U vector for that time-step
    def update(self, positions, velocities, normals, ang_vel):
        self.calculating = True

        # Update state data
        self.positions = positions.T.flatten()
        self.velocities = velocities.T.flatten()
        self.normals = normals.T.flatten()
        self.angVel = ang_vel

        # self.thread = Thread(target=self.startThread)
        # self.thread.start()
        self.startThread()


    def waitForThread(self):
        self.thread.join()

    def startThread(self):
        # Run optimization routine
        out = self.calculateControl()

        # Make sure the output is within the allowed limits, if the MPC is calculated correctly this shouldn't change
        # anything
        # if self.max_c > 0:
        #     out[out > self.max_c] = self.max_c
        #     out[out < - self.max_c] = - self.max_c

        self.output = out

        self.calculating = False

# Initial MPC with delta_u optimization and velocity penalty
class ControllerCase1(MPC_Controller):

    A_mat = np.zeros((1, 1))

    def __init__(self, numBots, botMass, delta_t, max_c=-1):
        super().__init__(numBots, botMass, delta_t, max_c=max_c)

        # Create A matrix
        self.A_mat = self.getAmat()
        self.ref = EllipseRefClosestPoint(np.array([0, 0]), 0, semis=[1.5, 1.5])

    def setReference(self, ref):
        self.ref = ref

    def getAmat(self):
        # Create ones
        I_N = np.eye(self.numBots * 2)

        # Create delta T
        dt = I_N * self.delta_t

        # Create zeros
        Z_N = I_N * 0

        # Return full matrix
        return np.block([[I_N, dt], [Z_N, I_N]])

    def getBmat(self, normals):
        p2 = np.ones(self.numBots * 2) * self.delta_t
        p1 = p2**2 / 2

        p1 = np.reshape(np.multiply(normals, p1), (self.numBots, 2)) / self.botMass
        p2 = np.reshape(np.multiply(normals, p2), (self.numBots, 2)) / self.botMass

        p1 = block_diag(*(p1)).T
        p2 = block_diag(*(p2)).T

        tmp = np.vstack((p1, p2))
        return tmp


    def calculateControl(self):

        # first we build the state vector and the reference state vector
        xi = np.reshape(np.hstack((self.positions, self.velocities)), self.numBots * 4)
        xi_r = self.ref.getReferences(self.positions, self.normals)
        xi_r = np.hstack((xi_r, np.zeros(self.numBots*2)))

        # System matrices
        A = self.A_mat
        B = self.getBmat(self.normals)
        #C = np.hstack((np.eye(self.numBots * 2), np.zeros((self.numBots * 2, self.numBots * 2))))
        # Output state is the position and velocity
        C = np.eye(self.numBots * 4)

        # Create augmented matrices
        A_a = block_diag(A, np.eye(len(xi_r)), np.eye(self.numBots))
        B_a = np.vstack((B, np.zeros((len(xi_r), self.numBots)), np.eye(self.numBots)))
        # C_a = np.hstack((C, -np.eye(self.numBots * 2), np.zeros((self.numBots * 2, self.numBots))))
        C_a = np.hstack((C, -np.eye(self.numBots * 4), np.zeros((self.numBots * 4, self.numBots))))

        # Create initial augmented state:
        xi_a = np.hstack((xi, xi_r, self.output))

        # Define optimization parameters, that is, cost of position error, velocity, actuation effort,
        # and horizon length
        kappa = 1

        # Q = C_a.T @ block_diag(np.eye(self.numBots * 2), .3 * np.eye(self.numBots * 2)) @ C_a
        # P = Q#C_a.T @ block_diag(np.eye(self.numBots * 2), 2 * np.eye(self.numBots * 2)) @ C_a
        Q = C_a.T @ block_diag(np.eye(self.numBots * 2), .5 * np.eye(self.numBots * 2)) @ C_a
        P = C_a.T @ block_diag(1 * np.eye(self.numBots * 2), 2 * np.eye(self.numBots * 2)) @ C_a
        R = np.eye(self.numBots) * kappa
        N = 10

        # Obtain matrices for horizon
        krhc, F, G, phi, rho, mask = self.getMPCMatrices(A_a, B_a, Q, R, P, N)

        # Define QP matrices
        q = (F @ xi_a).T

        # Define constraint matrices

        Aineq = np.kron(np.tri(N), np.eye(self.numBots))
        Aineq = np.vstack((Aineq, -Aineq))
        bineq = np.kron(np.ones((N, 1)),
                        np.vstack((np.ones((self.numBots, 1)) * self.max_c - np.reshape(self.output, (self.numBots, 1)),
                                   np.ones((self.numBots, 1)) * self.max_c + np.reshape(self.output, (self.numBots, 1)))))

        #Aineq = np.kron(np.eye(N), np.vstack((np.eye(self.numBots), - np.eye(self.numBots))))
        #bineq = np.kron(np.ones((N, 1)), np.vstack((np.ones((self.numBots, 1)) * self.max_c, np.ones((self.numBots, 1)) * self.max_c)))

        # Find optimal value
        u = cvxopt_solve_qp(G, q, Aineq=Aineq, bineq=bineq)

        # Convert optimization result to control value
        u = mask @ u + self.output

        if u is not None:
            return u
        else:
            return np.ones(self.numBots)

# Initial MPC with u optimization and velocity penalty
class ControllerCase2(MPC_Controller):

    A_mat = np.zeros((1, 1))

    def __init__(self, numBots, botMass, delta_t, max_c=-1):
        super().__init__(numBots, botMass, delta_t, max_c=max_c)

        # Create A matrix
        self.A_mat = self.getAmat()
        self.ref = EllipseRefClosestPoint(np.array([0, 0]), 0, semis=[1.5, 1.5])

    def setReference(self, ref):
        self.ref = ref

    def getAmat(self):
        # Create ones
        I_N = np.eye(self.numBots * 2)

        # Create delta T
        dt = I_N * self.delta_t

        # Create zeros
        Z_N = I_N * 0

        # Return full matrix
        return np.block([[I_N, dt], [Z_N, I_N]])

    def getBmat(self, normals):
        p2 = np.ones(self.numBots * 2) * self.delta_t
        p1 = p2**2 / 2

        p1 = np.reshape(np.multiply(normals, p1), (self.numBots, 2)) / self.botMass
        p2 = np.reshape(np.multiply(normals, p2), (self.numBots, 2)) / self.botMass

        p1 = block_diag(*(p1)).T
        p2 = block_diag(*(p2)).T

        tmp = np.vstack((p1, p2))
        return tmp

    def calculateControl(self):

        # first we build the state vector and the reference state vector
        xi = np.reshape(np.hstack((self.positions, self.velocities)), self.numBots * 4)
        xi_r = self.ref.getReferences(self.positions, self.normals)
        xi_r = np.hstack((xi_r, np.zeros(self.numBots*2)))

        # System matrices
        A = self.A_mat
        B = self.getBmat(self.normals)
        #C = np.hstack((np.eye(self.numBots * 2), np.zeros((self.numBots * 2, self.numBots * 2))))
        # Output state is the position and velocity
        C = np.eye(self.numBots * 4)

        # Create augmented matrices
        A_a = block_diag(A, np.eye(len(xi_r)))
        B_a = np.vstack((B, np.zeros((len(xi_r), self.numBots))))
        # C_a = np.hstack((C, -np.eye(self.numBots * 2), np.zeros((self.numBots * 2, self.numBots))))
        C_a = np.hstack((C, -np.eye(self.numBots * 4)))

        # Create initial augmented state:
        xi_a = np.hstack((xi, xi_r))


        tmp = C_a @ xi_a

        kappa = 0.5

        Q = C_a.T @ block_diag( np.eye(self.numBots * 2), 1.5 * np.eye(self.numBots * 2)) @ C_a
        P = C_a.T @ block_diag( np.eye(self.numBots * 2), 1.5 * np.eye(self.numBots * 2)) @ C_a
        R = np.eye(self.numBots) * kappa
        N = 10

        # Obtain matrices for horizon
        krhc, F, G, phi, rho, mask = self.getMPCMatrices(A_a, B_a, Q, R, P, N)

        # Define QP matrices
        q = (F @ xi_a).T

        # Define constraint matrices
        Aineq = np.kron(np.eye(N), np.vstack((np.eye(self.numBots), - np.eye(self.numBots))))
        bineq = np.kron(np.ones((N, 1)), np.vstack((np.ones((self.numBots, 1)) * self.max_c, np.ones((self.numBots, 1)) * self.max_c)))

        # Find optimal value
        u = cvxopt_solve_qp(G, q, Aineq=Aineq, bineq=bineq)

        # Convert optimization result to control value
        u = mask @ u

        if u is not None:
            return u
        else:
            return np.ones(self.numBots)

# Using fourier descriptors aproximate geometry
class ControllerCase3(MPC_Controller):

    A_mat = np.zeros((1, 1))

    def __init__(self, numBots, botMass, delta_t, max_c=-1):
        super().__init__(numBots, botMass, delta_t, max_c=max_c)

        # Create A matrix
        self.A_mat = self.getAmat()
        self.ref = EllipseRefClosestPoint(np.array([0, 0]), 0, semis=[1.5, 1.5])

    def setReference(self, ref):
        self.ref = ref

    def getAmat(self):
        # Create ones
        I_N = np.eye(self.numBots * 2)

        # Create delta T
        dt = I_N * self.delta_t

        # Create zeros
        Z_N = I_N * 0

        # Return full matrix
        return np.block([[I_N, dt], [Z_N, I_N]])

    def getBmat(self, normals):
        p2 = np.ones(self.numBots * 2) * self.delta_t
        p1 = p2**2 / 2

        p1 = np.reshape(np.multiply(normals, p1), (self.numBots, 2)) / self.botMass
        p2 = np.reshape(np.multiply(normals, p2), (self.numBots, 2)) / self.botMass

        p1 = block_diag(*(p1)).T
        p2 = block_diag(*(p2)).T

        tmp = np.vstack((p1, p2))
        return tmp

    def rot(self, angle):
        return np.array([[np.cos(angle), np.sin(angle)], [-np.sin(angle), np.cos(angle)]])

    def rot_1(self, angle):
        return np.array([[np.sinc(angle/np.pi), np.sinc(angle/2/np.pi) * np.sin(angle/2)],
                         [-np.sinc(angle/2/np.pi) * np.sin(angle/2), np.sinc(angle/np.pi)]])

    def rot_2(self, angle):
        if np.abs(angle) < 1e-6:
            return np.array([[np.sinc(angle/2)**2, 2 * angle / 3],
                             [- 2 * angle / 3, np.sinc(angle/2)**2]])
        else:
            return np.array([[np.sinc(angle/2)**2, 2 * (1 - np.sinc(angle)) / angle],
                             [- 2 * (1 - np.sinc(angle)) / angle, np.sinc(angle/2)**2]])

    # New function
    def getMPCMatricesAngularRate(self, A, B, R_ang, Q, R, P, N):
        m = np.array(B).shape[0] # State size
        n = np.array(B).shape[1] # Control size

        R_mat = block_diag(*([self.rot(rate) for rate in R_ang]))
        R_mat = block_diag(R_mat, R_mat)
        R_mat = block_diag(R_mat, np.zeros(R_mat.shape))

        R_inc = block_diag(*([self.rot_2(rate) for rate in R_ang]))
        R_inc = block_diag(R_inc, block_diag(*([self.rot_1(rate) for rate in R_ang])))
        R_inc = block_diag(R_inc, np.zeros(R_inc.shape))

        # First the matrices which allow x_bar to be calculated form x0 and u_bar:
        try:
            omega = np.kron(np.eye(N - 1), Q)
            omega = block_diag(omega, P)
        except:
            omega = P


        psi = np.kron(np.eye(N), R)

        # Now the matrices for the QP solver:

        phi = [[]]
        rho = [[]]
        A_prev = np.eye(m)
        for i in range(N):
            A_prev = A_prev @ A

            try:
                phi = np.vstack((phi, A_prev))
            except:
                phi = A_prev

        tmp = np.vstack((np.eye(A.shape[0]), phi[:-A.shape[0], :]))

        for i in range(N):
            B_tmp = R_mat @ np.linalg.matrix_power(R_inc, i) @ B
            try:
                rho = np.hstack((rho, tmp @ B_tmp))
            except:
                rho = tmp @ B_tmp

            tmp = np.vstack((np.zeros(A.shape), tmp[:-A.shape[0], :]))

        # test = rho' * omega * rho
        G = 2 * (psi + rho.T @ omega @ rho)
        G = (G + G.T)/2            # To ensure matlab will consider G symmetric
        F = 2 * rho.T @ omega @ phi

        mask = np.hstack((np.eye(n), np.zeros((n, n * (N - 1)))))

        krhc = - mask @ np.linalg.inv(G) @ F

        return krhc, F, G, phi, rho, mask

    def calculateControl(self):

        # first we build the state vector and the reference state vector
        xi = np.reshape(np.hstack((self.positions, self.velocities)), self.numBots * 4)
        xi_r = self.ref.getReferences(self.positions, self.normals)
        xi_r = np.hstack((xi_r, np.zeros(self.numBots*2)))

        # System matrices
        A = self.A_mat
        B = self.getBmat(self.normals)
        #C = np.hstack((np.eye(self.numBots * 2), np.zeros((self.numBots * 2, self.numBots * 2))))
        # Output state is the position and velocity
        C = np.eye(self.numBots * 4)

        # Create augmented matrices
        A_a = block_diag(A, np.eye(len(xi_r)))
        B_a = np.vstack((B, np.zeros((len(xi_r), self.numBots))))
        # C_a = np.hstack((C, -np.eye(self.numBots * 2), np.zeros((self.numBots * 2, self.numBots))))
        C_a = np.hstack((C, -np.eye(self.numBots * 4)))

        # Create initial augmented state:
        xi_a = np.hstack((xi, xi_r))

        kappa = 10

        Q = C_a.T @ block_diag(np.eye(self.numBots * 2), 1.5 * np.eye(self.numBots * 2)) @ C_a
        P = C_a.T @ block_diag(10 * np.eye(self.numBots * 2), 5 * np.eye(self.numBots * 2)) @ C_a
        R = np.eye(self.numBots) * kappa
        N = 3

        angular_rates = self.angVel * self.delta_t

        # Obtain matrices for horizon
        krhc, F, G, phi, rho, mask = self.getMPCMatricesAngularRate(A_a, B_a, angular_rates, Q, R, P, N)

        # Define QP matrices
        q = (F @ xi_a).T

        # Define constraint matrices
        Aineq = np.kron(np.eye(N), np.vstack((np.eye(self.numBots), - np.eye(self.numBots))))
        bineq = np.kron(np.ones((N, 1)),
                        np.vstack((np.ones((self.numBots, 1)) * self.max_c, np.ones((self.numBots, 1)) * self.max_c)))

        # Find optimal value
        u = cvxopt_solve_qp(G, q, Aineq=Aineq, bineq=bineq)

        # Convert optimization result to control value
        u = mask @ u

        if u is not None:
            return u
        else:
            return np.ones(self.numBots)


# More efficient implementation
class ControllerCase3e(MPC_Controller):

    A_mat = np.zeros((1, 1))

    def __init__(self, numBots, botMass, delta_t, max_c=-1):
        super().__init__(numBots, botMass, delta_t, max_c=max_c)

        # Create A matrix
        self.A_mat = self.getAmat()
        self.ref = EllipseRefClosestPoint(np.array([0, 0]), 0, semis=[1.5, 1.5])
        self.calculateStaticMatrices(5)

    def calculateStaticMatrices(self, N=5):
        # System matrices
        self.A = self.A_mat
        # B is non static
        self.C = np.eye(self.numBots * 4)

        # Create augmented matrices
        self.A_a = block_diag(self.A, np.eye(self.numBots * 4))
        # B_a is non static
        self.C_a = np.hstack((self.C, -np.eye(self.numBots * 4)))

        kappa = 1.5

        self.Q = self.C_a.T @ block_diag(np.eye(self.numBots * 2), 2 * np.eye(self.numBots * 2)) @ self.C_a
        self.P = self.C_a.T @ block_diag(10 * np.eye(self.numBots * 2), 5 * np.eye(self.numBots * 2)) @ self.C_a
        self.R = np.eye(self.numBots) * kappa
        self.N = N

        # Define constraint matrices
        self.Aineq = np.kron(np.eye(self.N), np.vstack((np.eye(self.numBots), - np.eye(self.numBots))))
        self.bineq = np.kron(np.ones((self.N, 1)),
                        np.vstack((np.ones((self.numBots, 1)) * self.max_c, np.ones((self.numBots, 1)) * self.max_c)))

        self.calculateMPCMatricesStatic()

    def setReference(self, ref):
        self.ref = ref

    def getAmat(self):
        # Create ones
        I_N = np.eye(self.numBots * 2)

        # Create delta T
        dt = I_N * self.delta_t

        # Create zeros
        Z_N = I_N * 0

        # Return full matrix
        return np.block([[I_N, dt], [Z_N, I_N]])

    def getBmat(self, normals):
        p2 = np.ones(self.numBots * 2) * self.delta_t
        p1 = p2**2 / 2

        p1 = np.reshape(np.multiply(normals, p1), (self.numBots, 2)) / self.botMass
        p2 = np.reshape(np.multiply(normals, p2), (self.numBots, 2)) / self.botMass

        p1 = block_diag(*(p1)).T
        p2 = block_diag(*(p2)).T

        tmp = np.vstack((p1, p2))
        return tmp

    def rot(self, angle):
        return np.array([[np.cos(angle), np.sin(angle)], [-np.sin(angle), np.cos(angle)]])

    def rot_1(self, angle):
        return np.array([[np.sinc(angle/np.pi), np.sinc(angle/2/np.pi) * np.sin(angle/2)],
                         [-np.sinc(angle/2/np.pi) * np.sin(angle/2), np.sinc(angle/np.pi)]])

    def rot_2(self, angle):
        if np.abs(angle) < 1e-6:
            return np.array([[np.sinc(angle/2)**2, 2 * angle / 3],
                             [- 2 * angle / 3, np.sinc(angle/2)**2]])
        else:
            return np.array([[np.sinc(angle/2)**2, 2 * (1 - np.sinc(angle)) / angle],
                             [- 2 * (1 - np.sinc(angle)) / angle, np.sinc(angle/2)**2]])

    # New function
    def calculateMPCMatricesStatic(self):
        m = np.array(self.A_a).shape[0] # State size
        n = self.numBots * self.N # Control size

        # First the matrices which allow x_bar to be calculated form x0 and u_bar:
        try:
            omega = np.kron(np.eye(self.N - 1), self.Q)
            omega = block_diag(omega, self.P)
        except:
            omega = self.P

        self.omega = omega

        self.psi = np.kron(np.eye(self.N), self.R)

        # Now the matrices for the QP solver:

        phi = [[]]
        A_prev = np.eye(m)
        for i in range(self.N):
            A_prev = A_prev @ self.A_a

            try:
                phi = np.vstack((phi, A_prev))
            except:
                phi = A_prev

        mask = np.hstack((np.eye(self.numBots), np.zeros((self.numBots, n - self.numBots))))

        self.phi = phi

        self.mask = mask

    def getNonStaticMPCMatrices(self, B, R_ang):
        m = np.array(self.A_a).shape[0]  # State size
        n = self.numBots * self.N  # Control size

        R_mat = block_diag(*([self.rot(rate) for rate in R_ang]))
        R_mat = block_diag(R_mat, R_mat)
        R_mat = block_diag(R_mat, np.zeros(R_mat.shape))

        R_inc = block_diag(*([self.rot_2(rate) for rate in R_ang]))
        R_inc = block_diag(R_inc, block_diag(*([self.rot_1(rate) for rate in R_ang])))
        R_inc = block_diag(R_inc, np.zeros(R_inc.shape))

        tmp = np.vstack((np.eye(self.A_a.shape[0]), self.phi[:-self.A_a.shape[0], :]))

        rho = [[]]
        for i in range(self.N):
            B_tmp = R_mat @ np.linalg.matrix_power(R_inc, i) @ B
            try:
                rho = np.hstack((rho, tmp @ B_tmp))
            except:
                rho = tmp @ B_tmp

            tmp = np.vstack((np.zeros(self.A_a.shape), tmp[:-self.A_a.shape[0], :]))

        # test = rho' * omega * rho
        G = 2 * (self.psi + rho.T @ self.omega @ rho)
        #G = (G + G.T) / 2  # To ensure matlab will consider G symmetric
        F = 2 * rho.T @ self.omega @ self.phi

        return F, G

    def calculateControl(self):

        # first we build the state vector and the reference state vector
        xi = np.reshape(np.hstack((self.positions, self.velocities)), self.numBots * 4)
        xi_r = self.ref.getReferences(self.positions, self.normals)
        xi_r = np.hstack((xi_r, np.zeros(self.numBots*2)))

        # Create initial augmented state:
        xi_a = np.hstack((xi, xi_r))

        B = self.getBmat(self.normals)
        B_a = np.vstack((B, np.zeros((self.numBots * 4, self.numBots))))

        # For rotation matrices
        angular_rates = self.angVel * self.delta_t

        # Obtain matrices for horizon
        F, G = self.getNonStaticMPCMatrices(B_a, angular_rates)

        # Define QP matrices
        q = (F @ xi_a).T

        # Find optimal value
        u = cvxopt_solve_qp(G, q, Aineq=self.Aineq, bineq=self.bineq)

        # Convert optimization result to control value
        u = self.mask @ u

        if u is not None:
            return u
        else:
            return np.ones(self.numBots)


class ControllerCase3er(MPC_Controller):

    A_mat = np.zeros((1, 1))
    costs = []
    distances = []
    times = []

    def __init__(self, numBots, botMass, delta_t, max_c=-1):
        super().__init__(numBots, botMass, delta_t, max_c=max_c)

        # Create A matrix
        self.A_mat = self.getAmat()
        self.ref = EllipseRefClosestPoint(np.array([0, 0]), 0, semis=[1.5, 1.5])
        #self.calculateStaticMatrices(5)

    def calculateStaticMatrices(self, N=5):
        # System matrices
        self.A = self.A_mat
        # B is non static
        self.C = np.eye(self.numBots * 4)

        # Create augmented matrices
        self.A_a = block_diag(self.A, np.eye(self.numBots * 4))
        # B_a is non static
        self.C_a = np.hstack((self.C, -np.eye(self.numBots * 4)))

        kappa = .051 * N * self.delta_t**2 / 0.1**2#1 * self.delta_t

        self.Q = self.C_a.T @ block_diag(np.eye(self.numBots * 2),   .01 * N * self.delta_t / 0.1 * np.eye(self.numBots * 2)) @ self.C_a
        self.P = self.C_a.T @ block_diag(5 * np.eye(self.numBots * 2), 2.5 * np.eye(self.numBots * 2)) @ self.C_a
        self.R = np.eye(self.numBots) * kappa
        self.N = N

        # Define constraint matrices
        self.Aineq = np.kron(np.eye(self.N), np.vstack((np.eye(self.numBots), - np.eye(self.numBots))))
        self.bineq = np.kron(np.ones((self.N, 1)),
                        np.vstack((np.ones((self.numBots, 1)) * self.max_c, np.ones((self.numBots, 1)) * self.max_c)))

        self.calculateMPCMatricesStatic()

    def setRotationSpeed(self, omega):
        self.ref_omega = omega
        R1 = self.rot(omega * self.delta_t)
        R2 = omega * self.rot(- np.pi / 2)
        R1 = np.kron(np.eye(self.numBots), R1)
        R2 = np.kron(np.eye(self.numBots), R2)

        RR_mat = np.vstack((R1, R2))
        RR_mat = np.hstack((RR_mat, np.zeros(RR_mat.shape)))

        # Create augmented matrices
        self.A_a = block_diag(self.A, RR_mat)

    def setReference(self, ref):
        self.ref = ref

    def getAmat(self):
        # Create ones
        I_N = np.eye(self.numBots * 2)

        # Create delta T
        dt = I_N * self.delta_t

        # Create zeros
        Z_N = I_N * 0

        # Return full matrix
        return np.block([[I_N, dt], [Z_N, I_N]])

    def getBmat(self, normals):
        p2 = np.ones(self.numBots * 2) * self.delta_t
        p1 = p2**2 / 2

        p1 = np.reshape(np.multiply(normals, p1), (self.numBots, 2)) / self.botMass
        p2 = np.reshape(np.multiply(normals, p2), (self.numBots, 2)) / self.botMass

        p1 = block_diag(*(p1)).T
        p2 = block_diag(*(p2)).T

        tmp = np.vstack((p1, p2))
        return tmp

    def rot(self, angle):
        return np.array([[np.cos(angle), np.sin(angle)], [-np.sin(angle), np.cos(angle)]])

    def rot_1(self, angle):
        return np.array([[np.sinc(angle/np.pi), np.sinc(angle/2/np.pi) * np.sin(angle/2)],
                         [-np.sinc(angle/2/np.pi) * np.sin(angle/2), np.sinc(angle/np.pi)]])

    def rot_2(self, angle):
        if np.abs(angle) < 1e-6:
            return np.array([[np.sinc(angle/2)**2, 2 * angle / 3],
                             [- 2 * angle / 3, np.sinc(angle/2)**2]])
        else:
            return np.array([[np.sinc(angle/2)**2, 2 * (1 - np.sinc(angle)) / angle],
                             [- 2 * (1 - np.sinc(angle)) / angle, np.sinc(angle/2)**2]])

    # New function
    def calculateMPCMatricesStatic(self):
        m = np.array(self.A_a).shape[0] # State size
        n = self.numBots * self.N # Control size

        # First the matrices which allow x_bar to be calculated form x0 and u_bar:
        #try:
        if self.N-1 > 10:
            tmp = np.kron(np.eye(10), self.Q)
            omega = tmp
            ni = 10
            while ni < self.N - 11:
                omega = block_diag(omega, tmp)
                ni += 10
            tmp2 = np.kron(np.eye(self.N - 1 - ni), self.Q)
            omega = block_diag(omega, tmp2)

        else:
            omega = np.kron(np.eye(self.N - 1), self.Q)
        omega = block_diag(omega, self.P)
        #except:
        #    omega = self.P

        self.omega = omega

        self.psi = np.kron(np.eye(self.N), self.R)

        # Now the matrices for the QP solver:

        phi = [[]]
        A_prev = np.eye(m)
        for i in range(self.N):
            A_prev = A_prev @ self.A_a

            try:
                phi = np.vstack((phi, A_prev))
            except:
                phi = A_prev

        mask = np.hstack((np.eye(self.numBots), np.zeros((self.numBots, n - self.numBots))))

        self.phi = phi

        self.mask = mask

    def getNonStaticMPCMatrices(self, B, R_ang):
        m = np.array(self.A_a).shape[0]  # State size
        n = self.numBots * self.N  # Control size

        R_mat = block_diag(*([self.rot(rate) for rate in R_ang]))
        R_mat = block_diag(R_mat, R_mat)
        R_mat = block_diag(R_mat, np.zeros(R_mat.shape))

        R_inc = block_diag(*([self.rot_2(rate) for rate in R_ang]))
        R_inc = block_diag(R_inc, block_diag(*([self.rot_1(rate) for rate in R_ang])))
        R_inc = block_diag(R_inc, np.zeros(R_inc.shape))

        tmp = np.vstack((np.eye(self.A_a.shape[0]), self.phi[:-self.A_a.shape[0], :]))

        rho = [[]]
        for i in range(self.N):
            B_tmp = R_mat @ np.linalg.matrix_power(R_inc, i) @ B
            try:
                rho = np.hstack((rho, tmp @ B_tmp))
            except:
                rho = tmp @ B_tmp

            tmp = np.vstack((np.zeros(self.A_a.shape), tmp[:-self.A_a.shape[0], :]))

        # test = rho' * omega * rho
        G = 2 * (self.psi + rho.T @ self.omega @ rho)
        #G = (G + G.T) / 2  # To ensure matlab will consider G symmetric
        F = 2 * rho.T @ self.omega @ self.phi

        return F, G

    def calculateControl(self):

        t0 = time()
        if abs(self.ref_omega) > 1e-5:
            self.ref.updateRef(self.ref.alfa + self.ref_omega * self.delta_t, self.ref.semis)

        # first we build the state vector and the reference state vector
        xi = np.reshape(np.hstack((self.positions, self.velocities)), self.numBots * 4)
        xi_r = self.ref.getReferences(self.positions, self.normals)
        xi_r = np.hstack((xi_r, np.zeros(self.numBots*2)))

        # Create initial augmented state:
        xi_a = np.hstack((xi, xi_r))

        B = self.getBmat(self.normals)
        B_a = np.vstack((B, np.zeros((self.numBots * 4, self.numBots))))

        # For rotation matrices
        angular_rates = self.angVel * self.delta_t

        # Obtain matrices for horizon
        F, G = self.getNonStaticMPCMatrices(B_a, angular_rates)

        # Define QP matrices
        q = (F @ xi_a)

        # Find optimal value
        u = cvxopt_solve_qp(G, q, Aineq=self.Aineq, bineq=self.bineq)

        cost = 0.5 * u.T @ G @ u - u @ q

        tmp = xi - xi_r
        tmp = np.average(np.sqrt(np.sum(np.reshape(tmp[:self.numBots*2], (self.numBots, 2)) ** 2, axis=1)))

        self.costs.append(cost)
        self.distances.append(tmp)

        # Convert optimization result to control value
        u = self.mask @ u
        t1 = time()
        self.times.append(t1 - t0)

        if u is not None:
            return u
        else:
            return np.ones(self.numBots)


class CurvatureControl(MPC_Controller):
    A_mat = np.zeros((1, 1))

    def __init__(self, numBots, botMass, delta_t, max_c=-1):
        super().__init__(numBots, botMass, delta_t, max_c=max_c)

        # Create A matrix
        self.A_mat = self.getAmat()
        self.ref = EllipseRefClosestPoint(np.array([0, 0]), 0, semis=[1.5, 1.5])
        self.calculateStaticMatrices(5)

    def calculateCurvature(self, points):
        crvt1 = np.gradient(points, axis=0)
        crvt2 = np.gradient(crvt1, axis=0)

        x_d = crvt1[:, 0]
        y_d = crvt1[:, 1]
        x_dd = crvt2[:, 0]
        y_dd = crvt2[:, 1]

        k = (x_d * y_dd - y_d * x_dd) / np.power(x_d ** 2 + y_d ** 2, 3 / 2)

        return k

    def calculateStaticMatrices(self, N=5):
        # System matrices
        self.A = self.A_mat
        # B is non static
        self.C = np.eye(self.numBots * 4)

        # Create augmented matrices
        self.A_a = block_diag(self.A, np.eye(self.numBots * 4))
        # B_a is non static
        self.C_a = np.hstack((self.C, -np.eye(self.numBots * 4)))

        kappa = 1.5

        self.Q = self.C_a.T @ block_diag(np.eye(self.numBots * 2), 2 * np.eye(self.numBots * 2)) @ self.C_a
        self.P = self.C_a.T @ block_diag(10 * np.eye(self.numBots * 2), 5 * np.eye(self.numBots * 2)) @ self.C_a
        self.R = np.eye(self.numBots) * kappa
        self.N = N

        # Define constraint matrices
        self.Aineq = np.kron(np.eye(self.N), np.vstack((np.eye(self.numBots), - np.eye(self.numBots))))
        self.bineq = np.kron(np.ones((self.N, 1)),
                             np.vstack(
                                 (np.ones((self.numBots, 1)) * self.max_c, np.ones((self.numBots, 1)) * self.max_c)))

        self.calculateMPCMatricesStatic()

    def setReference(self, ref):
        self.ref = ref

    def getAmat(self):
        # Create ones
        I_N = np.eye(self.numBots * 2)

        # Create delta T
        dt = I_N * self.delta_t

        # Create zeros
        Z_N = I_N * 0

        # Return full matrix
        return np.block([[I_N, dt], [Z_N, I_N]])

    def getBmat(self, normals):
        p2 = np.ones(self.numBots * 2) * self.delta_t
        p1 = p2 ** 2 / 2

        p1 = np.reshape(np.multiply(normals, p1), (self.numBots, 2)) / self.botMass
        p2 = np.reshape(np.multiply(normals, p2), (self.numBots, 2)) / self.botMass

        p1 = block_diag(*(p1)).T
        p2 = block_diag(*(p2)).T

        tmp = np.vstack((p1, p2))
        return tmp

    def rot(self, angle):
        return np.array([[np.cos(angle), np.sin(angle)], [-np.sin(angle), np.cos(angle)]])

    def rot_1(self, angle):
        return np.array([[np.sinc(angle / np.pi), np.sinc(angle / 2 / np.pi) * np.sin(angle / 2)],
                         [-np.sinc(angle / 2 / np.pi) * np.sin(angle / 2), np.sinc(angle / np.pi)]])

    def rot_2(self, angle):
        if np.abs(angle) < 1e-6:
            return np.array([[np.sinc(angle / 2) ** 2, 2 * angle / 3],
                             [- 2 * angle / 3, np.sinc(angle / 2) ** 2]])
        else:
            return np.array([[np.sinc(angle / 2) ** 2, 2 * (1 - np.sinc(angle)) / angle],
                             [- 2 * (1 - np.sinc(angle)) / angle, np.sinc(angle / 2) ** 2]])

    # New function
    def calculateMPCMatricesStatic(self):
        m = np.array(self.A_a).shape[0]  # State size
        n = self.numBots * self.N  # Control size

        # First the matrices which allow x_bar to be calculated form x0 and u_bar:
        try:
            omega = np.kron(np.eye(self.N - 1), self.Q)
            omega = block_diag(omega, self.P)
        except:
            omega = self.P

        self.omega = omega

        self.psi = np.kron(np.eye(self.N), self.R)

        # Now the matrices for the QP solver:

        phi = [[]]
        A_prev = np.eye(m)
        for i in range(self.N):
            A_prev = A_prev @ self.A_a

            try:
                phi = np.vstack((phi, A_prev))
            except:
                phi = A_prev

        mask = np.hstack((np.eye(self.numBots), np.zeros((self.numBots, n - self.numBots))))

        self.phi = phi

        self.mask = mask

    def getNonStaticMPCMatrices(self, B, R_ang):
        m = np.array(self.A_a).shape[0]  # State size
        n = self.numBots * self.N  # Control size

        R_mat = block_diag(*([self.rot(rate) for rate in R_ang]))
        R_mat = block_diag(R_mat, R_mat)
        R_mat = block_diag(R_mat, np.zeros(R_mat.shape))

        R_inc = block_diag(*([self.rot_2(rate) for rate in R_ang]))
        R_inc = block_diag(R_inc, block_diag(*([self.rot_1(rate) for rate in R_ang])))
        R_inc = block_diag(R_inc, np.zeros(R_inc.shape))

        tmp = np.vstack((np.eye(self.A_a.shape[0]), self.phi[:-self.A_a.shape[0], :]))

        rho = [[]]
        for i in range(self.N):
            B_tmp = R_mat @ np.linalg.matrix_power(R_inc, i) @ B
            try:
                rho = np.hstack((rho, tmp @ B_tmp))
            except:
                rho = tmp @ B_tmp

            tmp = np.vstack((np.zeros(self.A_a.shape), tmp[:-self.A_a.shape[0], :]))

        # test = rho' * omega * rho
        G = 2 * (self.psi + rho.T @ self.omega @ rho)
        # G = (G + G.T) / 2  # To ensure matlab will consider G symmetric
        F = 2 * rho.T @ self.omega @ self.phi

        return F, G

    def calculateControl(self):

        #c_r = self.calculateCurvature(np.reshape(self.positions, (self.numBots, 2)))
        #c_r = self.calculateCurvature(self.ref.getReferencesC(self.numBots))
        coefs_cr = self.ref.coefs#elliptic_fourier_descriptors(self.ref.getReferencesC(self.numBots), 10, normalize=True)


        # first we build the state vector and the reference state vector
        xi = np.reshape(np.hstack((self.positions, self.velocities)), self.numBots * 4)
        xi_r = self.ref.getReferences(self.positions, self.normals)
        xi_r = np.hstack((xi_r, np.zeros(self.numBots * 2)))

        # Create initial augmented state:
        xi_a = np.hstack((xi, xi_r))

        B = self.getBmat(self.normals)

        def calculateCost(X):

            # Calculate state for first two timesteps
            x_n1 = self.A @ xi + B @ X[:self.numBots]
            x_n2 = self.A @ x_n1 + B @ X[self.numBots:]

            err = 0

            # Calculate velocity error
            err += np.sum(np.reshape(x_n1[self.numBots * 2:], (self.numBots, 2))**2) * 1
            err += np.sum(np.reshape(x_n2[self.numBots * 2:], (self.numBots, 2))**2) * 1


            # Calculate contour error
            x_v1 = np.reshape(x_n1[self.numBots * 2:], (self.numBots, 2))
            x_n1 = np.reshape(x_n1[:self.numBots * 2], (self.numBots, 2))
            x_v2 = np.reshape(x_n2[self.numBots * 2:], (self.numBots, 2))
            x_n2 = np.reshape(x_n2[:self.numBots * 2], (self.numBots, 2))
            coefs1 = elliptic_fourier_descriptors(x_n1, self.ref.order, normalize=True)
            coefs2 = elliptic_fourier_descriptors(x_n2, self.ref.order, normalize=True)

            err += np.sum((coefs_cr - coefs1) **2)*2
            err += np.sum((coefs_cr - coefs2) **2)*2

            # Calculate rot_speed error
            center = np.average(x_n1, axis=0)
            err += np.sum(np.cross(x_n1 - center, x_v1)**2) * 0.02

            center = np.average(x_n2, axis=0)
            err += np.sum(np.cross(x_n2 - center, x_v2) ** 2) * 0.02

            # Calculate position error
            err += np.sum(x_n1[self.numBots * 2:] ** 2) * 0.03
            err += np.sum(x_n2[self.numBots * 2:] ** 2) * 0.03

            # Calculate controller effort
            err += np.sum((X + 0.1) ** 2) * .5


            return err

        out = minimize(calculateCost, np.zeros(self.numBots * 2), bounds=[(-self.max_c, self.max_c)] * self.numBots * 2)

        u = out.x[:self.numBots] #- 0.05#-.05

        if u is not None:
            return u
        else:
            return np.ones(self.numBots)



class FourierControlLQR(MPC_Controller):
    tocs = []
    first_iter = True

    def __init__(self, numBots, botMass, delta_t, reference, max_c=-1):
        super().__init__(numBots, botMass, delta_t, max_c=max_c)

        # Create A matrix
        self.ref = reference
        self.order = 10
        self.calculateStaticMatrices(5)
        self.output = np.zeros((self.numBots, 2))

    def setReference(self, ref):
        self.ref = ref
        self.order = ref.order

    def getABmats(self):
        # Create ones
        I_N = np.eye(self.order * 4 + 2)

        # Create delta T
        dt = I_N * self.delta_t

        # Create zeros
        Z_N = I_N * 0

        A = np.block([[I_N, dt], [Z_N, I_N]])

        B = np.block([[.5 * self.delta_t ** 2 * I_N],
                  [self.delta_t * I_N]]) #/ self.botMass

        # Return full matrix
        return A, B

    def updatePoseData(self):

        # Set center of image as (0, 0)
        p_tmp = np.reshape(self.positions, (self.numBots, 2)) - np.array(self.ref.originimage) / self.ref.scale
        v_tmp = np.reshape(self.velocities, (self.numBots, 2))


        # Calculate spread:
        dxy = np.diff(np.vstack((p_tmp, p_tmp[0, :])), axis=0)
        self.dt = np.sqrt(np.sum(dxy**2, axis=1))
        self.t = np.concatenate(([0], np.cumsum(self.dt)))[:-1]
        self.T = self.t[-1]

        # Calculate FFTs
        self.px_fft = nufft(p_tmp[:, 0], self.ref.order, self.dt[:-1], self.t)
        self.py_fft = nufft(p_tmp[:, 1], self.ref.order, self.dt[:-1], self.t)
        self.vx_fft = nufft(v_tmp[:, 0], self.ref.order, self.dt[:-1], self.t)
        self.vy_fft = nufft(v_tmp[:, 1], self.ref.order, self.dt[:-1], self.t)

    # New function
    def calculateStaticMatrices(self, N=5):
        self.N = N

        # Define A matrix
        A, B = self.getABmats()

        # Augment matrices to include reference
        Aa = block_diag(A, np.eye(A.shape[0]))
        Ba = np.block([[B], [np.zeros(B.shape)]])
        Ca = np.block([[np.eye(A.shape[0]), -np.eye(A.shape[0])]])

        # Calculate cost matrices
        # Q = block_diag(2*np.diag(np.concatenate((np.abs(self.ref.cnx)/np.max(np.abs(self.ref.cnx)),
        #                                        np.abs(self.ref.cny)/np.max(np.abs(self.ref.cny))))),
        #                .6 * self.N * self.delta_t / 0.1 * np.eye(self.order  * 4 + 2))
        # Q = Q + .01 * block_diag(2*np.eye(self.ref.order * 4 + 2),
        #                .5 * self.N * self.delta_t / 0.1 * np.eye(self.order  * 4 + 2))*.1

        Q = block_diag(1 * np.diag(np.concatenate((np.abs(self.ref.cnx) / np.max(np.abs(self.ref.cnx)),
                                                    np.abs(self.ref.cny) / np.max(np.abs(self.ref.cny))))),
                       .01 * 20 * self.delta_t / 0.1 * np.eye(self.order * 4 + 2))

        Q = Q*.8 + .2 * block_diag(1 * np.eye(self.order * 4 + 2),
                                 .01 * 20 * self.delta_t / 0.1 * np.eye(self.order * 4 + 2))

        Q[self.order, self.order] = 1e-5
        Q[(self.order) + (self.order * 2 + 1), (self.order) + (self.order * 2 + 1)] = 1e-5
        Q[(self.order) + (self.order * 2 + 1) * 2, (self.order) + (self.order * 2 + 1) * 2] = 1e-5
        Q[(self.order) + (self.order * 2 + 1) * 3, (self.order) + (self.order * 2 + 1) * 3] = 1e-5

        Qa = Ca.T @ Q @ Ca

        P = Q
        Pa = Qa
        R = np.eye(self.order * 4 + 2) * 2

        # Define constraint matrices
        DFT_Mat = dft(self.order  * 2 + 1)
        DFT_Mat = block_diag(DFT_Mat, DFT_Mat)
        DFT_Mat = np.kron(np.eye(self.N), DFT_Mat)
        self.Aineq = np.block([[DFT_Mat], [-DFT_Mat]])
        self.bineq = np.ones(((self.order  * 4 + 2) * self.N * 2, 1))

        krhc, self.F, self.G, phi, rho, mask = self.getMPCMatrices(Aa, Ba, Qa, R, Pa, self.N)

        # Save unconstrained LQR solution
        self.krhc = krhc

        # Calculate LQR solution
        self.klqr = solve_discrete_are(A, B, Q, R, balanced=False)
        self.klqr = -np.linalg.inv(R + B.T @ self.klqr @ B) @ B.T @ self.klqr @ A @ Ca

        self.corr_data_vect = 1 / (self.order * 2 + 1) * self.numBots * 2j * np.pi *\
                              np.linspace(-.5, .5, (self.order * 2 + 1))

    def tic(self):
        self.t0 = time()

    def toc(self):
        t1 = time()
        tmp = (t1 - self.t0) * 1000.
        print('Calculation time: ' + str(np.round(tmp, 6)) + ' ms')
        return tmp

    def calculateControl(self):

        self.tic()
        # Convert to frequency domain
        self.updatePoseData()


        # Get reference fourier series
        fser_x_r = self.ref.cnx
        fser_y_r = self.ref.cny

        # Calculate offset
        corr_x = np.real(np.fft.ifft(np.fft.ifftshift(self.px_fft * np.flip(fser_x_r))))
        est = (corr_x - np.min(corr_x))
        corr_x = np.sum(est / np.sum(est) * np.arange(0, len(corr_x), 1))

        corr_y = np.real(np.fft.ifft(np.fft.ifftshift(self.py_fft * np.flip(fser_y_r))))
        est = (corr_y - np.min(corr_y))
        corr_y = np.sum(est / np.sum(est) * np.arange(0, len(corr_y), 1))

        corr = -np.average([corr_x, corr_y])
        corr = np.exp(corr * self.corr_data_vect)

        if self.first_iter:
            self.corr = corr
            self.first_iter = False

        xi_r = np.concatenate((fser_x_r * self.corr, fser_y_r * self.corr))
        xi_r = np.concatenate((xi_r, np.zeros(len(xi_r))))


        # Calculate positions
        xi = np.concatenate((self.px_fft, self.py_fft, self.vx_fft, self.vy_fft))

        xi_a = np.concatenate((xi, xi_r))

        # Find optimal values
        u = self.klqr @ xi_a
        nu = len(u)//2
        ux = u[:nu]
        uy = u[nu:]

        idx = np.round(self.t * (self.numBots*100 - 1) / self.t[-1], 0).astype(int)

        ux = np.real(ifft_n(ux, self.numBots*100))[idx] #* self.dt
        uy = np.real(ifft_n(uy, self.numBots*100))[idx] #* self.dt

        # q = (self.F @ xi_a)
        # u = cvxopt_solve_qp(self.G, q, Aineq=self.Aineq, bineq=self.bineq)

        u = np.vstack((ux, uy)).T * self.botMass

        print('Max control action: ' + str(np.round(np.max(u), 6)))


        if u is not None:
            return u
        else:
            return np.ones((self.numBots, 2))

class FourierControlMPC(MPC_Controller):
    tocs = []
    first_iter = True

    def __init__(self, numBots, botMass, delta_t, reference, max_c=-1):
        super().__init__(numBots, botMass, delta_t, max_c=max_c)

        # Create A matrix
        self.ref = reference
        self.order = 10
        self.calculateStaticMatrices(5)
        self.output = np.zeros((self.numBots, 2))

        self.startPlots()

    def setReference(self, ref):
        self.ref = ref
        self.order = ref.order

    def getABmats(self):
        # Create ones
        I_N = np.eye(self.order * 2 + 2)

        # Create delta T
        dt = I_N * self.delta_t

        # Create zeros
        Z_N = I_N * 0

        A = np.block([[I_N, dt], [Z_N, I_N]])

        B = np.block([[.5 * self.delta_t ** 2 * I_N],
                  [self.delta_t * I_N]]) #/ self.botMass

        # Return full matrix
        return A, B

    def updatePoseData(self):

        # Set center of image as (0, 0)
        p_tmp = np.reshape(self.positions, (self.numBots, 2)) - np.array(self.ref.originimage) / self.ref.scale
        v_tmp = np.reshape(self.velocities, (self.numBots, 2))


        # Calculate spread:
        dxy = np.diff(np.vstack((p_tmp, p_tmp[0, :])), axis=0)
        self.dt = np.sqrt(np.sum(dxy**2, axis=1))
        self.t = np.concatenate(([0], np.cumsum(self.dt)))[:-1]
        self.T = self.t[-1]

        # Calculate FFTs
        self.px_fft = nurfft(p_tmp[:, 0], self.ref.order, self.dt[:-1], self.t)
        self.py_fft = nurfft(p_tmp[:, 1], self.ref.order, self.dt[:-1], self.t)
        self.vx_fft = nurfft(v_tmp[:, 0], self.ref.order, self.dt[:-1], self.t)
        self.vy_fft = nurfft(v_tmp[:, 1], self.ref.order, self.dt[:-1], self.t)

    # New function
    def calculateStaticMatrices(self, N=5):
        self.N = N

        # Define A matrix
        A, B = self.getABmats()

        # Augment matrices to include reference
        Aa = block_diag(A, np.eye(A.shape[0]))
        Ba = np.block([[B], [np.zeros(B.shape)]])
        Ca = np.block([[np.eye(A.shape[0]), -np.eye(A.shape[0])]])

        # Calculate cost matrices
        # Q = block_diag(2*np.diag(np.concatenate((np.abs(self.ref.cnx)/np.max(np.abs(self.ref.cnx)),
        #                                        np.abs(self.ref.cny)/np.max(np.abs(self.ref.cny))))),
        #                .6 * self.N * self.delta_t / 0.1 * np.eye(self.order  * 4 + 2))
        # Q = Q + .01 * block_diag(2*np.eye(self.ref.order * 4 + 2),
        #                .5 * self.N * self.delta_t / 0.1 * np.eye(self.order  * 4 + 2))*.1

        Q = block_diag(100 * np.diag(np.concatenate((np.abs(self.ref.cnx) / np.max(np.abs(self.ref.cnx)),
                                                   np.abs(self.ref.cny) / np.max(np.abs(self.ref.cny))))),
                       .05 * 20 * self.delta_t / 0.1 * np.eye(self.order * 2 + 2))
        Q = Q + .2 * block_diag(2 * np.eye(self.order * 2 + 2),
                                 .05 * 20 * self.delta_t / 0.1 * np.eye(self.order * 2 + 2))

        # l = np.linspace(0, .5, self.order + 1)
        # l = (l**2)*2
        # r = l# np.linspace(0, .5, self.order + 1) * 0
        # Q = block_diag(2 * np.diag(np.concatenate((l, l, r, r))))
        # Q = Q + 1e-5 * np.eye(self.order * 4 + 4)

        Q[0, 0] = .5
        Q[1, 1] = .5
        Q[self.order + 1, self.order + 1] = .1
        Q[self.order + 2, self.order + 2] = .1

        Qa = Ca.T @ Q @ Ca

        P = Q
        Pa = Qa
        R = np.eye(self.order * 2 + 2) * .5

        # Define constraint matrices
        K = self.order*2 + 1
        N = K#self.numBots
        n = np.arange(0, N, 1)
        k = np.arange(0, K//2 + K%2, 1)
        angles = 2 * np.pi * np.outer(n, k) / N
        cos_mat = np.cos(angles) @ block_diag([[.5]], np.eye(self.order))#/ N * self.order
        cos_mat = block_diag(cos_mat, cos_mat)
        sin_mat = np.sin(angles) @ block_diag([[.5]], np.eye(self.order))#/ N * self.numBots
        sin_mat = block_diag(sin_mat, sin_mat)
        full_mat = np.hstack((np.kron(np.eye(self.N), cos_mat), -np.kron(np.eye(self.N), sin_mat)))
        full_mat = np.vstack((full_mat, -full_mat))

        K = self.order * 2 + 1
        N = K  # self.numBots
        n = np.arange(0, N, 1)
        k = - np.arange(0, K // 2 + K%2, 1)
        angles = 2 * np.pi * np.outer(n, k) / N
        cos_mat = np.cos(angles) @ block_diag([[.5]], np.eye(self.order)) # / N * self.order
        cos_mat = block_diag(cos_mat, cos_mat)
        sin_mat = np.sin(angles) @ block_diag([[.5]], np.eye(self.order)) # / N * self.numBots
        sin_mat = block_diag(sin_mat, sin_mat)
        full_mat2 = np.hstack((np.kron(np.eye(self.N), cos_mat), np.kron(np.eye(self.N), sin_mat)))
        full_mat2 = np.vstack((full_mat2, -full_mat2))

        full_mat += full_mat2


        self.Aineq = full_mat * self.botMass
        self.bineq = np.ones((N * self.N * 4, 1)) * self.max_c

        krhc, self.F, self.G, phi, rho, mask = self.getMPCMatrices(Aa, Ba, Qa, R, Pa, self.N)

        # Define complex QP matrices:
        self.EE = block_diag(self.G, self.G)
        self.A_g = np.real(sqrtm(self.G))
        self.E = block_diag(self.A_g, self.A_g)
        self.A_g_inv = np.linalg.inv(self.A_g)

        # # Save unconstrained LQR solution
        # self.krhc = krhc
        #
        # # Calculate LQR solution
        # self.klqr = solve_discrete_are(A, B, Q, R, balanced=False)
        # self.klqr = -np.linalg.inv(R + B.T @ self.klqr @ B) @ B.T @ self.klqr @ A @ Ca

        self.corr_data_vect = 1 / (self.order + 1) * self.numBots * 2j * np.pi *\
                              np.linspace(0, .5, (self.order + 1))

    def tic(self):
        self.t0 = time()

    def toc(self):
        t1 = time()
        tmp = (t1 - self.t0) * 1000.
        print('Calculation time: ' + str(np.round(tmp, 6)) + ' ms')
        return tmp

    def startPlots(self):
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111)

        self.c1, = self.ax.plot([], [], 'x-')
        self.c2, = self.ax.plot([], [], 'x-')
        self.c3, = self.ax.plot([], [], 'x-')
        self.c4, = self.ax.plot([], [], 'x-')
        self.c5, = self.ax.plot([], [], 'k:')
        self.ax.set_xlim([-2, 2])
        self.ax.set_ylim([-2, 2])

    def calculateControl(self):

        self.tic()
        # Convert to frequency domain
        self.updatePoseData()


        # Get reference fourier series
        fser_x_r = self.ref.cnx
        fser_y_r = self.ref.cny

        f_shift_x = np.concatenate((np.flip(fser_x_r[1:]), fser_x_r))
        f_shift_y = np.concatenate((np.flip(fser_x_r[1:]), fser_x_r))
        f_px = np.concatenate((np.flip(self.px_fft[1:]), self.px_fft))
        f_py = np.concatenate((np.flip(self.py_fft[1:]), self.py_fft))
        # Calculate offset
        corr_x = np.real(np.fft.ifft(np.fft.ifftshift(f_px * np.flip(f_shift_x))))
        est = (corr_x - np.min(corr_x))
        corr_x = np.sum(est / np.sum(est) * np.arange(0, len(corr_x), 1))

        corr_y = np.real(np.fft.ifft(np.fft.ifftshift(f_py * np.flip(f_shift_y))))
        est = (corr_y - np.min(corr_y))
        corr_y = np.sum(est / np.sum(est) * np.arange(0, len(corr_y), 1))

        corr = -np.average([corr_x, corr_y])
        corr = np.exp(corr * self.corr_data_vect)

        if self.first_iter:
            self.corr = corr
            self.first_iter = False

        xi_r = np.concatenate((fser_x_r * corr, fser_y_r * corr))
        #xi_r = np.concatenate((fser_x_r * self.corr, fser_y_r * self.corr))
        xi_rz = np.zeros(len(xi_r))
        xi_r = np.concatenate((xi_r, xi_rz))


        # Calculate positions
        xi = np.concatenate((self.px_fft, self.py_fft, self.vx_fft, self.vy_fft))
        xi_a = np.concatenate((xi, xi_r))

        b = self.A_g_inv @ self.F @ xi_a
        h = np.hstack((np.real(b), np.imag(b)))
        q = 2 * h @ self.E

        out = cvxopt_solve_qp(self.EE, q, self.Aineq, self.bineq)

        u = np.block([np.eye((self.order * 2 + 2) * self.N), 1j*np.eye((self.order * 2 + 2) * self.N)]) @ out

        if self.N > 1:
            u = np.block([np.eye((self.order * 2 + 2)),
                          np.zeros(((self.order * 2 + 2), (self.order * 2 + 2) * (self.N - 1)))]) @ u

        # Find optimal values
        nu = len(u)//2
        ux = u[:nu]
        uy = u[nu:]

        idx = np.round(self.t * (self.numBots * 100 - 1) / self.t[-1], 0).astype(int)

        x = np.arange(0, self.numBots, 1)
        y = irfft_n(ux, self.numBots) * self.botMass


        # N = self.numBots
        # r = 1.5
        # y = irfft_n(ux, N * 100)[idx]
        # x = np.linspace(0, 2 * np.pi, N)
        #
        # rx = irfft_n(ux, N * 100)[idx] + r * np.cos(x)
        # ry = irfft_n(uy, N * 100)[idx] + r * np.sin(x)
        #
        # self.c1.set_xdata(rx)
        # self.c1.set_ydata(ry)
        #
        # y = irfft_n(uy, N * 100)[idx]
        #
        # p = np.reshape(self.positions, (self.numBots, 2)) - np.array(self.ref.originimage) / self.ref.scale
        #
        # self.c2.set_xdata(p[:, 0])
        # self.c2.set_ydata(p[:, 1])
        #
        # p_refx = irfft_n(fser_x_r * self.corr, self.numBots)
        # p_refy = irfft_n(fser_y_r * self.corr, self.numBots)
        #
        # self.c3.set_xdata(p_refx)
        # self.c3.set_ydata(p_refy)
        #
        # self.c5.set_xdata(r * np.cos(x))
        # self.c5.set_ydata(r * np.sin(x))

        # plt.grid()
        # plt.xlabel('s')
        # plt.ylabel('Ux')
        # plt.show()
        self.fig.canvas.draw()


        ux = irfft_n(ux, self.numBots*100)[idx] #* self.dt
        uy = irfft_n(uy, self.numBots*100)[idx] #* self.dt



        u = np.vstack((ux, uy)).T * self.botMass

        print('Max control action: ' + str(np.round(np.max(u), 6)))


        if u is not None:
            return u
        else:
            return np.ones((self.numBots, 2))

class FourierControlComplexMPC(MPC_Controller):
    tocs = []
    first_iter = True
    mapto = 0.
    obj_data = [0., 0.]

    def __init__(self, numBots, botMass, delta_t, reference, max_c=-1):
        super().__init__(numBots, botMass, delta_t, max_c=max_c)

        # Create A matrix
        self.ref = reference
        self.order = 10
        self.calculateStaticMatrices(5)
        self.output = np.zeros((self.numBots, 2))

    def setReference(self, ref):
        self.ref = ref
        self.order = ref.order

    def getABmats(self):
        # Create ones
        I_N = np.eye(self.order * 2 + 1)

        # Create delta T
        dt = I_N * self.delta_t

        # Create zeros
        Z_N = I_N * 0

        A = np.block([[I_N, dt], [Z_N, I_N]])

        B = np.block([[.5 * self.delta_t ** 2 * I_N], [self.delta_t * I_N]])

        # Return full matrix
        return A, B

    def updatePoseData(self):

        # Set center of image as (0, 0)
        p_tmp = np.reshape(self.positions, (self.numBots, 2)) - np.array(self.ref.originimage) / self.ref.scale
        v_tmp = np.reshape(self.velocities, (self.numBots, 2))


        # Calculate spread:
        dxy = np.diff(np.vstack((p_tmp, p_tmp[0, :])), axis=0)
        self.dt = np.sqrt(np.sum(dxy**2, axis=1))
        self.t = np.concatenate(([0], np.cumsum(self.dt)))[:-1]
        self.T = self.t[-1]

        # Calculate FFTs
        self.p_fft = nufft(p_tmp[:, 0] + 1j * p_tmp[:, 1], self.ref.order, self.dt[:-1], self.t)
        self.v_fft = nufft(v_tmp[:, 0] + 1j * v_tmp[:, 1], self.ref.order, self.dt[:-1], self.t)

    # New function
    def calculateStaticMatrices(self, N=5):
        self.N = N

        # Define A matrix
        A, B = self.getABmats()

        # Augment matrices to include reference
        Aa = block_diag(A, A)#np.eye(A.shape[0]))
        Ba = np.block([[B], [np.zeros(B.shape)]])
        Ca = np.block([[np.eye(A.shape[0]), -np.eye(A.shape[0])]])

        # Calculate cost matrices
        # Q = block_diag(2*np.diag(np.concatenate((np.abs(self.ref.cnx)/np.max(np.abs(self.ref.cnx)),
        #                                        np.abs(self.ref.cny)/np.max(np.abs(self.ref.cny))))),
        #                .6 * self.N * self.delta_t / 0.1 * np.eye(self.order  * 4 + 2))
        # Q = Q + .01 * block_diag(2*np.eye(self.ref.order * 4 + 2),
        #                .5 * self.N * self.delta_t / 0.1 * np.eye(self.order  * 4 + 2))*.1

        Q = block_diag(30 * np.diag((np.abs(self.ref.cn) / np.max(np.abs(self.ref.cn)))),
                       .05 * 20 * self.delta_t / 0.1 * np.eye(self.order * 2 + 1))
        Q = Q*.8 + .2 * block_diag(2 * np.eye(self.order * 2 + 1),
                                 .05 * 20 * self.delta_t / 0.1 * np.eye(self.order * 2 + 1))

        # l = np.linspace(0, .5, self.order + 1)
        # l = (l**2)*2
        # r = l# np.linspace(0, .5, self.order + 1) * 0
        # Q = block_diag(2 * np.diag(np.concatenate((l, l, r, r))))
        # Q = Q + 1e-5 * np.eye(self.order * 4 + 4)

        Q[0, 0] = .5
        Q[1, 1] = .5
        Q[self.order + 1, self.order + 1] = .5
        Q[self.order + 2, self.order + 2] = .5

        Qa = Ca.T @ Q @ Ca

        P = Q
        Pa = Qa
        R = np.eye(self.order * 2 + 1) * .5

        # Define constraint matrices
        K = self.order*2 + 1
        N = K
        n = np.arange(0, N, 1)
        k = np.arange(-(K//2), K//2 + (K%2), 1)

        angles = 2 * np.pi * np.outer(n, k) / N
        cos_mat = np.cos(angles)
        sin_mat = np.sin(angles)
        real_mat = np.hstack((np.kron(np.eye(self.N), cos_mat), -np.kron(np.eye(self.N), sin_mat)))
        real_mat = np.vstack((real_mat, -real_mat))
        imag_mat = np.hstack((np.kron(np.eye(self.N), cos_mat), np.kron(np.eye(self.N), sin_mat)))
        imag_mat = np.vstack((imag_mat, -imag_mat))
        full_mat = np.vstack((real_mat, imag_mat))

        self.Aineq = full_mat * self.botMass
        self.bineq = np.ones((N * self.N * 4, 1)) * self.max_c

        krhc, self.F, self.G, phi, rho, mask = self.getMPCMatrices(Aa, Ba, Qa, R, Pa, self.N)

        # Define complex QP matrices:
        self.EE = block_diag(self.G, self.G)
        self.A_g = np.real(sqrtm(self.G))
        self.E = block_diag(self.A_g, self.A_g)
        self.A_g_inv = np.linalg.inv(self.A_g)

        # # Save unconstrained LQR solution
        # self.krhc = krhc
        #
        # # Calculate LQR solution
        # self.klqr = solve_discrete_are(A, B, Q, R, balanced=False)
        # self.klqr = -np.linalg.inv(R + B.T @ self.klqr @ B) @ B.T @ self.klqr @ A @ Ca

        self.corr_data_vect = 1 / (self.order + 1) * self.numBots * 2j * np.pi *\
                              np.linspace(-.5, .5, (self.order * 2 + 1))

    def tic(self):
        self.t0 = time()

    def toc(self):
        t1 = time()
        tmp = (t1 - self.t0) * 1000.
        print('Calculation time: ' + str(np.round(tmp, 6)) + ' ms')
        return tmp

    def startPlots(self):
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111)

        self.c1, = self.ax.plot([], [], 'x-')
        self.c2, = self.ax.plot([], [], 'x-')
        self.c3, = self.ax.plot([], [], 'x-')
        self.c4, = self.ax.plot([], [], 'x-')
        self.c5, = self.ax.plot([], [], 'k:')
        self.ax.set_xlim([-2, 2])
        self.ax.set_ylim([-2, 2])

    def calculateControl(self):

        self.tic()
        # Convert to frequency domain
        self.updatePoseData()

        # Get reference fourier series
        fser_r = self.ref.cn

        l = self.mapto

        origin = np.array(self.ref.originimage) / self.ref.scale
        obj_p =  self.obj_data[0] + 1j * self.obj_data[1] - (origin[0] + 1j * origin[1])
        distance = np.abs(self.p_fft[self.order] - obj_p) / .5
        l = min(1.1, max(.6, (distance+.1)**2))
        mapping = lambda z: l ** 2 * (z) ** l

        fser_r = mapping(fser_r)
        angle = np.angle(obj_p - self.p_fft[self.order])
        fser_r *= np.exp(1j * angle)

        # Calculate offset
        corr_x = np.abs(np.fft.ifft(np.fft.ifftshift(self.p_fft * np.flip(fser_r))))
        est = (corr_x - np.min(corr_x))
        corr_x = np.sum(est / np.sum(est) * np.arange(0, len(corr_x), 1))

        corr = -corr_x
        corr = np.exp(corr * self.corr_data_vect)

        if self.first_iter:
            self.corr = corr
            self.first_iter = False

        xi_r = fser_r * self.corr
        xi_r[self.order] = obj_p
        #xi_r = fser_r
        xi_rz = np.zeros(len(xi_r), dtype=complex)

        xi_rz[self.order] = - self.mapto * obj_p / np.abs(obj_p) * 3

        xi_r = np.concatenate((xi_r, xi_rz))


        # Calculate positions
        xi = np.concatenate((self.p_fft, self.v_fft))
        xi_a = np.concatenate((xi, xi_r))

        b = self.A_g_inv @ self.F @ xi_a
        h = np.hstack((np.real(b), np.imag(b)))
        q = 2 * h @ self.E

        out = cvxopt_solve_qp(self.EE, q, self.Aineq, self.bineq)

        u = np.block([np.eye((self.order * 2 + 1) * self.N), 1j*np.eye((self.order * 2 + 1) * self.N)]) @ out

        if self.N > 1:
            u = np.block([np.eye((self.order * 2 + 1)),
                          np.zeros(((self.order * 2 + 1), (self.order * 2 + 1) * (self.N - 1)))]) @ u

        idx = np.round(self.t * (self.numBots * 100 - 1) / self.t[-1], 0).astype(int)



        tmp = ifft_n(u, self.numBots*100)[idx]
        ux = np.real(tmp)
        uy = np.imag(tmp)



        u = np.vstack((ux, uy)).T * self.botMass

        print('Max control action: ' + str(np.round(np.max(u), 6)))


        if u is not None:
            return u
        else:
            return np.ones((self.numBots, 2))
