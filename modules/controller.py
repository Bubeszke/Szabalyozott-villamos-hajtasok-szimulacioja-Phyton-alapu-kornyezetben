import numpy as np
from gekko import GEKKO
import math



class GenericController:
    def __init__(self, environment, tau):
        self.environment = environment
        self.state_names = environment.state_names
        self.motor_parameters = environment.physical_system.electrical_motor.motor_parameter

        # Store tau
        self.tau = tau

        # Store the labels of relevant references
        self.ref_idx_i_d = 'D'
        self.ref_idx_i_q = 'Q'

        # Store the indices of relevant state variables
        self.u_a_idx = self.state_names.index('u_a')
        self.u_b_idx = self.state_names.index('u_b')
        self.u_c_idx = self.state_names.index('u_c')
        self.u_sq_idx = self.state_names.index('u_sq')
        self.u_sd_idx = self.state_names.index('u_sd')
        self.i_sd_idx = self.state_names.index('i_sd')
        self.i_sq_idx = self.state_names.index('i_sq')
        self.omega_idx = self.state_names.index('omega')
        self.epsilon_idx = self.state_names.index('epsilon')

        # Store the motor parameters
        self.limits = environment.physical_system.limits
        self.l_q = environment.physical_system.electrical_motor.motor_parameter['l_q']
        self.l_d = environment.physical_system.electrical_motor.motor_parameter['l_d']
        self.psi_p = environment.physical_system.electrical_motor.motor_parameter['psi_p']
        self.r_s = environment.physical_system.electrical_motor.motor_parameter['r_s']
        self.p = environment.physical_system.electrical_motor.motor_parameter['p']

    def control(self, state, reference):
        raise NotImplementedError("Subclasses should implement this method")

class FOC_Controller(GenericController):
    def __init__(self, environment, tau):
        super().__init__(environment, tau)

    def clarke_transformation(self, u_a, u_b, u_c):
        """ Converts three-phase to two-phase stationary coordinates (α-β) """
        u_alpha = u_a - 0.5 * (u_b + u_c)
        u_beta = (np.sqrt(3) / 2) * (u_b - u_c)
        return u_alpha, u_beta

    def park_transformation(self, u_alpha, u_beta, theta):
        """ Transforms stationary coordinates (α-β) to rotating coordinates (d-q) using the angle theta """
        u_d = u_alpha * np.cos(theta) + u_beta * np.sin(theta)
        u_q = -u_alpha * np.sin(theta) + u_beta * np.cos(theta)
        return u_d, u_q

    def inverse_park_transformation(self, u_d, u_q, theta):
        """ Transforms rotating coordinates (d-q) back to stationary coordinates (α-β) """
        u_alpha = u_d * np.cos(theta) - u_q * np.sin(theta)
        u_beta = u_d * np.sin(theta) + u_q * np.cos(theta)
        return u_alpha, u_beta

    def inverse_clarke_transformation(self, u_alpha, u_beta):
        """ Converts two-phase stationary coordinates (α-β) back to three-phase coordinates (a, b, c) """
        u_a = u_alpha
        u_b = -0.5 * u_alpha + (np.sqrt(3) / 2) * u_beta
        u_c = -0.5 * u_alpha - (np.sqrt(3) / 2) * u_beta
        return u_a, u_b, u_c

    def control(self, state, reference):
        raise NotImplementedError("Subclasses should implement this method")

class PID_Controller(FOC_Controller):
    def __init__(self, environment, tau):
        super().__init__(environment, tau)
        self.tau = tau
        self.pole_pairs = self.motor_parameters.get("p")
        self.setup_pid(tau)

    def setup_pid(self, tau):
        # Initialize PID controller variables with validation
        self.previous_error_d = 0
        self.previous_error_q = 0
        self.integral_d = 0
        self.integral_q = 0

        self.K_P_d = 0.125
        self.K_I_d = 2500000 * tau
        self.K_D_d = 0.0000175
        
        self.K_P_q = 0.2
        self.K_I_q = 2500000 * tau
        self.K_D_q = 0.0000175

    def control(self, state, reference):
        epsilon = state[self.epsilon_idx]
        i_sd = state[self.i_sd_idx]
        i_sq = state[self.i_sq_idx]

        # Calculate errors
        error_d = reference['D'] - i_sd
        error_q = reference['Q'] - i_sq

        # Integral term
        self.integral_d += error_d * self.tau
        self.integral_q += error_q * self.tau

        # Derivative term
        derivative_d = (error_d - self.previous_error_d) / self.tau
        derivative_q = (error_q - self.previous_error_q) / self.tau

        # PID control
        actuating_d = self.K_P_d * error_d + self.K_I_d * self.integral_d + self.K_D_d * derivative_d
        actuating_q = self.K_P_q * error_q + self.K_I_q * self.integral_q + self.K_D_q * derivative_q

        # Update previous errors
        self.previous_error_d = error_d
        self.previous_error_q = error_q

        # Inverze Park Transformation
        actuating_alpha, actuating_beta = self.inverse_park_transformation(actuating_d, actuating_q, epsilon)

        # Inverze Clarke Transformation
        actuating_a, actuating_b, actuating_c = self.inverse_clarke_transformation(actuating_alpha, actuating_beta)

        action = [actuating_a, actuating_b, actuating_c] 

        return action

    def info(self):
        print(f"MCP Controller Info:\nEnvironment: {self.environment}\nPole Pairs: {self.pole_pairs}\nIndices: {self.u_a_idx}, {self.u_b_idx}, {self.u_c_idx}")


class MPC_Controller(FOC_Controller):
    def __init__(self, environment, tau, prediction_horizon=5):
        super().__init__(environment, tau)
        self.prediction_horizon = prediction_horizon

        t32 = environment.physical_system.electrical_motor.t_32
        q = environment.physical_system.electrical_motor.q
        self._backward_transformation = (lambda quantities, eps: t32(q(quantities[::-1], eps)))

    def ratio_det(self, state, reference):
        #omega = state[self.omega_idx] * self.limits[self.omega_idx] * 30 / math.pi
        omega = state[self.omega_idx] * self.limits[self.omega_idx] # rad/s
        omega_e = omega * self.p 
        epsilon = state[self.epsilon_idx] # * self.limits[self.epsilon_idx] 
        ud_prev = state[self.u_sd_idx] * self.limits[self.u_sd_idx]
        uq_prev = state[self.u_sq_idx] * self.limits[self.u_sq_idx]
        id_prev = state[self.i_sd_idx] * self.limits[self.i_sd_idx]
        iq_prev = state[self.i_sq_idx] * self.limits[self.i_sq_idx]

        print(omega)
        print(epsilon)
        u_a = state[self.u_a_idx]
        u_b = state[self.u_b_idx]
        u_c = state[self.u_c_idx]

        u_a /= self.limits[self.u_a_idx]
        u_b /= self.limits[self.u_b_idx]
        u_c /= self.limits[self.u_c_idx]

        action = [0, 0, 0]
        return action

    def control(self, state, reference):
        # Transform everything into SI
        omega = state[self.omega_idx] * self.limits[self.omega_idx] # rad/s
        omega_e = omega * self.p
        epsilon_prev = state[self.epsilon_idx] * self.limits[self.epsilon_idx]
        ud_prev = state[self.u_sd_idx] * self.limits[self.u_sd_idx]
        uq_prev = state[self.u_sq_idx] * self.limits[self.u_sq_idx]
        id_prev = state[self.i_sd_idx] * self.limits[self.i_sd_idx]
        iq_prev = state[self.i_sq_idx] * self.limits[self.i_sq_idx]

        refrence_D = []
        refrence_Q = []
        epsilons = []

        lim_a_up = []
        lim_a_low = []
        for i in range(self.prediction_horizon): # Load the data into the future dimensions
            refrence_D.append(reference["D"] * self.limits[self.i_sd_idx])
            refrence_Q.append(reference["Q"] * self.limits[self.i_sq_idx])
            epsilons.append(epsilon_prev + (i-1) * self.tau * omega_e) # i-1 bc the u_d, u_q, ... are from one step befora

            lim_a_up.append(2 * self.limits[self.u_a_idx])
            lim_a_low.append(-2 * self.limits[self.u_a_idx])
        # Initialize the GEKKO model
        m = GEKKO(remote=False)

        # Time setup
        m.time = np.linspace(self.tau, self.tau * self.prediction_horizon, self.prediction_horizon)

        # Control inputs
        u_d = m.MV(value=ud_prev)
        u_q = m.MV(value=uq_prev)
        u_d.STATUS = 1
        u_q.STATUS = 1
    
        # Tunable parameters
        # u_d.DCOST = 1  # Penalty for changes in u_d 
        # u_q.DCOST = 1e-1  # Penalty for changes in u_q

        # State variables
        i_d = m.SV(value=id_prev, lb=-self.limits[self.i_sd_idx], ub=self.limits[self.i_sd_idx])
        i_q = m.SV(value=iq_prev, lb=-self.limits[self.i_sq_idx], ub=self.limits[self.i_sq_idx])

        epsilon = m.Param(value=epsilons)

        # reference trajectory
        traj_d = m.Param(value=refrence_D)
        traj_q = m.Param(value=refrence_Q)


        # defenition of the constants
        omega_e = m.Const(value=omega_e)
        psi_p = m.Const(value=self.psi_p)
        r_s = m.Const(value=self.r_s)
        l_d = m.Const(value=self.l_d)
        l_q = m.Const(value=self.l_q)


        # control error
        e_d = m.CV()
        e_q = m.CV()
        e_d.STATUS = 1
        e_q.STATUS = 1

        # System dynamics
        m.Equations([
            l_d * i_d.dt() == u_d - r_s * i_d + omega_e * l_q * i_q,
            l_q * i_q.dt() == u_q - r_s * i_q - omega_e * l_d * i_d - omega_e * psi_p
        ])

        # voltage limitations
        u_a_lim_up = m.Param(value=lim_a_up)
        u_a_lim_low = m.Param(value=lim_a_low)
        m.Equation(u_a_lim_up >= 3/2 * m.cos(epsilon) * u_d - 3/2 * m.sin(epsilon) * u_q - math.sqrt(3)/2 * m.sin(epsilon) * u_d - math.sqrt(3)/2 * m.cos(epsilon) * u_q)
        m.Equation(u_a_lim_low <= 3 / 2 * m.cos(epsilon) * u_d - 3 / 2 * m.sin(epsilon) * u_q - math.sqrt(3) / 2 * m.sin(epsilon) * u_d - math.sqrt(3) / 2 * m.cos(epsilon) * u_q)
        m.Equation(u_a_lim_up >= math.sqrt(3) * m.sin(epsilon) * u_d + math.sqrt(3) * m.cos(epsilon) * u_q)
        m.Equation(u_a_lim_low <= math.sqrt(3) * m.sin(epsilon) * u_d + math.sqrt(3) * m.cos(epsilon) * u_q)
        m.Equation(u_a_lim_up >= -3 / 2 * m.cos(epsilon) * u_d + 3 / 2 * m.sin(epsilon) * u_q - math.sqrt(3) / 2 * m.sin(epsilon) * u_d - math.sqrt(3) / 2 * m.cos(epsilon) * u_q)
        m.Equation(u_a_lim_low <= -3 / 2 * m.cos(epsilon) * u_d + 3 / 2 * m.sin(epsilon) * u_q - math.sqrt(3) / 2 * m.sin(epsilon) * u_d - math.sqrt(3) / 2 * m.cos(epsilon) * u_q)

        # cost function
        m.Equations([e_d == (i_d - traj_d), e_q == (i_q - traj_q)])


        # Objects to minimize
        #m.Obj(e_d**2 + e_q**2)
        m.Obj(e_d)
        m.Obj(e_q)

        # MPC configuration
        m.options.IMODE = 6  # MPC mode

        #might be unnecessary
        m.options.CV_TYPE = 2
        m.options.solver = 3
        m.options.WEB = 0
        m.options.NODES = 2

        # Solve the optimization
        m.solve(disp=False)

        # Transformations might be needed here to compute phase voltages or other necessary forms
        #u_alpha, u_beta = self.inverse_park_transformation(u_d.NEWVAL, u_q.NEWVAL, epsilon_prev)
        #u_a, u_b, u_c = self.inverse_clarke_transformation(u_alpha, u_beta)
        # additional voltage limitation
        u_a, u_b, u_c = self._backward_transformation((u_q.NEWVAL, u_d.NEWVAL), epsilon_prev)
        u_max = max(np.absolute(u_a - u_b), np.absolute(u_b - u_c), np.absolute(u_c - u_a))
        if u_max >= 2 * self.limits[self.u_a_idx]:
            u_a = u_a / u_max * 2 * self.limits[self.u_a_idx]
            u_b = u_b / u_max * 2 * self.limits[self.u_a_idx]
            u_c = u_c / u_max * 2 * self.limits[self.u_a_idx]

        # Zero Point Shift
        u_0 = 0.5 * (max(u_a, u_b, u_c) + min(u_a, u_b, u_c))
        u_a -= u_0
        u_b -= u_0
        u_c -= u_0

        # Transform back from SI
        u_a /= self.limits[self.u_a_idx]
        u_b /= self.limits[self.u_b_idx]
        u_c /= self.limits[self.u_c_idx]

        action = [u_a, u_b, u_c]
        return action
