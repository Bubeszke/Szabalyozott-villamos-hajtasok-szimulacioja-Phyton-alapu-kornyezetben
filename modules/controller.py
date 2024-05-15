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
    def __init__(self, environment, tau, prediction_horizon=10):
        super().__init__(environment, tau)
        self.prediction_horizon = prediction_horizon

    def control(self, state, reference):
        u_d_prev =  state[self.u_sd_idx]
        u_q_prev =  state[self.u_sq_idx]
        i_d_prev =  state[self.i_sd_idx]
        i_q_prev =  state[self.i_sq_idx]
        omega = state[self.omega_idx]

        # Initialize the GEKKO model
        m = GEKKO(remote=False)

        # Time setup
        m.time = np.linspace(self.tau, self.tau * self.prediction_horizon, self.prediction_horizon)

        # Control inputs
        u_d = m.MV(value=u_d_prev, lb=-10, ub=10)
        u_q = m.MV(value=u_q_prev, lb=-10, ub=10)
        u_d.STATUS = 1
        u_q.STATUS = 1
    
        # Tunable parameters
        u_d.DCOST = 1  # Penalty for changes in u_d 
        u_q.DCOST = 1e-1  # Penalty for changes in u_q

        # State variables
        i_d = m.Var(value=i_d_prev)
        i_q = m.Var(value=i_q_prev)

        # control error
        e_d = m.CV()
        e_q = m.CV()
        e_d.STATUS = 1
        e_q.STATUS = 1

        # System dynamics
        m.Equations([
            self.l_d * i_d.dt() == u_d - self.r_s * i_d + omega * self.l_q * i_q,
            self.l_q * i_q.dt() == u_q - self.r_s * i_q - omega * self.l_d * i_d - omega * self.psi_p
        ])


        traj_d = m.Param(value=reference[self.ref_idx_i_d])
        traj_q = m.Param(value=reference[self.ref_idx_i_q])

        # cost function
        m.Equations([e_d == (i_d - traj_d), e_q == (i_q - traj_q)])

        # Objects to minimize
        m.Obj(e_d**2 + e_q**2)
        #m.Obj(e_q)

        # MPC configuration
        m.options.IMODE = 6  # MPC mode

        # Solve the optimization
        m.solve(disp=False)

        # Transformations might be needed here to compute phase voltages or other necessary forms
        actuating_alpha, actuating_beta = self.inverse_park_transformation(u_d.NEWVAL, u_q.NEWVAL, state[self.epsilon_idx])
        action = self.inverse_clarke_transformation(actuating_alpha, actuating_beta)

        return action
