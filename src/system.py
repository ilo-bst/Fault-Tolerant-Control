from regelum.system import System
from regelum.utils import rg
from regelum import callback
from regelum.system import InvertedPendulum, CartPole
import random 

class InvertedPendulum(InvertedPendulum):
    _parameters = {"m": 1, "g": 9.8, "l": 1.0}

    def _compute_state_dynamics(self, time, state, inputs):
        Dstate = rg.zeros(
            self.dim_state,
            prototype=(state, inputs),
        )

        m, g, l = (
            self._parameters["m"],
            self._parameters["g"],
            self._parameters["l"],
        )

        Dstate[0] = state[1]
        Dstate[1] = g / l * rg.sin(state[0]) + inputs[0] / (m * l**2)

        return Dstate


class InvertedPendulumWithFriction(InvertedPendulum):
    _parameters = {"m": 1, "g": 9.8, "l": 1.0, "c": 0.08}

    def _compute_state_dynamics(self, time, state, inputs):
        Dstate = rg.zeros(
            self.dim_state,
            prototype=(state, inputs),
        )

        m, g, l, friction_coef = (
            self._parameters["m"],
            self._parameters["g"],
            self._parameters["l"],
            self._parameters["c"],
        )

        Dstate[0] = state[1]
        Dstate[1] = (
            g / l * rg.sin(state[0])
            + inputs[0] / (m * l**2)
            - friction_coef * state[1] ** 2 * rg.sign(state[1])
        )

        return Dstate


class MyCartPole(CartPole):
    _dim_observation = 4
    _dim_state = 4
    _parameters = {"m_c": 0.1, "m_p": 2.0, "g": 9.81, "l": 0.5}
    _action_bounds = [[-300.0, 300.0]]

    def _compute_state_dynamics(self, time, state, inputs):
        Dstate = rg.zeros(
            self.dim_state,
            prototype=(state, inputs),
        )

        m_c, m_p, g, l = (
            self.parameters["m_c"],
            self.parameters["m_p"],
            self.parameters["g"],
            self.parameters["l"],
        )

        theta = state[0]
        theta_dot = state[2]
        x_dot = state[3]
        Force = inputs[0]
        sin_theta = rg.sin(theta)
        cos_theta = rg.cos(theta)

        #########################
        ## YOUR CODE GOES HERE ##

        a = g*sin_theta*(m_c + m_p) - cos_theta*(Force + m_p*l*theta_dot**2*sin_theta)
        b = 4/3*l*(m_c + m_p) - l*m_p*cos_theta**2
        Dstate[0] = theta_dot
        Dstate[1] = x_dot
        Dstate[2] = a/b
        c = Force + m_p*l*(theta_dot**2*sin_theta - Dstate[2]*cos_theta)
        d = m_c + m_p
        Dstate[3] = c/d

        ## YOUR CODE ENDS HERE ##
        #########################

        return Dstate

    def _get_observation(self, time, state, inputs):
        return state

class MyCartPoleFault(CartPole):
    _dim_state = 4
    _dim_inputs = 5
    _dim_observation = 4
    _parameters = {"m_c": 0.1, "m_p": 2.0, "g": 9.81, "l": 0.5}
    _observation_naming = _state_naming = [
        "angle [rad]",
        "x [m]",
        "angle_dot [rad/s]",
        "x_dot [m/s]",
    ]
    _inputs_naming = ["force [kg*m/s^2]", "noise_angle","noise_x","noise_omega","noise_velocity"]
    _action_bounds = [[-50.0, 50.0], [-10, 10], [-10, 10], [-10, 10], [-10, 10]]
    

    def _compute_state_dynamics(self, time, state, inputs):
        Dstate = rg.zeros(
            self.dim_state,
            prototype=(state, inputs),
        )

        m_c, m_p, g, l = (
            self.parameters["m_c"],
            self.parameters["m_p"],
            self.parameters["g"],
            self.parameters["l"],
        )

        theta = state[0]
        x = state[1]
        omega = state[2]
        vel = state[3]
        
        
        
        sin_theta = rg.sin(theta)
        cos_theta = rg.cos(theta)
        Force = inputs[0]
        #########################
        ## YOUR CODE GOES HERE ##
        
        noise_theta_dot = inputs[1]
        noise_x_dot = inputs[2]
        noise_omega_dot = inputs[3]
        noise_vel_dot = inputs[4]
        
        

        Dstate[0] = omega + noise_theta_dot # derivative of \vartheta 
        Dstate[1] =  vel + noise_x_dot # derivative of x
        Dstate[2] = ( g * sin_theta * (m_p + m_c) - cos_theta * (Force + m_p * l * sin_theta * omega**2 ) ) / (4*l/3 * (m_c + m_p) - l * m_p * cos_theta**2) + noise_omega_dot # derivative of \omega
        Dstate[3] = ( Force + m_p * l * (omega**2 * sin_theta - Dstate[2] * cos_theta)) / (m_p + m_c) + noise_vel_dot # derivative of v_x



        ## YOUR CODE ENDS HERE ##
        #########################

        return Dstate

    def _get_observation(self, time, state, inputs):
        return state
