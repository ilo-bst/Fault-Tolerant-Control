from regelum.system import System
from regelum.utils import rg
from regelum import callback
from regelum.system import InvertedPendulum, CartPole
import random 


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
