from regelum.system import System
from regelum.utils import rg
from regelum import callback
from regelum.system import CartPole


class MyCartPole(CartPole): # MyCartPoleFault
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

        mass_cart, mass_pole, grav_const, length_pole = (
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
        
        noise_theta_dot = inputs[1]
        noise_x_dot = inputs[2]
        noise_omega_dot = inputs[3]
        noise_vel_dot = inputs[4]
        
        Dstate[0] = omega + noise_theta_dot # derivative of \vartheta 
        Dstate[1] =  vel + noise_x_dot # derivative of x
        Dstate[2] = (grav_const * sin_theta * (mass_pole + mass_cart) - \
        cos_theta * (Force + mass_pole * length_pole * sin_theta * omega**2)) / (4 * \
        length_pole / 3 * (mass_cart + mass_pole) - length_pole * mass_pole * \
        cos_theta ** 2) + noise_omega_dot # derivative of \omega
        Dstate[3] = (Force + mass_pole * length_pole * (omega ** 2 * sin_theta - Dstate[2] * \
        cos_theta)) / (mass_pole + mass_cart) + noise_vel_dot # derivative of v_x

        return Dstate

    def _get_observation(self, time, state, inputs):
        return state