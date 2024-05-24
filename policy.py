from numpy.core.multiarray import array as array
from regelum.policy import Policy
import numpy as np
import random 
from scipy.special import expit
from src.system import InvertedPendulum, InvertedPendulumWithFriction, MyCartPole, MyCartPoleFault


def soft_switch(signal1, signal2, gate, loc=np.cos(np.pi / 4), scale=10):

    # Soft switch coefficient
    switch_coeff = expit((gate - loc) * scale)

    return (1 - switch_coeff) * signal1 + switch_coeff * signal2


def pd_based_on_sin(observation, pd_coefs=[20, 10]):
    return -pd_coefs[0] * np.sin(observation[0, 0]) - pd_coefs[1] * observation[0, 1]

def pd_task2(observation, pd_coefs=[100, 10, 20, 10]):  #   1000, 200, 600, 800  
    theta = observation[0, 0]
    x = observation[0, 1]
    omega = observation[0, 2]
    x_dot = observation[0, 3]
    theta = np.arctan2(np.sin(theta), np.cos(theta))
    x_clipped = np.clip(x, -1.0, 1.0)
    x_dot_clipped = np.clip(x_dot, -1.0, 1.0)

    return ( np.sin(theta) * pd_coefs[0]
            + x_clipped * pd_coefs[1]
            + omega * pd_coefs[2]
            + x_dot_clipped * pd_coefs[3])


class InvPendulumPolicyPD(Policy):
    def __init__(self, pd_coefs: np.ndarray, action_min: float, action_max: float):
        super().__init__()

        self.pid_coefs = np.array(pd_coefs).reshape(1, -1)
        self.action_min = action_min
        self.action_max = action_max

    def get_action(self, observation: np.ndarray):
        action = np.clip(
            (self.pid_coefs * observation).sum(),
            self.action_min,
            self.action_max,
        )
        return np.array([[action]])


class InvertedPendulumEnergyBased(Policy):
    def __init__(self, gain: float, action_min: float, action_max: float):
        super().__init__()
        self.gain = gain
        self.action_min = action_min
        self.action_max = action_max

    def get_action(self, observation: np.ndarray) -> np.ndarray:

        params = InvertedPendulum._parameters
        m, g, length = params["m"], params["g"], params["l"]

        theta = observation[0, 0]
        theta_vel = observation[0, 1]

        energy_total = (
            m * g * length * (np.cos(theta) - 1) + 0.5 * m * length**2 * theta_vel**2
        )
        energy_control_action = -self.gain * np.sign(theta_vel * energy_total)

        return np.array(
            [
                [
                    np.clip(
                        soft_switch(
                            signal1=energy_control_action,
                            signal2=pd_based_on_sin(observation),
                            gate=np.cos(theta),
                        ),
                        self.action_min,
                        self.action_max,
                    )
                ]
            ]
        )

class Fault(Policy):
    def __init__(self, action_min: float, action_max: float, sampling_time: float, learning_rate: float):
        super().__init__()

        # Add parameters
        # noise term for observation
        # noise term for action

        self.q_hat =0
        self.learning_rate = learning_rate
        self.dt  = sampling_time

        # for cartpole: 
        self.lamb = 1.0
        self.k = 1.0 

        # actions
        self.action_max = action_max
        self.action_min = action_min


    def get_action(self, observation: np.ndarray) -> np.ndarray:
        params = MyCartPole._parameters
        m_c, m_p, g, l = (
            params["m_c"],
            params["m_p"],
            params["g"],
            params["l"],
        )

        theta = observation[0, 0]
        omega = observation[0, 2]
        vel = observation[0, 3]
        sin_theta = np.sin(theta)
        cos_theta = np.cos(theta)

        ########################
        # YOUR CODE GOES HERE # Energy - based old 
        energy_total = (4/6 * m_p * l**2 * omega**2 + m_p*g*l*(cos_theta - 1))
        
        term1 = (m_c + m_p) * self.k * (energy_total * omega * cos_theta - self.lamb*vel)

        term2 = - m_p * l * omega**2 * sin_theta

        term3 = 3/4 * m_p * g * sin_theta * cos_theta

        term4 = - 3/4 * self.k * (energy_total * omega * cos_theta - self.lamb*vel) * m_p * cos_theta**2

        g_dagger_omega =  -cos_theta / (4 * l/3*(m_c + m_p) - l*m_p*cos_theta**2)

        g_dagger_vel = 1/(m_c+m_p)

        g_dag = np.array([0,0,g_dagger_omega,g_dagger_vel])

        grad_L = np.array([-sin_theta*energy_total*m_p*g*l, 
                            0,
                            energy_total*4/3 * l**2 * omega,
                            m_p*l*self.lamb * vel])
        
        q_hat_dot = self.learning_rate * grad_L

        self.q_hat += q_hat_dot* self.dt

        action = term1 + term2 + term3 + term4 # - g_dag @ self.q_hat


        act = np.clip(
                        # soft_switch(
                        #     signal1=action,
                        #     signal2=pd_from_task2(observation),
                        #     gate=np.cos(theta),
                        # ),
                        action,
                        self.action_min,
                        self.action_max,
                    )
        # YOUR CODE ENDS HERE #
        #######################
        return np.array(
            [[
                
                    act,
                    np.random.normal()
                
            ]]
        ) 

            
class InvPendulumEnergyBasedFrictionCompensation(Policy):

    def __init__(self, gain: float, action_min: float, action_max: float):
        super().__init__()
        self.gain = gain
        self.action_min = action_min
        self.action_max = action_max

    def get_action(self, observation: np.ndarray) -> np.ndarray:

        params = InvertedPendulumWithFriction._parameters
        m, g, length, friction_coef = params["m"], params["g"], params["l"], params["c"]

        theta = observation[0, 0]
        theta_vel = observation[0, 1]

        energy_total = (
            m * g * length * (np.cos(theta) - 1) + 0.5 * m * length**2 * theta_vel**2
        )
        energy_control_action = -self.gain * np.sign(
            theta_vel * energy_total
        ) + friction_coef * m * length * theta_vel * np.abs(theta_vel)

        return np.array(
            [
                [
                    np.clip(
                        soft_switch(
                            signal1=energy_control_action,
                            signal2=pd_based_on_sin(observation),
                            gate=np.cos(theta),
                        ),
                        self.action_min,
                        self.action_max,
                    )
                ]
            ]
        )


class InvPendulumEnergyBasedFrictionAdaptive(Policy):

    def __init__(
        self,
        gain: float,
        action_min: float,
        action_max: float,
        sampling_time: float,
        gain_adaptive: float,
        friction_coef_est_init: float = 0,
    ):
        super().__init__()
        self.gain = gain
        self.action_min = action_min
        self.action_max = action_max
        self.friction_coef_est = friction_coef_est_init
        self.sampling_time = sampling_time
        self.gain_adaptive = gain_adaptive

    def get_action(self, observation: np.ndarray) -> np.ndarray:

        params = InvertedPendulumWithFriction._parameters
        m, g, length = params["m"], params["g"], params["l"]

        theta = observation[0, 0]
        theta_vel = observation[0, 1]

        energy_total = (
            m * g * length * (np.cos(theta) - 1) + 0.5 * m * length**2 * theta_vel**2
        )
        energy_control_action = -self.gain * np.sign(
            theta_vel * energy_total
        ) + self.friction_coef_est * m * length * theta_vel * np.abs(theta_vel)

        # Parameter adaptation using Euler scheme
        self.friction_coef_est += (
            -self.gain_adaptive
            * energy_total
            * m
            * length**2
            * np.abs(theta_vel) ** 3
            * self.sampling_time
        )

        return np.array(
            [
                [
                    np.clip(
                        soft_switch(
                            signal1=energy_control_action,
                            signal2=pd_based_on_sin(observation),
                            gate=np.cos(theta),
                        ),
                        self.action_min,
                        self.action_max,
                    )
                ]
            ]
        )


class CartPolePD(Policy):

    def __init__(self, action_min: float, action_max: float):
        super().__init__()
        self.action_min = action_min
        self.action_max = action_max

        #########################
        ## YOUR CODE GOES HERE ##

        ## Find the coefficients of the PD controller
        ## It should be a list of 4 elements
        self.pd_coefs = [100, 10, 20, 10]

        ## YOUR CODE ENDS HERE ##
        #########################

    def get_action(self, observation: np.ndarray) -> np.ndarray:
        theta = observation[0, 0]
        x = observation[0, 1]
        omega = observation[0, 2]
        x_dot = observation[0, 3]
        theta = np.arctan2(np.sin(theta), np.cos(theta))

        x_clipped = np.clip(x, -1.0, 1.0)
        x_dot_clipped = np.clip(x_dot, -1.0, 1.0)

        action = (
            np.sin(theta) * self.pd_coefs[0]
            + x_clipped * self.pd_coefs[1]
            + omega * self.pd_coefs[2]
            + x_dot_clipped * self.pd_coefs[3]
        )

        return np.array(
            [
                [
                    np.clip(
                        action,
                        self.action_min,
                        self.action_max,
                    )
                ]
            ]
        )


class CartPoleEnergyBased(Policy):

    def __init__(self, action_min: float, action_max: float):
        super().__init__()
        self.action_max = action_max
        self.action_min = action_min

        #########################
        ## YOUR CODE GOES HERE ##

        ## Define hyperparameters if needed

        self.lambd = 1.0
        self.k = 1.0

        ## YOUR CODE ENDS HERE ##
        #########################

    def get_action(self, observation: np.ndarray) -> np.ndarray:
        params = MyCartPoleFault._parameters
        m_c, m_p, g, l = (
            params["m_c"],
            params["m_p"],
            params["g"],
            params["l"],
        )
        theta = observation[0, 0]
        omega = observation[0, 2]
        vel = observation[0, 3]
        sin_theta = np.sin(theta)
        cos_theta = np.cos(theta)

        ########################
        # YOUR CODE GOES HERE #

        E_p = (4/6 * m_p * l**2 * omega**2 + m_p*g*l*(cos_theta - 1))
        
        a1 = (m_c + m_p) * self.k * (E_p * omega * cos_theta - self.lambd*vel)

        a2 = - m_p * l * omega**2 * sin_theta

        a3 = 3/4 * m_p * g * sin_theta * cos_theta

        a4 = - 3/4 * self.k * (E_p * omega * cos_theta - self.lambd*vel) * m_p * cos_theta**2 

        action = a1 + a2 + a3 + a4

        # YOUR CODE ENDS HERE #
        #######################

        return np.array(
            [
                [
                     np.clip(
                    #     action,
                    #     self.action_min,
                    #     self.action_max,
                    soft_switch(
                            signal1=action,
                            signal2=pd_task2(observation),
                            gate=np.cos(theta),
                        ),
                        self.action_min,
                        self.action_max,
                    )
                ]
            ]
        )
