from numpy.core.multiarray import array as array
from regelum.policy import Policy
import numpy as np
from scipy.special import expit
from src.system import MyCartPole


def soft_switch(signal1, signal2, gate, loc=np.cos(np.pi / 4), scale=10):
    switch_coeff = expit((gate - loc) * scale)
    return (1 - switch_coeff) * signal1 + switch_coeff * signal2

def pd_from_task2(observation, pd_coefs=[100, 10, 25, 15]):
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

class CartPoleEnergyBased(Policy): # CartPoleEnergyBasedFault
    def __init__(self, action_min: float, action_max: float):
        super().__init__()
        self.action_max = action_max
        self.action_min = action_min
        self.lambd = 1.0
        self.k = 1.0
        self.learning_rate = 1.0
        self.q_hat = 0
        self.dt = 0.01


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


        energy_total = (4/6 * m_p * l**2 * omega**2 + m_p*g*l*(cos_theta - 1))
        
        term1 = (m_c + m_p) * self.k * (energy_total * omega * cos_theta - self.lambd*vel)

        term2 = - m_p * l * omega**2 * sin_theta

        term3 = 3/4 * m_p * g * sin_theta * cos_theta

        term4 = - 3/4 * self.k * (energy_total * omega * cos_theta - self.lambd*vel) * m_p * cos_theta**2

        g_dagger_omega = -cos_theta / (4 * l/3*(m_c + m_p) - l*m_p*cos_theta**2)

        g_dagger_vel = 1/(m_c+m_p - 3/4*m_p*cos_theta**2) 

        g_dag = np.linalg.pinv(np.array([[0],[0],[g_dagger_omega],[g_dagger_vel]])).reshape(4)

        grad_L = np.array([-sin_theta*energy_total*m_p*g*l, 
                            0,
                            energy_total*4/3 * l**2 * omega,
                            m_p*l*self.lambd * vel])
        
        q_hat_dot = self.learning_rate * grad_L

        self.q_hat += q_hat_dot* self.dt

        action = term1 + term2 + term3 + term4 - 100*g_dag @ self.q_hat

        act = np.clip(
                    soft_switch(
                            signal1=action,
                            signal2=pd_from_task2(observation),
                            gate=np.cos(theta),
                        ),
                        self.action_min,
                        self.action_max,
                    )
        return np.array(
            [
                [
                    act,
                    np.random.normal(0, 1),
                    np.random.normal(0, 2),
                    np.random.normal(0, 2),
                    np.random.normal(0, 1)
                ]
            ]
        )
