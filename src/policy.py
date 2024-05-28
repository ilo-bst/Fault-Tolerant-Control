from regelum.policy import Policy
import numpy as np
from scipy.special import expit
from src.system import MyCartPoleFault


def soft_switch(signal1, signal2, gate, loc=np.cos(np.pi / 4), scale=10):
    switch_coeff = expit((gate - loc) * scale)
    return (1 - switch_coeff) * signal1 + switch_coeff * signal2


def pd_control(observation, pd_coefs=[100, 10, 25, 15]):
    theta = observation[0, 0]
    x = observation[0, 1]
    omega = observation[0, 2]
    x_dot = observation[0, 3]
    theta = np.arctan2(np.sin(theta), np.cos(theta))
    x_clipped = np.clip(x, -1.0, 1.0)
    x_dot_clipped = np.clip(x_dot, -1.0, 1.0)
    return (np.sin(theta) * pd_coefs[0]
            + x_clipped * pd_coefs[1]
            + omega * pd_coefs[2]
            + x_dot_clipped * pd_coefs[3])


class CartPoleFault(Policy):
    def __init__(self, action_min: float, action_max: float):
        super().__init__()
        self.action_max = action_max
        self.action_min = action_min
        self.velocity_gain = 1.0
        self.energy_gain = 1.0
        self.learning_rate = 100
        self.q_hat = 0
        self.dt = 0.01

    def get_action(self, observation: np.ndarray) -> np.ndarray:
        params = MyCartPoleFault._parameters
        mass_cart, mass_pole, grav_const, length_pole = (
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

        energy_total = 2 / 3 * mass_pole * length_pole ** 2 * omega ** 2\
            + mass_pole * grav_const * length_pole * (cos_theta - 1)
        p1 = (mass_cart + mass_pole) * self.energy_gain * (
            energy_total * omega * cos_theta - self.velocity_gain * vel
        )
        p2 = - mass_pole * length_pole * omega**2 * sin_theta
        p3 = 3 / 4 * mass_pole * grav_const * sin_theta * cos_theta
        p4 = -3 / 4 * self.energy_gain * (
            energy_total * omega * cos_theta - self.velocity_gain * vel
        ) * mass_pole * cos_theta**2
        g_dagger_omega = -cos_theta / (4 * length_pole / 3 * (
            mass_cart + mass_pole
        ) - length_pole * mass_pole * cos_theta ** 2)
        g_dagger_vel = 1 / (
            mass_cart + mass_pole
        )
        g_dag = np.linalg.pinv(np.array([
            [0],
            [0],
            [g_dagger_omega],
            [g_dagger_vel]])
        ).reshape(4)
        grad_L = np.array([
            -sin_theta * energy_total * mass_pole * grav_const * length_pole,
            0,
            energy_total * 4 / 3 * length_pole ** 2 * omega,
            mass_pole * length_pole * self.velocity_gain * vel]
        )
        q_hat_dot = self.learning_rate * grad_L
        self.q_hat += q_hat_dot * self.dt
        u = p1 + p2 + p3 + p4 - g_dag @ self.q_hat

        act = np.clip(
                    soft_switch(
                            signal1=u,
                            signal2=pd_control(observation),
                            gate=np.cos(theta),
                        ), self.action_min, self.action_max,
                    )

        return np.array(
            [
                [
                    act,
                    np.random.normal(0, 1),
                    np.random.normal(0, 1),
                    np.random.normal(0, 1),
                    np.random.normal(0, 1)
                ]
            ]
        )


class CartPoleEnergyBased(Policy):
    def __init__(self, action_min: float, action_max: float):
        super().__init__()
        self.action_max = action_max
        self.action_min = action_min
        self.energy_gain = 1.0
        self.velocity_gain = 1.0

    def get_action(self, observation: np.ndarray) -> np.ndarray:
        params = MyCartPoleFault._parameters
        mass_cart, mass_pole, grav_const, length_pole = (
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
        energy_total = 2 / 3 * mass_pole * length_pole ** 2 * omega ** 2\
            + mass_pole * grav_const * length_pole * (cos_theta - 1)
        p1 = (mass_cart + mass_pole) * self.energy_gain * (
            energy_total * omega * cos_theta - self.velocity_gain * vel
        )
        p2 = - mass_pole * length_pole * omega**2 * sin_theta
        p3 = 3 / 4 * mass_pole * grav_const * sin_theta * cos_theta
        p4 = -3 / 4 * self.energy_gain * (
            energy_total * omega * cos_theta - self.velocity_gain * vel
        ) * mass_pole * cos_theta**2
        u = p1 + p2 + p3 + p4

                return np.array(
            [
                [
                    np.clip(
                        soft_switch(
                            signal1=u,
                            signal2=pd_control(observation),
                            gate=np.cos(theta),
                        ),
                        self.action_min,
                        self.action_max,
                    ),
                    np.random.normal(0, 1),
                    np.random.normal(0, 1),
                    np.random.normal(0, 1),
                    np.random.normal(0, 1)
                ]
            ]
        )
