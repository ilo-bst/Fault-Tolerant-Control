# Fault-Tolerant-Control

## Overview
This project aims to implement and evaluate fault-tolerant control strategies for dynamic systems. The goal is to ensure system reliability and robustness in the presence of faults.

- Course: Advanced Control Methods, Skoltech, 2024
- Team Members: [Ilona Basset](https://github.com/ilo-bst), [Mikhail Mishustin](https://github.com/mishkaaa31), [Ruslan Gladilov](https://github.com/RuslanGladilov)
- Final Presentation: [link](https://docs.google.com/presentation/d/1OndUG2DB0yXjvMWQS30-TukYlxjxPwfYRwaH3VeM5bQ/edit#slide=id.p)

---

## Table of Contents

- [Overview](#overview)
- [Problem Statement](#problem-statement)
- [Results](#results)
- [Run the Project](#run-the-project)
- [Other Section](#other-section)
- [Bibliography](#bibliography)

---

## Problem Statement

The need for stabilizing dynamic systems in the presence of faults exists. The objective is to implement and evaluate fault-tolerant control strategy for the Cartpole system.
Tasks to be done:
- develop a fault-tolerant control system;
- test the system under various fault conditions and noise levels;
- evaluate the impact of additional control terms on system performance.

The problem statement is as follows, there is a system of differential equations describing the dynamics of the system, the peculiarity of this system is that there is noise at each coordinate. We conducted the tests under the assumption that the noise is from a normal distribution.

![alt text](./gfx/system_equation.png)

---

## Results
We tried adding noise in different coordinates.
The result is the following:
The fault-tolerant control works by adding noise to any coordinates much faster than conventional energy-based controler.

Without fault-tolerant control:

![alt text](./gfx/image_without_fault.png)

Fault-tolerant control in use:

![alt text](./gfx/image_with_fault.png)

---

## Run the Project
### Requirements
List of prerequisites, dependencies, and environment setup necessary to run the project.
- Regelum 0.3.2
- Numpy 1.26.4
- Scipy 1.13.0


### Setup and Installation
Before installing the requirements, it is recommended to [create a virtual environment](https://github.com/OdinManiac/acm-2024-sem-1) for your project. And then run
```bash
git clone https://github.com/ilo-bst/Fault-Tolerant-Control.git
cd Fault-Tolerant-Control
pip install -r requirements.txt
code .
```


### Running the Code
To run fault-tolerant control:
```bash
python run.py policy=cartpole_fault initial_conditions=cartpole_swingup system=cartpole_fault --interactive --fps=3
```

To run basic control:
```bash
python run.py policy=cartpole_energy_based initial_conditions=cartpole_swingup system=cartpole_fault --interactive --fps=3
```
> **Note:**
>
> For the `--fps` parameter, you can select any suitable value to ensure a smooth experience (e.g., `--fps=2`, `--fps=10`, `--fps=20`, etc.).


## Bibliography/Links

- [CartPole](https://regelum.aidynamic.io/systems/cartpole/)
- [Fault-tolerant-control](https://gitflic.ru/project/aidynamicaction/classedu2024-advctrl/blob?file=lectures%2Flec-6%2FNotes_240416_184536.pdf&commit=67dd87ffbb6480eeee682b23db8588f1e584c7d8)
- [Instruction for creating virtual environment](https://github.com/OdinManiac/acm-2024-sem-1)
