# Project Title

## Overview
This section provides a brief introduction to the project conducted as part of the Advanced Control Methods course at Skoltech in 2024. It includes the fundamental objectives of the project, information about the team members, and a link to the final presentation.

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
This section delves into the specifics of the challenge tackled during the project.  It provides context, outlines the objectives, and discusses the significance of the problem.

This project aims to implement and evaluate fault-tolerant control strategies for dynamic systems. The goal is to ensure system reliability and robustness in the presence of faults.

### Subsection (if any)
Subsections may be added to further break down the problem, provide background information, or elaborate on specific aspects that are crucial to understanding the project's scope.

---

## Results
Detailed explanation of the findings, performance metrics, and outcomes of the project. This section may include graphs, tables, and other visual aids to support the results.

### Subsection (if any)
Subsections may be used to organize results into categories, discuss different algorithms or methods used, or compare various scenarios within the project.

---

## Run the Project
Step-by-step instructions on how to replicate the results obtained in this project. This should be clear enough for someone with basic knowledge of the tools used to follow.

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
python run.py policy=cartpole_energy_based initial_conditions=cartpole_swingup system=cartpole --interactive --fps=3
```

To run basic control:
```bash
python run.py policy=cartpole_energy_based initial_conditions=cartpole_swingup system=cartpole --interactive --fps=3
```
---

## Bibliography/Links
(If applicable) This section includes references to papers, articles, and other resources that informed the project's approach and methodology.

- [CartPole](https://regelum.aidynamic.io/systems/cartpole/)
- [Fault-tolerant-control](https://gitflic.ru/project/aidynamicaction/classedu2024-advctrl/blob?file=lectures%2Flec-6%2FNotes_240416_184536.pdf&commit=67dd87ffbb6480eeee682b23db8588f1e584c7d8)
- [Instruction for create virtual environment](https://github.com/OdinManiac/acm-2024-sem-1)








# Fault-Tolerant-Control

## Overview
This project aims to implement and evaluate fault-tolerant control strategies for dynamic systems. The goal is to ensure system reliability and robustness in the presence of faults.

## Project Structure

- `policy.py`: Contains the main control algorithms and equations.
- `system.py`: Models the system dynamics and simulates various scenarios.
- `README.md`: This document.

## Getting Started

### Prerequisites
- Python 3.x
- Required libraries: numpy, scipy, matplotlib

### Installation
Clone the repository:
```
git clone https://github.com/yourusername/Fault-Tolerant-Control.git
cd fault-tolerant-control
```

Install dependencies:
```
pip install -r requirements.txt
```

### Usage
Run the main simulation script:
```
python run.py policy=cartpole_energy_based initial_conditions=cartpole_swingup system=cartpole --interactive --fps=3
```

### Example
To simulate the system with noise:
```python
# Example code snippet
import system
system.simulate_with_noise()
```

# Project Details
### Objectives
- Develop a fault-tolerant control system.
- Test the system under various fault conditions and noise levels.
- Evaluate the impact of additional control terms on system performance.
  
### Approach
- System Modeling: Define the system equations and control laws.
- Fault Detection: Implement methods to detect and isolate faults.
- Control Reconfiguration: Adjust control strategies to maintain performance despite faults.
- Parameter Tuning: Optimize hyperparameters for robustness.

### Current Status
- Basic control system implemented.
- Preliminary tests show that the additional term improve performance under noise. Cart does not go left as much as without that term. The angle stabilization works pretty good in both cases.

  
### Future Work
- Refine fault detection and isolation algorithms.
- Explore different control reconfiguration strategies.
- Test on a simpler system like Inverted Pendulum.
  
### Contributors
Ilona Basset,
Mikhail Mishustin,
Ruslan Gladilov
