# Fault-Tolerant-Control

## Overview
This project aims to implement and evaluate fault-tolerant control strategies for dynamic systems. The goal is to ensure system reliability and robustness in the presence of faults.

## Problem Statement


## Results


## Run the Project


### Requirements 
(List of prerequisites, dependencies, and environment setup necessary to run the project.)

requirements.txt


### Setup and Installation
Clone the repository:
```bash
git clone https://github.com/yourusername/Fault-Tolerant-Control.git
cd fault-tolerant-control
```

Install dependencies:
```bash
pip install -r requirements.txt
```


### Running the Code













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
