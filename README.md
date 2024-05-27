# Project Title

## Overview
This section provides a brief introduction to the project conducted as part of the Advanced Control Methods course at Skoltech in 2024. It includes the fundamental objectives of the project, information about the team members, and a link to the final presentation.

- Course: Advanced Control Methods, Skoltech, 2024
- Team Members: [Member1], [Member2], [Member3], [Member4]
- Final Presentation: [https://docs.google.com/presentation/d/1OndUG2DB0yXjvMWQS30-TukYlxjxPwfYRwaH3VeM5bQ/edit#slide=id.p]

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

### Setup and Installation
Instructions for setting up the project environment, which may include:
- Installing dependencies: `pip install -r requirements.txt`
- Setting up a virtual environment
- Running a `setup.py` or `pyproject.toml` if necessary
- Building and running a Docker container using `Dockerfile`

### Running the Code
Exact commands to execute the project, such as:

```bash
python main.py
```

### Documentation
If available, provide links to the project documentation or instructions on how to generate it.

---

## Other Section
This is a placeholder for any additional sections that the team wishes to include. It could be methodology, discussions, acknowledgments, or any other relevant content that doesn't fit into the predefined sections.

---

## Bibliography
(If applicable) This section includes references to papers, articles, and other resources that informed the project's approach and methodology.

- Reference 1
- Reference 2
- Reference 3








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
