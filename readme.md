
# Sensor Positioning Optimization

This project provides tools to optimize sensor placement in a distribution environment using various classical and quantum models.
The goal is to minimize the number of sensors while covering the whole area. The position and angles of the sensors can be adjusted.

Is it recommended to use Python 3.11 and install the requirements for packages by using `requirements.txt`.

## Installation

### Virtual Environment (venv)
1. Create and activate a virtual environment:
   ```bash
   python3.11 -m venv venv
   source venv/bin/activate  # On Windows use `venv\Scripts\activate`
   ```
2. Install requirements:
   ```bash
   pip install -r requirements.txt
   ```

### Conda Environment
1. Create and activate a conda environment:
   ```bash
   conda create -n sp python=3.11
   conda activate sp
   ```
2. Install requirements:
   ```bash
   pip install -r requirements.txt
   ```

## Project Architecture

- **data**: Handles data generation and manipulation, specifically `SPData` for creating sensor positioning problems.
- **models**: Contains various optimization models, including classical and quantum approaches.
- **evaluation**: Used to evaluate and validate solutions based on constraints and objectives.
- **plotting**: Tools for visualizing problem setup and solutions.

## Use Case

This project addresses the **sensor positioning problem**, which involves determining optimal sensor placements to maximize area coverage while minimizing sensor count. The setup supports custom problems and real-world scenarios using data provided in GLB format.

## Model Classes

- `SPCplex`: Classical optimization with Cplex
- `SPQuboBinary`: QUBO formulation using binary encoding


### Model Descriptions

The models span classical, quantum, and hybrid approaches. The classical baseline (Cplex) provides initial solutions, while transformations to QUBO enable compatibility with quantum methods like D-Wave and QAOA. QUBO models use either binary or one-hot encoding. Quantum models like QAOA and QAOAnsatz employ advanced Hamiltonians for constrained optimization, and the Grover model leverages quantum search algorithms (might not work correctly).

## Example Usage

Here's an example of using `SPData`, `SPCplex`, and visualization tools:

```python
from data.sp_data import SPData
from models import SPCplex
from evaluation.evaluation import SPEvaluation
from plotting.sp_plot import SPPlot

params = {"lidar_density": 0.1, "street_point_density": 0.1}
data = SPData().create_problem_from_glb_file(**params)
plt = SPPlot(data).plot_problem()
plt.show()

cplex_model = SPCplex(data)
answer = cplex_model.solve()

evaluation = SPEvaluation(data, answer["solution"])
print(f"solution clean: {evaluation.solution}")

print(f"objective = {evaluation.get_objective()}")
for constraint, violations in evaluation.check_solution().items():
    if len(violations) > 0:
        print(f"constraint {constraint} was violated {len(violations)} times")

plt = SPPlot(data, evaluation).plot_solution(hide_never_covered = True)
plt.show()
```

## Additional Examples

The above provided example can be found in example_cplex_glb.py. There are two other examples: example_cplex_generator.py uses a simple toy problem and solves them with cplex and example_qubo_generator.py creates a simple toy problem and solves it with simulated annealing.