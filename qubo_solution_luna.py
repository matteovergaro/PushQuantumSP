import os

import numpy as np
from luna_sdk import LunaSolve
from luna_sdk.schemas import TokenProvider, QpuToken

from evaluation.evaluation import SPEvaluation
from plotting.sp_plot import SPPlot
from testing.sp_data import SPData
from testing.sp_qubo_binary import SPQuboBinary

encryption_token = "<>"

os.environ["LUNA_ENCRYPTION_KEY"] = encryption_token

dwave_token = "<>"
luna_token = "<>"
ls = LunaSolve(api_key=luna_token)


def run_with_luna(qubo_matrix):
    optimization = ls.optimization.create_from_qubo(name="My Quneo", matrix=qubo_matrix)

    job = ls.solution.create(
        optimization_id=optimization.id,
        solver_name="QA",
        provider="dwave",
        solver_parameters={
            "embedding": {
                "chain_strength": None,
                "chain_break_fraction": True,
                "embedding_parameters": {
                    "max_no_improvement": 10,
                    "random_seed": None,
                    "timeout": 1000,
                    "max_beta": None,
                    "tries": 10,
                    "inner_rounds": None,
                    "chainlength_patience": 10,
                    "max_fill": None,
                    "threads": 1,
                    "return_overlap": False,
                    "skip_initialization": False,
                    "initial_chains": (),
                    "fixed_chains": (),
                    "restrict_chains": (),
                    "suspend_chains": (),
                },
            },
            "sampling_params": {
                "anneal_offsets": None,
                "anneal_schedule": None,
                "annealing_time": None,
                "auto_scale": None,
                "fast_anneal": False,
                "flux_biases": None,
                "flux_drift_compensation": True,
                "h_gain_schedule": None,
                "initial_state": None,
                "max_answers": None,
                "num_reads": 1,
                "programming_thermalization": None,
                "readout_thermalization": None,
                "reduce_intersample_correlation": False,
                "reinitialize_state": None,
            },
        },
        qpu_tokens=TokenProvider(
            dwave=QpuToken(
                source="inline",
                token=dwave_token,
            ),
        ),
    )

    return job.id


params = {"lidar_density": 0.2, "street_point_density": 0.2}

data = SPData().create_problem_from_glb_file(**params)
# Range of the penalty coefficient for the constraint to test the QUBO formulation
penalties3 = range(1, 10, 3)
coverage_percentage_vs_penalty = list(range(len(penalties3)))
lidar_used_vs_penalty = list(range(len(penalties3)))
config = {"num_reads": 1000, "num_sweeps": 1000}
solve_func = run_with_luna
p_index = 0
job_ids = []
for P3 in penalties3:
    qubo_model_bin = SPQuboBinary(data, P3=P3, max_radius=data.rad_max)
    job_ids.append(run_with_luna(qubo_model_bin.model))

# Wait that all the jobs on the DWAVE quantum annelear are completed

for i in range(len(penalties3)):
    solution = ls.solution.get(job_ids[i])

    best_result = ls.solution.get_best_result(solution)

    evaluation = SPEvaluation(data, best_result)
    coverage_percentage_vs_penalty[p_index] = len(evaluation.listStreetPointsCovered) / (
            len(data.listStreetPoints3D) - len(data.listStreetPointsNeverCovered)) * 100
    lidar_used_vs_penalty[p_index] = evaluation.get_objective()
    p_index += 1

for i in range(len(penalties3)):
    print(f"penalty used: {penalties3[i]}\n"
          f"lidars used: {lidar_used_vs_penalty[i]:2f}\n"
          f"coverage percentage: {coverage_percentage_vs_penalty[i]}\n")

penalties3 = ['P3', *penalties3]
lidar_used_vs_penalty = ['number of lidars', *lidar_used_vs_penalty]
coverage_percentage_vs_penalty = ['coverage percentage', *coverage_percentage_vs_penalty]
np.savetxt('penalties_scores.csv', [p for p in zip(penalties3, lidar_used_vs_penalty, coverage_percentage_vs_penalty)],
           delimiter=',', fmt='%s')

plt = SPPlot(data, evaluation).plot_solution(hide_never_covered=True)
plt.show()
