import neal
import numpy as np

from evaluation.evaluation import SPEvaluation
from plotting.sp_plot import SPPlot
from testing.sp_data import SPData
from testing.sp_qubo_binary import SPQuboBinary

params = {"lidar_density": 0.2, "street_point_density": 0.2}


# We modified the class SPData to add the property that describes the edge weights, which describes how
# far are nodes between one another
data = SPData().create_problem_from_glb_file(**params)
# Range of the penalty coefficient for the constraint to test the QUBO formulation
penalties3 = range(1, 10, 3)
coverage_percentage_vs_penalty = list(range(len(penalties3)))
lidar_used_vs_penalty = list(range(len(penalties3)))
config = {"num_reads": 1000, "num_sweeps": 1000}
solve_func = neal.SimulatedAnnealingSampler().sample_qubo
p_index = 0
for P3 in penalties3:
    # We modified the SPQuboBinary class to add an optimization in the QUBO model,
    # we inserted a quadratic term that minimizes the probability to have two different LiDar
    # both near to the same street point.
    # In this way we obtain a better distribution of the sensors on the whole region allowing to
    # achieve better solutions
    qubo_model_bin = SPQuboBinary(data, P3=P3, max_radius=data.rad_max)
    answer = qubo_model_bin.solve(solve_func, **config)

    evaluation = SPEvaluation(data, answer['solution'])
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

# Plotting only the last solution
plt = SPPlot(data, evaluation).plot_solution(hide_never_covered=True)
plt.show()
