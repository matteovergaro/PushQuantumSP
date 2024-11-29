from data.sp_data import SPData
from models import SPCplex
from evaluation.evaluation import SPEvaluation
from plotting.sp_plot import SPPlot

params = {"version": 1, "num_cols": 5, "rad_max": 2.4}
data = SPData.gen_problem(**params)
plt = SPPlot(data).plot_problem()
plt.show()

cplex_model = SPCplex(data)
answer = cplex_model.solve(TimeLimit=1)

evaluation = SPEvaluation(data, answer["solution"])
print(f"solution clean: {evaluation.solution}")

print(f"objective = {evaluation.get_objective()}")
for constraint, violations in evaluation.check_solution().items():
    if len(violations) > 0:
        print(f"constraint {constraint} was violated {len(violations)} times")

plt = SPPlot(data, evaluation).plot_solution(hide_never_covered = True)
plt.show()
