import time

from docplex.mp.model import Model

class CPlexSP: 
    def __init__(self, gra) -> None:
        self.gra=gra
        self.__model = Model(name="SP")
        self.build_model()


    def solve(self, **config):

        TimeLimit = config.get("TimeLimit", 1)
        self.__model.set_time_limit(TimeLimit)

        start_time = time.time()
        self.__model.solve()
        runtime = time.time() - start_time  

        solution = {var.name.replace("m", "-"): var.solution_value for var in self.__model.iter_variables()}

        return {"solution": solution, "runtime": runtime}
    

    def build_model(self):
        x = self.__model.binary_var_dict(self.gra.listLidar3D, name='x')
        self.__model.objective_expr = sum(x[i] for i in self.gra.listLidar3D)
        self.__model.objective_sense = 'min'
        for node in self.gra.listStreetPoints3D:
            has_neighbour=0
            for v in self.gra.G.neighbors(node): 
                has_neighbour=1
            if has_neighbour:
                self.__model.add_constraint(1 <= sum(x[v] for v in self.gra.G.neighbors(node)))