from copy import deepcopy

class SPEvaluation():

    def __init__(self, data, solution):
        self.data = data
        self.solution = solution

        self.O = deepcopy(self.data.O)
        self.schemeGraph = deepcopy(self.data.schemeGraph)
        self.missing_achievable_coverage = self.data.missing_achievable_coverage

        self.listLidarActivated=[]
        self.listStreetPointsCovered=[]

        self.create_solution_graph(solution)
        self.create_optimized_connections()

        self.objective = len(self.listLidarActivated)
        self.never_covered = self.data.never_covered

    def print_evaluation(self):
        print(f"{self.objective} Lidars activated.")
        print(f"{self.missing_achievable_coverage} street points were not coverted")

    def get_objective(self):
        return self.objective
    
    def check_solution(self):
        error = {
            "missing_achievable_coverage": [1 for i in range(self.missing_achievable_coverage)]
        }
        return error
    
    def create_solution_graph(self, solution_dict):
        for key, value in solution_dict.items():
            if value==1:
                parts = key.split('_')
                lx=float(parts[1].replace("m","-"))
                ly=float(parts[2].replace("m","-"))
                lz=float(parts[3].replace("m","-"))
                dir=float(parts[4].replace("m","-"))
                pitch=float(parts[5].replace("m","-"))
                self.listLidarActivated.append((lx,ly,lz, dir, pitch))
        self.__generateOptimizedGraph()

    def __generateOptimizedGraph(self):
       
        points3 = []
         
        for i in self.schemeGraph['listCovering']:
            points3.append((i[0], i[1]))
        self.O.add_nodes_from(self.listLidarActivated)
        self.O.add_nodes_from(points3)


    def create_optimized_connections(self): 

        for s in self.data.listStreetPoints3D:
            not_covered=True
            for l in self.listLidarActivated:
                line=(l,s)
                
                if (self.data._in_range(l,s, self.data.rad_max, self.data.vert_ang_max_deg, self.data.vert_ang_min_deg, self.data.halber_oeffnungswinkel_deg)):
                    inters=0
                    for w in self.data.walls3D:
                        inters+=self.data._intersect(line,w) 
                        if inters>0: 
                            break   
                    if inters==0: 
                        self.O.add_edge((s[0], s[1]),(l[0], l[1]))
                        not_covered=False
            if not_covered:
                self.O.remove_node((s[0], s[1]))
            else: 
                self.listStreetPointsCovered.append((s[0], s[1]))
        self.missing_achievable_coverage=len(self.data.listStreetPoints3D)-len(self.listStreetPointsCovered)-len(self.data.listStreetPointsNeverCovered)

