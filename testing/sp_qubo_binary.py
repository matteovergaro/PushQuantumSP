import math
import sys
import time

import numpy as np


class SPQuboBinary:
    def __init__(self, gra, P1=1, P2=2, P3=5, max_radius=30) -> None:
        self.gra = gra

        self.usedLidars = []
        self.mandatoryLidars = []
        self.P1 = P1
        self.P2 = P2
        self.P3 = P3
        print(max_radius)
        self.model = self.__compute_QUBO_Matrix_binary(P1, P2, P3, max_radius)


    def __inverter_matrix(self, sample):
        solution_dict = {
            f"x_{self.usedLidars[i][0]}_{self.usedLidars[i][1]}_{self.usedLidars[i][2]}_{self.usedLidars[i][3]}_{self.usedLidars[i][4]}":
                sample[
                    i
                ]
            for i in range(len(self.usedLidars))
        }
        return solution_dict

    def solve(self, solve_func, **config):

        start_time = time.time()
        # print(self.model)
        answer = solve_func(Q=self.model, **config)
        # print(answer)
        solve_time = time.time() - start_time

        solution = self.__inverter_matrix(answer.first.sample)
        info = answer.info

        return {
            "solution": solution,
            "energy": answer.first.energy,
            "runtime": solve_time,
            "info": info,
        }

    def __needed_bitnum(self, decnum):
        if decnum == 0:
            return 0
        return int(math.ceil(math.log2(decnum)))

    def __is_in_list(self, mylist, target):
        for i in mylist:
            if target == i:
                return True
        return False

    def __compute_QUBO_Matrix_binary(self, P1, P2, P3, max_radius):
        slacksize = 0
        slacklist = []
        for s in self.gra.G.nodes:
            if len(s) == 3:

                slackbits = self.__needed_bitnum(len(self.gra.G.adj[s].items()))

                lidar_per_SP = []
                for ls in self.gra.G.adj[s].items():
                    self.usedLidars.append(ls[0])
                    lidar_per_SP.append(ls[0])
                    if slackbits == 0:
                        self.mandatoryLidars.append(ls[0])

                slacklist.append(
                    [lidar_per_SP, {slacksize + i + 1: 2 ** i for i in range(slackbits)}]
                )
                slacksize += slackbits
        self.usedLidars = list(set(self.usedLidars))
        ilist = list(range(len(self.usedLidars)))
        usedLidars_index = dict(zip(self.usedLidars, ilist))
        for s in slacklist:
            if s[1]:
                s[1] = {
                    key + len(self.usedLidars) - 1: -value
                    for key, value in s[1].items()
                }
        myQUBOsize = len(self.usedLidars) + slacksize
        myQUBOMatrix = np.zeros([myQUBOsize, myQUBOsize], dtype=float)

        for i in range(len(self.usedLidars)):
            myQUBOMatrix[i, i] = P1
            if self.__is_in_list(self.mandatoryLidars, self.usedLidars[i]):
                myQUBOMatrix[i, i] -= P2

            for j in range(len(self.usedLidars)):
                for s in self.gra.G.nodes:
                    if len(s) == 3:
                        v_si = self.gra.custom_edge_weights.get(frozenset([s, self.usedLidars[i]]), 0)
                        v_sj = self.gra.custom_edge_weights.get(frozenset([s, self.usedLidars[j]]), 0)
                        # The quadratic term we added needs to be calibrated based on the maximum radius of the sensors
                        optimization_coefficient = 0.1
                        if max_radius < 5:
                            optimization_coefficient = 0.5
                        if max_radius > 20:
                            optimization_coefficient = 0.01
                        myQUBOMatrix[i, j] += optimization_coefficient * v_sj * v_si

        # count = 0
        for s in slacklist:
            if s[1]:
                # count += 1
                sdict = s[1]
                ldict = {}
                for l in s[0]:
                    ldict[usedLidars_index[l]] = 1
                ldict.update(sdict)
                # print(count)
                # print(ldict)

                for i in ldict:
                    myQUBOMatrix[i, i] -= 2 * P3 * ldict[i]
                    for j in ldict:
                        myQUBOMatrix[i, j] += P3 * ldict[i] * ldict[j]

        np.set_printoptions(threshold=sys.maxsize)
        # print(myQUBOMatrix)

        # for i in myQUBOMatrix:
        #   for j in i:
        #      print(j, end=",")
        #    print()
        return myQUBOMatrix
