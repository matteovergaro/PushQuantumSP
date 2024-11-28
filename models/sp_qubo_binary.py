import itertools

from docplex.mp.model import Model
import math
from dwave.system import DWaveSampler, EmbeddingComposite
from dwave.samplers import SimulatedAnnealingSampler
#from transformations.from_cplex import FromCPLEX 
import numpy as np
import time

class QuboSPBinary: 
    def __init__(self, gra, P1 = 1, P2 = 2, P3 = 2) -> None:
        self.gra=gra
        #Ob jeder Straßenpunkt abgedeckt werden kann wird hier nicht geprüft
        #self.coverage_possible=-1

        #Braucht man nie?
        #self.mod = Model() 
        self.usedLidars=[]
        self.mandatoryLidars=[]
        self.P1=P1
        self.P2=P2
        self.P3=P3
        self.model=self.__compute_QUBO_Matrix_binary(P1, P2, P3)
               
    
    def __inverter_matrix(self, sample):
        solution_dict={f'x_{self.usedLidars[i][0]}_{self.usedLidars[i][1]}_{self.usedLidars[i][2]}_{self.usedLidars[i][3]}_{self.usedLidars[i][4]}': sample[i] for i in range(len(self.usedLidars))}
        return solution_dict
    
    def solve(self, solve_func, **config):

        start_time = time.time()
        answer = solve_func(Q=self.model, **config)
        solve_time = time.time() - start_time

        solution = self.__inverter_matrix(answer.first.sample)
        info = answer.info
        
        return {"solution": solution, "energy": answer.first.energy, "runtime": solve_time, "info": info}
      
    def solve_dict_dwave(self, num_reads, chain_strength):
        self.myQUBOMatrix=self.__compute_QUBO_Matrix_binary(self.P1, self.P2, self.P3)
        ds=DWaveSampler()
        sampler=EmbeddingComposite(ds)
        sampleset=sampler.sample_qubo(self.myQUBOMatrix.tolist(), num_reads=num_reads, chain_strength=chain_strength)
        solution_dict_sa=self.__inverter_matrix(sampleset.first.sample)

        return solution_dict_sa

    
    def solve_dict_annealing(self, num_reads, chain_strength):
        self.myQUBOMatrix=self.__compute_QUBO_Matrix_binary(self.P1, self.P2, self.P3)
        SA_sampler = SimulatedAnnealingSampler()
        sa_res = SA_sampler.sample_qubo(self.myQUBOMatrix, num_reads=num_reads, chain_strength=chain_strength)
        solution_dict_sa = self.__inverter_matrix(sa_res.first.sample)
        solution_dict_sa = {key.replace('m', '-'): value for key, value in solution_dict_sa.items()}
        #print(len(self.myQUBOMatrix))
        return solution_dict_sa

    def __needed_bitnum(self, decnum): 
        #? wann kann dieser Fall eintreten?
        if decnum==0: 
            return 0
        #print(decnum, " ", int(math.ceil(math.log2(decnum))))
        return int(math.ceil(math.log2(decnum)))
    
    def __is_in_list(self, mylist, target): 
        for i in mylist: 
            if target==i: 
                return True  
        return False

    def __compute_QUBO_Matrix_binary_2(self, P1, P2, P3):
        slacksize = 0
        SP_list = []
        for s in self.gra.G.nodes:
            # Strassenpunkt?
            if len(s) == 3:
                # Bits for needed slack variable
                # Slack variable running between 0 and (number of adjacent lidars-1)
                # Street points with only on Lidar dont need a slack variable, but need to be considered in the QUBO Matrix
                slackbits = self.__needed_bitnum(len(self.gra.G.adj[s].items()))

                lidar_per_SP = []
                for ls in self.gra.G.adj[s].items():
                    # Nur die Lidare, die einen Straßenpunkt covern, sollen in den solver. Schmeisst CPLEX die auch raus?
                    self.usedLidars.append(ls[0])
                    lidar_per_SP.append(ls[0])
                    if slackbits==0:
                        self.mandatoryLidars.append(ls[0])

                SP_list.append([lidar_per_SP, {slacksize + i: 2 ** i for i in range(slackbits)}])
                slacksize += slackbits

        self.usedLidars = list(set(self.usedLidars))
        ilist = list(range(len(self.usedLidars)))
        usedLidars_index = dict(zip(self.usedLidars, ilist))

        for s in SP_list:
            s[1]={key+len(self.usedLidars): value for key, value in s[1].items()}

        myQUBOsize = len(self.usedLidars) + slacksize
        #print("myQUBOsize", myQUBOsize)
        myQUBOMatrix = np.zeros([myQUBOsize, myQUBOsize], dtype=float)

        for i in range(0, len(self.usedLidars)):
            # Penalty P1 for each used Lidar
            myQUBOMatrix[i, i] = P1
            # Penalty P2, if mandatory Lidar is not used
            if self.__is_in_list(self.mandatoryLidars, self.usedLidars[i]):
                myQUBOMatrix[i, i] -= P2

        for s in SP_list:
            #print("s", s)
            if s[1]:
                sdict=s[1]
                ldict={}
                for l in s[0]:
                    #print(usedLidars_index[l])
                    ldict[usedLidars_index[l]]=1
                #print("sdict", sdict)
                #print("ldict", ldict)
                for (s_1,s_2) in itertools.product(list(sdict.keys()),list(sdict.keys())):
                    #print(f"(s_1,s_2){(s_1, s_2)}")
                    myQUBOMatrix[s_1][s_2] += int(P3 * sdict[s_1] * sdict[s_2])


                for (_s, _l) in itertools.product(list(sdict.keys()),list(ldict.keys())):
                    #print(f"(_s, _l) {(_s, _l)}")

                    myQUBOMatrix[_s][_l] -= int(2 * sdict[_s] * ldict[_l])

                for (l_1,l_2) in itertools.product(list(ldict.keys()),list(ldict.keys())):
                    #print(f"(l_1,l_2) {(l_1,l_2)}")

                    myQUBOMatrix[l_1][l_2] += P3 * int(ldict[l_1] * ldict[l_2])

        return myQUBOMatrix

    def __compute_QUBO_Matrix_binary(self, P1, P2, P3): 
        slacksize=0
        slacklist=[]
        for s in self.gra.G.nodes:
            #Strassenpunkt?
            if len(s)==3:
                #Bits for needed slack variable 
                #Slack variable running between 0 and (number of adjacent lidars-1)
                #Street points with only one Lidar dont need a slack variable, but need to be considered in the QUBO Matrix
                slackbits=self.__needed_bitnum(len(self.gra.G.adj[s].items()))
              
                

                lidar_per_SP=[]
                for ls in self.gra.G.adj[s].items():
                    #Nur die Lidare, die einen Straßenpunkt covern, sollen in den solver. Schmeisst CPLEX die auch raus?
                    self.usedLidars.append(ls[0])
                    lidar_per_SP.append(ls[0])
                    if slackbits==0: 
                        self.mandatoryLidars.append(ls[0])
                
                slacklist.append([lidar_per_SP,{slacksize+i+1:2**i for i in range(slackbits)}])
                slacksize+=slackbits

        # Remove double lidars
        self.usedLidars=list(set(self.usedLidars))
        ilist=list(range(len(self.usedLidars)))
        usedLidars_index=dict(zip(self.usedLidars, ilist))
        #Reindex slack variables and subtract: 
        # print(len(self.usedLidars))
        # print("slacklist", slacksize)
        for s in slacklist: 
            if s[1]:
                #print(s[1])
                s[1]={key+len(self.usedLidars)-1: -value for key, value in s[1].items()}
                #print(s[1])
                #print()

        myQUBOsize=len(self.usedLidars)+slacksize
        myQUBOMatrix=np.zeros([myQUBOsize,myQUBOsize],dtype=float)
        #print("self.usedLidars", self.usedLidars)
        for i in range(0,len(self.usedLidars)): 
            #Penalty P1 for each used Lidar
            myQUBOMatrix[i,i]=P1
            #print("P1", P1)
            #Penalty P2, if mandatory Lidar is not used
            if self.__is_in_list(self.mandatoryLidars, self.usedLidars[i]): 
                #print("P2", P2)
                myQUBOMatrix[i,i]-=P2


        #Penalty P3 for missing coverage of a streetpoint 

        #Letzer Index der Lidare in QUBO Matrix, ab dann gehts mit Slackvariablen y weiter
        

        #Schleife über Straßenpunkte
        for s in slacklist:
            #print(s)
            
            if s[1]:
                sdict=s[1]
                ldict={}
                for l in s[0]: 
                    #print(usedLidars_index[l])
                    ldict[usedLidars_index[l]]=1
                #print(ldict, sdict)
                ldict.update(sdict)
                #print(ldict)
            
                for i in ldict: 
                    myQUBOMatrix[i,i]-=2*P3*ldict[i]
                    for j in ldict: 
                        myQUBOMatrix[i,j]+=P3*ldict[i]*ldict[j]
            
            #?Stimmt das da?
            #self.coverage_possible=0


        return myQUBOMatrix



