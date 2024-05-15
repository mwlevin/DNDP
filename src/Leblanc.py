import time
import copy
from src import Network
from src import BB_node

class Leblanc:

    # instantiate Leblanc's branch-and-bound algorithm
    def __init__(self, network, timelimit):
        self.network = network
        self.BB_nodes = []
        self.inf = 1e+9
        self.tol = 1e-2        
        self.nit = 0
        self.LB = 0
        self.UB = self.inf
        self.timelimit = timelimit
        
        n = BB_node.BB_node(self.network, 0, 0, self.LB, self.inf, [], [])
        self.BB_nodes.append(n)
        
    def getCandidates(self):
        return [i for i in self.BB_nodes if i.active==True]
        
    def getLB(self, candidates):
        return min([i.LB for i in candidates])
        
    def getGap(self):           
        return (self.UB - self.LB)/self.UB
    
    def nodeSelection(self, candidates):
        min_LB = min([i.LB for i in candidates])
        min_LB_nodes = [i for i in candidates if i.LB==min_LB]
        return min_LB_nodes[0]
    
    def branch(self, can):
                             
        fixed00 = copy.deepcopy(can.fixed0)
        fixed00.append(can.ybr)
        fixed01 = copy.deepcopy(can.fixed1)
        fixed10 = copy.deepcopy(can.fixed0)
        fixed11 = copy.deepcopy(can.fixed1)
        fixed11.append(can.ybr)
        
        cnt = len(self.nodes) 
        print(self.nodes,cnt)
        
        BB_node_id = cnt+1
        can.children.append(BB_node_id)
        n0 = BB_node.BB_node(self.network, BB_node_id, can.id, can.LB, self.inf, fixed00, fixed01)
        self.nodes.append(n0)
        
        BB_node_id = cnt+2
        can.children.append(BB_node_id)
        n1 = BB_node.BB_node(self.network, BB_node_id, can.id, can.LB, can.UB, fixed10, fixed11)
        n1.solved = True
        self.nodes.append(n1)
    
        return     

    def BB(self):
        nBB = 0
        nSO = 0
        nUE = 0
 
        t0 = time.time()
    
        conv = False
        while conv == False:
             
            can = self.nodeSelection(self.getCandidates())
            status = can.check()
     
            prune = False
            integral = False
         
            if status == 'infeasible':
                print('--> prune by feasibility')
                prune = True
                 
                can.LB = self.inf
                can.UB = self.inf        
                 
            elif status == 'fixed' or status == 'stop':
                print('--> prune by check',status)
                prune = True
                integral = True
                 
                yUB = {}
                for a in self.network.links2:
                    if a in can.fixed1:
                        yUB[a] = 1
                    elif a in can.fixed0:
                        yUB[a] = 0 
                    else:
                        yUB[a] = 0
                        can.fixed0.append(a)        
                 
            else:
                if can.solved == False:
                    #---closed link child
                    y = {}
                    for a in self.network.links2:
                        if a in can.fixed1:
                            y[a] = 1
                        elif a in can.fixed0:
                            y[a] = 0 
                        else:
                            y[a] = 1
                                        
                    can.LB = self.network.tapas('SO', y)
                    nSO += 1
                 
                if can.LB >= self.UB:
                    print('--> prune by optimality')
                    prune = True
                     
                else:
                    prune = False
                    
                    for a in self.network.links2:
                        can.score[a] = a.x * a.getTravelTime(a.x, 'SO')                        
     
            if integral == True:
                                
                can.UB = self.network.tapas('UE', yUB)
                nUE += 1
                
                if can.UB < self.UB:            
                    self.UB = can.UB
                    yopt = {}
                    for a in self.network.links2:
                        if a in can.fixed0:
                            yopt[a] = 0
                        else:
                            yopt[a] = 1
                    print('*** update UB ***')
                    
                    for i in self.BB_nodes:                    
                        if i.active == True and i.LB >= self.UB:
                            i.active = False      

            if prune == False:                
                fixed = can.fixed0 + can.fixed1
                free = [a for a in self.network.links2 if a not in fixed]            
                free_sorted = sorted(free, key = lambda ele: can.score[ele], reverse = True)
                can.ybr = free_sorted[0]
                
                self.branch(can)                
         
            can.active = False
            candidates = self.getCandidates()
               
            if len(candidates) == 0:
                conv = True
                self.LB = self.UB
                gap = 0
                print('Convergence by inspection')
                break
                
            else:
                self.LB = self.getLB(candidates)            
                gap = self.getGap()
            
            print('==> %d\t%d\t%d\t%.1f\t%.1f\t%.2f' % (nBB,nSO,nUE,self.LB,self.UB,gap))
            
            if gap <= self.tol:
                conv = True
                self.LB = self.UB
                gap = 0
                print('Convergence by optimality')
                break
            
            if (time.time() - t0) >= self.timelimit:
                print('!!! time limit exceeded')
                break
            
            nBB += 1
 
        rt = time.time() - t0
 
        print(conv,rt,nBB,gap,self.UB,yopt)
        
        return
    