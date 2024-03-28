# Created on : Mar 27, 2024, 5:02:24 PM
# Author     : michaellevin

from src import Params
from collections import deque

class Bush:
    def __init__(self, network, origin):
        self.network = network
        self.flow = {}
        self.relevantPAS = []
        self.sorted = []
        self.branches = []
        self.origin = origin
        
        for l in self.network.links:
            self.flow[l] = 0
            
        self.loadDemand()
        
        
    def contains(self, l):
        return self.flow[l] > self.network.params.flow_epsilon

    def containsNode(n):
        if n == origin:
            return True


        for l in n.getIncoming():
            if contains(l):
                return True

        return False
     
    def loadDemand(self):
        self.network.dijkstras(self.origin)
        
        for s in self.network.zones:
            d = self.origin.getDemand(s)
            
            if d > 0:
                curr = s

                while curr != self.origin:
                    uv = curr.pred
                    self.addFlow(uv, d)
                    curr = uv.start
        
        self.topologicalSort()
    
    def topologicalSort(self):

        for n in self.network.nodes:
            n.in_degree = len(n.getBushIncoming(self))
            n.visited = False
            n.top_order = -1
        
        
        queue = deque()
        queue.append(self.origin)
        self.origin.visited = True
        
        self.sorted = []
        idx = 0
        
        while len(queue) > 0:
        
            vertex = queue.pop()
            self.sorted.append(vertex)
            vertex.top_order = idx
            idx += 1
            
            for ij in vertex.getBushOutgoing(self):
            
                j = ij.end
                
                
                if not j.visited:
                
                    j.in_degree -= 1
                    
                    if j.in_degree == 0:
                    
                        queue.append(j)
                        j.visited = True


        # check for nodes that were not completed
        for n in self.network.nodes:
        
            if n.in_degree < len(n.getBushIncoming(self)) and not n.visited:
                print("NOT DAG")



    def validateFlowConservation():

        for n in network.nodes:
            inflow = 0
            outflow = 0
            
            for l in n.getIncoming():
                inflow += self.flow[l]
            
            for l in n.getOutgoing():
                outflow += flow[l]
            
            if n == origin:
                inflow += origin.getTotalDemand()
            
            elif isInstance(n, Zone):
                outflow += origin.getDemand(n)
            

            if abs(inflow-outflow) > Params.flow_epsilon:
                return False

        return True

    
    def testTopologicalSort():
    
        topologicalSort()
        
        for l in flow:
            if contains(l) and l.getSource().top_order > l.getDest().top_order:
                return False;

        return True
    
    


    
    def checkReducedCosts(self):
    
        minPath()
        
        output = True
        for l in network.links:
        
            if flow[l] > Params.flow_epsilon and l.getReducedCost() < -Params.bush_gap:
                print("Negative reduced cost origin "+str(origin.id))
                #print(str(l)+"\t"+str(flow[l])+"\t"+str(l.getSource().cost)+"\t"+str(l.getTT())+"\t"+str(l.getDest().cost) + "\t"+ str(l.getDest().cost - (l.getSource().cost + l.getTT())))
                output = False

        return output
    
    
    
    def minUsedPath(self):
        for u in network.nodes:
            u.cost = Params.INFTY
            u.pred = None

        origin.cost = 0
        
        for u in sorted:
            for uv in u.getOutgoing(self):
                if flow[uv.getIdx()] < Params.flow_epsilon:
                    continue    
                
                v = uv.getDest()
                temp = uv.getTT() + u.cost
                
                if temp < v.cost:
                    v.cost = temp
                    v.pred = uv

        
        output = {}
        
        for n in sorted:
        
            output[n] = n.pred
        
        
        return output
    

    

    
    def tracePath(self, i, j):
        output = []
        
        curr = j;
        
        while curr != i:

            output.add(curr.pred)
            curr = curr.pred.getSource()
        
        return output
    
    def tracePath2(self, i, j):
        output = []
        
        curr = j;
        
        while curr != i:

            output.add(curr.pred)
            curr = curr.pred2.getSource()
        
        return output
    
    def tracePathSet(self, i, j):
        output = set()
        
        curr = j;
        
        while curr != i:

            output.add(curr.pred)
            curr = curr.pred.getSource()
        
        return output
    
   
    
    def minPath():
        for u in network.nodes:
            u.cost = Params.INFTY
            u.pred = None
        
        origin.cost = 0
        
        
        for u in sorted:
            
            for uv in u.getBushOutgoing(self):
                v = uv.getDest()
                
                temp = uv.getTT() + u.cost
                
                if temp < v.cost:
                    v.cost = temp
                    v.pred = uv
               

        output = {}
        
        for n in sorted:
            output[n] = n.pred;
        
        
        return output
   
    
    def branchShifts(self):
        
        
        for b in self.branches:
            b.init()
            b.flowShift()
            
        
        branches = []
    
  
    
    
    def validateDemand():

        for s in sorted:
            if s != origin and isInstance(s, Dest):
                d = origin.getDemand(s)
                
                actual = 0
                
                for id in s.getIncoming():
                    actual += flow[id]
                
                
                for dj in s.getOutgoing():
                    actual -= flow[dj]
                
                
                if abs(d - actual) > Params.flow_epsilon:
                    print("Origin "+str(origin)+": demand to "+str(s)+" is "+str(d)+" but flow is "+str(actual))
                    return False;
        return True
    
    
    def addFlow(self, l, x):
        l.x += x
        self.flow[l] += x
    

