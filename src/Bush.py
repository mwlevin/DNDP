# Created on : Mar 27, 2024, 5:02:24 PM
# Author     : michaellevin

from src import Params
from src import PASList
from src import Branch
import llist
from llist import sllist
from src import PAS
from src import NodeReturn
from src import Node

class Bush:
    def __init__(self, network, origin):
        self.network = network
        self.flow = {}
        self.relevantPAS = []
        self.sorted = list()
        self.branches = []
        self.origin = origin 
        self.relevantPAS = PASList.PASList()
        
        for l in self.network.links:
            self.flow[l] = 0
            
        self.loadDemand()
        
        
    def contains(self, l):
        return self.flow[l] > self.network.params.flow_epsilon

    def containsNode(n):
        if n == origin:
            return True


        for l in n.incoming:
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
        
        
        queue = sllist()
        queue.append(self.origin)
        self.origin.visited = True
        
        self.sorted = list()
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
            
            for l in n.incoming:
                inflow += self.flow[l]
            
            for l in n.outgoing:
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
    
        self.minPath()
        
        output = True
        for l in self.network.links:
        
            if self.flow[l] > self.network.params.flow_epsilon and l.getReducedCost() < -self.network.params.bush_gap:
                print("Negative reduced cost origin "+str(self.origin.id))
                #print(str(l)+"\t"+str(flow[l])+"\t"+str(l.getSource().cost)+"\t"+str(l.getTT())+"\t"+str(l.getDest().cost) + "\t"+ str(l.getDest().cost - (l.getSource().cost + l.getTT())))
                output = False

        return output
    
    
    
    def minUsedPath(self):
        for u in self.network.nodes:
            u.cost = Params.INFTY
            u.pred = None

        self.origin.cost = 0
        
        for u in sorted:
            for uv in u.getOutgoing(self):
                if self.flow[uv] < self.network.params.flow_epsilon:
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
    
   
    
    def minPath(self):
        for u in self.network.nodes:
            u.cost = Params.INFTY
            u.pred = None
        
        self.origin.cost = 0
        
        
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
                
                for id in s.incoming:
                    actual += flow[id]
                
                
                for dj in s.outgoing:
                    actual -= flow[dj]
                
                
                if abs(d - actual) > Params.flow_epsilon:
                    print("Origin "+str(origin)+": demand to "+str(s)+" is "+str(d)+" but flow is "+str(actual))
                    return False;
        return True
    
    
    def addFlow(self, l, x):
        l.x += x
        self.flow[l] += x
        
        
    def checkPAS(self):
        
        minPathTree = self.network.getSPTree(self.origin)
        
        # only create 1 PAS per link per origin per iteration
        included = set()
        
        # look for all used links not part of the tree of least cost routes
        # search in backwards topological order
        
        for n in self.sorted[::-1]:
        
            for l in n.incoming:
                # check for links with high reduced cost and positive flow, not just links not on the shortest path
                if l not in included and l.end != self.origin and l.start != self.origin and self.flow[l] > self.network.params.flow_epsilon and l.hasHighReducedCost(self.network.type, self.network.params.pas_cost_mu):

                    #System.out.println(l+" "+l.getDest().cost+" "+l.getSource().cost+" "+(l.getDest().cost-l.getSource().cost)+" "+l.getTT());
                    # we need a PAS!
                    if not self.hasRelevantPAS(l):

                        # should check if we can borrow one from network
                        fromNetwork = self.network.findPAS(l, self)
                        if fromNetwork is None:

                            if self.network.params.PRINT_PAS_INFO:
                                print("\nCreate PAS for " + str(l) + " for origin " + str(self.origin))
                            
                            newPAS = self.createPAS(minPathTree, l, self.flow[l] * self.network.params.pas_flow_mu)
                            if newPAS is None:
                                # branch shift
                                if self.network.params.PRINT_PAS_INFO:
                                    print("branch shift!")
         
                                branch = self.createBranch(l)
                                if branch is not None:
                                    self.branches.append(branch)

                            else:
                                for ij in newPAS.backwardlinks:
                                    included.add(ij)
                        else:
                            if self.network.params.PRINT_PAS_INFO:
                                print("Take PAS for " + str(l))
                                
                            fromNetwork.addRelevantOrigin(self.origin)

    

    def createBranch(self, endlink):
        minpath = self.tracePathSet(self.origin, endlink.end)
        
        output = Branch.Branch(self, endlink, minpath)
        
        return output
   
    
    def hasRelevantPAS(self, a):
        if a in self.relevantPAS.forward:
            for p in self.relevantPAS.forward[a]:
                if p.isCostEffective(a, self.network.params.pas_cost_mu) and p.isFlowEffective(type, self.network.params.pas_flow_mu):
                    return True
           
        if a in self.relevantPAS.backward:     
            for p in self.relevantPAS.backward[a]:
                if p.isCostEffective(a, self.network.params.pas_cost_mu) and p.isFlowEffective(type, self.network.params.pas_flow_mu):
                    return True
                
        return False
        
    def removeCycles(self):

        
        # right now this restarts the entire loop when a cycle is detected. I think we don't need to restart everything...
        cycleDetected = True
        while cycleDetected:
            for n in self.network.nodes:
                n.visited = False
                n.pred2 = None
                n.top_order = -1
            
            cycleDetected = False
            
            sorted = list()
            
            idx = len(self.network.nodes)-1
            
            unvisited = [] # this is a stack
            unvisited.append(self.origin)


            while len(unvisited) > 0:
                n = unvisited.pop()
                
                
                
                if isinstance(n, Node.Node):

                    if n.top_order >= 0:
                        continue
                    elif n.visited:
                        # remove the cycle
                        
                        #print("cycle check "+str(n)+" for origin "+str(self.origin))
                        
                        #for nb in self.network.nodes:
                        #    if nb.pred2 is not None:
                        #        print("\t"+str(nb)+" "+str(nb.pred2)+" "+str(self.flow[nb.pred2]))
                        
                        self.removeCycleAtNode(n)

                        cycleDetected = True
                        break
                    else:
                        n.visited = True

                        unvisited.append(NodeReturn.NodeReturn(n))

                        for l in n.outgoing:
                            if self.contains(l):
                                j = l.end
                                j.pred2 = l
                                unvisited.append(j)
                else:
                    node = n.node
                    if node.top_order < 0:
                        sorted.append(node)
                        node.top_order = idx
                        idx -= 1


    def removeCycleAtNode(self, n):
        # n is the root node of the cycle
        
        list = []
        
        curr = n
        
        loopbreak = True
        while loopbreak:
            #print("\t"+str(curr)+" "+str(curr.pred2))
            pred = curr.pred2
            list.append(pred)
            
            curr = pred.start
            
            loopbreak = curr != n
            
        
        n.pred2 = None
        
        maxflow = self.network.params.INFTY
        
        for l in list:
            maxflow = min(maxflow, self.flow[l])
        
        
        #print("removing "+str(maxflow)+" from "+str(list))
        for l in list:
            self.addFlow(l, -maxflow)

    def getPathAsNodeSet(self, minPathTree, i, j):
        output = set()
        
        output.add(j)
        
        curr = j
        
        while curr != i:
            curr = curr.pred.start
            output.add(curr)
            
        return output
        
    # create a PAS for link a
    def createPAS(self, minPathTree, a, minflow):
        output = PAS.PAS()
        
        # min path to a.dest
        minPath = self.getPathAsNodeSet(minPathTree, self.origin, a.end)
        
        if self.network.params.PRINT_PAS_INFO:
            print("minPath is "+str(minPath)+" for "+str(a.end))
        
        
        # store trace to avoid repeating breadth first search
        trace = {}
        
        unvisited = sllist()
        
        unvisited.append(a)
        
        firstSimilar = None
        
        while len(unvisited) > 0:
            jk = unvisited.pop()
            #print("consider "+str(jk)+" "+str(unvisited))
            j = jk.start
            
            if j in minPath:
                firstSimilar = jk
                break
            
            for ij in j.incoming:
                #print(str(ij.start)+" "+str(ij.end)+" "+str(self.flow[ij])+" "+str(minflow))
                if self.flow[ij] > minflow and ij.start.top_order < j.top_order:
                #if self.flow[ij] > minflow:
                    unvisited.append(ij)
                    trace[ij] = jk
        
        
        
        
        
        if self.network.params.PRINT_PAS_INFO:
            print("firstSimilar is "+str(firstSimilar))
            print(str(trace)+" "+str(minflow)+" "+str(self.flow[a])+" "+str(firstSimilar))
        
        if firstSimilar is None:
            return None
        

        
        # trace firstSimilar to a in used flow bush: this is the backward side of the PAS
        # if this loop breaks it's probably because we shifted flow and tried to find a PAS after shifting flow before re-running shortest path
        curr = firstSimilar
        output.backwardlinks.append(firstSimilar)
        while curr != a:
            
            #System.out.println("\t"+curr);
            curr = trace[curr]
            output.backwardlinks.append(curr)
        
        
        # trace firstSimilar to a in min path tree: this is the forward side of the PAS
        for l in self.network.traceTree(minPathTree, firstSimilar.start, a.end):
            output.forwardlinks.append(l)
            
        if len(output.forwardlinks) == 0:
            return None
        
        output.start = firstSimilar.end
        output.end = a.end
        
        if self.network.params.PRINT_PAS_INFO:
            print("PAS is "+str(output))
        
        
        output.addRelevantOrigin(self.origin)
        
        self.network.allPAS.add(output)
        
        return output