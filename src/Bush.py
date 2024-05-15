# Created on : Mar 27, 2024, 5:02:24 PM
# Author     : michaellevin
import sys
#print(sys.executable)
import llist
import contextlib
from collections import deque
import heapq

from src import Params
from src import PASList
from src import Branch
from src import Zone


import llist
#print('llist module is available.')

from llist import sllist
from src import PAS
from src import NodeReturn
from src import Node

class Bush:
    def __init__(self, network, origin):
        self.network = network
        self.flow = {}
        self.relevantPAS = []
        self.sortted = list()
        self.branches = []
        self.origin = origin 
        
        #print(origin)
        self.relevantPAS = PASList.PASList()
        
        #for l in self.network.links:
        #    self.flow[l] = 0
            
        self.loadDemand()
        
        
    def contains(self, l):
        return l in self.flow and self.flow[l] > self.network.params.flow_epsilon
        
    def getFlow(self, l):
        return self.flow.get(l, 0)
        
    #def containsNode(self, n):
        #if n == self.origin:
            #print(n)
            #return True


        #for l in n.incoming:
            #if self.contains(l):
                #return True

        #return False
     
    def loadDemand(self):
        #with open('result130.txt', 'w') as file, contextlib.redirect_stdout(file):
            self.network.dijkstras(self.origin, self.network.type)
            #with open('result37.txt', 'w') as file, contextlib.redirect_stdout(file):
            for s in self.network.zones:
                d = self.origin.getDemand(s)
                #print(d)
                if d > 0:
                    curr = s
                    
                    while curr != self.origin:
                        #print(f"self origin is{self.origin}")
                        uv = curr.pred
                        #print(f"uv is {uv}")
                        self.addFlow(uv, d)
                        #print(self.addFlow(uv, d))
                        curr = uv.start
                        #print(f"curr is {curr}")
                        #print(curr)

        
            self.topologicalSort()
        #print(self.topologicalSort)

    def topologicalSort(self):
        with open('result22.txt', 'a') as file, contextlib.redirect_stdout(file):
        # Initialize in-degrees and visited flags
            for n in self.network.nodes:
                n.in_degree = len(n.getBushIncoming(self))
                #print(n.in_degree)

                n.visited = False
                n.top_order = -1
            
            # Use a list as a priority queue
            queue = []
            heapq.heappush(queue, self.origin.id)  # Assume each node has a unique node_id
            self.origin.visited = True
            
            self.sortted = []
            idx = 0
            
            while queue:
                #print(queue)
                # Use heappop for consistent smallest element first
                vertex_id = heapq.heappop(queue)
                #print(vertex_id)
                vertex = self.network.findNode(vertex_id)  # You need to be able to fetch nodes by ID
                #print(vertex)
                self.sortted.append(vertex)
                vertex.top_order = idx
                idx += 1
                
                # Process outgoing edges
                #with open('result2.txt', 'a') as file, contextlib.redirect_stdout(file):
                for ij in vertex.getBushOutgoing(self):
                    #print(f"This is ij{ij}")
                    j = ij.end
                    #print(f"This is j{j}")
                    if not j.visited:
                        j.in_degree -= 1
                        if j.in_degree == 0:
                            heapq.heappush(queue, j.id)
                            j.visited = True
            for n in self.network.nodes:
                if n.in_degree < len(n.getBushIncoming(self)) and not n.visited:
                    print("NOT DAG")

        #with open('output_topological3.txt', 'w') as file, contextlib.redirect_stdout(file):
            #for n in self.network.nodes:
                #print(self.network.nodes)
                #n.in_degree = len(n.getBushIncoming(self))
                #print(n.getBushIncoming(self))
                #print(n.in_degree)
                #n.visited = False
                #n.top_order = -1
        
        
        #queue = deque()
        
        #queue.append(self.origin)
        #print(self.origin)
        #print(queue.append(self.origin))
        #self.origin.visited = True
        
        #self.sortted = list()
        #idx = 0
        #with open('output_topological.txt', 'w') as file, contextlib.redirect_stdout(file):
            #while len(queue) > 0:
                #print(queue)
                #vertex = queue.popleft()
                #print(vertex)
                #self.sortted.append(vertex)
                #vertex.top_order = idx
                #idx += 1
                
                #for ij in vertex.getBushOutgoing(self):
                
                    #j = ij.end
                    
                    #if not j.visited:
                        #j.in_degree -= 1
                        
                        #if j.in_degree == 0:
                        
                            #queue.append(j)
                            #j.visited = True


            # check for nodes that were not completed
            #for n in self.network.nodes:
            
                #if n.in_degree < len(n.getBushIncoming(self)) and not n.visited:
                    #print("NOT DAG")



    #def validateFlowConservation(self):

        #for n in self.network.nodes:
            #print(n)
            #inflow = 0
            #outflow = 0
            
            #for l in n.incoming:
                #inflow += self.flow[l]
            
            #outgoing_edges_list = list(n.outgoing)
            #outgoing_edges_list.sort(key=lambda e: (e.start.id, e.end.id))
            #for l in n.outgoing:
            #for l in sorted(n.outgoing, key=lambda edge: edge.end.id):
                #outflow += self.flow[l]
            
            #if n == self.origin:
                #inflow += self.origin.getTotalDemand()
            
            #elif isinstance(n, Zone):
                #outflow += self.origin.getDemand(n)
            

            #if abs(inflow-outflow) > Params.flow_epsilon:
                #return False

        #return True

    
    def testTopologicalSort(self):
    
        self.topologicalSort()
        
        for l in self.flow:
            if self.contains(l) and l.getSource().top_order > l.getDest().top_order:
                return False

        return True
    
    
    def removeLinks(self, removedlinks):

        minPathTree = self.network.getSPTree(self.origin)
        
        for l in removedlinks:
            if self.contains(l):
                if l.end == self.origin:
                    continue
                elif l.start == self.origin:
                    # need a PAS shift here
                    pas = self.createOriginLinkPAS(minPathTree, l)
                    pas.zeroBackwardFlow(self)
                else:
                    branch = self.createBranch(l)
                    if branch is not None:
                        #print("branch shift ", self.origin, l, self.getFlow(l))
                        branch.init()
                        branch.flowShift(type)
                        #print(self.getFlow(l))
                    #else:
                        #print("branch is null ", self.origin, l)
                    
   
            
    def addLinks(self, newlinks):  
        # does this create a new shortest path?
        
        included = set()
        
        minPathTree = self.network.getSPTree(self.origin)
        
        for l in newlinks:
            if l not in included and l.end != self.origin and l.hasHighReducedCost(self.network.type, self.network.params.pas_cost_mu):
                #print(f"included is {included}")
                #print(f"l.end is {l.end} and l.start is {l.start}")
                #print(f"self.flow of l is {self.flow}")
                #print(f"self.network.params.flow_epsilon is {self.network.params.flow_epsilon}")
                #print(f"the last thing {l.hasHighReducedCost(self.network.type, self.network.params.pas_cost_mu)}")
                #print(l)
                #print(f"selforigin is {self.origin}")
                #System.out.println(l+" "+l.getDest().cost+" "+l.getSource().cost+" "+(l.getDest().cost-l.getSource().cost)+" "+l.getTT());
                # we need a PAS!
                if not self.hasRelevantPAS(l):

                    # should check if we can borrow one from network
                    fromNetwork = self.network.findPAS(l, self)
                    if fromNetwork is None:

                        if self.network.params.PRINT_PAS_INFO:
                            print("\nCreate PAS for " + str(l) + " for origin " + str(self.origin))

                        newPAS = None
                        
                        if l.start != self.origin:
                            newPAS = self.createPAS(minPathTree, l, self.getFlow(l) * self.network.params.pas_flow_mu)
                        #else:
                        #    newPAS = self.createOriginLinkPAS(minPathTree, l)
                        #print(l)
                        
                        if newPAS != None:
                            self.savePAS(newPAS)
                            for ij in newPAS.backwardlinks:
                                included.add(ij)
                            
                    else:
                        if self.network.params.PRINT_PAS_INFO:
                            print("Take PAS for " + str(l))

                        fromNetwork.addRelevantOrigin(self.origin)
                        #print(self.origin)
   
    
    #def checkReducedCosts(self):
    
        #self.minPath()
        
        #output = True
        #for l in self.network.links:
        
            #if self.flow[l] > self.network.params.flow_epsilon and l.getReducedCost() < -self.network.params.bush_gap:
                #print("Negative reduced cost origin "+str(self.origin.id))
                #print(str(l)+"\t"+str(flow[l])+"\t"+str(l.getSource().cost)+"\t"+str(l.getTT())+"\t"+str(l.getDest().cost) + "\t"+ str(l.getDest().cost - (l.getSource().cost + l.getTT())))
                #output = False

        #return output
    
    
    
    '''def minUsedPath(self):
        for u in self.network.nodes:
            u.cost = Params.INFTY
            u.pred = None

        self.origin.cost = 0
        
        for u in self.sortted:
            for uv in u.getOutgoing(self):
                if self.flow[uv] < self.network.params.flow_epsilon:
                    continue    
                
                v = uv.getDest()
                temp = uv.getTT() + u.cost
                
                if temp < v.cost:
                    v.cost = temp
                    v.pred = uv
                    print(v.pred)

        
        output = {}
        
        for n in sortted:
        
            output[n] = n.pred
        
        
        return output'''
    

    

    
    '''def tracePath(self, i, j):
        output = []
        
        curr = j
        #print(j)
        
        while curr != i:

            output.append(curr.pred)
            print(output)
            curr = curr.pred.getSource()
        
        return output'''
    
    def tracePath2(self, i, j):
        output = []
        
        curr = j
        
        while curr != i:
            #print(str(curr)+" "+str(curr.pred2)+" "+str(i)+" "+str(j))
            output.append(curr.pred2)
            curr = curr.pred2.start
        
        return output
    
    def tracePathSet(self, i, j):
        output = []
        
        curr = j
        
        while curr != i:

            output.append(curr.pred)
            #print(curr.pred)
            curr = curr.pred.start
        
        return output
    
   
    
    def minPath(self):
        for u in self.network.nodes:
            u.cost = Params.INFTY
            u.pred = None
        
        self.origin.cost = 0
        
        
        for u in self.sortted:
            
            for uv in u.getBushOutgoing(self):
                v = uv.end
                
                temp = uv.getTravelTime(uv.x, self.network.type) + u.cost
                
                if temp < v.cost:
                    v.cost = temp
                    v.pred = uv
               

        output = {}
        
        for n in self.sortted:
            output[n] = n.pred
        
        
        return output
   
    
    def branchShifts(self):

        for b in self.branches:
            if self.getFlow(b.endlink) > self.network.params.flow_epsilon:
                b.init()
                b.flowShift(type)
                #print(b)
                #return(b.flowShift())
            
        
        self.branches = []
    
  
    
    
    #def validateDemand(self):

        #for s in self.sortted:
            #print(s)
            #if s != self.origin and isinstance(s, self.Dest):
                #d = self.origin.getDemand(s)
                
                #actual = 0
                
                #for id in s.incoming:
                    #actual += self.flow[id]
                
                
                #for dj in sorted(s.outgoing, key=lambda edge: edge.end.id):
                    #actual -= self.flow[dj]
                
                
                #if abs(d - actual) > Params.flow_epsilon:
                    #print("Origin "+str(self.origin)+": demand to "+str(s)+" is "+str(d)+" but flow is "+str(actual))
                    #return False
        #return True
    
    
    def addFlow(self, l, x):
        curr_flow = self.getFlow(l)
        #with open('result121.txt', 'a') as file, contextlib.redirect_stdout(file):
        if curr_flow + x < -self.network.params.flow_epsilon:
            raise("attempt to add flow to link "+str(l.id)+" from bush "+str(self.origin)+" of flow "+str(self.flow[l])+" "+str(x))

            #print(f"the l.x is {l.x}")
        l.x += x
            #print(f"The l.x after adding x is {l.x} and x is {x}")
        curr_flow += x
        
        if curr_flow > self.network.params.flow_epsilon:
            self.flow[l] = curr_flow
        elif l in self.flow:
            del self.flow[l]
            #print(f"The self.flow of l is {self.flow[l]}")

        
    def checkPAS(self):
        
        minPathTree = self.network.getSPTree(self.origin)
        #print(self.origin)
        
        # only create 1 PAS per link per origin per iteration
        included = set()
        
        # look for all used links not part of the tree of least cost routes
        # search in backwards topological order
        with open('result171.txt', 'a') as file, contextlib.redirect_stdout(file):
            for n in self.sortted[::-1]:
                #print(n)
            
                #for l in n.incoming:
                #print(f"n.incoming is {n.incoming}")
                #for l in sorted(n.incoming, key=lambda l: (l.start.id, l.end.id)): 
                for l in n.incoming:
                    #print(sorted(n.incoming, key=lambda l: (l.start.id, l.end.id)))
                    #print(f"l before {l}")
                    #print(n.incoming)
                    # check for links with high reduced cost and positive flow, not just links not on the shortest path
                    if l not in included and l.end != self.origin and self.getFlow(l) > self.network.params.flow_epsilon and l.hasHighReducedCost(self.network.type, self.network.params.pas_cost_mu):
                        #print(f"included is {included}")
                        #print(f"l.end is {l.end} and l.start is {l.start}")
                        #print(f"self.flow of l is {self.flow}")
                        #print(f"self.network.params.flow_epsilon is {self.network.params.flow_epsilon}")
                        #print(f"the last thing {l.hasHighReducedCost(self.network.type, self.network.params.pas_cost_mu)}")
                        #print(l)
                        #print(f"selforigin is {self.origin}")
                        #System.out.println(l+" "+l.getDest().cost+" "+l.getSource().cost+" "+(l.getDest().cost-l.getSource().cost)+" "+l.getTT());
                        # we need a PAS!
                        if not self.hasRelevantPAS(l):

                            # should check if we can borrow one from network
                            fromNetwork = self.network.findPAS(l, self)
                            if fromNetwork is None:

                                if self.network.params.PRINT_PAS_INFO:
                                    print("\nCreate PAS for " + str(l) + " for origin " + str(self.origin))

                                newPAS = None
                                
                                if l.start != self.origin:
                                    newPAS = self.createPAS(minPathTree, l, self.getFlow(l) * self.network.params.pas_flow_mu)
                                # this was causing a crash??
                                #else:
                                #    newPAS = self.createOriginLinkPAS(minPathTree, l)
                                    
                                #print(l)
                                if newPAS is None:
                                    if l.start != self.origin:
                                        # branch shift
                                        if self.network.params.PRINT_PAS_INFO:
                                            print("branch shift!")

                                        branch = self.createBranch(l)
                                        if branch is not None:
                                            self.branches.append(branch)

                                else:
                                    self.savePAS(newPAS)
                                    for ij in newPAS.backwardlinks:
                                        included.add(ij)
                            else:
                                if self.network.params.PRINT_PAS_INFO:
                                    print("Take PAS for " + str(l))

                                fromNetwork.addRelevantOrigin(self.origin)
                                #print(self.origin)


    

    def createBranch(self, endlink):
        minpath = self.tracePathSet(self.origin, endlink.end)
        
        output = Branch.Branch(self, endlink, minpath)
        
        return output
   
    
    def hasRelevantPAS(self, a):
        #with open('hasrelevantPAS4.txt', 'a') as file, contextlib.redirect_stdout(file):
            #print(a)
            if a in self.relevantPAS.forward:
                #print(f"a is {a}")
                for p in self.relevantPAS.forward[a]:
                    if p.isCostEffective(a, self.network.params.pas_cost_mu) and p.isFlowEffective(type, self.network.params.pas_flow_mu):
                        return True
            
            if a in self.relevantPAS.backward:
                #print(a)     
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
            
            sortted = list()
            
            idx = len(self.network.nodes)-1
            
            #unvisited = [] # this is a stack
            #unvisited.append(self.origin)
            unvisited = [self.origin]
            #print(self.origin)

            
            while len(unvisited) > 0:
                #unvisited = sorted(unvisited, key=lambda n: n.node.id)
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

                        #outgoing_edges_list = list(n.outgoing)
                        #outgoing_edges_list.sort(key=lambda e: (e.start.id, e.end.id))
                        #for l in n.outgoing:
                        #for l in sorted(n.outgoing, key=lambda edge: edge.end.id):
                        for l in n.outgoing:
                            if self.contains(l):
                                j = l.end
                                j.pred2 = l
                                unvisited.append(j)
                else:
                    node = n.node
                    if node.top_order < 0:
                        sortted.append(node)
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
            #if pred is None:
                #print(f"Error: Attempt to access 'start' attribute of None at node {curr}")
                #break  #
            list.append(pred)
            
            curr = pred.start
            
            loopbreak = curr != n
            
        
        n.pred2 = None
        
        maxflow = Params.INFTY
        #print(maxflow)
        
        #with open('result119.txt', 'a') as file, contextlib.redirect_stdout(file):
        for l in list:
            maxflow = min(maxflow, self.getFlow(l))
            #print(f"maxflow is {maxflow}")
            #print(f"self.flow[l] is {self.flow[l]}")
        
        
        #print("removing "+str(maxflow)+" from "+str(list))
        #with open('result116.txt', 'a') as file, contextlib.redirect_stdout(file):
        for l in list:
            self.addFlow(l, -maxflow)
            #print(-maxflow)

    def getPathAsNodeSet(self, minPathTree, i, j):
        #with open('result53.txt', 'a') as file, contextlib.redirect_stdout(file):
            output = set()
            
            output.add(j)
            
            curr = j
            
            while curr != i:
                curr = curr.pred.start
                output.add(curr)
                #print(curr)
                
            return output
        
    def createOriginLinkPAS(self, minPathTree, a):
        output = PAS.PAS()

            
        output.backwardlinks.append(a)
        
        for l in self.network.traceTree(minPathTree, a.start, a.end):
            #print(f'tree entry {minPathTree}, r {firstSimilar.start}, s {a.end}')
            output.forwardlinks.append(l)
            #print(l)
            
        output.start = a.start
        output.end = a.end

        if self.network.params.PRINT_PAS_INFO:
            print("PAS is "+str(output))
                
        return output
    
    # create a PAS for link a
    def createPAS(self, minPathTree, a, minflow):
        #with open('result149.txt', 'a') as file, contextlib.redirect_stdout(file):
            output = PAS.PAS()
            #print(a)
            # min path to a.dest
            minPath = self.getPathAsNodeSet(minPathTree, self.origin, a.end)
            #print(self.origin)
            if self.network.params.PRINT_PAS_INFO:
                print("minPath is "+str(minPath)+" for "+str(a.end))
            
            #with open('result147.txt', 'a') as file, contextlib.redirect_stdout(file):
        # store trace to avoid repeating breadth first search
            trace = {}
            unvisited = deque([a])
            #print(f"a is {a}")
        
            #unvisited = sllist()
            #print(unvisited)
            
            #unvisited.append(a)
            #print(f"unvisited is {unvisited}")
            
            firstSimilar = None
            
            #with open('result145.txt', 'a') as file, contextlib.redirect_stdout(file):
            while len(unvisited) > 0:
                jk = unvisited.popleft()
                #print(f"jk is {jk}")
                #print("consider "+str(jk)+" "+str(unvisited))
                j = jk.start
                
                if j in minPath:
                    firstSimilar = jk
                    #print(jk)
                    break
                
                for ij in j.incoming:
                    #print(str(ij.start)+" "+str(ij.end)+" "+str(self.flow[ij])+" "+str(minflow))
                    if self.getFlow(ij) > minflow and ij.start.top_order < j.top_order:
                    #if self.flow[ij] > minflow:
                        unvisited.append(ij)
                        trace[ij] = jk
            
            
            
            
            
            if self.network.params.PRINT_PAS_INFO:
                print("firstSimilar is "+str(firstSimilar))
                print(str(trace)+" "+str(minflow)+" "+str(self.getFlow(a))+" "+str(firstSimilar))
            
            if firstSimilar is None:
                return None
            

            
            # trace firstSimilar to a in used flow bush: this is the backward side of the PAS
            # if this loop breaks it's probably because we shifted flow and tried to find a PAS after shifting flow before re-running shortest path
            #with open('result24.txt', 'a') as file, contextlib.redirect_stdout(file):
            curr = firstSimilar
            #print(curr)
            output.backwardlinks.append(firstSimilar)
            while curr != a:
                
                #System.out.println("\t"+curr);
                curr = trace[curr]
                output.backwardlinks.append(curr)
                
            
            
            # trace firstSimilar to a in min path tree: this is the forward side of the PAS
            with open('result141.txt', 'a') as file, contextlib.redirect_stdout(file):
                for l in self.network.traceTree(minPathTree, firstSimilar.start, a.end):
                    #print(f'tree entry {minPathTree}, r {firstSimilar.start}, s {a.end}')
                    output.forwardlinks.append(l)
                    #print(l)
                    
            if len(output.forwardlinks) == 0:
                return None
            
            output.start = firstSimilar.end
            output.end = a.end
            
            if self.network.params.PRINT_PAS_INFO:
                print("PAS is "+str(output))
            
            #print(self.origin)
            return output
            
    def savePAS(self, pas):
        pas.addRelevantOrigin(self.origin)
             
        self.network.allPAS.add(pas)