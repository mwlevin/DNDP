import contextlib
from src import Node
from src import Link
from src import Path
from src import Zone
from src import Bush
from src import Params
from src import PASList
import math
from gurobipy import Model, GRB, quicksum
import sys

class OptNetwork:

    # construct this Network with the name; read files associated with network name
    def __init__(self, name,B_prop,m,scal_time,scal_flow,timelimit):
        self.nodes = [] 
        self.links = []
        self.zones = []
        
        self.links2 = {}
        self.type = 'UE'
        self.TD = 0
        self.TC = 0 # total cost
        self.params = Params.Params()
        
        self.allPAS = PASList.PASList()
        
        
        self.inf = 1e+9
        self.tol = 1e-2
        
        self.readNetwork("data/"+name+"/net.txt",m,scal_time,scal_flow,timelimit)
        self.readTrips("data/"+name+"/trips.txt",m,scal_time,scal_flow,timelimit)
        
        self.B = self.TC * B_prop # budget
        
        
        print('Total scaled demand',self.TD)
        print('Total cost',self.TC,'Budget',self.B)
        
    def setType(self, type):
        self.type = type
        
    # read file "/net.txt"
    def readNetwork(self, netFile,m,scal_time,scal_flow,timelimit):

        
        firstThruNode = 1
        numZones = 0
        numNodes = 0
        numLinks = 0
        newLinks = 0
        #with open('result8.txt', 'a') as file, contextlib.redirect_stdout(file):
        file = open(netFile, "r")

        line = ""
        
        while line.strip() != "<END OF METADATA>":
            line = file.readline()
            if "<NUMBER OF ZONES>" in line:
            
                numZones = int(line[line.index('>') + 1:].strip());
            
            elif "<NUMBER OF NODES>" in line:
                numNodes = int(line[line.index('>') + 1:].strip())
            elif "<NUMBER OF LINKS>" in line:
                numLinks = int(line[line.index('>') + 1:].strip())
            elif "<NUMBER OF NEW LINKS>" in line:
                newLinks = int(line[line.index('>') + 1:].strip())
            elif "<FIRST THRU NODE>" in line:
                firstThruNode = int(line[line.index('>') + 1:].strip())

        for i in range(0, numZones):
            self.zones.append(Zone.Zone(i + 1))

        for i in range(0, numNodes):
            if i < numZones:
                self.nodes.append(self.zones[i])
                
                if i + 1 < firstThruNode:
                    self.zones[i].setThruNode(False)

            else:
                self.nodes.append(Node.Node(i + 1))

        line = ""
        id = 0
        while len(line) == 0:
            line = file.readline().strip()
        #with open('result6.txt', 'a') as file, contextlib.redirect_stdout(file):
        for i in range(0, numLinks + newLinks):
            line = file.readline().split()
            if len(line) == 0:
                continue
            start = self.nodes[int(line[0]) - 1]
            end = self.nodes[int(line[1]) - 1]
            C = float(line[2]) * scal_flow

            t_ff = float(line[4]) * scal_time
            alpha = float(line[5])
            beta = float(line[6])
            
            cost = float(line[10])
            
            self.TC += cost
            
            #with open('result5.txt', 'w') as file, contextlib.redirect_stdout(file):
            link = Link.Link(id, start ,end, t_ff, C, alpha, beta, cost)
            id = id +1
            #print(start,end)
            self.links.append(link)
            #print(self.links)
            
            if i > numLinks:
                self.links2[(start, end)] = link
            
            #with open('result6.txt', 'a') as file, contextlib.redirect_stdout(file):
            #print(f"Start Node: {start}, End Node: {end}")
            #print(link)
        file.close()

        #with open('result6.txt', 'a') as file, contextlib.redirect_stdout(file):
            #print(start,end)
            #print(self.links)


    def readTrips(self, tripsFile,m,scal_time,scal_flow,timelimit):

        # **********
        # Exercise 5(d)
        # ********** 
        
        file = open(tripsFile, "r")
        
        lines = file.readlines()
        
        line_idx = 0
        
        while lines[line_idx].strip() != "<END OF METADATA>":
            line_idx += 1
            
        line_idx += 1
        
        while lines[line_idx].strip() == "":
            line_idx += 1
            
        r = None
        
        idx = 0
        
        splitted = lines[line_idx].split()
        #print(splitted)
        
        while len(lines) < line_idx or idx < len(splitted):
            #print(line_idx)
            #print(idx)
            #print(len(lines))This is the total number of line
            #print()
        #while lines < len(line_idx) or idx < len(splitted):

            next = splitted[idx]
            #print(next)
            if next == "Origin":
                
                idx += 1
                r = self.zones[int(splitted[idx]) - 1]

                #print(int(splitted[idx]))
            else:
                #print(int(splitted[idx]))
                #print(line)
                #print(f"idx: {idx}, splitted[idx]: {splitted[idx]}")
                #print(f"Length of self.zones: {len(self.zones)}")
                #print(int(splitted[idx]) - 1)
                s = self.zones[int(splitted[idx]) - 1]

                #print(s)
                idx += 2
                next = splitted[idx]
                d = float(next[0:len(next) - 1]) * scal_flow *5
                
                r.addDemand(s, d)
                self.TD += d


            #if [int(splitted[idx]) - 1] != 343:

            idx += 1

            if idx >= len(splitted):
                line_idx += 1
                while line_idx < len(lines) and lines[line_idx].strip() == "":
                    line_idx += 1
                    
                if line_idx < len(lines):
                    line = lines[line_idx].strip()
                    splitted = line.split()
                    idx = 0
            
        file.close()

    def getLinks(self):
        return self.links
    
    def getNodes(self):
        return self.nodes
    
    def getZones(self):
        return self.zones

    # **********
    # Exercise 5(e)
    # ********** 
    # find the node with the given id
    def findNode(self, id):
        if id <= 0 or id > len(self.nodes):
            return None
        return self.nodes[id - 1]

    # find the link with the given start and end nodes
    def findLink(self, i, j):
        if i is None or j is None:
            return None

        for link in i.getOutgoing():
            if link.getEnd() == j:
                return link

        return None

    # find the node with the smallest cost (node.cost)
    def argmin1(self, set):
        best = None
        min = float("inf")

        for n in set:
            if n.cost < min:
                min = n.cost
                best = n

        return best

    #def argmin(self, set):
        # Sort the set based on the cost of the nodes
        #sorted_nodes = sorted(set, key=lambda x: x.cost)
        
        # Return a copy of the last node in the sortted list (which has the minimum cost)
        #return sorted_nodes[-1]

    # change
    def argmin(self, set):
        cost, node = min((n.cost, n) for n in set)
        return node

    def dijkstras(self, origin):
        #with open('result69.txt', 'a') as file, contextlib.redirect_stdout(file):
        
            for n in self.nodes:
                n.cost = self.params.INFTY
                n.pred = None

            origin.cost = 0.0

            Q = {origin}
            #print(f"This is Q {Q}")

            while len(Q) > 0:

                u = self.argmin(Q)
                #print(f"This is u {u}")
                Q.remove(u)
                #outgoing_edges = sortted(u.outgoing, key=lambda e: (e.start.id, e.end.id))
                #outgoing_edges_list = list(u.outgoing)
                #outgoing_edges_list.sort(key=lambda e: (e.start.id, e.end.id))
                #for u in sortted:


                #for uv in sorted(u.outgoing, key=lambda edge: (edge.end.id, edge.start.id)):
                for uv in u.outgoing:
                #for uv in u.outgoing:
                      # This will print all the properties and methods of the 'Zone' object
                    #break 
                    v = uv.end

                    #print(f"this is v {v}")
                    #print(f"uv.x before {uv.x}")
                    tt = uv.getTravelTime(uv.x, self.type)
                    #print(f"here is uv.x {uv.x}")
                    #print(self.type)
                    #print(f"this is tt {tt}")
                    if u.cost + tt < v.cost:
                        v.cost = u.cost + tt
                        v.pred = uv
                        #print(v.pred)

                        if v.isThruNode():
                            Q.add(v)
            
    


    def trace(self, r, s):
        curr = s
        #print(f"s is {s}")

        output = Path.Path()
        
        while curr != r and curr is not None:
            ij = curr.pred
            #print(ij)

            if ij is not None:
                output.addFront(ij)
                curr = curr.pred.start
              
        return output
        
    def traceTree(self, tree, r, s):
        #with open('trace8.txt', 'a') as file, contextlib.redirect_stdout(file):
            curr = s
            #print(s)

            output = []
            #with open('trace4.txt', 'a') as file, contextlib.redirect_stdout(file):
            while curr != r and curr is not None:
                ij = tree[curr]
                #print(ij)

                if ij is not None:
                    output.append(ij)
                    #print(output)
                    curr = ij.start
            
            return output

    def getSPTree(self, r):
        self.dijkstras(r)
        
        output = {}
        
        for n in self.nodes:
            if n != r and n.cost < self.params.INFTY:
                output[n] = n.pred
                

        
        return output
    
    # returns the total system travel time
    def getTSTT(self):
        output = 0.0
        for ij in self.links:
            tt = ij.getTravelTime(ij.x, self.type)
            output += ij.x * tt
            #print(ij.x)
            #print(str(link)+ "\t" + ij, 'flow'+ "\t" +ij.x,'travel time'+ "\t" +tt, 'free flow travel time'+ "\t" +ij.t_ff, 'alpha'+ "\t" +ij.alpha, 'beta'+ "\t" +ij.beta, ij.C)
            #print('link: {}\tflow: {}\ttravel time: {}\tfree flow travel time: {}\talpha: {}\tbeta: {}\tC: {}'.format(str(ij), ij.x, tt, ij.t_ff, ij.alpha, ij.beta, ij.C))

        return output

    # returns the total system travel time if all demand is on the shortest path
    def getSPTT(self):
        output = 0.0

        for r in self.zones:
            if r.getProductions() > 0:
                self.dijkstras(r)

                for s in self.zones:
                    if r.getDemand(s) > 0:
                        output += r.getDemand(s) * s.cost

        return output

    # returns the total number of trips in the network
    def getTotalTrips(self):
        output = 0.0

        for r in self.zones:
            output += r.getProductions()

        return output

    # returns the average excess cost
    def getAEC(self):
        return (self.getTSTT() - self.getSPTT()) / self.getTotalTrips()


    # find the step size for the given iteration number
    def calculateStepsize(self, iteration):
        return 1.0 / iteration
        #print(1.0 / iteration)


    # calculate the new X for all links based on the given step size
    def calculateNewX(self, stepsize):
        for ij in self.links:
            ij.calculateNewX(stepsize)


    # calculate the all-or-nothing assignment
    def calculateAON(self):
        for r in self.zones:
            if r.getProductions() > 0:
                self.dijkstras(r)

                for s in self.zones:
                    if r.getDemand(s) > 0:
                        pi_star = self.trace(r, s)
                        pi_star.addHstar(r.getDemand(s))


    def msa(self, type, lbd, y, xinit):

        self.setType(type)
        max_iteration = 1000
        
        for ij in self.links:
            i = ij.start
            j = ij.end
            if (i,j) in y:            
                ij.setlbdCost(lbd[(i,j)]*y[(i,j)] + self.inf*(1 - y[(i,j)]))
        
        
        output = "Iteration\tAEC\n"
        
        
        

        for iteration in range(1, max_iteration + 1):
            self.calculateAON()
            stepsize = self.calculateStepsize(iteration)
            
            self.calculateNewX(stepsize)
            
            output += str(iteration) + "\t" + str(self.getAEC()) + "\n"
        
        return self.getLx(lbd, y), self.getXDict(), self.getTSTT()
        





network = OptNetwork("SiouxFalls",0.5,500,1e-0,1e-0,600)

def setup_and_solve_optimization(network, mode):
    # Initialize the model
    m = Model("Transport_Optimization")

    # Decision Variables
    flows = {}
    for link in network.getLinks():
        flows[(link.start.id, link.end.id)] = m.addVar(lb=0, name=f"flow_{link.start.id}_{link.end.id}")

    # Auxiliary variable for the maximum expression
    max_var = m.addVar(name="max_var")

    # Update model to integrate new variables
    m.update()

    # Parameters
    alpha = 1.0  # Penalty factor
    N_v = 100    # Total number of vehicles, adjust as needed

    # Objective Function
    # Constructing the penalty function expression
    psi_expr = quicksum(flows[(link.start.id, link.end.id)] * link.getTravelTime(link.x, mode) for link in network.getLinks()) - N_v

    # Max condition implemented via an auxiliary variable
    m.addConstr(max_var >= 0, "max_var_non_negative")
    m.addConstr(max_var >= psi_expr, "max_var_ge_psi_expr")

    # Minimize the squared maximum of psi_expr and 0, scaled by alpha
    m.setObjective(alpha * max_var ** 2, GRB.MINIMIZE)

    # Constraints
    # Flow constraints for demands
    for zone in network.getZones():
        for link in zone.outgoing:
            if (link.start.id, link.end.id) in flows:
                m.addConstr(flows[(link.start.id, link.end.id)] >= zone.getDemand(link.end), f"Demand_{link.start.id}_{link.end.id}")

    # Flow conservation constraints
    for node in network.getNodes():
        m.addConstr(
            quicksum(flows[(link.start.id, link.end.id)] for link in node.outgoing) -
            quicksum(flows[(link.start.id, link.end.id)] for link in node.incoming) ==
            0, f"FlowCons_{node.id}"
        )

    # Solve the model
    m.optimize()

    # Check if the solution is optimal and print the results
    if m.status == GRB.OPTIMAL:
        print("Optimal solution found:")
        for v in m.getVars():
            print(f"{v.varName}: {v.x}")
    else:
        print("No optimal solution found")

# Assuming 'network' is properly initialized and set up as shown in your script
# setup_and_solve_optimization(network, "Uber/Lyft")
setup_and_solve_optimization(network, "Uber/Lyft")