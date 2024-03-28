from src import Node
from src import Link
from src import Path
from src import Zone
from src import Bush
from src import Params

class Network:

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
        while len(line) == 0:
            line = file.readline().strip()
        
        for i in range(0, numLinks + newLinks):
            line = file.readline().split()
            
            start = self.nodes[int(line[0]) - 1]
            end = self.nodes[int(line[1]) - 1]
            C = float(line[2]) * scal_flow

            t_ff = float(line[4]) * scal_time
            alpha = float(line[5])
            beta = float(line[6])
            
            cost = float(line[10])
            
            
            
            self.TC += cost
            
            link = Link.Link(start, end, t_ff, C, alpha, beta, cost)
            self.links.append(link)
            
            if i > numLinks:
                self.links2[(start, end)] = link
            
        file.close()
        
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
        
        while len(lines) < line_idx or idx < len(splitted):

            next = splitted[idx]
            if next == "Origin":
                idx += 1
                r = self.zones[int(splitted[idx]) - 1]
            else:
                s = self.zones[int(splitted[idx]) - 1]
                idx += 2
                next = splitted[idx]
                d = float(next[0:len(next) - 1]) * scal_flow
                
                r.addDemand(s, d)
                self.TD += d
                
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
    def argmin(self, set):
        best = None
        min = float("inf")

        for n in set:
            if n.cost < min:
                min = n.cost
                best = n

        return best

    def dijkstras(self, origin):

        
        for n in self.nodes:
            n.cost = self.params.INFTY
            n.pred = None

        origin.cost = 0.0

        Q = {origin}

        # **********
        # Exercise 6(c)
        # ********** 
        
        while len(Q) > 0:

            u = self.argmin(Q)
            Q.remove(u)

            for uv in u.getOutgoing():
                v = uv.end

                tt = uv.getTravelTime(uv.x, self.type)
                if u.cost + tt < v.cost:
                    v.cost = u.cost + tt
                    v.pred = uv

                    if v.isThruNode():
                        Q.add(v)
            
  
    # **********
    # Exercise 6(d)
    # ********** 

    def trace(self, r, s):
        curr = s

        output = Path.Path()
        
        while curr != r and curr is not None:
            ij = curr.pred

            if ij is not None:
                output.addFront(ij)
                curr = curr.pred.start
              
        return output

    # **********
    # Exercise 7
    # ********** 
    # returns the total system travel time
    def getTSTT(self):
        output = 0.0
        for ij in self.links:
            output += ij.x * ij.getTravelTime(ij.x, self.type)
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

    # **********
    # Exercise 8(a)
    # ********** 
    # find the step size for the given iteration number
    def calculateStepsize(self, iteration):
        return 1.0 / iteration

    # **********
    # Exercise 8(b)
    # ********** 
    # calculate the new X for all links based on the given step size
    def calculateNewX(self, stepsize):
        for ij in self.links:
            ij.calculateNewX(stepsize)

    # **********
    # Exercise 8(c)
    # ********** 
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
        
    def tapas(self, type, lbd, y, xinit):
        self.setType(type)
        
        for r in self.zones:
            r.bush = Bush.Bush(self, r)

    def getLx(self, lbd, y):
        Lx = 0
        for ij in self.links:
            Lx += ij.x * ij.getTravelTime(ij.x, 'TT')
            i = ij.start
            j = ij.end
            if (i,j) in y:
                Lx += lbd[(i,j)]*ij.x
            
        return Lx
            
    def getXDict(self):
        output = {}
        for ij in self.links:
            output[(ij.start, ij.end)] = ij.x
        return output