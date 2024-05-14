import contextlib
from src import Node
from src import Link
from src import Path
from src import Zone
from src import Bush
from src import Params
from src import PASList
import math

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

    # def readTrips(self, tripsFile, m, scal_time, scal_flow, timelimit):
    #     with open(tripsFile, "r") as file:
    #         lines = file.readlines()

    #     line_idx = 0
    #     while lines[line_idx].strip() != "<END OF METADATA>":
    #         line_idx += 1

    # # Move past the metadata and any blank lines
    #     line_idx += 1
    #     while line_idx < len(lines) and not lines[line_idx].strip():
    #         line_idx += 1

    # # Ensure there is data to process
    #     if line_idx >= len(lines):
    #         print("No trip data available after metadata.")
    #         return
    #     total_scaled_demand = 0
    #     total_cost = 0

    #     while line_idx < len(lines):
    #         line = lines[line_idx].strip()
    #         if line:
    #             splitted = line.split()
    #             idx = 0
    #             while idx < len(splitted):
    #                 try:
    #                 # Validate and clean the data before processing
    #                     if ':' in splitted[idx]:
    #                        idx += 1  # Skip invalid entries
    #                        continue
                    
    #                     number = splitted[idx].replace(';', '')  # Remove semicolons if present
    #                     if number.isdigit():
    #                         zone_index = int(number) - 1
    #                         if 0 <= zone_index < len(self.zones):
    #                             r = self.zones[zone_index]
    #                         # Process demands or other relevant data
    #                             idx += 1  # Move to the next relevant data after zone index
    #                     idx += 1
    #                 except (IndexError, ValueError) as e:
    #                     print(f"Error processing line {line_idx}: {e}")
    #                     idx += 1  # Move to the next entry to avoid stuck loops

    #         line_idx += 1  # Move to the next line
    #         while line_idx < len(lines) and not lines[line_idx].strip():
    #             line_idx += 1  # Skip empty lines within data section

    #     print(f"Total scaled demand {total_scaled_demand}")
    #     print(f"Total cost {total_cost} Budget 0.0")


    
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
        if line_idx < len(lines):
            splitted = lines[line_idx].split()
        
        while len(lines) < line_idx or idx < len(splitted):

            next = splitted[idx]
            if next == "Origin":
                idx += 1
                if idx < len(splitted) and 0 <= int(splitted[idx]) - 1 < len(self.zones):
                    r = self.zones[int(splitted[idx]) - 1]
                else:
                    print(f"Invalid index at Origin: {idx} with value {splitted[idx]}")

            else:
                if idx < len(splitted) and 0 <= int(splitted[idx]) - 1 < len(self.zones):
                    s = self.zones[int(splitted[idx]) - 1]
                else:
                    print(f"Invalid index: {idx} with value {splitted[idx]}")
                    break 
                idx += 2
                if idx < len(splitted):
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
            # print("Index being accessed:", idx)
            # print("Split line length:", len(splitted))
            # print("Split line content:", splitted)

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
        
    def tapas(self, type, lbd, y, xinit):
        self.setType(type)
        
        max_iter = 50
        min_gap = 1E-4
        
        #self.params.line_search_gap = pow(10, math.floor(math.log10(self.TD) - 6))
        
        
        last_iter_gap = 1
        
        for r in self.zones:
            r.bush = Bush.Bush(self, r)
        
    #with open('output_tapas21.txt', 'w') as file, contextlib.redirect_stdout(file):
    #with open('output_tapas11.txt', 'w') as file:
        for iter in range(1, max_iter+1):
                        
            #custom_x = {link: link.x for link in self.links}
            #print(custom_x)
            
            # for every origin
            for r in self.zones:
                if r.bush is None:
                    continue
            
                # remove all cyclic flows and topological sort
                r.bush.removeCycles()
                # find tree of least cost routes
                            
                r.bush.checkPAS()
                # for every link used by the origin which is not part of the tree
                    # if there is an existing effective PAS
                        # make sure the origin is listed as relevant
                    # else
                        # construct a new PAS    
                                    
                # choose a random subset of active PASs
                # shift flow within each chosen PAS
                                
                r.bush.branchShifts()

                            
                for a in r.bush.relevantPAS.forward:
                    for p in r.bush.relevantPAS.forward[a]:
                        p.flowShift(self.type, self.params.pas_cost_mu, self.params.pas_flow_mu, self.params.line_search_gap)
                        
                        # for every active PAS
                        
            modified = False
            for shiftIter in range(0, self.params.tapas_equilibrate_iter):
                # check if it should be eliminated
                self.removePAS(iter)
                # perform flow shift to equilibrate costs
                modified = self.equilibratePAS(iter)
                # redistribute flows between origins by the proportionality condition
                            
                # in the case that no flow shifting occurred, do not try to equilibrate more
                if not modified:
                    break

                            
            tstt = self.getTSTT()
            sptt = self.getSPTT()
            gap = (tstt - sptt)/tstt
            aec = (tstt - sptt)/self.TD
                        
                        #with open('output_tapas.txt', 'w') as file, contextlib.redirect_stdout(file):
                        
                            # The rest of your original code here...
                        #print(str(iter)+"\tGap: "+str(gap)+"\tAEC: "+str(aec))  # Example print, redirected to file
                            

            print(str(iter)+"\t"+str(tstt)+"\t"+str(sptt)+"\t"+str(gap)+"\t"+str(aec))
                
                #printLinkFlows();
                
            if gap < min_gap:
                break
                
                
            # there's an issue where PAS are labeled as not cost effective because the difference in cost is small, less than 5% of the reduced cost
            # for low network gaps, this is causing PAS to not flow shift
            # when the gap is low, increase the flow shift sensitivity
            if (last_iter_gap - gap) / gap < 0.01:
                    self.params.pas_cost_mu = max(self.params.pas_cost_mu/10, 1e-9)
                    self.params.line_search_gap = max(self.params.line_search_gap/10, 1e-9)
                    
                    if self.params.PRINT_TAPAS_INFO:
                        print("Adjusting parameters due to small gap "+str(self.params.pas_cost_mu)+" "+str(self.params.line_search_gap))
                        
            
            last_iter_gap = gap
                
                
    def getLx(self, lbd, y):
        Lx = 0
        for ij in self.links:
            Lx += ij.x * ij.getTravelTime(ij.x, 'TT')
            i = ij.start
            j = ij.end
            if (i,j) in y:
                Lx += lbd[(i,j)]*ij.x
            
        return Lx
            
    #def getXDict(self):
    #    output = {}
    #    for ij in self.links:
    #        print(self.links)
    #        output[(ij.start, ij.end)] = ij.x
    #        print(ij.x)
    #    return output
        
    def findPAS(self, ij, bush):
        
        if not self.allPAS.containsKey(ij):
            return None
        
        best = None
        max = self.params.bush_gap
        
        if ij in self.allPAS.forward:
            for p in self.allPAS.forward[ij]:
                temp = p.maxBackwardBushFlowShift(bush)

                if temp > max and p.isCostEffective(ij, self.params.pas_cost_mu):
                    max = temp
                    best = p
                
        if ij in self.allPAS.backward:
            for p in self.allPAS.backward[ij]:
                temp = p.maxForwardBushFlowShift(bush)

                if temp > max and p.isCostEffectiveForLink(ij, self.type, self.params.pas_cost_mu):
                    max = temp
                    best = p
        
        return best
        
        
        
    def equilibratePAS(self, iter):
        output = False
        

        for a in self.allPAS.forward:
            for p in self.allPAS.forward[a]:

                if p.flowShift(self.type, self.params.pas_cost_mu, self.params.pas_flow_mu, self.params.line_search_gap):
                    output = True
                    p.lastIterFlowShift = iter

        return output
        
        
    def removeAPAS(self, p):
        self.allPAS.remove(p)
            
        for r in p.relevant:
            r.bush.relevantPAS.remove(p)
    
    def removePAS(self, iter):
        removed = []
        
        for a in self.allPAS.forward:
            for p in self.allPAS.forward[a]:
                if p.lastIterFlowShift < iter-2:
                    removed.append(p)
        
        for p in removed:
            self.removeAPAS(p)
