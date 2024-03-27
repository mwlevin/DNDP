

class Link:

    # construct this Link with the given parameters
    def __init__(self, start, end, t_ff, C, alpha, beta, cost):
        self.start = start
        self.end = end
        self.t_ff = t_ff
        self.C = C
        self.alpha = alpha
        self.beta = beta
        self.x = 0
        self.cost = cost # for DNDP
        
        if start is not None:
            start.addOutgoingLink(self)
        self.xstar = 0
        self.lbdcost = 0
        
    def setlbdCost(self, lbdcost):
        self.lbdcost = lbdcost

    # updates the flow to the given value
    def setFlow(self, x):
        self.x = x
    
    def __repr__(self):
        return str(self)
        

    def getTravelTime(self, x, type):
        output = self.t_ff * (1 + self.alpha * pow(self.x / self.C, self.beta))
        
        if type == 'SO':
            output += self.t_ff * self.alpha * self.beta * pow(self.x / self.C, self.beta-1) / self.C
        
        if type != 'TT':
            output += self.lbdcost
        return output
        

    def getCapacity(self):
        return self.C
    
    def getFlow(self):
        return self.x
        
    # **********
    # Exercise 3(a)
    # **********  
    def getStart(self):
        return self.start
    
    def getEnd(self):
        return self.end
        
    # **********
    # Exercise 3(c)
    # **********   
    def __str__(self):
        return "(" + str(self.start.getId()) + ", " + str(self.end.getId()) + ")"
        
    # **********
    # Exercise 8(a)
    # **********   
    def addXstar(self, flow):
        self.xstar += flow
        
    # **********
    # Exercise 8(b)
    # **********   
    def calculateNewX(self, stepsize):
        self.x = (1 - stepsize) * self.x + stepsize * self.xstar
        self.xstar = 0