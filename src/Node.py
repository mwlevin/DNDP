

class Node:

    # construct this Node with the given id
    def __init__(self, id):
        # used for Dijkstra's implementation
        self.cost = 0.0
        self.id = id
        self.outgoing = set()
        self.incoming = set()
        
        self.top_order = -1
        self.visited = False
        self.in_degree = 0
    
        self.cost = 0
        # pred2 is used by bush when not finding shortest paths
        self.pred = None
        self.pred2 = None
        
    def __repr__(self):
        return str(self)
        
        

    
    def getBushOutgoing(self, b):
    
        output = []
        
        for l in self.outgoing:
            if b.contains(l):
                output.append(l)
        
        return output
    
    def getBushIncoming(self, b):
    
        output = []
        
        for l in self.incoming:
            if b.contains(l):
                output.append(l)
        
        return output
    

        
    # returns a list of links containing the outgoing links of this node
    def getOutgoing(self):
        return self.outgoing
        
    def getIncoming(self):
        return self.incoming

    # returns True if this node is a thru node
    def isThruNode(self):
        return True
    
    # **********
    # Exercise 3(b)
    # **********    
    # returns the id of this node
    def getId(self):
        return self.id
    
    # **********
    # Exercise 3(c)
    # **********   
    def __str__(self):
        return str(self.id)
    
    # **********
    # Exercise 3(d)
    # **********    
    # adds ij to list of outgoing links
    def addOutgoingLink(self, ij):
        self.outgoing.add(ij)
    
    def addIncomingLink(self, ij):
        self.incoming.add(ij)