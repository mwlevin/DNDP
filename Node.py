

class Node:

    # construct this Node with the given id
    def __init__(self, id):
        # used for Dijkstra's implementation
        self.cost = 0.0
        self.predecessor = None
        self.id = id
        self.outgoing = []
        
    def __repr__(self):
        return str(self)
        
    # returns a list of links containing the outgoing links of this node
    def getOutgoing(self):
        return self.outgoing

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
        self.outgoing.append(ij)