class BB_node:
    
    def __init__(self, network, id, parent, LB, UB, fixed0, fixed1):
        self.network = network
        self.parent = parent
        self.id = id
        self.children = []
        self.LB = LB
        self.UB = UB        
        self.fixed0 = fixed0 #list of A2 links fixed to 0
        self.fixed1 = fixed1 #list of A2 links fixed to 1
        self.active = True
        self.solved = False
        self.score = {}
        self.ybr = None
        
    def check(self):
        status = 'solve'
        cost = sum(a.cost for a in self.fixed1)
        
        if cost > self.network.B:
            status = 'infeasible'
            return status
            
        fixed = self.fixed0 + self.fixed1
        free = [a for a in self.network.links2 if a not in fixed]
        
        if len(free) == 0:
            status = 'fixed'
            return status
            
        else:
            left = self.network.B - cost
            mincost = min([a.cost for a in free])
            
            if left < mincost:
                status = 'stop'
                return status
            
        return status
