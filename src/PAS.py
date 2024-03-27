# Created on : Mar 27, 2024, 4:37:50 PM
# Author     : michaellevin


# This is a sample Python script.

# Press Shift+F6 to execute it or replace it with your code.
from src import Params

class PAS:
    def __init__(self):
        self.forwardlinks = []
        self.backwardlinks = []
        
        self.relevant = set()
        self.lastIterFlowShift = 0
        self.start = None
    
    def addRelevantOrigin(self, r):
        self.relevant.append(r)
        r.bush.addRelevantPAS(self)
    
    def getTT(self, topshift):
        output = 0
        
        for l in self.forwardlinks:
            output += l.getTT(l.x + topshift)
            
        for l in self.backwardlinks:
            output -= l.getTT(l.x - topshift)
        
        return output
        
    def isBackwards(self):
        forwardcost = self.getForwardCost()
        backwardcost = self.getBackwardCost()
        
        return backwardcost < forwardcost
        
        
    def isCostEffective():
        
        forwardcost = getForwardCost()
        backwardcost = getBackwardCost()
        
        costdiff = backwardcost - forwardcost
        
        # maybe the forward and backward costs will be reversed sometimes
        return costdiff > Params.pas_cost_mu * forwardcost or -costdiff > Params.pas_cost_mu * backwardcost
    
    def isCostEffectiveForLink(self, a):
        forwardcost = self.getForwardCost()
        backwardcost = self.getBackwardCost()
        
        costdiff = 0
        
        
        if a == self.getEndLinkBwd():
            costdiff = backwardcost - forwardcost
            return costdiff > Params.pas_cost_mu * forwardcost
        elif a == self.getEndLinkFwd():
            costdiff = forwardcost - backwardcost
            return costdiff > Params.pas_cost_mu * backwardcost

        return False
    
    def getForwardCost(self):
        forwardcost = 0
        
        for l in forwardlinks:
            forwardcost += l.getTT(l.x)
        
        
        return forwardcost
    
    
    def getBackwardCost(self):
        backwardcost = 0
        
        for l in backwardlinks:
            backwardcost += l.getTT(l.x)
        
        
        return backwardcost
    
    
    def isFlowEffective(self):
        # min flow of high cost segment
        # high cost segment is backwards links
        minflow = 1e9; 
        flowlastsegment = 0
        
        lookat = self.backwardlinks
        
        if self.isBackwards():
            lookat = self.forwardlinks
            
        for l in lookat:
            # only look at high cost segment
            totalFlow = 0
            for r in relevant:
                totalFlow += r.bush.getFlow(l)

            minflow = min(minflow, totalFlow)
            if l.getDest() == getEnd():
                flowlastsegment = totalFlow
            
        
        
        return minflow >= Params.pas_flow_mu * flowlastsegment
        
    def maxFlowShift(self, bush)
        max = 1E9
            
        for l in backwardlinks:
            # check flow on link if l in backwards direction
            max = min(max, b.getFlow(l))

        
        return max
    
    
    def maxBackwardFlowShift(self, bush):
        max = 1E9
            
        for l in forwardlinks:
            # check flow on link if l in backwards direction
            max = min(max, bush.getFlow(l))

        
        return max
    
    def maxFlowShift(self):
        
        maxFlowPerBush = {}
        
        for r in relevant:
            maxFlowPerBush[r.bush] = self.maxFlowShift(r.bush)
        
        
        return maxFlowPerBush
    
    
    
    def maxBackwardFlowShift(self):
        
        maxFlowPerBush = {}
        
        for r in relevant:
            maxFlowPerBush[r.bush] = self.maxBackwardFlowShift(r.bush))
        
        
        return maxFlowPerBush
    
    def flowShift(self)
        
        forwardcost = self.getForwardCost()
        backwardcost = self.getBackwardCost()
        
        costdiff = backwardcost - forwardcost
        
        # maybe the forward and backward costs will be reversed sometimes
        if costdiff > Params.pas_cost_mu * forwardcost or -costdiff > Params.pas_cost_mu * backwardcost:
            return False
        
        
        backwards = 1
        if backwardcost < forwardcost:
            backwards = -1

        
        overallMaxShift = 0
        
        maxFlowShift = {}
        
        if backwards > 0:
            maxFlowShift = self.maxFlowShift();
        
        else
            maxFlowShift = self.maxBackwardFlowShift()
        
        
        for b in maxFlowShift.keySet():
            overallMaxShift += maxFlowShift[b]
        
        
        if overallMaxShift < Params.pas_flow_mu:
            return False
        

        #System.out.println("max shift "+overallMaxShift+" "+backwards+" "+backwardcost+" "+forwardcost);
        
        bot = 0
        top = overallMaxShift
        
        while top - bot > Params.line_search_gap:
            mid = (top + bot)/2
            
            check = self.getTT(mid * backwards)
            
            #System.out.println("\t"+bot+" "+top+" "+mid+" "+check);
            
            if check*backwards < 0:
                bot = mid
            
            elseL
                top = mid
            
        
        

        for l in forwardlinks:
            for b in relevant:
                # proportion allocated to bush is bush max shift / total max shift
                b.addFlow(l, maxFlowShift[b] / overallMaxShift * bot * backwards)
            
        
        
        for l in backwardlinks:
            for b in relevant:
                # proportion allocated to bush is bush max shift / total max shift
                b.addFlow(l, -maxFlowShift[b] / overallMaxShift * bot * backwards)
            
        
        
        #System.out.println("after shift "+getTT(0));
        
        return true;
    }