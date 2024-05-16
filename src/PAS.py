# Created on : Mar 27, 2024, 4:37:50 PM
# Author     : michaellevin


# This is a sample Python script.

# Press Shift+F6 to execute it or replace it with your code.
from src import Params
import contextlib
from src import Network
class PAS:
    def __init__(self):
        self.forwardlinks = []
        self.backwardlinks = []
        
        self.relevant = set()
        self.lastIterFlowShift = 0
        self.start = None
        self.end = None
    
    def addRelevantOrigin(self, r):
        self.relevant.add(r)
        #print(r)
        r.bush.relevantPAS.add(self)
    
    def getTT(self, topshift, type):
        output = 0
        #with open('getTT5.txt', 'a') as file, contextlib.redirect_stdout(file):
        for l in self.forwardlinks:
            #print(f"self.forwardlinks {self.forwardlinks}")
            output += l.getTravelTime(l.x + topshift, type)
            #output += l.getTravelTime(l.x, type)
            #print(f"output is {output}----")
            
        for l in self.backwardlinks:
            output -= l.getTravelTime(l.x - topshift, type)
            #print(output)
        
        return output
        
    def isBackwards(self, type):
        forwardcost = self.getForwardCost(type)
        backwardcost = self.getBackwardCost(type)
        
        return backwardcost < forwardcost
        
    def getEndLinkFwd(self):
        return self.forwardlinks[0]
    
    def getEndLinkBwd(self):
        #with open('result175.txt', 'a') as file, contextlib.redirect_stdout(file):
            answer = self.backwardlinks[-1]
            #print(self.backwardlinks)
            #print(answer)
            return self.backwardlinks[-1]
        
    
    def isCostEffective(self, type, cost_mu):
        
        forwardcost = self.getForwardCost(type)
        backwardcost = self.getBackwardCost(type)
        
        costdiff = backwardcost - forwardcost
        
        # maybe the forward and backward costs will be reversed sometimes
        return costdiff > cost_mu * forwardcost or -costdiff > cost_mu * backwardcost
    
    def isCostEffectiveForLink(self, a, type, cost_mu):
        forwardcost = self.getForwardCost(type)
        backwardcost = self.getBackwardCost(type)
        
        costdiff = 0
        
        
        if a == self.getEndLinkBwd():
            #print(a)
            costdiff = backwardcost - forwardcost
            return costdiff > cost_mu * forwardcost
        elif a == self.getEndLinkFwd():
            costdiff = forwardcost - backwardcost
            return costdiff > cost_mu * backwardcost

        return False
    
    def getForwardCost(self, type):
        forwardcost = 0
        
        for l in self.forwardlinks:
            forwardcost += l.getTravelTime(l.x, type)
        
        
        return forwardcost
    
    
    def getBackwardCost(self, type):
        backwardcost = 0
        
        for l in self.backwardlinks:
            backwardcost += l.getTravelTime(l.x, type)
        
        
        return backwardcost
    
    
    def isFlowEffective(self, type, flow_mu):
        # min flow of high cost segment
        # high cost segment is backwards links
        minflow = 1e9; 
        flowlastsegment = 0
        
        lookat = self.backwardlinks
        
        if self.isBackwards(type):
            lookat = self.forwardlinks
            
        for l in lookat:
            # only look at high cost segment
            totalFlow = 0
            for r in sorted(self.relevant, key=lambda x: x.id):
                totalFlow += r.bush.flow[l]

            minflow = min(minflow, totalFlow)
            if l.end == self.end:
                flowlastsegment = totalFlow
            
        
        
        return minflow >= flow_mu * flowlastsegment
        
    def maxForwardBushFlowShift(self, bush):
        max = 1E9
        #with open('forwardBush4.txt', 'a') as file, contextlib.redirect_stdout(file):
        for l in self.forwardlinks:
            #print(self.forwardlinks)
            # check flow on link if l in backwards direction
            #print(max)
            max = min(max, bush.flow[l])
            #print(bush.flow[l])
            #print(f"The result is {max}------------------")

        
        return max
    
    
    def maxBackwardBushFlowShift(self, bush):
        max = 1E9
        #print(self.backwardlinks)
    #with open('maxB2.txt', 'a') as file, contextlib.redirect_stdout(file):
        for l in self.backwardlinks:

            # check flow on link if l in backwards direction
            max = min(max, bush.flow[l])
            #print(f"max {max}-----")
            #print(l.id, bush.flow[l], bush.origin)
            

        
        return max
    
    def maxForwardFlowShift(self):
        
        maxFlowPerBush = {}
        
        #for r in self.relevant:
        for r in sorted(self.relevant, key=lambda x: x.id):
            maxFlowPerBush[r.bush] = self.maxForwardBushFlowShift(r.bush)
        
        
        return maxFlowPerBush
    
    
    
    def maxBackwardFlowShift(self):
        #with open('result139.txt', 'a') as file, contextlib.redirect_stdout(file):
            maxFlowPerBush = {}
            #sorted_relevant = sorted(self.relevant, key=lambda r: r.id)
            
            #print(f"before sorting {self.relevant}")
            #print(self.relevant)
            for r in sorted(self.relevant, key=lambda r: r.id):
                #print(self.forwardlinks, self.backwardlinks, sorted(self.relevant, key=lambda r: r.id))
                #print(f"r.id is {r.id}")
                #print(f"r is {r}")
                maxFlowPerBush[r.bush] = self.maxBackwardBushFlowShift(r.bush)
                #print(maxFlowPerBush[r.bush])

            return maxFlowPerBush
    
    def flowShift(self, type, cost_mu, flow_mu, line_search_gap):
        
        forwardcost = self.getForwardCost(type)
        backwardcost = self.getBackwardCost(type)
        
        costdiff = backwardcost - forwardcost
        
        
        
        backwards = 1
        if backwardcost < forwardcost:
            backwards = -1
            
        #print("flow shift? " +str(costdiff)+" "+str(forwardcost)+" "+str(backwardcost)+" "+str(cost_mu)+" "+str(backwards))
        
        # maybe the forward and backward costs will be reversed sometimes
        if (backwards == 1 and costdiff < cost_mu * forwardcost) or (backwards == -1 and -costdiff < cost_mu * backwardcost):
        
            return False

        
        #print(backwards)

        
        overallMaxShift = 0
        
        maxFlowShift = {}
        
        if backwards > 0:
            #print(backwards)
            maxFlowShift = self.maxBackwardFlowShift()
        
        else:
            maxFlowShift = self.maxForwardFlowShift()
        
        
        for b in maxFlowShift:
            overallMaxShift += maxFlowShift[b]
        
        
        if overallMaxShift < flow_mu:
            return False
        

        #print("max shift "+str(overallMaxShift)+" "+str(backwards)+" "+str(backwardcost)+" "+str(forwardcost))
        
        #print("backwards")
        #for l in self.backwardlinks:
        #    for r in self.relevant:
        #        print("\t"+str(l.start)+" "+str(l.end)+" "+str(r)+" "+str(r.bush.flow[l]))
        #print("forwards")
        #for l in self.forwardlinks:
        #    for r in self.relevant:
        #        print("\t"+str(l.start)+" "+str(l.end)+" "+str(r)+" "+str(r.bush.flow[l]))

        #for r in self.relevant:
        #    print(str(r)+" "+str(maxFlowShift[r.bush]))
        
        bot = 0
        top = overallMaxShift
        #with open('flowShift3.txt', 'a') as file, contextlib.redirect_stdout(file):
        while top - bot > line_search_gap:
            #print(line_search_gap)
            mid = (top + bot)/2
            
            check = self.getTT(mid * backwards, type)
            #print(mid * backwards)
            #print("\t"+str(bot)+" "+str(top)+" "+str(mid)+" "+str(check))
            
            if check*backwards < 0:
                bot = mid
            
            else:
                top = mid



    #with open('result132.txt', 'a') as file, contextlib.redirect_stdout(file):
        for l in self.forwardlinks:

            for r in sorted(self.relevant, key=lambda x: x.id):
                # proportion allocated to bush is bush max shift / total max shift
                r.bush.addFlow(l, maxFlowShift[r.bush] / overallMaxShift * bot * backwards)
                #print(f"maxFlowShift[r.bush] {maxFlowShift[r.bush]}")
                #print(f"overallMaxShift is {overallMaxShift}")
                #print(f"The bot is {bot}")
                #print(f"The backwards is {backwards}")
        
        
        
        for l in self.backwardlinks:
            for r in sorted(self.relevant, key=lambda x: x.id):
                # proportion allocated to bush is bush max shift / total max shift
                #print(-maxFlowShift[r.bush] / overallMaxShift * bot * backwards)
                #print(f'-maxFlowShift[r.bush] {-maxFlowShift[r.bush]}, overallMaxShift {overallMaxShift},bot{bot},backwards{backwards}')
                r.bush.addFlow(l, -maxFlowShift[r.bush] / overallMaxShift * bot * backwards)
            
        #print(self.getForwardCost(type), self.getBackwardCost(type), self.getForwardCost(type)-self.getBackwardCost(type))
        #print(bot, overallMaxShift)
        
        #print("after shift "+str(bot)+" "+str(self.getTT(0, type)))
        
        return True