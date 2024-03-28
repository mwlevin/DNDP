# Created on : Mar 28, 2024, 2:08:29 PM
# Author     : michaellevin

class Branch:

    # this is just a placeholder that says this branch has a specific endlink and a minpath. The actual links in the branch will be determined later.
    def __init__(self, bush, endlink, minpath):
        self.bush = bush;
        linkflows = {}
        
        self.endlink = endlink
        self.minpath = minpath
        self.maxflow = 0
        
        
    def init():
        
        
        for n in self.bush.network.nodes:
            n.visited = False
        
        
        unvisited = []
        
        branchlinks = set()
        
        unvisited.add(endlink.start)
        
        while len(unvisited) > 0:
            j = unvisited.pop()
            
            for ij in j.getIncoming():
 
                if bush.contains(ij):
                    i = ij.start
                    
                    branchlinks.add(ij)
                    
                    
                    if not i.visited:
                        unvisited.append(i)
                        i.visited = True

        self.maxflow = self.bush.getFlow(endlink)
        # now do Ford-Fulkerson to figure out branch flow on each link
        # the "capacities" are the bush flow on each link
        # due to conservation of flow I don't need to add flow in reverse. DFS will be sufficient.
        
        
        for l in branchlinks:
            linkflows[l] = 0.0
        
        
        start = self.bush.origin
        end = self.endlink.start
        linkflows[endlink] = self.maxflow
        
        
        assignedFlow = 0
        
        # while there is flow left to assign
        # use flow epsilon to avoid numerical error causing infinite loop
        while self.maxflow - assignedFlow > self.bush.network.params.flow_epsilon:
            
            #System.out.println(maxflow+" "+assignedFlow);
            
            # DFS find path
            unvisited = []
            unvisited.append(start)
            
            for n in self.bush.network.nodes:
                n.visited = False
                n.pred2 = None
            
            start.visited = True
            
            while len(unvisited) > 0:
                i = unvisited.pop()
                

                # once DFS finds a path, stop and add flow. That path will become unusable
                if i == end:
                    break
                
                expanded = []
                for ij in i.outgoing:
                    # only expand links with positive bush flow - temporary branch flow
                    if ij in branchlinks and not ij.end.visited and self.bush.flow[ij] - linkflows[ij] > self.bush.network.params.flow_epsilon:
                        expanded.add(ij)
                
                # sort in order of decreasing flow
                Collections.sort(expanded, new Comparator<Link>(){
                    public int compare(Link i, Link j){
                        double flowi = bush.getFlow(i) - linkflows.get(i);
                        double flowj = bush.getFlow(j) - linkflows.get(j);
                        return (int)Math.ceil(flowj - flowi);
                    }
                });
                
                for ij in expanded:
                    j = ij.end
                    j.pred2 = ij
                    j.visited = True
                    unvisited.append(j)
            
            # trace path and label flows
            augmentedPath = self.bush.tracePath2(start, end)
            
            
            sendFlow = self.maxflow - assignedFlow
            
            for l in augmentedPath:
                sendFlow = min(sendFlow, self.bush.flow[l] - linkflows[l])
            
            
            for l in augmentedPath:
                linkflows[l] = linkflows[l] + sendFlow
            
            assignedFlow += sendFlow
     
     
           
    def flowShift(self):
        avgTT = self.getAvgTT(0)
        minTT = self.getMinTT(0)
        
        #System.out.println("cost difference is "+avgTT+" "+minTT);
        
        # difference is too small to be worth shifting
        if avgTT - minTT < minTT * self.bush.network.params.pas_cost_mu:
            return 0
        
        bot = 0
        top = self.maxflow
        
        while top - bot > self.bush.network.params.line_search_gap:
            mid = (bot+top)/2
            
            newTTDiff = getAvgTT(mid) - getMinTT(mid)
            
            #System.out.println(bot+" "+mid+" "+top+" "+getAvgTT(mid)+" "+getMinTT(mid)+" "+newTTDiff);
            
            if newTTDiff > 0:
                # shift more
                bot = mid
            else
                # shift less
                top = mid
        
        self.propAddFlow(bot)
        
        if self.bush.network.params.PRINT_PAS_INFO:
            print("after shift is "+str(getAvgTT(0))+" "+str(getMinTT(0))+" "+str(maxflow))

        return bot

    
    public double getMinTT(double shift){
        double output = 0;
        
        // this is used in case the min links are also on the branch
        double prop = shift/maxflow;
        
        for(Link l : minpath){
            double newflow = l.getFlow();
            
            // add flow to the minpath
            newflow += shift;
            
            // if link is in branch, some flow will be shifted:
            if(linkflows.containsKey(l)){
                newflow -= prop * linkflows.get(l);
            }
            
            output += l.getTT(newflow);
        }
        
        return output;
        
    }
    
    
    // consider shifting flow from branch to minpath
    public double getAvgTT(double shift){
        double output = 0;
        
        // proportional shift based on how much of the branch flow is on each link
        double prop = shift/maxflow;

        
        for(Link l : linkflows.keySet()){
            
            // subtract the flowshift
            double flowchange = linkflows.get(l)*prop;
            double newflow = l.getFlow() - flowchange;
            
            // if link is on minpath, then add the entire shift to it
            
            if(minpath.contains(l)){
                newflow += shift;
            }


            output += (linkflows.get(l) - flowchange) * l.getTT(newflow);
        }
        
        return output / (maxflow - shift);
    }
    
    public void propAddFlow(double shift){
        double prop = shift/maxflow;

        
        for(Link l : linkflows.keySet()){
            bush.addFlow(l, -linkflows.get(l)*prop);
            
            // this isn't needed: the branch will be discarded after equilibrating.
            if(Params.DEBUG_CHECKS){
                linkflows.put(l, linkflows.get(l) * (1-prop));
            }
        }
        
        for(Link l : minpath){
            bush.addFlow(l, shift);
            
            // this isn't needed: the branch will be discarded after equilibrating.
            if(Params.DEBUG_CHECKS){
                linkflows.put(l, linkflows.get(l) + shift);
            }
        }
        
        maxflow -= shift;
    }