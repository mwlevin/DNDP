# Created on : Mar 27, 2024, 5:02:24 PM
# Author     : michaellevin

from src import Params
import collections import deque

class Bush:
    def __init__(self, network, origin):
        self.network = network
        self.flow = {}
        self.relevantPAS = []
        self.sorted = []
        self.branches = []
        self.origin = origin
        origin.bush = self
        
        for l in self.network.links:
            self.flow[l] = 0
            
        self.loadDemand()
        
        def contains(self, l):
            return self.flow[l] > Params.flow_epsilon
            
        def containsNode(n):
            if n == origin:
                return True
            

            for l in n.getIncoming():
                if contains(l):
                    return True

            return False
     
    def topologicalSort(self):

        for n in network.nodes:
            n.in_degree = n.getBushIncoming(self).size()
            n.visited = False
            n.top_order = -1
        
        
        queue = deque()
        queue.append(origin)
        origin.visited = True
        
        idx = 0
        
        while len(queue) > 0:
        
            vertex = queue.pop()
            sorted.add(vertex)
            vertex.top_order = idx
            idx += 1
            
            for ij in vertex.getBushOutgoing(self)
            
                j = ij.getDest()
                
                
                if notj.visited:
                
                    j.in_degree -= 1
                    
                    if j.in_degree == 0
                    
                        queue.append(j)
                        j.visited = true


        # check for nodes that were not completed
        for n in network.nodes:
        
            if n.in_degree < n.getBushIncoming(self).size() and not n.visited:
                print("NOT DAG")



    def validateFlowConservation():

        for n in network.nodes:
            inflow = 0
            outflow = 0
            
            for l in n.getIncoming():
                inflow += self.flow[l]
            
            for l : n.getOutgoing():
                outflow += flow[l]
            
            if n == origin:
                inflow += origin.getTotalDemand()
            
            elif n instanceof Zone:
                outflow += origin.getDemand(n)
            

            if abs(inflow-outflow) > Params.flow_epsilon:
                return False

        return True

    
    def testTopologicalSort():
    
        topologicalSort()
        
        for l in flow:
            if contains(l) and l.getSource().top_order > l.getDest().top_order:
                return False;

        return True
    
    


    
    def checkReducedCosts(self):
    
        minPath();
        
        output = True
        for l in network.links)
        
            if flow[l] > Params.flow_epsilon and l.getReducedCost() < -Params.bush_gap:
                
                    print("Negative reduced cost origin "+str(origin.id)
                    print(str(l)+"\t"+str(flow[l])+"\t"+str(l.getSource().cost)+"\t"+str(l.getTT())+"\t"+str(l.getDest().cost) + "\t"+ str(l.getDest().cost - (l.getSource().cost + l.getTT())))
                output = False

        return output;
    
    
    
    
    
    public Tree minUsedPath()
    {
        for(Node u : network.nodes)
        {
            u.cost = Params.INFTY;
            u.pred = null;
        }
        
        origin.cost = 0;
        
        origin.cost = 0;
        
        for(Node u : sorted)
        {
            for(Link uv : u.getBushOutgoing(this))
            {
                if(flow[uv.getIdx()] == 0)
                {
                    continue;
                }
                
                Node v = uv.getDest();
                double temp = uv.getTT() + u.cost;
                
                if(temp < v.cost)
                {
                    v.cost = temp;
                    v.pred = uv;
                }
            }
        }
        
        Tree output = new Tree(origin, network);
        
        for(Node n : sorted)
        {
            output.put(n.pred);
        }
        
        return output;
    }
    
    
    public Branch createBranch(Link endlink){
        
        
        
        
        Set<Link> minpath = tracePathSet(origin, endlink.getDest());
        
        Branch output = new Branch(this, endlink, minpath);
        
        return output;
    }
    
    

    
    def tracePath(self, i, j):
        output = []
        
        curr = j;
        
        while curr != i:

            output.add(curr.pred)
            curr = curr.pred.getSource()
        
        return output
    
    def tracePath2(self, i, j):
        output = []
        
        curr = j;
        
        while curr != i:

            output.add(curr.pred)
            curr = curr.pred2.getSource()
        
        return output
    
    def tracePathSet(self, i, j):
        output = set()
        
        curr = j;
        
        while curr != i:

            output.add(curr.pred)
            curr = curr.pred.getSource()
        
        return output
    
   
    
    public Tree minPath()
    {
        for(Node u : network.nodes)
        {
            u.cost = Params.INFTY;
            u.pred = null;
        }
        
        origin.cost = 0;
        
        
        for(Node u : sorted)
        {
            
            for(Link uv : u.getBushOutgoing(this))
            {
                
                Node v = uv.getDest();
                
                double temp = uv.getTT() + u.cost;
                
                if(temp < v.cost)
                {
                    v.cost = temp;
                    v.pred = uv;
                }
            }

        }


        Tree output = new Tree(origin, network);
        
        for(Node n : sorted)
        {
            output.put(n.pred);
        }
        
        return output;
    }
    
    
    // look for links that need to be included in a PAS
    public void checkPAS(){
        
        Tree minPathTree = network.getSPTree(origin);
        
        // only create 1 PAS per link per origin per iteration
        Set<Link> included = new HashSet<>();
        
        // look for all used links not part of the tree of least cost routes
        // search in backwards topological order
        for(int i = sorted.size()-1; i >= 0; i--){
            Node n = sorted.get(i);
        
            for(Link l : n.getIncoming()){
                // check for links with high reduced cost and positive flow, not just links not on the shortest path
                if(!included.contains(l) && l.getDest() != origin && l.getSource() != origin && getFlow(l) > Params.flow_epsilon && l.hasHighReducedCost(Params.pas_cost_mu)){

                    //System.out.println(l+" "+l.getDest().cost+" "+l.getSource().cost+" "+(l.getDest().cost-l.getSource().cost)+" "+l.getTT());
                    // we need a PAS!
                    if(!hasRelevantPAS(l)){

                        // should check if we can borrow one from network
                        PAS fromNetwork = network.findPAS(l, this);
                        if(fromNetwork == null){

                            if(Params.PRINT_PAS_INFO){
                                System.out.println("\nCreate PAS for "+l+" for origin "+getOrigin());
                            }

                            PAS newPAS = createPAS(minPathTree, l, getFlow(l)*Params.pas_flow_mu);
                            if(newPAS == null){
                                // branch shift
                                if(Params.PRINT_PAS_INFO){
                                    System.out.println("branch shift!");
                                }


                                Branch branch = createBranch(l);

                                branches.add(branch);

                            }
                            else{
                                for(Link ij : newPAS.getBackwardsLinks()){
                                    included.add(ij);
                                }
                            }
                        }
                        else{
                            if(Params.PRINT_PAS_INFO){
                                System.out.println("Take PAS for "+l);
                            }


                            fromNetwork.addRelevantOrigin(origin);
                        }
                    }
                }
            }
        }
        
    }
    
    def branchShifts(self):
        
        
        for b in self.branches):
            b.init()
            b.flowShift()
            
        
        branches = []
    
    
    public boolean hasRelevantPAS(Link a){
        if(!relevantPAS.containsKey(a)){
            return false;
        }
        for(PAS p : relevantPAS.get(a)){
            if(p.isCostEffective(a) && p.isFlowEffective()){
                return true;
            }
        }
        
        return false;
    }
    
    public void removePAS(PAS p){
        relevantPAS.remove(p);
    }
    
    
    // create a PAS for link a
    public PAS createPAS(Tree minPathTree, Link a, double minflow){


        PAS output = new PAS();
        
        // min path to a.dest
        Set<Node> minPath = minPathTree.getPathAsNodeSet(a.getDest());
        
        if(Params.PRINT_PAS_INFO){
            System.out.println("minPath is "+minPathTree.getPath(a.getDest()));
        }
        
        // store trace to avoid repeating breadth first search
        Map<Link, Link> trace = new HashMap<>();
        
        Queue<Link> unvisited = new LinkedList<>();
        
        unvisited.add(a);
        
        Link firstSimilar = null;
        
        while(!unvisited.isEmpty()){
            Link jk = unvisited.remove();
            Node j = jk.getSource();
            
            if(minPath.contains(j)){
                firstSimilar = jk;
                break;
            }
            
            
            for(Link ij : j.getIncoming()){
                
                if(getFlow(ij) > minflow && ij.getSource().top_order < j.top_order){
                    unvisited.add(ij);
                    trace.put(ij, jk);
                }
            }
        }
        
        
        
        if(firstSimilar == null){
            return null;
        }
        
        if(Params.PRINT_PAS_INFO){
            System.out.println("firstSimilar is "+firstSimilar);
            System.out.println(trace+" "+minflow+" "+getFlow(a)+" "+firstSimilar);
        }
        
        
        

        
        // trace firstSimilar to a in used flow bush: this is the backward side of the PAS
        // if this loop breaks it's probably because we shifted flow and tried to find a PAS after shifting flow before re-running shortest path
        Link curr = firstSimilar;
        output.addBackwardLink(firstSimilar);
        while(curr != a){
            
            //System.out.println("\t"+curr);
            curr = trace.get(curr);
            output.addBackwardLink(curr);
            
        }
        
        
        // trace firstSimilar to a in min path tree: this is the forward side of the PAS
        for(Link l : minPathTree.trace(firstSimilar.getSource(), a.getDest())){
            output.addForwardLink(l);
        }
        
        output.setStart(firstSimilar.getDest());
        
        
        if(Params.PRINT_PAS_INFO){
            System.out.println("PAS is "+output);
        }
        
        output.addRelevantOrigin(origin);
        
        network.addPAS(output);
        
        return output;
    }
    
    public PASList getRelevantPAS(){
        return relevantPAS;
    }
    
    public void removeCycles(){
        
        // right now this restarts the entire loop when a cycle is detected. I think we don't need to restart everything...
        outer:while(true){
            for(Node n : network.nodes){
                n.visited = false;
                n.pred2 = null;
                n.top_order = -1;
            }
            
            sorted.clear();
            
            int idx = network.nodes.length-1;
            
            Stack unvisited = new Stack();
            unvisited.add(origin);


            while(!unvisited.isEmpty()){
                Object o = unvisited.pop();
                
                if(o instanceof Node){
                    Node n = (Node)o;

                    if(n.top_order >= 0){
                        continue;
                    }
                    else if(n.visited){
                        // remove the cycle
                        
                        removeCycle(n);
                        

                        continue outer;
                    }
                    else{
                        n.visited = true;

                        unvisited.push(new NodeReturn(n));

                        for(Link l : n.getOutgoing()){
                            if(contains(l)){
                                Node j = l.getDest();
                                j.pred2 = l;
                                unvisited.add(j);
                            }
                        }
                    }
                }
                else{
                    Node n = ((NodeReturn)o).node;
                    if(n.top_order < 0){
                        sorted.add(n);
                        n.top_order = idx--;
                    }
                }
            }
            
            break outer;
        }
        
        /*
        for(Node n : network.nodes){
            System.out.println(n+" "+n.pred2);
        }
        System.out.println(sorted);
        */
    }
    
    
    
    class NodeReturn{
        public Node node;
        
        public NodeReturn(Node n){
            node = n;
        }
        
        public String toString(){
            return "sort "+node;
        }
    }
    
    // n is the root node of the cycle
    private void removeCycle(Node n){
        
        
        List<Link> list = new ArrayList<>();
        
        Node curr = n;
        
        
        do{
            //System.out.println("\t"+curr+" "+curr.pred);
            Link pred = curr.pred2;
            list.add(pred);
            
            curr = pred.getSource();
            
            
        }
        while(curr != n);
        
        n.pred2 = null;
        
        double maxflow = Double.MAX_VALUE;
        
        for(Link l : list){
            maxflow = Math.min(maxflow, getFlow(l));
        }
        
        //System.out.print("removing "+maxflow+" from ");
        for(Link l : list){
            addFlow(l, -maxflow);
            
            //System.out.print(l+", ");
            
            /*
            // if link is no longer included in bush after removing flow, we need to recheck the visited indices for the destination node
            if(!addFlow(l, -maxflow)){
                // ok to do it within this loop because each node only has at most 1 incoming link in the cycle
                
                Node j = l.getDest();
                
                //System.out.println("Check connectivity for "+j);
                
                boolean contains = false;
                for(Link ij : j.getIncoming()){
                    //System.out.println("\t"+ij+" "+getFlow(ij)+" "+contains(ij)+" "+ij.getSource().visited);
                    if(contains(ij) && ij.getSource().visited){
                        j.pred2 = ij;
                        //System.out.println("New pred is "+ij);
                        contains = true;
                        break;
                    }
                }
                */
                /*
                j.visited = contains;
                if(!contains){
                    j.pred = null;
                }
            }*/
        }
        //System.out.println();
        
    }
    
    
    public Tree maxUsedPath()
    {
        for(Node u : network.nodes)
        {
            u.cost = -Params.INFTY;
            u.pred = null;
        }
        
        origin.cost = 0;
        
        origin.cost = 0;
        
        for(Node u : sorted)
        {
            for(Link uv : u.getBushOutgoing(this))
            {
                if(flow[uv.getIdx()] == 0)
                {
                    continue;
                }
                
                Node v = uv.getDest();
                double temp = uv.getTT() + u.cost;
                
                if(temp > v.cost)
                {
                    v.cost = temp;
                    v.pred = uv;
                }
            }
        }
              
        Tree output = new Tree(origin, network);
        
        for(Node n : sorted)
        {
            output.put(n.pred);
        }
        
        return output;
    }
    
    public void equilibrate(double bush_gap)
    {
        topologicalSort();
        
        if(Params.printBushEquilibrate)
        {
            System.out.println("Origin "+origin);
        }
        
        int swapIter = 0;
        
        double difference = Params.INFTY;
        
        for(int bushIter = 1; !(bushIter > 2 && swapIter < 2); bushIter++)
        {
            difference = 0.0;
            
            swapIter = 0;
            
            
            
            do
            {
                
                difference = swapFlows();
                
                if(Params.printBushEquilibrate)
                {
                    System.out.println("\t\tSwap: "+String.format("%.2f", difference));
                }
                
                swapIter ++;
            }
            while(!checkReducedCosts());
            //while(difference > bush_gap);
            
            if(!checkReducedCosts(true))
            {
                Tree min = minPath();
                Tree max = maxUsedPath();
                
                int id = 11;
                Path minPath = min.getPath(network.findNode(id));
                System.out.println(minPath+"\t"+minPath.getTT());
                Path maxPath = max.getPath(network.findNode(id));
                System.out.println(maxPath +"\t"+maxPath.getTT()+"\t"+getMaxFlow(maxPath));
                
                System.out.println(swapFlow(minPath, maxPath));
                
                System.out.println(minPath+"\t"+minPath.getTT());
                System.out.println(maxPath +"\t"+maxPath.getTT()+"\t"+getMaxFlow(maxPath));
                
                checkReducedCosts();
                
                System.exit(0);
            }
            
            if(Params.printBushEquilibrate)
            {
                System.out.println("\t"+bushIter+"\t"+difference+"\t"+testTopologicalSort());
            }
            
            improveBush();
        }
        
        
        //validateDemand();
    }
    
    public double getMaxFlow(Path path)
    {
        double max_moved = Params.INFTY;
        for(Link l : path)
        {
            max_moved = Math.min(flow[l.getIdx()], max_moved);
        }
        return max_moved;
    }
    

    public double swapFlows()
    {

        Tree max = maxUsedPath();
        Tree min = minPath();
        

        // start and end of common path segments
        Node m = null;
        Node n = null;
        
        double max_diff = 0.0;
        
        for(int ids = network.getFirstDest(); ids <= network.getLastDest(); ids++)
        {
            
            Node s = network.nodes[ids];
            
            
            if(origin.getDemand((Dest)s) == 0)
            {
                continue;
            }
            
            Path min_path = min.getPath(s);
            Path max_path = max.getPath(s);
            
            max_diff = Math.max(max_diff, swapFlow(min_path, max_path));
            
            /*
            
            n = s;
            while(n != origin)
            {
                
                Iterable<Link> min_iter = min.iterator(n);
                Iterable<Link> max_iter = max.iterator(n);

                Set<Node> visited_min = new HashSet<Node>();


                for(Link l : min_iter)
                {
                    visited_min.add(l.getSource());
                }
                
                
                for(Link l : max_iter)
                {
                    if(visited_min.contains(l.getSource()))
                    {
                        m = l.getSource();
                        break;
                    }
                }
                
                if(m==null)
                {
                    System.out.println("origin="+origin);
                    
                    
                    System.out.println("n="+n+" "+n.getBushIncoming(this)+"\t"+n.top_order+"\t"+n.cost);
                    
                    
                    int id = 164;
                    Node node = network.findNode(id);
                    System.out.println(node+"\t"+node.getBushIncoming(this)+"\t"+node.getBushOutgoing(this)+"\t"+
                            node.top_order+"\t"+node.cost+"\t"+sorted.contains(node)+"\t"+node.in_degree);
                    id = 162;
                    node = network.findNode(id);
                    System.out.println(node+"\t"+node.getBushIncoming(this)+"\t"+node.getBushOutgoing(this)+"\t"+
                            node.top_order+"\t"+node.cost+"\t"+sorted.contains(node)+"\t"+node.in_degree);
                    System.out.println(visited_min);
                    
                    Link link = network.findLink(162, 164);
                    System.out.println(link+"\t"+link.getTT()+"\t"+flow[link.getIdx()]);
                    
                    link = network.findLink(164, 162);
                    System.out.println(link+"\t"+link.getTT()+"\t"+flow[link.getIdx()]);
                    
                    max_iter = max.iterator(n);
                    
                    for(Link l : max_iter)
                    {
                        System.out.print(l+" ");
                    }
                    System.out.println();
                    
                    throw new RuntimeException("m is null");
                }
                

                min_iter = min.iterator(n);
                max_iter = max.iterator(n);

                Path min_path = new Path(true);
                Path max_path = new Path(true);

                for(Link l : min_iter)
                {
                    min_path.add(l);
                    if(l.getSource() == m)
                    {
                        break;
                    }
                }

                for(Link l : max_iter)
                {
                    max_path.add(l);
                    if(l.getSource() == m)
                    {
                        break;
                    }
                }

                //System.out.println(n+" "+m+" "+min_path+" "+max_path);
                
                max_diff = Math.max(max_diff, swapFlow(min_path, max_path));
                
                
                n = m;
                
            }
            */
        }
        
        return max_diff;
    }
    
    /**
     * return how much flow was swapped
     */
    public double swapFlow(Path min_path, Path max_path)
    {
        if(min_path.equals(max_path))
        {
            return 0.0;
        }
        
        double max_moved = getMaxFlow(max_path);
        
        double stepsize = 1;
        
        
        
        //System.out.println(max_path);
        
        double difference = max_path.getTT() - min_path.getTT();
        
        /*
        if(origin.getId() == 1)
        {
            System.out.println(origin+"\t"+min_path.get(min_path.size()-1).getDest()+"\t"+max_moved);
            System.out.println("\t"+min_path.getTT()+"\t"+difference);
            for(Link l : min_path)
            {
                System.out.println("\t\t"+l+"\t"+l.getReducedCost());
            }
            System.out.println("\t"+max_path.getTT()+"\t"+difference);
            for(Link l : max_path)
            {
                System.out.println("\t\t"+l+"\t"+l.getReducedCost());
            }
        }
        */

        while(max_moved > 0  && difference > Params.flow_epsilon)
        {
 
            double deriv = max_path.getDeriv_TT() + min_path.getDeriv_TT();
            
            double y;
            if(max_moved < Params.flow_epsilon)
            {
                y = max_moved;
            }
            else
            {
                y = Math.min(max_moved, stepsize * difference / deriv);
            }

            
            //System.out.println(deriv+" "+y+" "+difference+" "+stepsize+" "+max_moved);

            for(Link l : max_path)
            {
                addFlow(l, -y);
            }
            
            for(Link l : min_path)
            {
                addFlow(l, y);
            }


            max_moved -= y;

            difference = max_path.getTT() - min_path.getTT();


        }
        

        return difference;
        
    }
    
    public boolean checkCost(Link l)
    {
        return l.getReducedCost() >= 0;
        
    }
    
    public void improveBush()
    {

        minPath();
        
        List<Link> remove = new ArrayList<>();
        
        for(int idx = 0; idx < flow.length; idx++)
        {
            Link l = network.links[idx];
            
            if(flow[idx] < Params.flow_epsilon && !checkCost(l))
            {
                remove.add(l);
            }
        }
        
        for(Link l : remove)
        {
            flow[l.getIdx()] = 0;
        }
        
        /*
        for(int idx = 0; idx < network.links.length; idx++)
        {
            Link l = network.links[idx];
            if(!contains(l) && checkCost(l))
            {
                contains[idx] = true;
            }
        }
        */
        
        topologicalSort();
    }
    
    public void loadDemand()
    {
        network.dijkstras(origin);
        
        for(int ids = network.getFirstDest(); ids <= network.getLastDest(); ids++)
        {
            Node s = network.nodes[ids];
            
            double d = origin.getDemand((Dest)s);
            
            Node curr = s;
            
            /*
            for(Node n : network.nodes)
            {
                if(n.getId() == s.getId())
                {
                    curr = n;
                    break;
                }
            }
            */
            
            
            while(curr != origin)
            {
                Link uv = curr.pred;
                addFlow(uv, d);
                curr = uv.getSource();
            }
        }
        
        topologicalSort();
    }
    
    
    
    
    def validateDemand():

        for s in sorted):
            if s != origin and (s instanceof Dest)):
                d = origin.getDemand(s)
                
                actual = 0
                
                for id in s.getIncoming():
                    actual += flow[id]
                
                
                for dj in s.getOutgoing():
                    actual -= flow[dj]
                
                
                if abs(d - actual) > Params.flow_epsilon:
                    print("Origin "+str(origin)+": demand to "+str(s)+" is "+str(d)+" but flow is "+str(actual))
                    return False;
        return True
    
    
    def addFlow(self, l, x):
        l.x += x
        flow[l] += x;
    
    // maybe store this as a map<Link, list<PAS>> mapping end link of PAS
    public void addRelevantPAS(PAS p){
        relevantPAS.add(p);
    }
    
    
}
   