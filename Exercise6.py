import Autograde
import Link
import Node
import Path
import Network


def test():

    network = Network.Network("SiouxFalls")
        
        
    zones = network.getZones()
    nodes = network.getNodes()
    links = network.getLinks()
        
    for i in range(0, len(links)):
        links[i].setFlow(1021 + i*500)
    
        
    path = Path.Path()
    if len(links) > 13:
        path.add(links[0])
        path.add(links[3])
        path.add(links[14])
        path.add(links[12])
        

        print(path.isConnected())
        print(path.getTravelTime())    
        
        
        path2 = Path.Path()
        
        if len(links) > 4:
            for i in range(0, 4):
                path2.add(links[i])
  
        
        print(path2.isConnected())
        print(path2.getTravelTime()) 
        
        print(str(path.getSource()) + " " + str(path.getDest()))
        print(str(path2.getSource()) + " " + str(path2.getDest()))
        
        if len(nodes) > 0:
        
            network.dijkstras(nodes[0])

            for i in range(0, len(nodes)):
                msg = str(nodes[i]) + " " + str(nodes[i].cost)
                msg += " " + str(nodes[i].predecessor)
                print(msg)
                print(network.trace(nodes[0], nodes[i]))

            network.dijkstras(nodes[0])

            for i in range(0, len(nodes)):
                msg = "Path from " + str(nodes[0]) + " to " + str(nodes[i])
                msg += ": " + str(network.trace(nodes[0], nodes[i]))
                print(msg)

    autograde()
    
def autograde():
    auto = Autograde.Autograde()

    network = Network.Network("SiouxFalls")


    zones = network.getZones()
    nodes = network.getNodes()
    links = network.getLinks()

    flowdata = [
        [0, 1, 2],
        [1, 1, 3],
        [2, 2, 1],
        [3, 2, 6],
        [4, 3, 1],
        [5, 3, 4],
        [6, 3, 12],
        [7, 4, 3],
        [8, 4, 5],
        [9, 4, 11],
        [10, 5, 4],
        [11, 5, 6],
        [12, 5, 9],
        [13, 6, 2],
        [14, 6, 5],
        [15, 6, 8],
        [16, 7, 8],
        [17, 7, 18],
        [18, 8, 6],
        [19, 8, 7],
        [20, 8, 9],
        [21, 8, 16],
        [22, 9, 5],
        [23, 9, 8],
        [24, 9, 10],
        [25, 10, 9],
        [26, 10, 11],
        [27, 10, 15],
        [28, 10, 16],
        [29, 10, 17],
        [30, 11, 4],
        [31, 11, 10],
        [32, 11, 12],
        [33, 11, 14],
        [34, 12, 3],
        [35, 12, 11],
        [36, 12, 13],
        [37, 13, 12],
        [38, 13, 24],
        [39, 14, 11],
        [40, 14, 15],
        [41, 14, 23],
        [42, 15, 10],
        [43, 15, 14],
        [44, 15, 19],
        [45, 15, 22],
        [46, 16, 8],
        [47, 16, 10],
        [48, 16, 17],
        [49, 16, 18],
        [50, 17, 10],
        [51, 17, 16],
        [52, 17, 19],
        [53, 18, 7],
        [54, 18, 16],
        [55, 18, 20],
        [56, 19, 15],
        [57, 19, 17],
        [58, 19, 20],
        [59, 20, 18],
        [60, 20, 19],
        [61, 20, 21],
        [62, 20, 22],
        [63, 21, 20],
        [64, 21, 22],
        [65, 21, 24],
        [66, 22, 15],
        [67, 22, 20],
        [68, 22, 21],
        [69, 22, 23],
        [70, 23, 14],
        [71, 23, 22],
        [72, 23, 24],
        [73, 24, 13],
        [74, 24, 21],
        [75, 24, 23]
    ]

    for r in range(0, len(flowdata)):
        i = flowdata[r][0]
        startid = flowdata[r][1]
        endid = flowdata[r][2]

        start = network.findNode(startid)
        end = network.findNode(endid)

        if start is not None and end is not None:
            link = network.findLink(start, end)

            if link is not None:
                link.setFlow(1021 + i*500)
            
    path = Path.Path()

    if len(links) > 12:
        path.add(links[0])
        path.add(links[3])
        path.add(links[14])
        path.add(links[12])
    
    auto.test(path.isConnected());

    path2 = Path.Path()

    if len(links) > 4:
        for i in range(0, 4):
            path2.add(links[i])

    auto.test(not path2.isConnected())

    auto.flush("Path.isConnected()")

    if path.size() > 0:
        auto.test(path.getSource().getId() == 1)
    
    if path2.size() > 0:
        auto.test(path2.getSource().getId() == 1)

    auto.flush("Path.getSource()")

    if path.size() > 0:
        auto.test(path.getDest().getId() == 9)

    if path2.size() > 0:
        auto.test(path2.getDest().getId() == 6)

    auto.flush("Path.getDest()")



    auto.test(abs(path.getTravelTime() - 24.37569064685568) < 0.01)
    auto.test(abs(path2.getTravelTime() - 21.050172255908606) < 0.01)   


    auto.flush("Path.getTravelTime()")


    for n in nodes:
        n.cost = 1;
        n.predecessor = n;

    if len(nodes) > 20:
        network.dijkstras(nodes[20])
        auto.test(nodes[20].predecessor is None)

    for i in range(1, 24):
        if i < len(nodes):
            if i == 20:
                auto.test(nodes[i].cost == 0)
            else:
                auto.test(nodes[i].cost > 1) 
        else:
            auto.test(False)

    for n in nodes:
        n.cost = 1
        n.predecessor = n
        
    if len(nodes) > 10:
        network.dijkstras(nodes[10])
        auto.test(nodes[10].predecessor is None)
    
    for i in range(1, 24):
        if i < len(nodes):
            if i == 10:
                auto.test(nodes[i].cost == 0)
            
            else:
                auto.test(nodes[i].cost > 1)
        else:
            auto.test(False);

    auto.flush("Dijkstra's initialization")


    if len(nodes) > 0:
        network.dijkstras(nodes[0])
    

    costs = [
        0.0, 
        6.000002173366477, 
        4.000010704018527, 
        8.001086579821319, 
        10.002993278827729, 
        11.05012818657351, 
        20.254815898571675, 
        15.796757080158615, 
        15.185238922340416, 
        18.530190870067695, 
        15.441219860476464, 
        8.000533540987554, 
        11.131431652920096, 
        119.42992460379457, 
        25.730656873570418, 
        27.11253935123187, 
        138.53485409204677, 
        22.26303321972979, 
        31.539301160373306, 
        27.58642669899411, 
        650.1485820323414, 
        44.95245518013293, 
        342.23125672182744, 
        158.61247346764176
    ]

    for i in range(0, 24):
        if i < len(nodes):
            auto.test(abs(costs[i] - nodes[i].cost) < 0.01)
        else:
            auto.test(False)

    if len(nodes) > 10:
        network.dijkstras(nodes[10])

    costs = [
        30.561574457187128, 
        35.335356588859646, 
        26.561407873336385, 
        22.558483478099397, 
        20.554540759663677, 
        26.36459203655937, 
        35.569279748557534, 
        31.111220930144476, 
        13.988425749509492, 
        10.587359148869036, 
        0.0, 
        30.56193071030541, 
        33.69282882223795, 
        103.98870474331811, 
        17.787825152371756, 
        42.42700320121773, 
        130.5920223708481, 
        37.57749706971565, 
        23.59646943917464, 
        42.900890548979966, 
        642.2057503111428, 
        37.00962345893427, 
        326.790036861351, 
        181.1738706369596
    ]

    for i in range(0, 24):
        if i < len(nodes):
            auto.test(abs(costs[i] - nodes[i].cost) < 0.01)
        
        else:
            auto.test(False)

    auto.flush("Dijkstra's cost labels")



    if len(nodes) > 0:
        network.dijkstras(nodes[0])
    

    preds = [-1, 1, 1, 3, 4, 2, 8, 6, 5, 9, 4, 3, 12, 11, 10, 18, 10, 7, 15, 18, 22, 15, 14, 13]

    for i in range(0, 24):
        if i < len(nodes):
            auto.test(nodes[i].predecessor == network.findNode(preds[i]))
        
        else:
        
            auto.test(False);

    preds = [3, 6, 4, 5, 9, 5, 8, 6, 10, 11, -1, 3, 12, 11, 10, 18, 10, 7, 15, 18, 22, 15, 14, 13]

    if len(nodes) > 10:
        network.dijkstras(nodes[10])

    for i in range(0, 24):
        if i < len(nodes):
            auto.test(nodes[i].predecessor == network.findNode(preds[i]))
        
        else:
            auto.test(false);

    auto.flush("Dijkstra's predecessor labels")

    traces = [
        "[]", 
        "[(1, 2)]", 
        "[(1, 3)]", 
        "[(1, 3), (3, 4)]", 
        "[(1, 3), (3, 4), (4, 5)]", 
        "[(1, 2), (2, 6)]", 
        "[(1, 2), (2, 6), (6, 8), (8, 7)]", 
        "[(1, 2), (2, 6), (6, 8)]", 
        "[(1, 3), (3, 4), (4, 5), (5, 9)]", 
        "[(1, 3), (3, 4), (4, 5), (5, 9), (9, 10)]", 
        "[(1, 3), (3, 4), (4, 11)]", 
        "[(1, 3), (3, 12)]", 
        "[(1, 3), (3, 12), (12, 13)]", 
        "[(1, 3), (3, 4), (4, 11), (11, 14)]", 
        "[(1, 3), (3, 4), (4, 5), (5, 9), (9, 10), (10, 15)]", 
        "[(1, 2), (2, 6), (6, 8), (8, 7), (7, 18), (18, 16)]", 
        "[(1, 3), (3, 4), (4, 5), (5, 9), (9, 10), (10, 17)]", 
        "[(1, 2), (2, 6), (6, 8), (8, 7), (7, 18)]", 
        "[(1, 3), (3, 4), (4, 5), (5, 9), (9, 10), (10, 15), (15, 19)]", 
        "[(1, 2), (2, 6), (6, 8), (8, 7), (7, 18), (18, 20)]", 
        "[(1, 3), (3, 4), (4, 5), (5, 9), (9, 10), (10, 15), (15, 22), (22, 21)]", 
        "[(1, 3), (3, 4), (4, 5), (5, 9), (9, 10), (10, 15), (15, 22)]", 
        "[(1, 3), (3, 4), (4, 11), (11, 14), (14, 23)]", 
        "[(1, 3), (3, 12), (12, 13), (13, 24)]"
    ]
        
 
    if len(nodes) > 0:
        network.dijkstras(nodes[0]);

    for i in range(0, 24):
        if i < len(nodes):
            auto.test(str(network.trace(nodes[0], nodes[i])) == traces[i])
        else:
            auto.test(False)


    traces = [
        "[(11, 10), (10, 9), (9, 5), (5, 4), (4, 3), (3, 1)]", 
        "[(11, 10), (10, 9), (9, 5), (5, 6), (6, 2)]", 
        "[(11, 10), (10, 9), (9, 5), (5, 4), (4, 3)]", 
        "[(11, 10), (10, 9), (9, 5), (5, 4)]", 
        "[(11, 10), (10, 9), (9, 5)]", 
        "[(11, 10), (10, 9), (9, 5), (5, 6)]", 
        "[(11, 10), (10, 9), (9, 5), (5, 6), (6, 8), (8, 7)]", 
        "[(11, 10), (10, 9), (9, 5), (5, 6), (6, 8)]", 
        "[(11, 10), (10, 9)]", "[(11, 10)]", 
        "[]", 
        "[(11, 10), (10, 9), (9, 5), (5, 4), (4, 3), (3, 12)]", 
        "[(11, 10), (10, 9), (9, 5), (5, 4), (4, 3), (3, 12), (12, 13)]", 
        "[(11, 14)]", "[(11, 10), (10, 15)]", 
        "[(11, 10), (10, 9), (9, 5), (5, 6), (6, 8), (8, 7), (7, 18), (18, 16)]", "[(11, 10), (10, 17)]", 
        "[(11, 10), (10, 9), (9, 5), (5, 6), (6, 8), (8, 7), (7, 18)]", "[(11, 10), (10, 15), (15, 19)]", 
        "[(11, 10), (10, 9), (9, 5), (5, 6), (6, 8), (8, 7), (7, 18), (18, 20)]", 
        "[(11, 10), (10, 15), (15, 22), (22, 21)]", 
        "[(11, 10), (10, 15), (15, 22)]", "[(11, 14), (14, 23)]", 
        "[(11, 10), (10, 9), (9, 5), (5, 4), (4, 3), (3, 12), (12, 13), (13, 24)]"
    ]


    if len(nodes) > 10:
        network.dijkstras(nodes[10]);

    for i in range(0, 24):
        if i < len(nodes):
            auto.test(str(network.trace(nodes[10], nodes[i])) == traces[i])
        
        else:  
            auto.test(False)

    auto.flush("trace() after Dijkstra's")



    costs = [
        0.0, 
        6.000002173366477, 
        4.000010704018527, 
        8.001086579821319, 
        10.002993278827729, 
        11.05012818657351, 
        20.254815898571675, 
        15.796757080158615, 
        15.185238922340416, 
        18.530190870067695, 
        15.441219860476464, 
        8.000533540987554, 
        11.131431652920096, 
        119.42992460379457, 
        25.730656873570418, 
        27.11253935123187, 
        138.53485409204677, 
        22.26303321972979, 
        31.539301160373306, 
        27.58642669899411, 
        650.1485820323414, 
        44.95245518013293, 
        342.23125672182744, 
        158.61247346764176
    ]

    if len(nodes) > 0:
        network.dijkstras(nodes[0])
    
    for i in range(0, 24):
        if i < len(nodes):
            auto.test(abs(network.trace(nodes[0], nodes[i]).getTravelTime() - costs[i]) < 0.01)

    costs = [
        30.561574457187128, 
        35.335356588859646, 
        26.561407873336385, 
        22.558483478099397, 
        20.554540759663677, 
        26.36459203655937, 
        35.569279748557534, 
        31.111220930144476, 
        13.988425749509492, 
        10.587359148869036, 
        0.0, 
        30.56193071030541, 
        33.69282882223795, 
        103.98870474331811, 
        17.787825152371756, 
        42.42700320121773, 
        130.5920223708481, 
        37.57749706971565, 
        23.59646943917464, 
        42.900890548979966, 
        642.2057503111428, 
        37.00962345893427, 
        326.790036861351, 
        181.1738706369596
    ]

    if len(nodes) > 10:
        network.dijkstras(nodes[10])
    
    for i in range(0, 24):
        if i < len(nodes):
            auto.test(abs(network.trace(nodes[10], nodes[i]).getTravelTime() - costs[i]) < 0.01)

    auto.flush("costs of paths found by trace()")

    auto.end()