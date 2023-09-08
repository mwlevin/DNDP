from src import Autograde
from src import Link
from src import Node
from src import Path
from src import Network


def test():

    network = Network.Network("Braess")


    zones = network.getZones()
    nodes = network.getNodes()
    links = network.getLinks()

    for i in range(0, len(zones)):
        msg = str(zones[i]) + " "
        msg += str(zones[i] == nodes[i]) + " " + str(zones[i].getOutgoing())
        print(msg)
    
    for i in range(len(zones), len(nodes)):
        print(str(nodes[i]) + " " + str(nodes[i].getOutgoing()))
    
    for link in links:
        msg = str(link) + "\t"
        msg += str(link.getCapacity()) + "\t" + str(link.getTravelTime())
        print(msg)
    

    for r in range(0, len(zones)):
        for s in range(0, len(zones)):
            msg = str(zones[r]) + " " + str(zones[s]) + " "
            msg += str(zones[r].getDemand(zones[s]))
            print(msg)

    for i in range(0, len(nodes)):
        if i < len(zones):
            print(network.findNode(nodes[i].getId()) == zones[i])
        
        else:
            print(network.findNode(nodes[i].getId()) == nodes[i])

    print(network.findNode(100))
    print(network.findNode(-1))

    for i in range(0, len(links)):
        start = network.findNode(links[i].getStart().getId())
        end = network.findNode(links[i].getEnd().getId())
        print(network.findLink(start, end) == links[i])

    if len(nodes) > 0:
        print(network.findLink(None, nodes[0]))

    autograde()


def autograde():
    auto = Autograde.Autograde()

    network = Network.Network("Braess")

    


    zones = network.getZones()
    nodes = network.getNodes()
    links = network.getLinks()
    
    auto.test(len(nodes) == 4)
    auto.test(len(links) == 5)
    auto.test(len(zones) == 4)

    auto.flush("reading metadata of \"net.txt\"")

    for i in range(0, 4):
        if i < len(nodes):
            auto.test(zones[i] == nodes[i] and zones[i] is not None)
        else:
            auto.test(False)

    for i in range(len(zones), len(nodes)):
        auto.test(nodes[i] is not None)
    
    auto.flush("construction of zones and nodes")

    for i in range(0, 4):
        if i < len(nodes):
            auto.test(nodes[i].isThruNode() == (i + 1 >= 2))
        else:
            auto.test(False)

    auto.flush("isThruNode() parameter")

    for i in range(0, 4):
        if i < len(nodes):
       
            auto.test(nodes[i].getId() == i+1)
        else:
            auto.test(False)

    auto.flush("correct ids for nodes")

    starts = [1, 2, 1, 3, 2]
    ends = [2, 4, 3, 4, 3]

    for i in range(0, len(starts)):
        if len(links) > i:
            auto.test(links[i].getStart().getId() == starts[i])
        else:            
            auto.test(False)

    auto.flush("start nodes from data file")

    for i in range(0, len(starts)):
        if len(links) > i:
        
            auto.test(links[i].getEnd().getId() == ends[i])
        else:
            auto.test(False)

    auto.flush("end nodes from data file")

    if len(nodes) > 0 and len(links) > 0:
        auto.test(links[0] in nodes[0].getOutgoing())
    else:
        auto.test(False)

    if len(nodes) > 0 and len(links) > 2:
        auto.test(links[2] in nodes[0].getOutgoing())
    else:
        auto.test(False)

    if len(nodes) > 0:
        auto.test(len(nodes[0].getOutgoing()) == 2)
    else:
        auto.test(False)

    if len(nodes) > 1 and len(links) > 1:
        auto.test(links[1] in nodes[1].getOutgoing())
    else:
        auto.test(False)
   
    if len(nodes) > 1 and len(links) > 4:
        auto.test(links[4] in nodes[1].getOutgoing())
    else:
        auto.test(False)

    if len(nodes) > 1:
        auto.test(len(nodes[1].getOutgoing()) == 2)
    else:
        auto.test(False)

    if len(nodes) > 2 and len(links) > 3:
        auto.test(links[3] in nodes[2].getOutgoing())
    else:
        auto.test(False)

    if len(nodes) > 2:
        auto.test(len(nodes[2].getOutgoing()) == 1)
    else:
        auto.test(False)

    if len(nodes) > 3:
        auto.test(len(nodes[3].getOutgoing()) == 0)
    else:
        auto.test(False)

    auto.flush("outgoing links")

    TTs = [1, 50, 45, 1, 5]
    capacities = [100, 1000000, 1000000, 200, 1000000]

    for i in range(0, len(TTs)):
        if len(links) > i:
            auto.test(links[i].getTravelTime() == TTs[i])
        else:
            auto.test(False)
            
    auto.flush("link free flow travel times")

    for i in range(0, len(capacities)):
        if len(links) > i:
            auto.test(abs(links[i].getCapacity() - capacities[i]) < 0.1)
        else:
            auto.test(False)

    auto.flush("link capacities")

    for r in range(0, 4):
        for s in range(0, 4):
            if r < len(zones) and s < len(zones):
                if r == 0 and s == 3:
                    auto.test(zones[r].getDemand(zones[s]) == 4000)
                else:
                    auto.test(zones[r].getDemand(zones[s]) == 0)
            else:
                auto.test(False)

    auto.flush("trips.txt file")



    network = Network.Network("SiouxFalls")


    zones = network.getZones()
    nodes = network.getNodes()
    links = network.getLinks()

    outgoing = [2, 2, 3, 3, 3, 3, 2, 4, 3, 5, 4, 3, 2, 3, 4, 4, 3, 3, 3, 4, 3, 4, 3, 3]

    for i in range(0, 4):
        if i < len(nodes):
            auto.test(len(nodes[i].getOutgoing()) == outgoing[i])
        else:

            auto.test(False)

    auto.flush("outgoing links of Sioux Falls")

    productions = [
        8800.0, 
        4000.0, 
        2800.0, 
        11600.0, 
        6100.0, 
        7600.0, 
        12100.0, 
        16700.0, 
        16200.0, 
        45200.0, 
        22300.0, 
        13900.0, 
        14600.0, 
        14100.0, 
        21400.0, 
        26100.0, 
        23400.0, 
        4800.0, 
        12800.0, 
        18500.0, 
        11000.0, 
        24400.0, 
        14500.0, 
        7700.0
    ]

    for i in range(0, 24):
        if i < len(zones):
            auto.test(zones[i].getProductions() == productions[i])
        else:
            auto.test(False)

    auto.flush("productions of Sioux Falls")

    for i in range(0, 4):
        if i < len(nodes):
            if i < len(zones):
                auto.test(network.findNode(nodes[i].getId()) == zones[i])
            else:
                auto.test(network.findNode(nodes[i].getId()) == nodes[i])
        else:
            auto.test(False)

    auto.flush("Network.findNode()")

    for i in range(0, len(links)):
        start = network.findNode(links[i].getStart().getId())
        end = network.findNode(links[i].getEnd().getId())
        auto.test(network.findLink(start, end) == links[i])

    if len(nodes) > 0:
        auto.test(network.findLink(None, nodes[0]) is None)
    else:
        auto.test(False)

    auto.flush("Network.findLink()")

    auto.end()
