from src import Autograde
from src import Link
from src import Node
from src import Zone


def test(): 
    n1 = Zone.Zone(1)
    n2 = Zone.Zone(2)
    n3 = Node.Node(3)
    n4 = Node.Node(4)


    print(n1.getId())
    print(n2.getId())

    n1.addDemand(n2, 15)
    n1.addDemand(n3, 20)
    n1.addDemand(n3, 9)
    n2.addDemand(n3, 19)
    n2.addDemand(n4, 23)

    print(n1.getDemand(n2))
    print(n1.getDemand(n3))
    print(n1.getDemand(n4))
    print(n2.getDemand(n1))
    print(n2.getDemand(n3))
    print(n2.getDemand(n4))

    print(n1.getProductions())
    print(n2.getProductions())

    n1.setThruNode(False)

    print(n1.isThruNode())
    print(n2.isThruNode())


    autograde()

def autograde():
    auto = Autograde.Autograde()
        
    n1 = Zone.Zone(1)
    n2 = Zone.Zone(2)
    n3 = Node.Node(3)
    n4 = Node.Node(4)
        
    auto.test(n1.getId() == 1)
    auto.test(n2.getId() == 2)
        
    auto.flush("Zone constructor")
        
    n1.addDemand(n2, 15)
    n1.addDemand(n3, 20)
    n1.addDemand(n3, 9)
    n2.addDemand(n3, 19)
    n2.addDemand(n4, 23)
        
    auto.test(n1.getDemand(n2) == 15)
    auto.test(n1.getDemand(n3) == 29)
    auto.test(n1.getDemand(n4) == 0)
    auto.test(n2.getDemand(n1) == 0)
    auto.test(n2.getDemand(n3) == 19)
    auto.test(n2.getDemand(n4) == 23)
        
    auto.flush("Zone.addDemand() and Zone.getDemand()")
        
    auto.test(n1.getProductions() == 44)
    auto.test(n2.getProductions() == 42)
        
    auto.flush("getProductions()")
        
    n1.setThruNode(False)
        
    auto.test(not n1.isThruNode())
    auto.test(n2.isThruNode())
        
    auto.flush("Zone.isThruNode()")
        
    auto.end()
    
