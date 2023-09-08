from src import Autograde
from src import Link
from src import Node
from src import Path
from src import Network


def test():

    network = Network.Network("SiouxFalls")
                
    links = network.getLinks()
        
    for i in range(0, len(links)):
        links[i].setFlow(1021 + i * 500)
        
        print(network.getTSTT())
        print(network.getSPTT())
        print(network.getTotalTrips())
        print(network.getAEC())
        
    autograde()
    
    
def autograde():
    auto = Autograde.Autograde()

    network = Network.Network("SiouxFalls")

    links = network.getLinks()

    for i in range(0, len(links)):
        links[i].setFlow(1021 + i * 500)
    
    auto.test(abs(network.getTSTT() - 8.007500975406816E8) < 1)

    auto.flush("getTSTT()")

    auto.test(abs(network.getSPTT() - 7.666724374221587E7) < 1)

    auto.flush("getSPTT()")

    auto.test(abs(network.getTotalTrips() - 360600.0) < 1)

    auto.flush("getTotalTrips()")

    auto.test(abs(network.getAEC() - 2007.9946028798274) < 0.1)

    auto.flush("getAEC()")

    auto.end()
        
