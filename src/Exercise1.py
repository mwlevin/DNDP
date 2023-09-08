from src import Autograde
from src import Link


def test():
    
    link1 = Link.Link(None, None, 10, 2580, 0.15, 4)
    link2 = Link.Link(None, None, 12, 1900, 0.35, 2)

    link1.setFlow(1320.2)
    print(link1.getTravelTime())

    link2.setFlow(570)
    print(link2.getTravelTime())

    link1.setFlow(0)
    link2.setFlow(2512)

    print(link1.getTravelTime())
    print(link2.getTravelTime())
    
    autograde()


def autograde():
    auto = Autograde.Autograde()

    link1 = Link.Link(None, None, 10, 2580, 0.15, 4)
    link2 = Link.Link(None, None, 12, 1900, 0.35, 2)

    link1.setFlow(1320.2)
    auto.test(abs(link1.getTravelTime() - 10.077538130554997) < 0.01)

    auto.test(abs(link2.getTravelTime() - 12.378) < 0.01)
        
    link1.setFlow(0)
    link2.setFlow(2512)
        

    auto.test(abs(link1.getTravelTime() - 10.0) < 0.01)
    auto.test(abs(link2.getTravelTime() - 19.341441772853184) < 0.01)
    
    auto.flush("Link.getTravelTime()");    
    auto.end()