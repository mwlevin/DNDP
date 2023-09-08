from src import Autograde
from src import Link
from src import Node


def test():

    n1 = Node.Node(1)
    n2 = Node.Node(2)
    n3 = Node.Node(3)
    n4 = Node.Node(4)

    array = [
        Link.Link(n1, n2, 10, 2580, 0.15, 4),
        Link.Link(n1, n3, 11, 1130, 0.15, 4),
        Link.Link(n2, n4, 6, 1280, 0.15, 4),
        Link.Link(n3, n4, 12, 1900, 0.35, 2),
        Link.Link(n2, n3, 13, 1460, 0.15, 4)
    ]

    print(str(n1.getId()) + " " + str(n1.getOutgoing()))
    print(str(n2.getId()) + " " + str(n2.getOutgoing()))
    print(str(n3.getId()) + " " + str(n3.getOutgoing()))
    print(str(n4.getId()) + " " + str(n4.getOutgoing()))

    for ij in array:
        print(str(ij) + " " + str(ij.getStart()) + " " + str(ij.getEnd()))

    print(n1.getOutgoing())
    print(n2.getOutgoing())
    print(n3.getOutgoing())
    print(n4.getOutgoing())
               
    autograde()

def autograde():
        
    n1 = Node.Node(1)
    n2 = Node.Node(2)
    n3 = Node.Node(3)
    n4 = Node.Node(4)

    flows = [1200, 1300, 1400, 1500, 1600]

    array = [
        Link.Link(n1, n2, 10, 2580, 0.15, 4),
        Link.Link(n1, n3, 11, 1130, 0.15, 4),
        Link.Link(n2, n4, 6, 1280, 0.15, 4),
        Link.Link(n3, n4, 12, 1900, 0.35, 2),
        Link.Link(n2, n3, 13, 1460, 0.15, 4)
    ]

    starts = [n1, n1, n2, n3, n2]
    ends = [n2, n3, n4, n4, n3]

    for i in range(0, len(array)):
        array[i].setFlow(flows[i])       

    auto = Autograde.Autograde()

    for i in range(0, len(array)):
        auto.test(array[i].getStart() == starts[i])

    auto.flush("Node.getStart()")

    for i in range(0, len(array)):
        auto.test(array[i].getEnd() == ends[i])

    auto.flush("Node.getEnd()")

    auto.test(n1.getId() == 1)
    auto.test(n2.getId() == 2)
    auto.test(n3.getId() == 3)
    auto.test(n4.getId() == 4)

    auto.flush("Node.getId()")

    auto.test(str(n1) == "1")
    auto.test(str(n2) == "2")
    auto.test(str(n3) == "3")
    auto.test(str(n4) == "4")

    auto.flush("Node.toString()")

    auto.test(str(array[0]) == "(1, 2)")
    auto.test(str(array[1]) == "(1, 3)")
    auto.test(str(array[2]) == "(2, 4)")
    auto.test(str(array[3]) == "(3, 4)")
    auto.test(str(array[4]) == "(2, 3)")

    auto.flush("Link.toString()")

    auto.test(str(n1.getOutgoing()) == "[(1, 2), (1, 3)]" or str(n1.getOutgoing()) == "[(1, 3), (1, 2)]")
    auto.test(str(n2.getOutgoing()) == "[(2, 3), (2, 4)]" or str(n2.getOutgoing()) == "[(2, 4), (2, 3)]")
    auto.test(str(n3.getOutgoing()) == "[(3, 4)]")
    auto.test(str(n4.getOutgoing()) == "[]")

    auto.flush("Node.getOutgoing()")

    auto.end()
