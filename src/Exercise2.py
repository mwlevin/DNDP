from src import Autograde
from src import Link


def test():
    array = [
        Link.Link(None, None, 10, 2580, 0.15, 4),
        Link.Link(None, None, 6, 1100, 0.15, 4),
        Link.Link(None, None, 12, 1900, 0.35, 2),
        Link.Link(None, None, 23, 1280, 0.15, 4)
    ]
        
    for ij in array:
        ij.setFlow(1500)
        
    print(findCongestedLinks(array))
        
    autograde()
    
    # **********
    # Exercise 2(b)
    # ********** 
    findCongestedLinks(array)
    

# return a string with the following:
# travel times of all links. 
# Then for each link, if it has volume/capacity > 1, "yes". 
# Otherwise, "no". 
# Print each link on a separate line. 
def findCongestedLinks(array):     
    output = ""
    for i in range(0, len(array)):

        output += "link " + str(i + 1) + " "
        output += str(array[i].getTravelTime()) + " "
        if array[i].getFlow() / array[i].getCapacity() > 1:
            output += "yes"
        else:
            output += "no"
        output += "\n"     
        
    return output


def autograde():

    auto = Autograde.Autograde()

    capacities = [2580, 1130, 1280, 1900, 1460]
    flows = [1200, 1300, 1400, 1500, 1600]

    array = [
        Link.Link(None, None, 10, 2580, 0.15, 4),
        Link.Link(None, None, 11, 1130, 0.15, 4),
        Link.Link(None, None, 6, 1280, 0.15, 4),
        Link.Link(None, None, 12, 1900, 0.35, 2),
        Link.Link(None, None, 13, 1460, 0.15, 4)
    ]

    for i in range(0, len(array)):
        array[i].setFlow(flows[i])

    answers = [
        10.070200049666536, 
        13.89030436990636, 
        7.287996768951416, 
        14.617728531855956, 
        15.812568567186961
    ]
    answers2 = ["no", "yes", "yes", "no", "yes"]

    for i in range(0, len(array)):
        auto.test(array[i].getCapacity() == capacities[i])

    auto.flush("Link.getCapacity()")

    for i in range(0, len(array)):
        auto.test(array[i].getFlow() == flows[i])

    auto.flush("Link.getFlow()")

    output = findCongestedLinks(array).split("\n")

    for i in range(0, len(array)):
        link_output = output[i].split()

        auto.test(link_output[0] == "link")
        auto.test(int(link_output[1]) == i + 1)
        auto.test(abs(float(link_output[2]) - answers[i]) < 0.01)
        auto.test(link_output[3] == answers2[i])

    auto.flush("findCongestedLinks()")
    auto.end()

        
    