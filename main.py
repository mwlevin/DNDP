# -*- coding: utf-8 -*-
"""
Created on Wed Mar 27 18:46:55 2024

@author: david
"""

#---modules
from src import Network
from src import Leblanc

#net = 'SF'
#ins = 'DNDP_10_1'
#data = inout.read_instance(net,ins,0.5,500,1e-0,1e-3,600)

network = Network.Network("SiouxFalls",0.5,500,1e-0,1e-3,600)
#network = Network.Network("grid3",1,1,1,1,1)
#print(net,ins)
#print()


y1 = {ij:1 for ij in network.links2}
y0 = {ij:0 for ij in network.links2}
l0 = {ij:0 for ij in network.links2}

#---initialize link flows in the network (0 is default)
x0 = {ij:0 for ij in network.links2}

#---arbitrary (nonzero) lambda
#lbd = {(i,j):1 for (i,j) in network.links2}

y1[network.findLink(network.findNode(11), network.findNode(150))] = 0

#print(network.tapas('SO', y1))





#print(network.tapas('SO', y1))

#for ij in network.links2:
#    print(ij, ij.x)



timelimit = 600

leblanc = Leblanc.Leblanc(network, timelimit)

leblanc.BB()