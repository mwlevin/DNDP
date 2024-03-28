# -*- coding: utf-8 -*-
"""
Created on Wed Mar 27 18:46:55 2024

@author: david
"""

#---modules
from src import Network

#net = 'SF'
#ins = 'DNDP_10_1'
#data = inout.read_instance(net,ins,0.5,500,1e-0,1e-3,600)

network = Network.Network("SiouxFalls",0.5,500,1e-0,1e-3,600)

#print(net,ins)
#print()



y1 = {(i,j):1 for (i,j) in network.links2}
l0 = {(i,j):0 for (i,j) in network.links2}

#---initialize link flows in the network (0 is default)
x0 = {(i,j):0 for (i,j) in network.links2}

#---arbitrary (nonzero) lambda
lbd = {(i,j):1 for (i,j) in network.links2}

network.tapas('UE', l0, y1, x0)

#---solve UE with lambda=0 
Lx, x, tstt = network.msa('UE',l0,y1,x0)
#Lx, x, tstt = test_optim.TAP(data,G,'UE','MSA',l0,y1,x0)
print('Lx: %.1f' % Lx)
print('tstt: %.1f' % tstt)

#---solve SO with lambda=lbd
Lx, x, tstt = network.msa('SO',lbd,y1,x0)
#Lx, x, tstt = test_optim.TAP(data,G,'SO','MSA',lbd,y1,x0)
print('Lx: %.1f' % Lx)
print('tstt: %.1f' % tstt)