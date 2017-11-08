#!/usr/bin/python

from collections import namedtuple
from pprint import pprint as pp
from collections import defaultdict
import sys
import copy
import heapq

Node = namedtuple('Node', ['cost', 'shop', 'collected_fish', 'visited_neigh'])
N, M, K = raw_input("").split(" ")
N, M, K = int(N), int(M), int(K)
#print N, M, K
sold_fish = [0]
final_nodes = []
neighbours = defaultdict(list)
for i in range(N):
    v = raw_input("").split(" ")
    v = map(lambda x: int(x), v)
    b = 0x0000
    for j in range(v[0]): 
        b = b | (1<<(v[j+1]-1))
    sold_fish.append(b)
for i in range(M):
    v = raw_input("").split(" ")
    v = map(lambda x: int(x), v)
    neighbours[v[0]].append([v[2],v[1]])
    neighbours[v[1]].append([v[2],v[0]])
for i in neighbours.keys():
    neighbours[i].sort()
all_fish = 2**K - 1
#print bin(all_fish)

fringe = []

dd = defaultdict(int)
nodes_by_shop = defaultdict(list)
initial_node = Node(0, 1, sold_fish[1], dd)
heapq.heappush(fringe, initial_node)

import time
counter=0
big_fish_and = 0
while True:
    t_start = time.time()
    counter = counter + 1
    node = heapq.heappop(fringe)
    #print fringe
    print "Fringe size {}, Exploring cost {}, node {}, collected_fish {}".format(len(fringe), node.cost, node.shop, bin(node.collected_fish))

#    if node.shop == N:
#        #print "Final_node {}".format(node)
#        big_fish_and = big_fish_and & node.collected_fish
#        #print "path found {}, big_fish_or {}".format(bin(node.collected_fish), bin(big_fish_and))
#        fn = copy.deepcopy(final_nodes)
#        for n in fn:
#            if ((n.collected_fish | node.collected_fish) == all_fish):
#                print node.cost
#                sys.exit()
#            if ((n.collected_fish | node.collected_fish) == node.collected_fish):
#                final_nodes.remove(n)
#        final_nodes.append(node)
#        #for n in final_nodes:
#        #    print "path {}".format(bin(n.collected_fish))

    if node.shop == N:
        #print "Final_node {}".format(node)
        final_nodes.append(node)
        for n in final_nodes:
            if ((n.collected_fish | node.collected_fish) == all_fish):
                print node.cost
                #print final_nodes
                sys.exit()


    skipped = 0
    fringe_needs_heapify = False
    for neighbour in neighbours[node.shop]:
        new_shop = neighbour[1]
        visited_neigh = copy.copy(node.visited_neigh)
        if visited_neigh[(node.shop, new_shop)] == 1:
            skipped = skipped + 1
            continue
        visited_neigh[(node.shop, new_shop)] = 1
        new_node = Node(node.cost + neighbour[0],
                        new_shop,
                        node.collected_fish | sold_fish[new_shop],
                        visited_neigh)
        remove = 0
        need_add_node = True
        #similar_nodes = list(filter(lambda x: x.shop == new_node.shop, fringe))
        similar_nodes = nodes_by_shop[new_shop]
        for n in similar_nodes:
            if ((new_node.collected_fish == n.collected_fish) and new_node.cost >= n.cost):
                need_add_node = False
                #break                
            elif ((new_node.collected_fish | n.collected_fish) ==  new_node.collected_fish) and new_node.cost <= n.cost:
                #print "removing {}".format(n.shop)
                #print "        removing cost {}, node {}, fish {}".format(n.cost, n.shop, bin(n.collected_fish))
                fringe.remove(n)
                nodes_by_shop[new_shop].remove(n)
                remove = remove + 1
                fringe_needs_heapify = True
            elif ((new_node.collected_fish | n.collected_fish) == n.collected_fish and new_node.cost >= n.cost):
                need_add_node = False
                #break
        #if remove > 0:
        #    heapq.heapify(fringe)
        if need_add_node:
            #print "    adding cost {}, node {}, fish {}".format(new_node.cost, new_node.shop, bin(new_node.collected_fish))
            heapq.heappush(fringe, new_node)
            nodes_by_shop[new_shop].append(new_node)
    if fringe_needs_heapify:
        heapq.heapify(fringe)
    t_end = time.time()
    #print "counter {}, fringe {}, neighs {}, removed {}, skipped {}, time = {}".format(counter, len(fringe), len(neighbours[node.shop]), remove, skipped, t_end - t_start)
