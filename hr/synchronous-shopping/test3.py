#!/usr/bin/python

from collections import namedtuple
from pprint import pprint as pp
from collections import defaultdict
import sys
import copy
import heapq
import time

class Vertex:
    def __init__(self, node):
        self.id = node
        self.adjacent = {}
        # Set distance to infinity for all nodes
        self.distance = sys.maxint
        # Mark all nodes unvisited        
        self.visited = False  
        # Predecessor
        self.previous = None

    def add_neighbor(self, neighbor, weight=0):
        self.adjacent[neighbor] = weight

    def get_connections(self):
        return self.adjacent.keys()  

    def get_id(self):
        return self.id

    def get_weight(self, neighbor):
        return self.adjacent[neighbor]

    def set_distance(self, dist):
        self.distance = dist

    def get_distance(self):
        return self.distance

    def set_previous(self, prev):
        self.previous = prev

    def set_visited(self):
        self.visited = True

    def __str__(self):
        return str(self.id) + ' adjacent: ' + str([x.id for x in self.adjacent])

class Graph:
    def __init__(self):
        self.vert_dict = {}
        self.num_vertices = 0

    def __iter__(self):
        return iter(self.vert_dict.values())

    def add_vertex(self, node):
        self.num_vertices = self.num_vertices + 1
        new_vertex = Vertex(node)
        self.vert_dict[node] = new_vertex
        return new_vertex

    def get_vertex(self, n):
        if n in self.vert_dict:
            return self.vert_dict[n]
        else:
            return None

    def add_edge(self, frm, to, cost = 0):
        if frm not in self.vert_dict:
            self.add_vertex(frm)
        if to not in self.vert_dict:
            self.add_vertex(to)

        self.vert_dict[frm].add_neighbor(self.vert_dict[to], cost)
        self.vert_dict[to].add_neighbor(self.vert_dict[frm], cost)

    def get_vertices(self):
        return self.vert_dict.keys()

    def set_previous(self, current):
        self.previous = current

    def get_previous(self, current):
        return self.previous

def shortest(v, path):
    ''' make shortest path from v.previous'''
    if v.previous:
        path.append(v.previous.get_id())
        shortest(v.previous, path)
    return


def collected_fish(v):
    f = sold_fish[v.get_id()]
    if v.previous:
        return f | collected_fish(v.previous)
    else:
        return f

def bitcount(bitmap):
    count = 0
    while (bitmap != 0):
        if bitmap & 0x1:
            count += 1
        bitmap = bitmap >> 1
    return count

def dijkstra(aGraph, start):
    start.set_distance(0)

    fringe = []
    in_fringe = {}
    heapq.heapify(fringe)
    s = (0, start)
    heapq.heappush(fringe, s)
    in_fringe[start.get_id()] = s
    t_start = time.time()
    while True:
        try:
            uv = heapq.heappop(fringe)
            del in_fringe[uv[1].get_id()]
        except:
            t_end = time.time()
            print "dijkstra took {}".format(t_end - t_start)
            return
        current = uv[1]
        current.set_visited()

        for next in current.adjacent:
            if next.visited:
                continue
            new_dist = current.get_distance() + current.get_weight(next)
            
            if new_dist < next.get_distance():
                next.set_distance(new_dist)
                next.set_previous(current)
                a = (new_dist, next)
                if next.get_id() not in in_fringe.keys():
                    heapq.heappush(fringe, a)
                else:
                    fringe.remove(in_fringe[next.get_id()])
                    heapq.heapify(fringe)
                    heapq.heappush(fringe, a)
                in_fringe[next.get_id()] = a
            else:
                pass

def try_merge(next, current, neighbours, sold_fish):
    next_neighs = map(lambda x: x[1], neighbours[next])
    print "try_merge neighs {}".format(next_neighs)
    if len(next_neighs) == 2 and sold_fish[next] == 0:
        print "removing %d" % next
        w = map(lambda x: x[0], neighbours[next])
        total_w = w[0] + w[1]
        neighbours[next] = []
        neighbours[next_neighs[0]].append([total_w, next_neighs[1]])
        neighbours[next_neighs[1]].append([total_w, next_neighs[0]])
        return True
    return False

def bfs_remove(neighbours, sold_fish, start):
    fringe = []
    visited = set()
    heapq.heapify(fringe)
    s = (0, start)
    heapq.heappush(fringe, s)
    while True:
        try:
            uv = heapq.heappop(fringe)
        except:
            return
        current = uv[1]
        print current
        print visited
        next_nodes = map(lambda x: x[1], neighbours[current])
        print next_nodes
        for next in next_nodes:
            if next in visited:
                print "not adding %d" % next
                continue
            print "adding %d" % next
            if not try_merge(next, current, neighbours, sold_fish):
                heapq.heappush(fringe, (0, next))
            visited.add(next)

Node = namedtuple('Node', ['total_cost', 'cost', 'shop', 'collected_fish', 'visited_neigh'])
N, M, K = raw_input("").split(" ")
N, M, K = int(N), int(M), int(K)
#print N, M, K
sold_fish = [0]
final_nodes = []
neighbours = defaultdict(list)

g = Graph()

for i in range(N):
    g.add_vertex(i+1)
    v = raw_input("").split(" ")
    v = map(lambda x: int(x), v)
    b = 0x0000
    for j in range(v[0]): 
        b = b | (1<<(v[j+1]-1))
    sold_fish.append(b)
    

for i in range(M):
    v = raw_input("").split(" ")
    v = map(lambda x: int(x), v)
    g.add_edge(v[0], v[1], v[2])
    neighbours[v[0]].append([v[2],v[1]])
    neighbours[v[1]].append([v[2],v[0]])

#print 'Graph data:'
#for v in g:
#    for w in v.get_connections():
#        vid = v.get_id()
#        wid = w.get_id()
#        print '( %s , %s, %3d)'  % ( vid, wid, v.get_weight(w))

shops_to_consider = []
i = 0
for shop in sold_fish:
    if shop != 0:
        shops_to_consider.append(i)
    i += 1

for shop in shops_to_consider:
    print "Shop {}: {}".format(shop, bin(sold_fish[shop]))

shops_by_fish = defaultdict(list)
for shop in shops_to_consider:
    shops_by_fish[bin(sold_fish[shop])].append(shop)

for fishes in shops_by_fish.keys():
    print "{} sold in {}".format(fishes, shops_by_fish[fishes])

print len(shops_to_consider)
print N, M, K

bfs_remove(neighbours, sold_fish, 1)

g = Graph()
for i in neighbours.keys():
    if neighbours[i] != []:
        g.add_vertex(i)

for i in neighbours.keys():
    if neighbours[i] != []:
        for j in neighbours[i]:
            g.add_edge(i, j[1], j[0])
dijkstra(g, g.get_vertex(N))


min_distance_to_N = {}
for i in range(1, N+1):
    v = g.get_vertex(i)
    min_distance_to_N[i] = v.get_distance() 
    #print v.get_distance()

fish_bitmap_to_N = {}
fish_to_N = {}
for i in range(1, N+1):
    v = g.get_vertex(i)
    fish_bitmap_to_N[i] = collected_fish(v)
    fish_to_N[i] = bitcount(fish_bitmap_to_N[i])
    print fish_bitmap_to_N[i], fish_to_N[i]

for i in neighbours.keys():
    neighbours[i].sort()

all_fish = 2**K - 1

#print bin(all_fish)


fringe = []

dd = defaultdict(int)
nodes_by_shop = defaultdict(list)
initial_node = Node(0 , 0, 1, sold_fish[1], dd)
heapq.heappush(fringe, initial_node)

import time
counter=0
big_fish_and = 0
while True:
    t_start = time.time()
    counter = counter + 1
    node = heapq.heappop(fringe)
    print "Fringe size {}, Exploring total_cost {} cost {}, node {}, collected_fish {}".format(len(fringe), node.total_cost, node.cost, node.shop, bin(node.collected_fish))

    if node.shop == N:
        final_nodes.append(node)
        for n in final_nodes:
            if ((n.collected_fish | node.collected_fish) == all_fish):
                print node.cost
                sys.exit()


    skipped = 0
    fringe_needs_heapify = False
    for neighbour in neighbours[node.shop]:
        new_shop = neighbour[1]

        """
        visited_neigh = copy.copy(node.visited_neigh)
        if visited_neigh[(node.shop, new_shop)] == 1:
            skipped = skipped + 1
            continue
        visited_neigh[(node.shop, new_shop)] = 1
        """
        #visited_neigh = set()
        #new_node = Node(min_distance_to_N[new_shop] + node.cost + neighbour[0] + 10 - fish_to_N[new_shop],
        new_node = Node(min_distance_to_N[new_shop] + node.cost + neighbour[0],
        #new_node = Node(node.cost + neighbour[0],
                        node.cost + neighbour[0],
                        new_shop,
                        node.collected_fish | sold_fish[new_shop],
                        node.visited_neigh)
        remove = 0
        need_add_node = True
        similar_nodes = nodes_by_shop[new_shop]
        for n in similar_nodes:
            if ((new_node.collected_fish == n.collected_fish) and new_node.cost >= n.cost):
                need_add_node = False
                break                
            elif ((new_node.collected_fish | n.collected_fish) ==  new_node.collected_fish) and new_node.cost <= n.cost:
                fringe.remove(n)
                nodes_by_shop[new_shop].remove(n)
                remove = remove + 1
                fringe_needs_heapify = True
            elif ((new_node.collected_fish | n.collected_fish) == n.collected_fish and new_node.cost >= n.cost):
                need_add_node = False
                break
        if need_add_node:
            heapq.heappush(fringe, new_node)
            nodes_by_shop[new_shop].append(new_node)
    if fringe_needs_heapify:
        heapq.heapify(fringe)
    t_end = time.time()
    #print "counter {}, fringe {}, neighs {}, removed {}, skipped {}, time = {}".format(counter, len(fringe), len(neighbours[node.shop]), remove, skipped, t_end - t_start)
