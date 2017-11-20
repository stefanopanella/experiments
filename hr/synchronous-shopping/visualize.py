#!/usr/bin/python

N, M, K = raw_input("").split(" ")
N, M, K = int(N), int(M), int(K)

print "graph {"

for i in range(N):
    v = raw_input("").split(" ")
    v = map(lambda x: int(x), v)
    num_fish = v[0]
    v = v[1:]

    if i==0:
        extra = " color=green,style=filled"
    elif i==N-1:
        extra = " color=red,style=filled"
    else:
        extra = ""

    if num_fish > 0:
        print " n%d [label=\"Node %d\\n%s\"%s];" % (i+1, i+1, v, extra)
    else:
        print " n%d [label=\"Node %d\"%s];" % (i+1, i+1, extra)

# edges
for i in range(M):
    v = raw_input("").split(" ")
    v = map(lambda x: int(x), v)
    print " n%d -- n%d [label=\"%d\", weight=%d, penwidth=%d];" % (v[0], v[1], v[2], v[2], v[2]/100)

print "}"
