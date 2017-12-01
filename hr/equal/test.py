#!/bin/python

import sys

cache = {}

def n_to_increase(d):
    try:
        return cache[d]
    except:
        first = d/5
        r = d%5
        second = r/2
        r = r%2
        third = r
        cache[d] = first + second + third
        return cache[d]

def getMoves(n, c):
    c.sort()
    #c0 = min(c)
    min_acc = sys.maxint
    for rm in [0,1,2]:
        c0_rm = c[0] - rm
        acc = n_to_increase(rm)
        for i in range(1,len(c)):
            acc += n_to_increase(c[i] - c0_rm)
        if acc < min_acc:
            min_acc = acc
    return min_acc

T = raw_input().strip()
for i in range(int(T)):
    n = raw_input().strip()
    c = map(int, raw_input().strip().split(' '))
    moves = getMoves(n, c)
    print moves
