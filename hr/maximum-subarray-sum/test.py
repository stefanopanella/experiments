#!/usr/bin/python
from collections import defaultdict
import sys

def maximumSum(a, m):
    vec_len = long(len(a))
    v = [long(0) for i in range(vec_len)]
    max_mod = 0
    for sub_array_size in range(vec_len):
        for start in range(vec_len - sub_array_size):
            v[start] = v[start] + long(a[start + sub_array_size])
            mod = v[start] % m
            max_mod = max(max_mod, mod)
    return max_mod

if __name__ == "__main__":
    q = int(raw_input().strip())
    for a0 in xrange(q):
        n, m = raw_input().strip().split(' ')
        n, m = [int(n), long(m)]
        a = map(long, raw_input().strip().split(' '))
        result = maximumSum(a, m)
        print result
