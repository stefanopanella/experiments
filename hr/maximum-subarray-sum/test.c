#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <assert.h>
#include <limits.h>
#include <stdbool.h>

long int maximumSum(int a_size, long int* a, long int m) {
  long int v[a_size];
  int i;
  for (i = 0; i < a_size; i++) {
    v[i] = 0;
  }
  long int mod, max_mod = 0;
  int sub_array_size;
  int start;
  for (sub_array_size = 0; sub_array_size < a_size; sub_array_size++) {
    for (start = 0; start < (a_size - sub_array_size); start++) {
      v[start] += a[start + sub_array_size];
      mod = v[start] % m;
      if (mod > max_mod) {
	max_mod = mod;
      }
    }
  }
  return max_mod;
}

int main() {
  int q; 
  scanf("%i", &q);
  for(int a0 = 0; a0 < q; a0++){
    int n; 
    long int m; 
    scanf("%i %li", &n, &m);
    long int *a = malloc(sizeof(long int) * n);
    for (int a_i = 0; a_i < n; a_i++) {
      scanf("%li",&a[a_i]);
    }
    long int result = maximumSum(n, a, m);
    printf("%ld\n", result);
  }
  return 0;
}
