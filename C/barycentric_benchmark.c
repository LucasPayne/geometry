/*
 * Benchmark: Benchmarking script should link in successive FUNCNAME_altN functions
 * and run this.
 */

#include <stdio.h>
#include <stdlib.h>
#include "barycentric.h"

    // Floating point random from K&R
#define frand() ((double) rand() / (RAND_MAX+1.0))

void randomize(double values[], int n, double mul)
{
    for (int i = 0; i < n; i++) {
        values[i] = mul * frand();
    }
}

int main(int argc, char *argv[])
{
    if (argc != 2) {
        fprintf(stderr, "give good args");
        exit(EXIT_FAILURE);
    }
    int num_runs;
    sscanf(argv[1], "%d", &num_runs);

    for (int i = 0; i < num_runs; i++) {
        double vals[8];
        randomize(vals, 8, 3.0);
        double u,v,w;
        cartesian_to_barycentric(vals[0], vals[1], vals[2],
                                 vals[3], vals[4], vals[5],
                                 vals[6], vals[7], &u, &v, &w);
    }
    return 0;
}
