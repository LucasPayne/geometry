/*
 * Testing for correctness (versus speed/benchmarking)
 */

#include <stdio.h>
#include "barycentric.h"

int main(void)
{
    // points on triangle, centroid, (2,1,1), (3/4, 1/4, 0) ...
    double pointsx[10] = {1, 0, 0, 1.0/3.0, 1.0/2.0, 3.0/4.0, -1, 0.44, 9.32, 100};
    double pointsy[10] = {1, 1, 0, 2.0/3.0, 3.0/4.0, 1, 38, 100, 3.1, 1};
    for (int i = 0; i < 10; i++) {
        double u, v, w;
        cartesian_to_barycentric(1, 1, 0, 1, 0, 0, pointsx[i], pointsy[i], &u, &v, &w);
        printf("(%.2lf,%.2lf)\n\t:(u v w) =\n\t(%.6lf\n\t%.6lf\n\t%.6lf)\n",
                        pointsx[i],
                        pointsy[i],
                        u, v, w);
        double x, y;
        barycentric_to_cartesian(1, 1, 0, 1, 0, 0, u, v, w, &x, &y);
        printf("\t\t===> (%.2lf, %.2lf)\n", x, y);
    }
}
