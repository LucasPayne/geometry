/*
 * Using barycentric coordinates
 * [in the plane]
 * Conversion to and from cartesian coordinates
 */

#include <stdio.h>
#include "barycentric.h"


void cartesian_to_barycentric(double Ax, double Ay, double Bx, double By, double Cx, double Cy,
                              double Px, double Py,
                              double *baryU, double *baryV, double *baryW)
{
    /* computes the barycentric coordinates of P from triangle ABC.
     * work with vectors A->B, A->C, A->P, and find coefficients for A->B and A->C
     * as a basis combining to A->P (inverting a 2x2 matrix).
     */
    
    double v1x, v1y, v2x, v2y, v3x, v3y;
    v1x = Bx - Ax; v1y = By - Ay;
    v2x = Cx - Ax; v2y = Cy - Ay;
    v3x = Px - Ax; v3y = Py - Ay;
    
    double det_weight = 1 / (v1x*v2y - v2x*v1y);
    *baryV = det_weight * (v3x*v2y - v3y*v2x);
    *baryW = det_weight * (-v3x*v1y + v3y*v1x);

    *baryU = 1 - *baryV - *baryW;
    double normalizing_weight = 1 / (*baryU + *baryV + *baryW);
    *baryU *= normalizing_weight;
    *baryV *= normalizing_weight;
    *baryW *= normalizing_weight;
}
// Alterations: benchmarking {{{
void cartesian_to_barycentric_alt1(double Ax, double Ay, double Bx, double By, double Cx, double Cy,
                              double Px, double Py,
                              double *baryU, double *baryV, double *baryW)
{
    /* More divisions */
    
    double v1x, v1y, v2x, v2y, v3x, v3y;
    v1x = Bx - Ax; v1y = By - Ay;
    v2x = Cx - Ax; v2y = Cy - Ay;
    v3x = Px - Ax; v3y = Py - Ay;
    
    double det_weight = 1 / (v1x*v2y - v2x*v1y);
    *baryV = det_weight * (v3x*v2y - v3y*v2x);
    *baryW = det_weight * (-v3x*v1y + v3y*v1x);

    *baryU = 1 - *baryV - *baryW;
    double weight = 1 / (*baryU + *baryV + *baryW);
    *baryU /= weight;
    *baryV /= weight;
    *baryW /= weight;
}
// }}}


void barycentric_to_cartesian(double Ax, double Ay, double Bx, double By, double Cx, double Cy,
                              double u, double v, double w,
                              double *x, double *y)
{
    double v1x, v1y, v2x, v2y;
    v1x = Bx - Ax; v1y = By - Ay;
    v2x = Cx - Ax; v2y = Cy - Ay;

    *x = Ax + v * v1x + w * v2x;
    *y = Ay + w * v1y + w * v2y;
}


