#ifndef BC_HEADER_DEF
#define BC_HEADER_DEF

void cartesian_to_barycentric(double Ax, double Ay, double Bx, double By, double Cx, double Cy,
                              double Px, double Py,
                              double *baryU, double *baryV, double *baryW);

void barycentric_to_cartesian(double Ax, double Ay, double Bx, double By, double Cx, double Cy,
                              double u, double v, double w,
                              double *x, double *y);

#endif
