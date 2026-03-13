#include <math.h>
#include <stdbool.h>
#include <stdio.h>

void thinprism(
    const double M[3],          // 3D Point in camera system [X, Y, Z]
    const double params[12],    // Model parameters - fx, fy, cx, cy, k1, k2, p1, p2, k3, k4, sx1, sy1
    double *x, double *y)       // 2D Projection [x, y]
{
    double X = M[0];
    double Y = M[1];
    double Z = M[2];

    double fx  = params[0];
    double fy  = params[1];
    double cx  = params[2];
    double cy  = params[3];
    double k1  = params[4];
    double k2  = params[5];
    double p1  = params[6];
    double p2  = params[7];
    double k3  = params[8];
    double k4  = params[9];
    double sx1 = params[10];
    double sy1 = params[11];

    // 1. Pinhole normalization
    double xn = X / Z;
    double yn = Y / Z;

    // 2. Fisheye projection (equidistant)
    double r = sqrt(xn * xn + yn * yn);

    double u = xn;
    double v = yn;

    if (r > 1e-12) {
        double theta = atan(r);
        double scale = theta / r;
        u = xn * scale;
        v = yn * scale;
    }

    // 3. Distortion
    double u2 = u * u;
    double v2 = v * v;
    double uv = u * v;

    double r2 = u2 + v2;
    double r4 = r2 * r2;
    double r6 = r4 * r2;
    double r8 = r6 * r2;

    double radial = k1*r2 + k2*r4 + k3*r6 + k4*r8;

    double du = u * radial + 2.0 * p1 * uv + p2 * (r2 + 2.0 * u2) + sx1 * r2;
    double dv = v * radial + 2.0 * p2 * uv + p1 * (r2 + 2.0 * v2) + sy1 * r2;

    double ud = u + du;
    double vd = v + dv;

    // 4. Pixel coordinates
    *x = fx * ud + cx;
    *y = fy * vd + cy;
}


int main() {
    double M[3] = {200.0, 400.0, 300.0};
    double params[12] = {852.13705237092313, 850.17193586488634, 1440, 1440,
                        0.089108710858519902, -0.040448487168936217, -0.00051923858012622775, -0.00032993437201914437,
                        0.016053928938995387, -0.0038407855887184119, 0.0019045576073914582, 0.0032420175002338338};
    double x, y;

    thinprism(M, params, &x, &y);

    printf("Projected point: x = %f, y = %f\n", x, y);
    return 0;
}


// gcc thinprism.c -o thinprism.exe