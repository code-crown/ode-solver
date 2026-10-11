#include "odeslv.h"

void ode4(ode_function fun, double t, double xk[], int dim, double dt, void *parameters) {
    double k1[dim], k2[dim], k3[dim], k4[dim];
    double xk_temp[dim];

    fun(t, xk, k1, parameters);

    for(int i = 0; i < dim; i++) {xk_temp[i] = xk[i] + 0.5 * dt * k1[i];}
    fun(t, xk_temp, k2, parameters);

    for(int i = 0; i < dim; i++) {xk_temp[i] = xk[i] + 0.5 * dt * k2[i];}
    fun(t, xk_temp, k3, parameters);

    for(int i = 0; i < dim; i++) {xk_temp[i] = xk[i] + dt * k3[i];}
    fun(t, xk_temp, k4, parameters);

    for(int i = 0; i < dim; i++) {
        xk[i] += (dt/6.00) * (k1[i] + 2*k2[i] + 2*k3[i] + k4[i]);
    }
}