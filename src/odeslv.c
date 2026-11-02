#include <math.h>
#include "odeslv.h"
#include <stdlib.h>

// forward euler
void ode1(ode_function fun, double *t, double xk[], double u[], int dim, double *dt, void *parameters) {
    double k1[dim];

    fun(*t, xk, k1, u, parameters);

    for(int i = 0; i < dim; i++) {
        xk[i] += *dt * k1[i];
    }

    *t += *dt;
}

// heun
void ode2(ode_function fun, double *t, double xk[], double u[], int dim, double *dt, void *parameters) {
    double k1[dim], k2[dim];
    double xk_temp[dim];

    fun(*t, xk, k1, u, parameters);

    for(int i = 0; i < dim; i++) {xk_temp[i] = xk[i] + 0.5 * (*dt);}
    fun(*t + *dt, xk_temp, k2, u, parameters);

    for(int i = 0; i < dim; i++) {
        xk[i] += (*dt) * (0.5 * k1[i] + 0.5 * k2[i]);
    }

    *t += *dt;
}

// 4th order runge kutta
void ode4(ode_function fun, double *t, double xk[], double u[], int dim, double *dt, void *parameters) {
    double k1[dim], k2[dim], k3[dim], k4[dim];
    double xk_temp[dim];

    fun(*t, xk, k1, u, parameters);

    for(int i = 0; i < dim; i++) {xk_temp[i] = xk[i] + 0.5 * *dt * k1[i];}
    fun(*t + 0.5 * *dt, xk_temp, k2, u, parameters);

    for(int i = 0; i < dim; i++) {xk_temp[i] = xk[i] + 0.5 * *dt * k2[i];}
    fun(*t + 0.5 * *dt, xk_temp, k3, u, parameters);

    for(int i = 0; i < dim; i++) {xk_temp[i] = xk[i] + *dt * k3[i];}
    fun(*t + *dt, xk_temp, k4, u, parameters);

    for(int i = 0; i < dim; i++) {
        xk[i] += (*dt/6.00) * (k1[i] + 2*k2[i] + 2*k3[i] + k4[i]);
    }

    *t += *dt;
}

// dormand prince
void ode45(ode_function fun, double *t, double xk[], double u[], int dim, double *dt, void *parameters) {
    double k1[dim], k2[dim], k3[dim], k4[dim], k5[dim], k6[dim], k7[dim];
    double xk_temp[dim], zk[dim], s[dim];
    double t_temp, dt_opt;
    double smin = 1e9;
    const double epsilon = 1e-5;

    fun(*t, xk, k1, u, parameters); // k1

    t_temp = *t + 0.2 * *dt;
    for(int i = 0; i < dim; i++) {xk_temp[i] = xk[i] + 0.2 * *dt * k1[i];};
    fun(t_temp, xk_temp, k2, u, parameters); // k2

    t_temp = *t + 0.3 * *dt;
    for(int i = 0; i < dim; i++) {xk_temp[i] = xk[i] + 0.075 * *dt * k1[i] + 0.225 * *dt * k2[i];}
    fun(t_temp, xk_temp, k3, u, parameters); // k3

    t_temp = *t + 0.8 * *dt;
    for(int i = 0; i < dim; i++) {xk_temp[i] = xk[i] + (44.0/45.0) * *dt * k1[i] - (56.0/15.0) * *dt * k2[i] + (32.0/9.0) * *dt * k3[i];}
    fun(t_temp, xk_temp, k4, u, parameters); // k4

    t_temp = *t + (8.0/9.0) * *dt;
    for(int i = 0; i < dim; i++) {xk_temp[i] = xk[i] + (19372.0/6561.0) * *dt * k1[i] - (25360.0/2187.0) * *dt * k2[i] + (64448.0/6561.0) * *dt * k3[i] - (212.0/729.0) * *dt * k4[i];}
    fun(t_temp, xk_temp, k5, u, parameters); // k5

    t_temp = *t + *dt;
    for(int i = 0; i < dim; i++) {xk_temp[i] = xk[i] + (9017.0/3168.0) * *dt * k1[i] - (355.0/33.0) * *dt * k2[i] - (46732.0/5247.0) * *dt * k3[i] + (49.0/176.0) * *dt * k4[i] - (5103.0/18656.0) * *dt * k5[i];}
    fun(t_temp, xk_temp, k6, u, parameters); // k6

    t_temp = *t + *dt;
    for(int i = 0; i < dim; i++) {xk_temp[i] = xk[i] + (35.0/384.0) * *dt * k1[i] + (500.0/1113.0) * *dt * k3[i] + (125.0/192.0) * *dt * k4[i] - (2187.0/6784.0) * *dt * k5[i] + (11.0/84.0) * *dt * k6[i];}
    fun(t_temp, xk_temp, k7, u, parameters); // k7

    for(int i = 0; i < dim; i++) {
        xk_temp[i] = xk[i] + *dt * ((35.0/384.0) * k1[i] + (500.0/1113.0) * k3[i] + (125.0/192.0) * k4[i] - (2187.0/6784.0) * k5[i] + (11.0/84.0) * k6[i]);
        zk[i] = xk[i] + *dt * ((5179.0/57600.0) * k1[i] + (7571.0/16695.0) * k3[i] + (393.0/640.0) * k4[i] - (92097.0/339200.0) * k5[i] + (187.0/2100.0) * k6[i] + (1.0/40.0) * k7[i]);
        
        double diff = fabs(zk[i] - xk_temp[i]);
        if (diff < 1e-15) diff = 1e-15;

        s[i] = pow(((epsilon) / (2 * diff)), 0.2);
        if (s[i] < smin) { smin = s[i]; };
    }

    *t += *dt; 

    dt_opt = smin * *dt;
    *dt = dt_opt;
    
    for (int i = 0; i < dim; i++) {
        xk[i] = xk_temp[i];
    }
}