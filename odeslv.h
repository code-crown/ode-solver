#ifndef ODESLV_H
#define ODESLV_H

typedef void(*ode_function)(double t, const double x[], double dx[], double u[], void *parameters);

void ode1(ode_function fun, double t, double xk[], double u[], int dim, double dt, void *parameters);
void ode4(ode_function fun, double t, double xk[], double u[], int dim, double dt, void *parameters);

#endif