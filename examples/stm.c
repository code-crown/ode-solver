#include <math.h>
#include "odeslv.h"
#include <stdio.h>


struct parameters {
    double mass, l_f, l_r, inertia_z;
    double pky1, pky2, fz0;
    double static_loads[4];
    double c_alpha[2];      // cornering stiffness front and rear [rad/N]

    // scaling
    double lambda_fz0, lambda_ky;      // scale factor of nominal (rated) load
}bmw5;

void stm(double t, const double x[], double dx[], double u[],void *parameters);
void calc_cornering_stiff(void *parameters);

int main(void) {
    bmw5.mass = 1600.004,
    bmw5.l_f = 1.466,
    bmw5.l_r = 1.509,
    bmw5.pky1 = -25.376280505938563,
    bmw5.pky2 = 2.0604900161557698,
    bmw5.fz0 = 5350.0,
    bmw5.static_loads[0] = 3921.3656,
    bmw5.static_loads[1] = 3921.3656,
    bmw5.static_loads[2] = 3922.1269,
    bmw5.static_loads[3] = 3922.1267,
    bmw5.inertia_z = 2986.624,

    // linear tyre behavior
    bmw5.lambda_fz0 = 1.000;
    bmw5.lambda_ky = 1.000;

    calc_cornering_stiff(&bmw5);

    double xk_ode4[] = {0.00, 0.00, 10.00};
    double xk_ode1[] = {0.00, 0.00, 10.00};
    double time = 0.000;
    double dt = 0.001;
    double u[] = {0.2}; // input steering angle at axle front [rad]

    for (int i = 0; i <= 300; i++) {
        ode4(stm, time, xk_ode4, u, 3, dt, &bmw5);
        ode1(stm, time, xk_ode1, u, 3, dt, &bmw5);
        printf("yaw rate: ode4 %f ode1 %f\t sideslip angle: %f\t time: %f\n", xk_ode4[0], xk_ode1[0], xk_ode4[1], time);
        time += dt;
    };
}

void stm(double t, const double x[], double dx[], double u[], void *parameters) {
    struct parameters *p = (struct parameters *)parameters;

    // system behavior
    double a11 = -(p->c_alpha[0] * pow(p->l_f,2) + p->c_alpha[1] * pow(p->l_r,2))/(x[2] * p->inertia_z);
    double a12 = -(p->c_alpha[0] - p->c_alpha[1])/(x[2] * p->inertia_z);
    double a21 = -1.0 - (p->c_alpha[0] * p->l_f - p->c_alpha[1] * p->l_r)/(pow(x[2],2) * p->mass);
    double a22 = -(p->c_alpha[0]+ p->c_alpha[1])/(x[2] * p->mass);

    // control array
    double b11 = p->c_alpha[0] * p->l_f / p->inertia_z;
    double b21 = p->c_alpha[0] / (x[2] * p->mass);

    //compute output values
    dx[0] = a11 * x[0] + a12 * x[1] + b11 * u[0];       // yaw acceleration [rad/s^2]
    dx[1] = a21 * x[0] + a22 * x[1] + b21 * u[0];       //sideslip angle velocity [rad/s]
    dx[2] = 0.00;                                       // longitudinal acceleration
};

void calc_cornering_stiff(void *parameters){
    struct parameters *p = (struct parameters *)parameters;

    double ky_fl = p->pky1 * p->fz0 * sin(2 * atan(p->static_loads[0]/(p->pky2 * p->fz0 * p->lambda_fz0))) * p->lambda_ky;
    double ky_fr = p->pky1 * p->fz0 * sin(2 * atan(p->static_loads[1]/(p->pky2 * p->fz0 * p->lambda_fz0))) * p->lambda_ky;
    double ky_rl = p->pky1 * p->fz0 * sin(2 * atan(p->static_loads[2]/(p->pky2 * p->fz0 * p->lambda_fz0))) * p->lambda_ky;
    double ky_rr = p->pky1 * p->fz0 * sin(2 * atan(p->static_loads[3]/(p->pky2 * p->fz0 * p->lambda_fz0))) * p->lambda_ky;

    p->c_alpha[0] = -(ky_fl + ky_fr);
    p->c_alpha[1] = -(ky_rl + ky_rr);
}