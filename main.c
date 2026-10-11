#include <math.h>
#include "odeslv.h"
#include <stdio.h>


struct parameters {
    const double mass;
    const double l_f;
    const double l_r;
    const double pky1;      // maximum value of stiffness Kfy/Fznom
    const double pky2;      // load at which Kfy reaches maximum value
    const double fz0;
    const double fz_fl;
    const double fz_fr;
    const double fz_rl;
    const double fz_rr;
    const double inertia_z; // total yaw inertia [kgm^2]

    double c_alpha[2];      // cornering stiffness front and rear [rad/N]

    // scaling
    double lambda_fz0;      // scale factor of nominal (rated) load
    double lambda_ky;        // scale factor of Fy cornering stiffness

    double delta;
    double velocity;
};

void stm(double t, const double x[], double dx[], void *parameters);

int main(void) {
    struct parameters bmw5 = {
        .mass = 1600.004,
        .l_f = 1.466,
        .l_r = 1.509,
        .pky1 = -25.376280505938563,
        .pky2 = 2.0604900161557698,
        .fz0 = 5350.0,
        .fz_fl = 3921.3656,
        .fz_fr = 3921.3656,
        .fz_rl = 3922.1269,
        .fz_rr = 3922.1267,
        .inertia_z = 2986.624,
    };

    // linear tyre behavior
    bmw5.lambda_fz0 = 1.000;
    bmw5.lambda_ky = 1.000;

    double ky_fl = bmw5.pky1 * bmw5.fz0 * sin(2 * atan(bmw5.fz_fl/(bmw5.pky2 * bmw5.fz0 * bmw5.lambda_fz0))) * bmw5.lambda_ky;
    double ky_fr = bmw5.pky1 * bmw5.fz0 * sin(2 * atan(bmw5.fz_fr/(bmw5.pky2 * bmw5.fz0 * bmw5.lambda_fz0))) * bmw5.lambda_ky;
    double ky_rl = bmw5.pky1 * bmw5.fz0 * sin(2 * atan(bmw5.fz_rl/(bmw5.pky2 * bmw5.fz0 * bmw5.lambda_fz0))) * bmw5.lambda_ky;
    double ky_rr = bmw5.pky1 * bmw5.fz0 * sin(2 * atan(bmw5.fz_rr/(bmw5.pky2 * bmw5.fz0 * bmw5.lambda_fz0))) * bmw5.lambda_ky;

    bmw5.c_alpha[0] = -(ky_fl + ky_fr);
    bmw5.c_alpha[1] = -(ky_rl + ky_rr);

    bmw5.velocity = 10.00;    // input velocity of cog [m/s]
    bmw5.delta = 0.20;        // input steering angle at axle front [rad]

    double xk[] = {0.00, 0.00};
    double time = 0.000;
    double dt = 0.1;

    for (int i = 0; i <= 10; i++) {
        ode4(stm, time, xk, 2, dt, &bmw5);
        printf("yaw rate: %f\t sideslip angle: %f\t time: %f\n", xk[0], xk[1], time);
        time += dt;
    };
}

void stm(double t, const double x[], double dx[], void *parameters) {
    struct parameters *p = (struct parameters *)parameters;

    // system behavior
    double a11 = -(p->c_alpha[0] * pow(p->l_f,2) + p->c_alpha[1] * pow(p->l_r,2))/(p->velocity * p->inertia_z);
    double a12 = -(p->c_alpha[0] - p->c_alpha[1])/(p->velocity * p->inertia_z);
    double a21 = -1.0 - (p->c_alpha[0] * p->l_f - p->c_alpha[1] * p->l_r)/(pow(p->velocity,2) * p->mass);
    double a22 = -(p->c_alpha[0]+ p->c_alpha[1])/(p->velocity * p->mass);

    double a[] = {a11, a12, a21, a22};

    // control array
    double b11 = p->c_alpha[0] * p->l_f / p->inertia_z;
    double b21 = p->c_alpha[0] / (p->velocity * p->mass);

    double b[] = {b11, b21};

    //compute output values
    dx[0] = a11 * x[0] + a12 * x[1] + b11 * p->delta;    // yaw acceleration [rad/s^2]
    dx[1] = a21 * x[0] + a22 * x[1] + b21 * p->delta;    //sideslip angle velocity [rad/s]
};