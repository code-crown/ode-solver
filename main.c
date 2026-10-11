#include <stdio.h>
#include <math.h>

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

};

void stm(double dx[], double xk[], double velocity, double delta, struct parameters p);
void ode4(double dt, double xk[], double velocity, double delta, struct parameters p);

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

    double velocity = 10.00;    // input velocity of cog [m/s]
    double delta = 0.20;        // input steering angle at axle front [rad]
    double xk[] = {0.00, 0.00};

    double dt = 0.001;

    for (int i = 0; i < 1000; i++) {
        ode4(dt, xk, velocity, delta, bmw5);

        printf("yaw rate: %f\t sideslip angle: %f\n", xk[0], xk[1]);
    };
}

void stm(double dx[], double xk[], double velocity, double delta, struct parameters p) {
    // system behavior
    double a11 = -(p.c_alpha[0] * pow(p.l_f,2) + p.c_alpha[1] * pow(p.l_r,2))/(velocity * p.inertia_z);
    double a12 = -(p.c_alpha[0] - p.c_alpha[1])/(velocity * p.inertia_z);
    double a21 = -1.0 - (p.c_alpha[0] * p.l_f - p.c_alpha[1] * p.l_r)/(pow(velocity,2) * p.mass);
    double a22 = -(p.c_alpha[0]+ p.c_alpha[1])/(velocity * p.mass);

    double a[] = {a11, a12, a21, a22};

    // control array
    double b11 = p.c_alpha[0] * p.l_f / p.inertia_z;
    double b21 = p.c_alpha[0] / (velocity * p.mass);

    double b[] = {b11, b21};

    //compute output values
    dx[0] = a11 * xk[0] + a12 * xk[1] + b11 * delta;    // yaw acceleration [rad/s^2]
    dx[1] = a21 * xk[0] + a22 * xk[1] + b21 * delta;    //sideslip angle velocity [rad/s]
};

void ode4(double dt, double xk[], double velocity, double delta, struct parameters p) {
    double k1[2], k2[2], k3[2], k4[2];
    double xtemp[2];
    stm(k1, xk, velocity, delta, p);

    xtemp[0] = xk[0] + 0.5 * dt * k1[0];
    xtemp[1] = xk[1] + 0.5 * dt * k1[1];
    stm(k2, xtemp, velocity, delta, p);

    xtemp[0] = xk[0] + 0.5 * dt * k2[0];
    xtemp[1] = xk[1] + 0.5 * dt * k2[1];
    stm(k3, xtemp, velocity, delta, p);

    xtemp[0] = xk[0] + dt * k3[0];
    xtemp[1] = xk[1] + dt * k3[1];
    stm(k4, xtemp, velocity, delta, p);

    xk[0] += (dt/6.00) * (k1[0] + 2*k2[0] + 2*k3[0] + k4[0]);
    xk[1] += (dt/6.00) * (k1[1] + 2*k2[1] + 2*k3[1] + k4[1]);
}