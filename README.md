# ode-solver Library (odeslv)
A lightweight, high-performance C library for numerical integration of Ordinary Differential Equations (ODEs). It provides implementations of explicit single-step and multi-stage integration methods (such as Forward Euler and the classic 4th-order Runge-Kutta method) optimized for simulation systems, physical models, and control applications.


## What is this library?

odeslv is a shared C library designed to solve systems of first-order ordinary differential equations. It is structured to handle time-dependent state updates cleanly, allowing users to pass custom parameters and control inputs to their system dynamics functions.

The repository includes:

- odeslv.h & odeslv.c: Core solver functions (ode1 for Forward Euler and ode4 for 4th-order Runge-Kutta).

- Makefile: Automation script for compiling the shared library and building runnable example binaries.

- Examples: Demonstration programs (e.g., single-track vehicle dynamics models) showing how to link against the library.

## What Type of Input Equations Are Required?

The library expects systems formulated as a set of first-order ordinary differential equations in explicit form:

$$\frac{d\mathbf{x}}{dt} = \mathbf{f}(t, \mathbf{x}, \mathbf{u}, \text{parameters})$$

Your input function must conform to the ode_function signature defined in odeslv.h:

typedef void(*ode_function)(double t, const double x[], double dx[], double u[], void *parameters);


Where:

- t: Current time (double).

- x[]: Input array representing the current state vector of dimension dim (const double[]).

- dx[]: Output array where your function must write the computed derivatives (double[]).

- u[]: Array of external control inputs or driving forces (double[]).

- parameters: Pointer to a user-defined structure containing constant parameters (e.g., physical constants, vehicle dimensions, coefficients) (void *).

## How to Build and Use

1. Build the Library

To compile the shared library (libodeslv.so) inside the bin/ directory, run:

```console
make all
```

2. Build and Run Examples

The project is set up to compile example programs located in the examples/ directory. For example, if you have an example source file named examples/main.c, you can build it by typing:

```console
make main
```


This command compiles main.c, links it dynamically against libodeslv.so, and places the executable binary inside bin/.

3. Run the Binary

Due to the built-in RPATH configuration ($ORIGIN on Linux, @loader_path on macOS), the binary can locate the shared library directly from the bin/ folder without needing manual LD_LIBRARY_PATH exports. Run it from the root directory or inside bin/:

```console
./bin/main
```


4. Cleanup

To remove all compiled object files and binaries, run:

```console
make clean
```


## License

MIT License. See the LICENSE file for details.
