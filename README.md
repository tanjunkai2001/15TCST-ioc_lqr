# Inverse Optimal Control

## Inverse reinforcement learning or inverse optimal control problem.

This is the case for exact recovery of LQR case. Formulated as an SDP problem, solved by `YALMIP` solver.

The code is loosely based on:

> [1] K. Mombaur, A. Truong, and J.-P. Laumond, “From human to humanoid locomotion—an inverse optimal control approach,” *Auton Robot*, vol. 28, no. 3, pp. 369–383, Apr. 2010, doi: [10.1007/s10514-009-9170-7](https://doi.org/10.1007/s10514-009-9170-7).

## Improvement

Relaxing the equation condition in the LMI solution is more conducive to the convergence of the solution

Refer to the file `TCST15_inverse_lqr.m`
