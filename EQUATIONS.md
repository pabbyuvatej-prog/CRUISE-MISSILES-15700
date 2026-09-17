# Core equations

## Geometry
A_ref = pi R_max^2

## Aerodynamic coefficients
CL = CL0 + CL_alpha alpha + CL_delta delta

CD = CD0 + CD_delta delta

Cm = Cm0 + Cm_q (q c / 2V) + Cm_delta delta

## Forces
F_D = q_inf A_ref CD
F_L = q_inf A_ref CL
M_y = q_inf A_ref c Cm

## Rigid-body rotation
I omega_dot = M - omega x (I omega)

## Quaternion
q_dot = 1/2 Omega(omega) q

## Actuator
tau^2 delta_ddot + 2 zeta tau delta_dot + delta = delta_cmd

## PD controller
delta_cmd = Kp(q-q_cmd) + Kd q_dot

## Thermal bounded model
The repository treats the electrode/thermal input as an externally bounded source.
It does not equate deposited energy with kinetic energy and does not optimize destructive effects.

## Thermoelasticity
epsilon = epsilon_mech + epsilon_th

epsilon_th,i = integral(alpha_i(T) dT)

## Orthotropic compliance
epsilon = S sigma + epsilon_th

## Species conservation
d(rho Y_k)/dt + div(rho u Y_k)
= -div(J_k) + omega_k

This repository keeps the chemistry interface reduced and explicitly parametric.
