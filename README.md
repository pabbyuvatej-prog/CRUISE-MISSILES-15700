# AV Impacttetur Simulation Repository

Repository-ready numerical simulation framework for the AV Impacttetur experimental vehicle model.

## Quick start

```bash
python -m pip install -r requirements.txt
python run_simulation.py
```

Results are written to `outputs/`. This repository is a **numerical research model**, not a flight-certified or physically validated vehicle model.

## Included domains

- C2 biconic geometry
- atmosphere and reference condition
- reduced-order aerodynamics
- mass / CG / inertia
- quaternion rigid-body 6-DOF equations
- actuator and P/PD control
- bounded thermal source
- compressible-flow reduced model
- viscous/FSI bookkeeping
- orthotropic thermoelastic surrogate
- reduced finite-rate chemistry interface
- regression and conservation checks
- CSV result export

## Important

Parameters marked PARAMETRIC are assumptions/envelopes from the project record. Numerical convergence means the implemented equations converge; it does not establish physical validation.
