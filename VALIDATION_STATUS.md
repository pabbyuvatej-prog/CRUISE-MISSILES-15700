# Verification status

The project archive contains many numerical regression/convergence results. Those results establish properties of the implemented mathematical models, such as conservation residuals, quaternion normalization, mesh/timestep convergence and tensor consistency.

They do **not** establish:
- flight certification,
- wind-tunnel agreement,
- material qualification,
- real-gas chemistry validation,
- actual turbine drag reduction,
- structural safety margins without allowables.

Key archived numerical results include:
- mass residual < 1e-7 kg
- energy residual < 1e-6 J
- torque residual about < 1e-13
- quaternion drift about < 1e-14
- latest reduced reacting-flow temperature: 821.45 K
- latest model pressure: 45.951 kPa
- latest model von Mises stress: 124.52 MPa
- latest model interlaminar shear: 16.12 MPa
- latest model radial deformation: 0.1851 mm

These are case-specific solver outputs.
