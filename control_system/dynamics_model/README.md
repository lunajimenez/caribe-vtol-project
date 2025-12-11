# Dynamics Model

This module contains the full mathematical modeling of the VTOL vehicle, including:
- Rotational dynamics (Euler equations)
- Translational dynamics in NED and body frames
- Motor thrust models for VTOL rotors and forward propulsion
- Aerodynamic surfaces (elevator, ailerons, rudder)
- Lift and drag coefficient tables
- Transition model between VTOL and fixed-wing flight
- Initial stability validation

Subfolders:
- `python/` contains Python-based dynamic models and numerical simulations.
- `simulink/` contains MATLAB/Simulink models for advanced validation.
- `validation/` includes comparison graphs, logs, and numerical verification.