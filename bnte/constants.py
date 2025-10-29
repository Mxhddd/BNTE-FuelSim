"""Global constants for BNTE simulations."""
from __future__ import annotations

G0 = 9.80665  # m/s^2
SIGMA = 5.670374419e-8  # Stefan-Boltzmann constant W/m^2/K^4
BOLTZMANN = 1.380649e-23
HYDROGEN_R = 4124.0  # J/(kg*K)
HYDROGEN_CP = 14300.0  # J/(kg*K)
HYDROGEN_LH_VAP = 4.45e5  # J/kg approximate latent heat for H2 at ~20 K
E_CHARGE = 1.602176634e-19  # C

__all__ = [
    "G0",
    "SIGMA",
    "BOLTZMANN",
    "HYDROGEN_R",
    "HYDROGEN_CP",
    "HYDROGEN_LH_VAP",
    "E_CHARGE",
]
