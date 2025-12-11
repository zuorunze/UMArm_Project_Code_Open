import numpy
from numpy import cos, sin, tan, pi


# measure the fibre angle, radius, length at the initial, uninstalled state.

def get_force_from_pressure_length(FA, R, L, dL, p):
    Bf = L / cos(FA)
    Nf = L / (2 * pi * R) * tan(FA)

    output_force = pi * (Bf**2 - 3*(L + dL)**2) / (2*pi*Nf)**2 * p
    return output_force

def get_length_from_pressure_force(FA, R, L, F, p):
    Bf = L / cos(FA)
    Nf = L / (2 * pi * R) * tan(FA)
    
    output_dL = ((1.0/3.0) * (Bf**2 - 4*pi*(Nf**2)*F / p))**0.5 - L
    return output_dL

def get_pressure_from_force_length(FA, R, L, F, dL):
    Bf = L / cos(FA)
    Nf = L / (2 * pi * R) * tan(FA)

    output_p = (4*pi*Nf**2) / (Bf**2 - 3*(L+dL)**2) * F
    return output_p



def get_p_from_theta():
    return


if __name__ == "__main__":
    FA = 0.5
    R = 0.015
    L = 0.13
    
    dL = -0.03
    p = 300000


    force_out = get_force_from_pressure_length(FA, R, L, dL, p)
    print(force_out)

    dL_out = get_length_from_pressure_force(FA, R, L, force_out, p)
    print(dL_out)

    p_out = get_pressure_from_force_length(FA, R, L, force_out, dL_out)
    print(p)