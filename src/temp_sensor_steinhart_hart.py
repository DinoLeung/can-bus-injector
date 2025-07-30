import numpy as np

# Lookup table values
T_c = np.array([-40, -30, -20, -10, 0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120, 130, 140, 150], dtype=float)
R = np.array([44864, 25524, 15067, 9195, 5784, 3740, 2480, 1683, 1167, 824, 594, 434.9, 323.4, 244.0, 186.6, 144.5, 113.3, 89.9, 71.9, 58.1], dtype=float)

# Convert temperatures to Kelvin
T_k = T_c + 273.15

# Prepare for least-squares: 1/T = A + B*ln(R) + C*(ln(R))^3
lnR = np.log(R)
Y = 1.0 / T_k
X = np.column_stack([np.ones_like(lnR), lnR, lnR**3])

# Solve for A, B, C
A, B, C = np.linalg.lstsq(X, Y, rcond=None)[0]

print(f"A = {A:.10f}")
print(f"B = {B:.10e}")
print(f"C = {C:.10e}")