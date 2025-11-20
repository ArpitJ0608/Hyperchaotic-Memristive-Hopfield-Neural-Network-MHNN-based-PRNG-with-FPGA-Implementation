import numpy as np
import time
import os
# --------------------------
# MHNN Parameters
# --------------------------
a, b, c = 1.5, 2.9, 0.7
d, e, ky = 2.0, 1.18, 0.1
f, g, h = 2.957, 23.0, 0.49
alpha, beta = 1.0, 1.0
m, n = 0.5, 1.0
r = 1.5
h_step = 0.01

try:
    float_type = np.float128
except AttributeError:
    float_type = np.float64
 
def von_neumann_debias(bitstream: str) -> str:
    result = []
    for i in range(0, len(bitstream) - 1, 2):
        if bitstream[i] != bitstream[i + 1]:
            result.append(bitstream[i])
    return ''.join(result)


# --------------------------
# MHNN Dynamics
# --------------------------
def approx_tanh(i):
    if i >= 2:
         return 1.0
    elif i <= -2:
         return -1.0
    elif i >= 0:
         return i * (1 - 0.25 * i)
    else:
         return i * (1 + 0.25 * i)
        
        
def tanh_r(x):
    return approx_tanh(r * x)

def W(phi):
    return alpha - beta * abs(phi)

'''
||Note||
W(phi) modulates neuron y using the memory state (w):
when phi increases, W(phi) decreases. Neuron y drives the others
and w feeds back into y via (dw = m*y - n*w), creating nonlinear memory influence.
'''

def mhhn_derivatives(state):
     x, y, z, w = state
     dx = -x + a * tanh_r(x) + b * tanh_r(y) - c * tanh_r(z)
     dy = -y - d * tanh_r(x) + e * tanh_r(y) + ky * W(w)
     dz = -z + f * tanh_r(x) - g * tanh_r(y) + h * tanh_r(z)
     dw = m * y - n * w
     return np.array([dx, dy, dz, dw], dtype=float_type)
    
def rk4_step(state, noise_strength=1e-14):
     k1 = h_step * mhhn_derivatives(state)
     k2 = h_step * mhhn_derivatives(state + 0.5 * k1)
     k3 = h_step * mhhn_derivatives(state + 0.5 * k2)
     k4 = h_step * mhhn_derivatives(state + k3)
     update = (k1 + 2 * k2 + 2 * k3 + k4) / 6
     
     # Add tiny chaotic noise
     noise = np.random.uniform(-noise_strength, noise_strength, size=state.shape).astype(float_type)
     return state + update + noise

    
# --------------------------
# New Chaotic Bit Extraction
# --------------------------
def generate_prng_bits(state):
     """
     Extract 22 bits from each of the 4 chaotic state variables using LSBs.
     Produces an 88-bit string per call.
     """
     # Scale to push fractional chaos into integer LSBs
     int_x = int(state[0] * 1e14)
     int_y = int(state[1] * 1e14)
     int_z = int(state[2] * 1e14)
     int_w = int(state[3] * 1e14)
     bits = []
     for _ in range(22):
         bits.append(str(int_x & 1))
         bits.append(str(int_y & 1))
         bits.append(str(int_z & 1))
         bits.append(str(int_w & 1))

         int_x >>= 1
         int_y >>= 1
         int_z >>= 1
         int_w >>= 1
     return "".join(bits) # 88 bits total

    
# --------------------------
# Save PRNG output to file
# --------------------------
def save_prng_to_file(state, iterations=1200000,
    path="C:/Users/dell/Documents/proj_prng_pyt_optfile.txt"):
    
    with open(path, 'w') as f:
        for _ in range(iterations):
            state = rk4_step(state)
            raw_bits = generate_prng_bits(state)
            debiased_bits = von_neumann_debias(raw_bits)
            
            # Fallback if debiased too short
            if len(debiased_bits) < 64:
                bits_to_write = raw_bits
            else:
                bits_to_write = debiased_bits
                f.write(bits_to_write + '\n')
                
# --------------------------
# Initial State
# --------------------------
seed = int.from_bytes(os.urandom(8), 'little') + int(time.time() * 1e6)
initial_state = np.array([
     1.002 + 0.00000001 * seed,
     1.0003 + 0.00000001 * (seed // 2),
     0.0001 + 0.00000001 * (seed // 3),
     0.005 + 0.00000001 * (seed // 4)
], dtype=float_type)

save_prng_to_file(initial_state, iterations=1200000,
path="C:/Users/dell/Documents/proj_prng_pyt_optfile.txt")
