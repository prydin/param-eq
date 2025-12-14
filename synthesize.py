import math
import scipy.signal
import matplotlib.pyplot as plt
import numpy as np

# Peaking EQ filter coefficient calculation
G = 15
fc = 5000
fs = 48000
Q = 10


def precalc(fc, fs, Q, G):
    A=math.sqrt(10**(G/20))
    omega_c = 2*math.pi*fc/fs
    cos_omega_c = math.cos(omega_c)
    sin_omega_c = math.sin(omega_c)
    alpha = sin_omega_c /(2*Q)
    beta = math.sqrt(A)/Q
    return A, omega_c, cos_omega_c, sin_omega_c, alpha, beta


def peaking_filter(fc, fs, Q, G):
    A, omega_c, cos_omega_c, sin_omega_c, alpha, beta = precalc(fc, fs, Q, G)
    b0 = 1 + alpha*A
    b1 = -2*cos_omega_c
    b2 = 1 - alpha*A
    a0 = 1 + alpha/A
    a1 = -2*cos_omega_c
    a2 = 1 - alpha/A
    return [b0/a0, b1/a0, b2/a0], [1, a1/a0, a2/a0]

def shelving_filter(fc, fs, Q, G, high):
    A, omega_c, cos_omega_c, sin_omega_c, alpha, beta = precalc(fc, fs, Q, G)
    sign = 1 if high else -1
    b0 = A*((A+1)+(A-1)*cos_omega_c + beta*sin_omega_c)
    b1 = -sign*2*A*((A-1)+(A+1)*cos_omega_c)
    b2 = A*((A+1)+(A-1)*cos_omega_c - beta*sin_omega_c)
    a0 = (A+1)-(A-1)*cos_omega_c + beta*sin_omega_c
    a1 = sign*2*((A-1)-(A+1)*cos_omega_c)
    a2 = (A+1)-(A-1)*cos_omega_c - beta*sin_omega_c
    return [b0 / a0, b1 / a0, b2 / a0], [1, a1 / a0, a2 / a0]


(b, a) = peaking_filter(fc, fs, Q, G)


#[b, a] = shelving_filter(fc, fs, Q, G, high=False)
sos = scipy.signal.tf2sos(b, a)
for section in sos:
    print(f"{section[0]:.12e}, {section[1]:.12e}, {section[2]:.12e}, {section[4]:.12e}, {section[5]:.12e},")

# Plot filter transfer function
w, h = scipy.signal.sosfreqz(sos, worN=2000, fs=fs)
plt.plot(w, 20 * np.log10(abs(h)))
plt.title('Peaking EQ Filter Frequency Response')
plt.xlabel('Frequency [Hz]')
plt.ylabel('Amplitude [dB]')
plt.grid()
plt.show()
