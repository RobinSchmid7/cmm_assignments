# Simple plotting script for numerical integration schemes

import pandas as pd
import matplotlib.pyplot as plt

pee = ['../cmake-build-release/src/app/explicit_euler_0.001000.txt'
, '../cmake-build-release/src/app/explicit_euler_0.005000.txt'
, '../cmake-build-release/src/app/explicit_euler_0.010000.txt'
, '../cmake-build-release/src/app/explicit_euler_0.050000.txt'
, '../cmake-build-release/src/app/explicit_euler_0.100000.txt']

pse = ['../cmake-build-release/src/app/sympletic_euler_0.001000.txt'
, '../cmake-build-release/src/app/sympletic_euler_0.005000.txt'
, '../cmake-build-release/src/app/sympletic_euler_0.010000.txt'
, '../cmake-build-release/src/app/sympletic_euler_0.050000.txt'
, '../cmake-build-release/src/app/sympletic_euler_0.100000.txt']

pem = ['../cmake-build-release/src/app/explicit_midpoint_0.001000.txt'
, '../cmake-build-release/src/app/explicit_midpoint_0.005000.txt'
, '../cmake-build-release/src/app/explicit_midpoint_0.010000.txt'
, '../cmake-build-release/src/app/explicit_midpoint_0.050000.txt'
, '../cmake-build-release/src/app/explicit_midpoint_0.100000.txt']

for p in pse:

    data = pd.read_csv(p, sep=', ',header=None)
    data = pd.DataFrame(data)

    x = data[0]
    y = data[1]

    plt.xlim([0, 200])
    plt.ylim([5.75,7.25])
    plt.plot(x, y)

plt.title("Sympletic Euler")
plt.legend([0.001, 0.005, 0.01, 0.05, 0.1])
plt.xlabel("Iterations")
plt.ylabel("Hamiltonian")
plt.savefig('pse.png')
plt.show()