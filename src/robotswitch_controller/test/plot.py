import numpy as np
import matplotlib.pyplot as plt

# Import data
ahrs_data = np.loadtxt('ahrs_static1.txt')
ahrs = {
    "Time": ahrs_data[:, 0],
    "acc": ahrs_data[:, 1:4],
    "realacc": ahrs_data[:, 4:7],
    "pos": ahrs_data[:, 7:10],
    "qua": ahrs_data[:, 10:13]
}


# Plot
plt.figure("real acc")
# plt.plot(ahrs["vel"][:, 0], 'm', label='X')
# plt.plot(ahrs["vel"][:, 1], 'k', label='Y')
# plt.plot(ahrs["vel"][:, 2], 'y', label='Z')
plt.plot(ahrs["realacc"][:, 0], 'r', label='X')
plt.plot(ahrs["realacc"][:, 1], 'g', label='Y')
plt.plot(ahrs["realacc"][:, 2], 'b', label='Z')
plt.xlabel('sample')
plt.ylabel('dps')
plt.title('position')
plt.legend()
plt.show()

# Plot
plt.figure("position")
# plt.plot(ahrs["vel"][:, 0], 'm', label='X')
# plt.plot(ahrs["vel"][:, 1], 'k', label='Y')
# plt.plot(ahrs["vel"][:, 2], 'y', label='Z')
plt.plot(ahrs["pos"][:, 0], 'r', label='X')
plt.plot(ahrs["pos"][:, 1], 'g', label='Y')
plt.plot(ahrs["pos"][:, 2], 'b', label='Z')
plt.xlabel('sample')
plt.ylabel('dps')
plt.title('position')
plt.legend()
plt.show()