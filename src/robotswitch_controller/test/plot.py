import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import mpl_toolkits.mplot3d.axes3d

# static3 all calibration
# static4 perfect only z cali
# static6 all calibration
# dyn 2 circle 50Hz
# dyn6 no acc filter
# Import data  
ahrs_data = np.loadtxt('ahrs_dyn_hpf2_0.1.txt')
ahrs = {
    "Time": ahrs_data[:, 0],
    "acc": ahrs_data[:, 1:4],
    "realacc": ahrs_data[:, 4:7],
    "vel": ahrs_data[:, 7:10],
    "pos": ahrs_data[:, 10:13],
    "qua":ahrs_data[:, 13:16]
}


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
plt.title('pos filter')
plt.legend()
plt.show()

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
plt.title('acc')
plt.legend()
plt.show()

plt.figure(figsize=(10, 6))

# 画出realacc的X轴数据
plt.plot(ahrs["realacc"][:, 2], 'r', label='Real Acc - X')
# 画出acc的X轴数据
plt.plot(ahrs["acc"][:, 2], 'b--', label='Acc - X')

plt.xlabel('sample')
plt.ylabel('dps')
plt.title('Comparison of acc and realacc for X axis')
plt.legend()
plt.grid(True)  # 为了更好地观察延迟，可以加上网格线
plt.tight_layout()
plt.show()
