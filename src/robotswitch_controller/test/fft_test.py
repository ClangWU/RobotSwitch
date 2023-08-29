import numpy as np
import matplotlib.pyplot as plt
# static3 all calibration
# static4 perfect only z cali
# static6 all calibration
# dyn 2 circle 50Hz
# Import data  
ahrs_data = np.loadtxt('ahrs_dyn6.txt')
ahrs = {
    "Time": ahrs_data[:, 0],
    "acc": ahrs_data[:, 1:4],
    "realacc": ahrs_data[:, 4:7],
    "vel": ahrs_data[:, 7:10],
    "pos": ahrs_data[:, 10:13],
    "qua":ahrs_data[:, 13:16]
}

# 选择加速度数据中的X方向进行FFT
acceleration_data_x = ahrs["pos"][:, 1]
acceleration_filtered_x = ahrs["realacc"][:, 1]

# 快速傅里叶变换
num_samples = len(acceleration_filtered_x)
sample_period = 0.005  # 计算采样间隔
print(sample_period)
frequencies = np.fft.rfftfreq(num_samples, sample_period)
fft_values = np.fft.rfft(acceleration_filtered_x)
magnitude = np.abs(fft_values)

# 画图
plt.figure(figsize=(10, 5))
plt.plot(frequencies, magnitude)
plt.title("Frequency Spectrum of Acceleration Filtered(X-axis)")
plt.xlabel("Frequency (Hz)")
plt.ylabel("Magnitude")
plt.grid(True)
plt.show()

# 选择加速度数据中的X方向进行FFT

# 快速傅里叶变换
num_samples = len(acceleration_data_x)
sample_period = 0.005  # 计算采样间隔
print(sample_period)
frequencies = np.fft.rfftfreq(num_samples, sample_period)
fft_values = np.fft.rfft(acceleration_data_x)
magnitude = np.abs(fft_values)

# 画图
plt.figure(figsize=(10, 5))
plt.plot(frequencies, magnitude)
plt.title("Frequency Spectrum of Acceleration (X-axis)")
plt.xlabel("Frequency (Hz)")
plt.ylabel("Magnitude")
plt.grid(True)
plt.show()
