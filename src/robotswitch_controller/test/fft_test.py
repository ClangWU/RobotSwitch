import numpy as np
import matplotlib.pyplot as plt

# 加载数据
ahrs_data = np.loadtxt('ahrs_position.txt')
ahrs = {
    "Time": ahrs_data[:, 0],
    "acc": ahrs_data[:, 1:4],
    "vel": ahrs_data[:, 5:8],
    "pos": ahrs_data[:, 8:11]
}

# 选择加速度数据中的X方向进行FFT
acceleration_data_x = ahrs["acc"][:, 1]

# 快速傅里叶变换
num_samples = len(acceleration_data_x)
sample_period = ahrs["Time"][1] - ahrs["Time"][0]  # 计算采样间隔
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
