from madgwickahrs import MadgwickAHRS
from pytroykaimu import TroykaIMU


"""
imu = TroykaIMU()
# Чтобы пример работал корректно, нужно предварительно получить 
# калибровочную матрицу для вашего модуля
# Как это сделать описано в разделе "Калибровка магнитометра"
calibration_matrix = [[0.983175, 0.022738, -0.018581],
                      [0.022738, 0.942140, -0.022467],
                      [-0.018581, -0.022467, 1.016113]]

# raw measurements only
bias = [962.391696, -162.681348, 11832.188828]
"""


def readIMU(imu, calibration_matrix, bias):
    imu.magnetometer.calibrate_matrix(calibration_matrix, bias)

    filter = MadgwickAHRS(beta=1, sampleperiod=1/256)

    filter.update(imu.gyroscope.read_radians_per_second_xyz(),
                    imu.accelerometer.read_gxyz(),
                    imu.magnetometer.read_calibrate_gauss_xyz())
    data = filter.quaternion.to_angle_axis()

    dataencode = str(data).encode('utf-8')
    if dataencode:
        print(data)
    
    return