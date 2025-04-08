#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
import ydlidar
import time
import math
import matplotlib.pyplot as plt
import numpy as np

def salvar_imagem_lidar(scan_msg, contador):
    # Converte os ângulos e distâncias em coordenadas cartesianas
    angulos = np.arange(scan_msg.angle_min, scan_msg.angle_max, scan_msg.angle_increment)
    distancias = np.array(scan_msg.ranges)
    if angulos.shape[0] != distancias.shape[0]:
        return

    x = distancias * np.cos(angulos)
    y = distancias * np.sin(angulos)

    # Plota os pontos
    plt.figure(figsize=(6, 6))
    plt.scatter(x, y, s=10, c='blue', alpha=0.75)
    plt.title(f'Leitura do LiDAR - Frame {contador}')
    plt.xlabel('Distância X (m)')
    plt.ylabel('Distância Y (m)')
    plt.grid(True)

    # Define os limites dos eixos para uma escala fixa de 6x6 metros
    plt.xlim(-1.5, 1.5)
    plt.ylim(-1.5, 1.5)

    # Salva a imagem com um nome sequencial
    nome_arquivo = f'lidar_frame.png'
    plt.savefig(nome_arquivo)
    plt.close()

class YDLidarNode:
    def __init__(self):
        rospy.init_node('ydlidar_node', anonymous=True)
        self.scan_pub = rospy.Publisher('/LaserScan', LaserScan, queue_size=10)
        
        # Initialize YDLIDAR
        ydlidar.os_init()
        ports = ydlidar.lidarPortList()
        port = "/dev/ydlidar"
        for key, value in ports.items():
            port = value
            rospy.loginfo(f'Using LiDAR on port: {port}')
        
        self.laser = ydlidar.CYdLidar()
        self.laser.setlidaropt(ydlidar.LidarPropSerialPort, port)
        self.laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, 115200)
        self.laser.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TRIANGLE)
        self.laser.setlidaropt(ydlidar.LidarPropDeviceType, ydlidar.YDLIDAR_TYPE_SERIAL)
        self.laser.setlidaropt(ydlidar.LidarPropScanFrequency, 10.0)
        self.laser.setlidaropt(ydlidar.LidarPropSampleRate, 1)
        self.laser.setlidaropt(ydlidar.LidarPropSingleChannel, False)
        self.laser.setlidaropt(ydlidar.LidarPropMaxAngle, 150.0)
        self.laser.setlidaropt(ydlidar.LidarPropMinAngle, -210.0)
        self.laser.setlidaropt(ydlidar.LidarPropMaxRange, 16.0)
        self.laser.setlidaropt(ydlidar.LidarPropMinRange, 0.28)
        self.laser.setlidaropt(ydlidar.LidarPropIntenstiy, False)

        self.cnt = 0

        if not self.laser.initialize():
            rospy.logerr("Failed to initialize YDLIDAR")
            return
        if not self.laser.turnOn():
            rospy.logerr("Failed to turn on YDLIDAR")
            return

        rospy.loginfo("LiDAR initialized successfully.")
        
    def scan_callback(self):
        scan = ydlidar.LaserScan()
        if self.laser.doProcessSimple(scan):
            scan_msg = LaserScan()
            scan_msg.header.stamp = rospy.Time.now()
            scan_msg.header.frame_id = "laser_frame"
            
            scan_msg.angle_min = scan.config.min_angle + (np.pi / 6)
            scan_msg.angle_max = scan.config.max_angle + (np.pi / 6)
            scan_msg.angle_increment = scan.config.angle_increment
            scan_msg.scan_time = scan.config.scan_time
            scan_msg.time_increment = scan.config.time_increment
            scan_msg.range_min = scan.config.min_range
            scan_msg.range_max = scan.config.max_range  
            
            size = int((scan.config.max_angle - scan.config.min_angle) / scan.config.angle_increment) + 1
            scan_msg.ranges = [float("inf")] * size
            scan_msg.intensities = [0.0] * size
            for point in scan.points:
                index = int(math.ceil((point.angle - scan.config.min_angle) / scan.config.angle_increment))
                if (index >= 0 and index < 25 + size // 2) or (index > size - 25 and index < size):
                # print(f"angle: {abs(point.angle)}, deg: {np.pi / 2}, is_true: {abs(point.angle) < np.pi / 2}")
                # if index >= 0 and index < size and abs(point.angle) < np.pi / 2:
                    if point.range >= scan.config.min_range:
                        scan_msg.ranges[size - index] = point.range
                        scan_msg.intensities[size - index] = point.intensity
            self.scan_pub.publish(scan_msg)
            # print("size: ", size)

            # salvar_imagem_lidar(scan_msg, self.cnt)
            self.cnt += 1
        else:
            rospy.logwarn("Failed to get Lidar Data")

    def run(self):
        rate = rospy.Rate(4)  # 10 Hz
        while not rospy.is_shutdown():
            self.scan_callback()
            rate.sleep()
        self.shutdown()

    def shutdown(self):
        rospy.loginfo("Shutting down LiDAR node...")
        self.laser.turnOff()
        self.laser.disconnecting()

def main():
    node = YDLidarNode()
    try:
        node.run()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()