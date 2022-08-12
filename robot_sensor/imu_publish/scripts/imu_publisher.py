#!/usr/bin/python3
#coding:utf-8
## imu节点，发布mpu6050传感器数据

from multiprocessing.connection import wait
import smbus
import time
import math
import rospy, tf.transformations
from sensor_msgs.msg import Imu


class MPU6050Reader():
    def __init__(self) -> None:
        # 外接I2C设备的地址
        self.address = 0x68
        # 电源控制寄存器地址
        power_cfg_regist = 0x6b
        # 陀螺仪配置寄存器
        gyro_cfg_regist = 0x1b
        # 加速度传感器配置寄存器
        accel_cfg_regist = 0x1c
        # 陀螺仪采样率分频寄存器
        smplrt_div_regist = 0x19
        # I2C模块初始化
        self.bus = smbus.SMBus(1)
        
        try:
            # 复位、设置电源模式
            self.bus.write_byte_data(self.address, power_cfg_regist, 0x00)
            # 设置陀螺仪量程为+-500/s，灵敏度为65536/1000=65.5 LSB/(°/s)
            self.bus.write_byte_data(self.address, gyro_cfg_regist, 0x08)
            # 设置陀螺仪量程为+-2g/s，灵敏度为65536/4=16384 LSB/g
            self.bus.write_byte_data(self.address, accel_cfg_regist, 0x00)
            # 采样频率=输出频率/(1+SMPLRT_DIV)
            self.bus.write_byte_data(self.address, smplrt_div_regist, 0x00)
        except OSError:
            rospy.logerr("imu: Remote I/O error 请重启设备！")
            rospy.signal_shutdown("imu: Remote I/O error 请重启设备！")
            exit()

        # 传感器初始校准值
        self.g_offset = [2.165, 0.870, -0.436]
        self.a_offset = [-0.0631, 0.0128, 0.0551]
        
        # 测量校准，初始化
        self.initial()
        self.calibrate()
        
        
    def initial(self):
        self.gx = 0
        self.gy = 0
        self.gz = 0
        self.ax = 0
        self.ay = 0
        self.az = 0
        self.rx = 0
        self.ry = 0
        self.rz = 0
        self.gx_ofs = self.g_offset[0]
        self.gy_ofs = self.g_offset[1]
        self.gz_ofs = self.g_offset[2]
        self.ax_ofs = self.a_offset[0]
        self.ay_ofs = self.a_offset[1]
        self.az_ofs = self.a_offset[2]
        self.last_time = rospy.Time.now()
        

    # 读取一个字长度的数据(16位)
    def read_word(self, adr):
        high = self.bus.read_byte_data(self.address, adr)
        low = self.bus.read_byte_data(self.address, adr+1)
        val = (high << 8) + low
        return val


    # 将读取到的数据转换为原码 (有符号数本身是采用补码方式存储的)
    def read_word_2c(self, adr):
        val = self.read_word(adr)
        x = 0xffff
        # 首位为1 表示是负数
        if (val >= 0x8000):
            # 求原码
            return -((x - val)+1)
        else:
            return val


    # 获得平方和
    def dist(self, a, b):
        return math.sqrt((a*a)+(b*b))


    # 计算rpy角，单位为弧度
    def update_rotation_by_accel(self):
        self.rx = math.atan2(self.ay, self.dist(self.ax, self.az))
        self.ry = math.atan2(self.ax, self.dist(self.ay, self.az))

        max = math.radians(360)
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()
        drz = self.gz * dt
        self.rz += drz
        self.rz = self.rz if self.rz < max else self.rz - max
        self.rz = self.rz if self.rz > -max else self.rz + max
        self.last_time = current_time       
    
    
    # 陀螺仪角速度积分得到角度值
    def update_rotation_by_gyro(self):
        max = math.radians(360)
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()
        drx = self.gx * dt
        dry = self.gy * dt
        drz = self.gz * dt
        self.rx += drx
        self.ry += dry
        self.rz += drz
        self.rx = self.rx if self.rx < max else self.rx - max
        self.rz = self.rz if self.rz < max else self.rz - max
        self.ry = self.ry if self.ry < max else self.ry - max
        self.ry = self.ry if self.ry > -max else self.ry + max
        self.rx = self.rx if self.rx > -max else self.rx + max
        self.rz = self.rz if self.rz > -max else self.rz + max
        self.last_time = current_time       

    
    def update_data(self):
        try:
            # 获得原始数据(除以灵敏度)，再转换为弧度
            self.gx = math.radians(self.read_word_2c(0x43) / 65.5 + self.gx_ofs)
            self.gy = math.radians(self.read_word_2c(0x45) / 65.5 + self.gy_ofs)
            self.gz = math.radians(self.read_word_2c(0x47) / 65.5 + self.gz_ofs)
            self.ax = math.radians(self.read_word_2c(0x3b) / 16384.0 + self.ax_ofs)
            self.ay = math.radians(self.read_word_2c(0x3d) / 16384.0 + self.ay_ofs)
            self.az = math.radians(self.read_word_2c(0x3f) / 16384.0 + self.az_ofs)
            self.update_rotation_by_accel()
        except OSError:
            rospy.logwarn(f"imu: Remote I/O error")

    
    # 此函数用于校准MPU6050
    def calibrate(self):
        sum = [0] * 6
        avg = [0] * 6
        cnt = 0
        clb_t0 = 1
        
        try:
            rospy.logwarn(f"自动校准中，请将小车水平静止放置，等待{clb_t0}s...")
            start = time.time()
            while (time.time() - start) < clb_t0:        
                cnt += 1
                sum[0] += self.read_word_2c(0x43) / 65.5
                sum[1] += self.read_word_2c(0x45) / 65.5
                sum[2] += self.read_word_2c(0x47) / 65.5
                sum[3] += self.read_word_2c(0x3b) / 16384.0
                sum[4] += self.read_word_2c(0x3d) / 16384.0
                sum[5] += self.read_word_2c(0x3f) / 16384.0
                avg[0] = sum[0] / cnt
                avg[1] = sum[1] / cnt
                avg[2] = sum[2] / cnt
                avg[3] = sum[3] / cnt
                avg[4] = sum[4] / cnt
                avg[5] = sum[5] / cnt
                time.sleep(0.004)
        except OSError:
            rospy.logerr("imu: Remote I/O error")
            rospy.logerr("校准过程中出现读取错误，请重启设备！")
            rospy.signal_shutdown("校准过程中出现读取错误，请重启设备！")
            exit()
        
        
        self.g_offset[0] = -avg[0]
        self.g_offset[1] = -avg[1]
        self.g_offset[2] = -avg[2]
        self.a_offset[0] = -avg[3]
        self.a_offset[1] = -avg[4]
        self.a_offset[2] = -avg[5] + 1
        rospy.loginfo(f"陀螺仪校准值:{self.g_offset}")
        rospy.loginfo(f"加速计校准值:{self.a_offset}")
        
        self.initial()


class IMUPublisher():
    def __init__(self, nodeName) -> None:
        pub_t0 = 50 # imu发布频率
        
        # ROS初始化
        rospy.init_node(nodeName, anonymous=True)
        self.imu_pub = rospy.Publisher("imu/data_raw", Imu, queue_size=20)
        rate = rospy.Rate(pub_t0)

        # MPU6050读取类实例化
        self.imu_reader = MPU6050Reader()
        
        while not rospy.is_shutdown():
            self.publish_imu()
            # print("===========================================================")
            # print(f"x角速度:{self.imu_reader.gx:.10f}")
            # print(f"y角速度:{self.imu_reader.gy:.10f}")
            # print(f"z角速度:{self.imu_reader.gz:.10f} {self.imu_reader.rz:.5f}")
            # print(f"x加速度:{self.imu_reader.ax:.10f} {self.imu_reader.rx:.5f}")
            # print(f"y加速度:{self.imu_reader.ay:.10f} {self.imu_reader.ry:.5f}")
            # print(f"z加速度:{self.imu_reader.az:.10f}")
            rate.sleep()


    def publish_imu(self):
        imu_msg = Imu()
        imu_msg.header.stamp = rospy.Time.now()
        imu_msg.header.frame_id = "base_footprint"
        
        # 读取imu数据
        self.imu_reader.update_data()
        
        # 四元数
        qtn = tf.transformations.quaternion_from_euler(
            0, 0, self.imu_reader.rz)
        imu_msg.orientation.x = qtn[0]
        imu_msg.orientation.y = qtn[1]
        imu_msg.orientation.z = qtn[2]
        imu_msg.orientation.w = qtn[3]
        # 角速度    
        imu_msg.angular_velocity.x = 0
        imu_msg.angular_velocity.y = 0
        imu_msg.angular_velocity.z = self.imu_reader.gz
        imu_msg.linear_acceleration.x = self.imu_reader.ax
        imu_msg.linear_acceleration.y = self.imu_reader.ay
        imu_msg.linear_acceleration.z = self.imu_reader.az

        self.imu_pub.publish(imu_msg)
        

if __name__ == '__main__':
    imu_publisher = IMUPublisher("imu_publisher")