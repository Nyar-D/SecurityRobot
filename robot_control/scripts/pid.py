#!/usr/bin/python
# coding:utf-8
import RPi.GPIO as gpio
import time
import threading
import math
import rospy


# 向前行驶，左右轮需要不同速度
def set_pwm(pwm1, pwm2, t_time):
    left = pwm1 * -1 if pwm1 < 0 else pwm1
    l_isPosi = pwm1 > 0
    PL.ChangeDutyCycle(left)
    gpio.output(l_front, l_isPosi)
    gpio.output(l_back, not l_isPosi)

    right = pwm2 * -1 if pwm2 < 0 else pwm2
    r_isPosi = pwm2 > 0
    PR.ChangeDutyCycle(right)
    gpio.output(r_front, r_isPosi)
    gpio.output(r_back, not r_isPosi)

    time.sleep(t_time)


# 释放GPIO资源
def resetIO():
    PL.stop()
    PR.stop()
    gpio.output((l_pwm ,r_pwm), False)
    gpio.cleanup()


def count_la(l_a):                                       # 边缘检测回调函数
    global counter_le
    global isForward_l
    global lastOri_l
    global sameOriCnt_l
    global changeOriThreshold_l

    # 根据真值表判断正反转，相同次数到达改变方向的阈值才改变
    curOrient = (gpio.input(l_a) == gpio.input(l_b))
    sameOriCnt_l = sameOriCnt_l + 1 if curOrient == lastOri_l else 1
    isForward_l = curOrient if sameOriCnt_l == changeOriThreshold_l else isForward_l
    lastOri_l = curOrient

    if gpio.event_detected(l_a):                   
        counter_le += 1


# def count_lb(l_b): 
#     global counter_le , isForward_l
#     # isForward_l = (gpio.input(l_b) != gpio.input(l_a))
#     if gpio.event_detected(l_b):  
#         counter_le += 1


def count_ra(r_a):
    global counter_re
    global isForward_r, lastOri_r, sameOriCnt_r, changeOriThreshold_r

    # 左右轮对着安装，所以编码器的输出是相反的
    # 相同次数到达改变方向的阈值才改变
    curOri = (gpio.input(r_a) != gpio.input(r_b))   
    sameOriCnt_r = sameOriCnt_r + 1 if curOri == lastOri_r else 1
    isForward_r = curOri if sameOriCnt_r == changeOriThreshold_r else isForward_r
    lastOri_r = curOri

    if gpio.event_detected(r_a):
        counter_re += 1


# def count_rb(r_b):  
#     global counter_re, isForward_r
#     # isForward_r = (gpio.input(r_b) != gpio.input(r_a))
#     if gpio.event_detected(r_b):
#         counter_re += 1


# def compute_speed():
#     global u_time, cur_lv, cur_rv
#     global counter_le
#     global counter_re
    
#     if not isExit:
#         counter_le_last = counter_le
#         counter_re_last = counter_re

#         # 等待u_time时间
#         time.sleep(u_time)

#         # 左轮速度        
#         e = counter_le - counter_le_last
#         cur_lv = coe * e * (1 if isForward_l else -1)
#         # 右轮速度        
#         e = counter_re - counter_re_last
#         cur_rv = coe * e * (1 if isForward_r else -1)
        
#         # 开启下一次定时器
#         u_t0 = threading.Timer(u_interval, compute_speed)
#         u_t0.start()


def compute_speed():
    global u_time, cur_lv, cur_rv
    global counter_le, counter_le_last
    global counter_re, counter_re_last
    
    if not isExit:
        # 左轮速度        
        e = counter_le - counter_le_last
        cur_lv = coe * e * (1 if isForward_l else -1)
        # 右轮速度        
        e = counter_re - counter_re_last
        cur_rv = coe * e * (1 if isForward_r else -1)
        
        counter_le_last = counter_le
        counter_re_last = counter_re
        
        # 开启下一次定时器
        u_t0 = threading.Timer(u_time, compute_speed)
        u_t0.start()


# PID类
class pid:
    def __init__(self, speedToSet, currentSpeed, lastErr, totalErr):

        self.setPwm = speedToSet
        self.curV = currentSpeed
        self.lastErr = lastErr                      # 上一个误差值初始化
        self.totalErr = totalErr                    # 误差累加初始化


    def adjust(self, targetSpeed):
        
        if targetSpeed == 0:
            self.output = 0
            self.setPwm = 0
            pass

        error = targetSpeed - self.curV                                               # 计算偏差
        if (error < dead_w):                                                          # 死区控制
            self.output = 0
            pass

        self.totalErr = self.totalErr + error                                         # 偏差累加
        self.output = Kp * error + Ki * self.totalErr + Kd * (error - self.lastErr)   # PID运算
        self.lastErr = error                                                          # 将本次偏差赋给上次一偏差


def pid_adjust():
    global l_pid, r_pid
    global trg_lv, trg_rv
    
    while not isExit:
        l_pid.curV = cur_lv
        r_pid.curV = cur_rv

        # 目标速度和当前速度调整i
        l_pid.adjust(trg_lv)
        r_pid.adjust(trg_rv)
        
        # 根据PID输出调整轮子的pwm，即转速
        l_pid.setPwm += l_pid.output
        r_pid.setPwm += r_pid.output
        
        # 假如左右轮的速度理论值大于等于95，则令其等于95（原因：速度最大为100，以及避免电机长时间满负荷工作）
        # 速度为负代表反转
        l_pid.setPwm = 95 if l_pid.setPwm > 95 else l_pid.setPwm
        r_pid.setPwm = 95 if r_pid.setPwm > 95 else r_pid.setPwm
        l_pid.setPwm = -95 if l_pid.setPwm < -95 else l_pid.setPwm
        r_pid.setPwm = -95 if r_pid.setPwm < -95 else r_pid.setPwm

        set_pwm(l_pid.setPwm, r_pid.setPwm, 0)
        
        time.sleep(pid_t0)


def start():

    PL.start(0)  # 启动pwm功能，参数为初始占空比
    PR.start(0)
    
    # 四倍频检测
    gpio.add_event_detect(l_a, gpio.BOTH, callback=count_la)  # 在引脚上添加上升临界值检测再回调
    # gpio.add_event_detect(l_b, gpio.BOTH, callback=count_lb)
    gpio.add_event_detect(r_a, gpio.BOTH, callback=count_ra)
    # gpio.add_event_detect(r_b, gpio.BOTH, callback=count_rb)

    th_speed.start()
    th_pid.start()


# =========================================

# 左轮，右轮
l_front = 21    # 左轮正转
l_back = 26     # 左轮反转
l_pwm = 20
r_front = 19    # 右轮正转
r_back = 16     # 右轮反转
r_pwm = 13

# 左轮，右轮编码器
l_a = 17       # 左轮c1输出
l_b = 27       # 左轮c2输出
r_a = 18
r_b = 23

# gpio设置
gpio.setwarnings(False)              # 屏蔽警告
gpio.setmode(gpio.BCM)               # 设置引脚模式BCM
gpio.setup(l_pwm, gpio.OUT)  
gpio.setup(l_front, gpio.OUT)
gpio.setup(l_back, gpio.OUT)
gpio.setup(r_pwm, gpio.OUT)
gpio.setup(r_front, gpio.OUT)
gpio.setup(r_back, gpio.OUT)
gpio.setup(l_a, gpio.IN)  
gpio.setup(l_b, gpio.IN)
gpio.setup(r_a, gpio.IN)
gpio.setup(r_b, gpio.IN)

# 初始化脉冲计数器
counter_le = 0 
counter_re = 0
counter_le_last = 0
counter_re_last = 0

# 初始化速度计算参数，速度单位(rad/s)
# e = 0                             # 单位时间内编码器脉冲个数
u_time = 0.005                      # 单位时间
n = 3960 / 2                        # 编码器分辨率
l = 0.205                           # 轮胎间距
r = 0.023
coe = (2 * math.pi * r) / (u_time * n)  # 计算e的系数
rospy.loginfo(f"计算得到e的系数: {coe}")

# PID控制参数初始化
Kp = 10.0
Ki = 0.0
Kd = 0.4
dead_w = 0.1                  # 死区宽度
u_interval = 0.005            # 计算速度的间隔
pid_t0 = 0.005                 # PID计算间隔 

# 设定速度，当前速度
trg_lv = 0
trg_rv = 0
cur_lv = 0                       
cur_rv = 0 

# 退出标志，有c按下子线程就将isExit改为True
isExit = 0   

# 车轮方向判断    
isForward_l = 0       
isForward_r = 0  
sameOriCnt_l = 1
sameOriCnt_r = 1
lastOri_l = -1
lastOri_r = -1
changeOriThreshold_l = 4
changeOriThreshold_r = 4

# 左轮pwm，右轮pwm
PL = gpio.PWM(l_pwm, 50)
PR = gpio.PWM(r_pwm, 50)

# 初始化pid对象
l_pid = pid(trg_lv, 0, 0, 0)
r_pid = pid(trg_rv, 0, 0, 0)

# 线程定时器定时执行计算速度
th_speed = threading.Thread(target=compute_speed)
th_speed.setDaemon(True)

# 启动PID控制线程
th_pid = threading.Thread(target=pid_adjust)
th_pid.setDaemon(True)

# =========================================


# 不作为模块时单独运行
if __name__ == '__main__':
    try:
        start()
        while True:
            time.sleep(100)
            # pass # 用pass会影响编码器计数，原因未知

    except KeyboardInterrupt:
        resetIO()

    except SystemExit:
        resetIO()
    