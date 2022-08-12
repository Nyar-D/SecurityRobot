#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "robot_control/WheelSpeed.h"
#include <wiringPi.h>

int base_control_initial(void)
{
    if(wiringPiSetupGpio() == -1)
        exit(1)
    
    

}


// 向前行驶，左右轮需要不同速度
void set_pwm(pwm1, pwm2, t_time):
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


// 释放GPIO资源
void resetIO():
    PL.stop()
    PR.stop()
    gpio.output((l_pwm, l_front, l_back, r_pwm, r_front, r_back), False)
    gpio.cleanup()


// TODO
// w: 前进
// s: 后退
// a: 左转
// d: 右转
// q: 加速10%
// e: 减速10%

void count_lc1(l_c1):  // 边缘检测回调函数
    global counter_le  // 设置为全局变量
    if gpio.event_detected(l_c1):  // 检测到一个脉冲则脉冲数加1
        counter_le += 1

void count_lc2(l_c2):  // 边缘检测回调函数
    global counter_le  // 设置为全局变量
    if gpio.event_detected(l_c2):  // 检测到一个脉冲则脉冲数加1
        counter_le += 1

void count_rc1(r_c1):  // 这里的channel和channel1无须赋确定值，但笔者测试过，不能不写
    global counter_re
    if gpio.event_detected(r_c1):
        counter_re += 1

void count_rc2(r_c2):  // 这里的channel和channel1无须赋确定值，但笔者测试过，不能不写
    global counter_re
    if gpio.event_detected(r_c2):
        counter_re += 1


void compute_speed():
    global u_time, cur_lv, cur_rv
    global counter_le, counter_le_last
    global counter_re, counter_re_last
    
    if not isExit:
        
        // 左轮速度        
        e = counter_le - counter_le_last
        cur_lv = coe * e 
        counter_le_last = counter_le
        
        // 右轮速度        
        e = counter_re - counter_re_last
        cur_rv = coe * e
        counter_re_last = counter_re
        
        // 开启下一次定时器
        n_t0 = threading.Timer(u_time, compute_speed)
        n_t0.start()


// 位置式PID
class pid:
    void __init__(self, P: float, I: float, D: float):
        self.Kp = P        
        self.Ki = I        
        self.Kd = D        
        self.setV = 0
        self.lastErr = 0                                   // 上一次误差值初始化
        self.totalErr = 0                                  // 误差累加初始化


    void adjust(self, targetSpeed, currentSpeed):
        error = targetSpeed - currentSpeed                 // 计算偏差
        self.totalErr = self.totalErr + error              // 偏差累加

        KpWork = self.Kp * error
        KiWork = self.Ki * self.totalErr
        KdWork = self.Kd * (error - self.lastErr)
        self.output = KpWork + KiWork + KdWork
        
        self.lastErr = error                               // 将本次偏差赋给上一次偏差
        self.setV += self.output                           // 根据输出调整轮子的pwm

        // print(f"error:{error} lastErr:{self.lastErr}, totalErr:{self.totalErr}, output:{self.output}")


void pid_adjust():
    global l_pid, r_pid
    global trg_lv, trg_rv
    
    while not isExit:
        // 传入目标速度和当前速度
        l_pid.adjust(trg_lv, cur_lv)   
        r_pid.adjust(trg_rv, cur_rv)
        
        // 假如左右轮的速度理论值大于等于95，则令其等于95（原因：速度最大为100，以及避免电机长时间满负荷工作）
        // 假如左右轮的速度理论值小于等于0，则令其等于0（原因：速度最小为0）
        l_pid.setV = 95 if l_pid.setV > 95 else l_pid.setV
        r_pid.setV = 95 if r_pid.setV > 95 else r_pid.setV
        l_pid.setV = 0 if l_pid.setV < 0 else l_pid.setV
        r_pid.setV = 0 if r_pid.setV < 0 else r_pid.setV

        set_pwm(l_pid.setV, r_pid.setV, 0)
        
        print("++++++++++++++++++++++")
        print(f"PID输出:{l_pid.output} {r_pid.output}")
        print(f"目标速度:{trg_lv} {trg_rv}")
        print(f"当前速度:{cur_lv} {cur_rv}")
        print(f"调整速度:{l_pid.setV} {r_pid.setV}")
        print("++++++++++++++++++++++")
        
        time.sleep(pid_t0)


// 左轮，右轮
l_front = 21    // 左轮正转
l_back = 26     // 左轮反转
l_pwm = 20
r_front = 19    // 右轮正转
r_back = 16     // 右轮反转
r_pwm = 13

// 左轮，右轮编码器
l_c1 = 17       // 左轮c1输出
l_c2 = 27       // 左轮c2输出
r_c1 = 18
r_c2 = 23

// gpio设置
gpio.setwarnings(False)              // 屏蔽警告
gpio.setmode(gpio.BCM)               // 设置引脚模式BCM
gpio.setup(l_pwm, gpio.OUT)  
gpio.setup(l_front, gpio.OUT)
gpio.setup(l_back, gpio.OUT)
gpio.setup(r_pwm, gpio.OUT)
gpio.setup(r_front, gpio.OUT)
gpio.setup(r_back, gpio.OUT)
gpio.setup(l_c1, gpio.IN)  
gpio.setup(l_c2, gpio.IN)
gpio.setup(r_c1, gpio.IN)
gpio.setup(r_c2, gpio.IN)

// 初始化脉冲计数器
counter_le = 0 
counter_re = 0
counter_le_last = 0
counter_re_last = 0

// 初始化速度计算参数，速度单位(rad/s)
// e = 0                             // 单位时间内编码器脉冲个数
u_time = 0.005                      // 单位时间
n = 3960                            // 编码器分辨率
l = 0.19                            // 轮胎间距
coe = (2 * math.pi) / (u_time * n)  // 计算e的系数
print(f"计算得到e的系数: {coe}")

// PID控制参数初始化
Kp = 0.85
Ki = 0.0
Kd = 0.0
pid_t0 = 0.3                        // PID计算间隔 

// 设定速度，当前速度
trg_lv = 3
trg_rv = 3
cur_lv = 0                       
cur_rv = 0 

// 退出标志，有c按下子线程就将isExit改为True
isExit = 0           

// 左轮pwm，右轮pwm
PL = gpio.PWM(l_pwm, 50)
PR = gpio.PWM(r_pwm, 50)

// 初始化pid对象
l_pid = pid(Kp, Ki, Kd)
r_pid = pid(Kp, Ki, Kd)

// 线程定时器定时执行计算速度
th_speed = threading.Thread(target=compute_speed)
th_speed.setDaemon(True)

// 启动PID控制线程
th_pid = threading.Thread(target=pid_adjust)
th_pid.setDaemon(True)


void start():

    PL.start(0)  // 启动pwm功能，参数为初始占空比
    PR.start(0)
    
    // 四倍频检测
    gpio.add_event_detect(l_c1, gpio.BOTH, callback=count_lc1)  // 在引脚上添加上升临界值检测再回调
    gpio.add_event_detect(l_c2, gpio.BOTH, callback=count_lc2)
    gpio.add_event_detect(r_c1, gpio.BOTH, callback=count_rc1)
    gpio.add_event_detect(r_c2, gpio.BOTH, callback=count_rc2)

    th_speed.start()
    th_pid.start()
