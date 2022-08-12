import sys
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import rospy
from robot_control.msg import WheelSpeed


class SubscriberNode():
    def __init__(self, nodeName) -> None:
        self.cur_lv = 0
        self.cur_rv = 0
        self.trg_lv = 0
        self.trg_rv = 0
        
        # 初始化节点
        rospy.init_node(nodeName, anonymous=True)
        
        #2.创建订阅者对象
        rospy.Subscriber("wheel_vel", WheelSpeed, self.change_speed, queue_size=100)
    
    
    def change_speed(self, data):
        self.cur_lv = data.current.left
        self.cur_rv = data.current.right
        self.trg_lv = data.target.left
        self.trg_rv = data.target.right
        
        print(f"当前:{self.cur_lv:.2f} {self.cur_rv:.2f}", end=" ")
        print(f"目标:{self.trg_lv:.2f} {self.trg_rv:.2f}", end=" ")
        print(f"偏差:{self.cur_lv - self.trg_lv:.2f} {self.cur_rv - self.trg_rv:.2f}")

        # rospy.spin() 


class Visualizer():
    def __init__(self) -> None:
         # 图像参数
        min_x = 0   
        self.max_x = 300
        min_y = -10
        max_y = 10
        # 动态图更新间隔
        interval = 100
                
        fig = plt.figure(figsize=(20,7))
        plt.grid(ls='--')
        plt.ion()   
        
        self.x = np.linspace(0, self.max_x, self.max_x)
        self.lcurY = [0] * self.max_x
        self.rcurY = [0] * self.max_x
        self.ltrgY = [sub_node.trg_lv] * self.max_x
        self.rtrgY = [sub_node.trg_rv] * self.max_x
        
        plt.xlim(min_x, self.max_x)
        plt.ylim(min_y, max_y)
        
        self.lcur_ani = plt.plot(self.x, self.lcurY, 'red')[0]
        self.rcur_ani = plt.plot(self.x, self.rcurY, 'blue')[0]  
        self.ltrg_ani = plt.plot(self.x, self.ltrgY, 'black')[0]
        self.rtrg_ani = plt.plot(self.x, self.ltrgY, 'black')[0]
        self.ani = animation.FuncAnimation(fig=fig, func=self.update, frames=np.arange(min_x, self.max_x), interval=interval)
        
        # 显示图片
        plt.ioff()
        plt.show()
        
        
    def update(self, index):
        global sub_node

        if index == 0:
            self.lcurY = [0] * self.max_x
            self.rcurY = [0] * self.max_x

        self.lcurY[index] = sub_node.cur_lv
        self.rcurY[index] = sub_node.cur_rv
        self.ltrgY = [sub_node.trg_lv] * self.max_x
        self.rtrgY = [sub_node.trg_rv] * self.max_x
        self.lcur_ani.set_data(self.x, self.lcurY)
        self.rcur_ani.set_data(self.x, self.rcurY)
        self.ltrg_ani.set_data(self.x, self.ltrgY)
        self.rtrg_ani.set_data(self.x, self.rtrgY)
            
        return [self.lcur_ani, self.rcur_ani, self.ltrgY, self.rtrgY]    


if __name__ == '__main__':

    try:
        sub_node = SubscriberNode('visualizer')
        visualizer = Visualizer()
        
    except KeyboardInterrupt:
        sys.exit(0)