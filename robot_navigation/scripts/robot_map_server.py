#/usr/bin/python3
# coding:utf-8
## 从rosparam获取地图参数，进行障碍物添加、裁剪等操作

import os, yaml
import rospy, tf.transformations
from robot_navigation.srv import *
from nav_msgs.msg import OccupancyGrid
import time
import threading


class MapManager():
    def __init__(self, namespace) -> None:
        self.image = "/opt/ROS_ws/my_robot_ws/src/robot_navigation/maps/aic.pgm"
        self.origin = []
        self.negate = 0
        self.resolution = 0
        self.free_thresh = 0
        self.occupied_thresh = 0
        self.__head, self.raster = self.read_map()
        self.__map_dir = self.yaml_path = os.path.join(
            os.path.split(os.path.realpath(__file__))[0], "../maps")

        self.update_config(namespace)


    def update_config(self, namespace):
        try:
            # 获取地图名称
            # yamf = rospy.get_param(f"{namespace}/map_name") + ".yaml"
            yamf = "aic.yaml"

            # 读取yaml配置文件
            yaml_path = os.path.join(self.__map_dir, yamf)
            file = open(yaml_path, 'r', encoding="utf-8")
            cfg = yaml.load(file, Loader=yaml.FullLoader)
            file.close()

            self.image = cfg.get("image")
            self.resolution = cfg.get("resolution")
            self.origin = cfg.get("origin")
            self.negate = cfg.get("negate")
            self.occupied_thresh = cfg.get("occupied_thrash")
            self.free_thresh = cfg.get("free_thresh")
            rospy.loginfo("地图参数加载完成")

        except IOError:
            rospy.logerr("Map Editor: 地图参数未正确加载！")
            rospy.signal_shutdown("Map Editor: 地图参数未正确加载")
            exit()


    def read_map(self):
        pgmf = open(self.image, 'rb')
        
        # 读取文件头
        head = []
        for i in range(4):
            head.append(pgmf.readline())
        assert head[0] == b'P5\n'
        height, width = [int(i) for i in head[2].split()]
        depth = int(head[3])
        assert depth <= 255

        # 读取栅格数据, 顺序(从左到右，从上到下)
        raster = []
        for i in range(height):
            row = []
            for j in range(width):
                row.append(ord(pgmf.read(1)))
            raster.append(row)
            
        pgmf.close()
        return head, raster      


    def write_map(self, newImageName):
        # 将修改后的图片写入文件
        height, width = [int(i) for i in self.__head[2].split()]
        rospy.loginfo(f"New map: {width} {height}")
        
        path = self.__map_dir + newImageName
        pgmf = open(newImageName, 'wb')

        for info in self.__head:
            pgmf.write(info)

        for j in range(height):
            for i in range(width):
                pgmf.write((self.raster[j][i]).to_bytes(1, byteorder="little"))
                
        pgmf.close()
       

    def get_size(self):
        return [int(i) for i in self.__head[2].split()]
        

    def get_map_dir(self):
        return self.__map_dir


    def get_raster(self):
        return self.raster


    def set_pixel(self, x, y, value):
        self.raster[y][x] = value


class StaticMapPublisher():
    def __init__(self, robotMapServer) -> None:
        self.__map_man = robotMapServer.get_map_manager()
        # 静态地图发布, latch参数保证后起节点能接收到地图
        self.map_pub = rospy.Publisher("map", OccupancyGrid, queue_size=1, latch=True)
        self.publisher_static_map()


    def publisher_static_map(self):
        pub_th = threading.Thread(target=self.publish)
        pub_th.setDaemon(True)
        pub_th.start()
        

    def publish(self):
        width, height = self.__map_man.get_size()
        map_msg = OccupancyGrid()
        map_msg.header.stamp = rospy.Time.now()
        map_msg.header.frame_id = "map"

        map_msg.info.map_load_time = rospy.Time.now()
        map_msg.info.resolution = self.__map_man.resolution
        map_msg.info.width = width
        map_msg.info.height = height
        map_msg.info.origin.position.x = self.__map_man.origin[0]
        map_msg.info.origin.position.y = self.__map_man.origin[1]
        map_msg.info.origin.position.z = self.__map_man.origin[2]
        qtn = tf.transformations.quaternion_from_euler(0, 0, 0)
        map_msg.info.origin.orientation.x = qtn[0]
        map_msg.info.origin.orientation.y = qtn[1]
        map_msg.info.origin.orientation.z = qtn[2]
        map_msg.info.origin.orientation.w = qtn[3]
        raster = self.__map_man.get_raster()
        # 将gmapping保存的图片栅格数据转换成map_server发布的标准
        for j in range(height - 1, -1, -1):
            for i in range(width):
                pixel = raster[j][i]
                if pixel == 205:
                    map_msg.data.append(-1)
                elif pixel == 254:
                    map_msg.data.append(0)
                elif pixel == 0:
                    map_msg.data.append(100)

        # 发布地图消息
        self.map_pub.publish(map_msg)
        rospy.loginfo("Map Server: 地图发布成功!")


class MapEditor():
    def __init__(self, robotMapServer) -> None:
        self.__context = robotMapServer
        self.__map_man = self.__context.get_map_manager()
        # 创建服务
        self.add_obstacle_server = rospy.Service("MapAddObstacle", MapAddObstacle, self.map_add_obstacle_response)
        # self.server = rospy.Service("MapCrop", MapCrop, self.map_crop_response)
        rospy.loginfo("Map Editor: 服务已经启动...")


    def map_add_obstacle_response(self, req):     
        rospy.loginfo("Map Editor: 开始处理...")
        
        try:
            assert req.gen_map_name.find('.') != -1
            mapname, suffix = req.gen_map_name.split('.')
            assert mapname != ""
            assert suffix == "pgm"
        except AssertionError:
            rospy.logwarn("Map Editor: 地图名称不规范！")
            return MapAddObstacleResponse("failed: gen_map_name is empty")

        # 获取障碍物数据
        obstacle = []
        for p in req.points:
            obstacle.append((p.x, p.y))

        # 添加障碍物
        self.add_obstacle(obstacle)
        path = self.__map_man.get_map_dir() + req.gen_map_name
        self.__map_man.write_map(path)
        
        # 重启地图服务
        rospy.loginfo("Map Editor: 障碍物添加成功")
        rospy.loginfo("Map Editor: 正在重新发布地图...")
        self.__context.get_static_map_publisher().publisher_static_map()
        return MapAddObstacleResponse("success")
            

    def map_crop_response(self, req):
        self.head, self.raster = self.crop_map(750, 750, 1250, 1250)


    def add_obstacle(self, obstacle):  # TODO: 障碍物数据重复的点太多了，待优化！
        for map_x, map_y in obstacle:
            # 修改灰度数值
            raster_x, raster_y = self.trans_map_to_raster(map_x, map_y)
            print(f"({raster_x}, {raster_y})", end="")
            if raster_x != -1:  
                self.__map_man.set_pixel(raster_x, raster_y, 0)
        print()


    def crop_map(self, topLeftX, topLeftY, bottomRightX, bottomRightY):
        left = topLeftX
        bottom = topLeftY
        top = bottomRightX
        right = bottomRightY

        self.head[2] = f'{right - left} {top - bottom}\n'.encode()

        new_raster = []
        for j in range(bottom, top):
            row = []
            for i in range(left, right):
                row.append(self.raster[j][i])
            new_raster.append(row)
                
        return self.head, new_raster


    def trans_map_to_raster(self, mapX, mapY):
        # 坐标转换 map to raster
        width, height = self.__map_man.get_size()
        raster_x = round((mapX - self.__map_man.origin[0]) / self.__map_man.resolution) 
        raster_y = height - round(((mapY - self.__map_man.origin[1]) / self.__map_man.resolution))

        if raster_x < 0  or raster_x > width or raster_y < 0 or raster_y > height:
            rospy.logwarn("Map Editor: 超出地图范围，将忽略此坐标")
            return (-1, -1)

        return raster_x, raster_y


class RobotMaperServer():
    def __init__(self, nodeName) -> None:
        # 初始化节点，创建服务对象
        rospy.init_node(nodeName, anonymous=False)
        self.__map_manager = MapManager("map_config")  
        self.__map_editor = MapEditor(self)
        self.__static_map_pub = StaticMapPublisher(self)


    def get_map_manager(self):
        return self.__map_manager


    def get_map_editor(self):
        return self.__map_editor


    def get_static_map_publisher(self):
        return self.__static_map_pub


if __name__ == '__main__':
    robot_map_server = RobotMaperServer("robot_map_server")
    rospy.spin()
