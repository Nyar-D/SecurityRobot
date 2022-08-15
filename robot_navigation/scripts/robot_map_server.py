#/usr/bin/python3
# coding:utf-8
## Robot的地图服务，提供地图发布、障碍物增删、裁剪服务


from curses import curs_set
from email.errors import ObsoleteHeaderDefect
import os, yaml
from re import I
from tkinter.tix import Tree
from xml.etree.ElementPath import ops
from wsgiref.util import request_uri
import threading
import rospy, tf.transformations
from robot_navigation.srv import *
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid


class MapManager():
    def __init__(self, robotMapServer) -> None:
        self.__lock = threading.Lock()
        self.__context = robotMapServer
        self.__map_dir = self.__context.get_map_dir()
        # 图片信息
        self.__image = ""
        self.__origin = []
        self.__negate = 0
        self.__resolution = 0
        self.__free_thresh = 0
        self.__occupied_thresh = 0
        self.__head = []
        self.__raster = []
        # 障碍物继承关系、障碍物数据
        self.__inherit_map = {}
        self.__obstalcle = {}


    def update_config(self, map):
        try:
            # 如果内存中有这个图片的数据则不从文件中读取
            if map in self.__obstalcle:
                return

            # 读取yaml配置文件
            yamp = os.path.join(self.__map_dir, map + ".yaml")
            file = open(yamp, 'r', encoding="utf-8")
            cfg = yaml.load(file, Loader=yaml.FullLoader)
            file.close()

            self.__image = cfg.get("image")
            self.__resolution = cfg.get("resolution")
            self.__origin = cfg.get("origin")
            self.__negate = cfg.get("negate")
            self.__occupied_thresh = cfg.get("occupied_thresh")
            self.__free_thresh = cfg.get("free_thresh")

            # 读取图片内容
            self.__head, self.__raster = self.read_map()

            # 初始化obstacle和inherit_map
            self.__obstalcle.clear()
            self.__obstalcle[map] = {}
            self.__inherit_map.clear()
            self.__inherit_map[map] = ""

            rospy.loginfo("地图参数加载完成")

        except IOError as e:
            rospy.logerr(e)
            rospy.logerr("Map Editor: 地图参数未正确加载！")


    def read_map(self):
        pgmf = open(self.__image, 'rb')
        
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


    def generate_map(self, genMap, coordinate, pixelValue, thread=False):
        # 是否新地图, 是否已添加障碍物
        is_new = True if not genMap in self.__context.get_map_list() else False
        is_obsted = True if genMap in self.__obstalcle else False
        cur_map = self.__context.get_cur_map()
        obs_map = self.__obstalcle
        inh_map = self.__inherit_map
        
        if not is_obsted:
            obs_map[genMap] = dict()

        # format:{ map_name:{(x, y):value, ... }, ... }

        if genMap == cur_map:
            for (x, y) in coordinate:
                obs_map[genMap][(x, y)] = pixelValue
        else:
            # 修改继承关系
            # 找到genMap的后继节点
            for succ, pre in inh_map.items():
                if pre == genMap:

                    # 将genMap的障碍物加到后继节点中
                    for point, value in obs_map[genMap]:
                        succ_obs = obs_map[succ]
                        succ_obs[point] = value

                    # 将succ的前继设为genMap的前继节点
                    inh_map[succ] = inh_map[genMap]
                    break

            # 将genMap的前继节点设为当前地图
            inh_map[genMap] = cur_map                    

            # 更新obs[genMap]
            obs_map[genMap].clear()
            for (x, y) in coordinate:
                obs_map[genMap][(x, y)] = pixelValue

        print(self.__inherit_map)

        # 是否使用线程写文件
        if thread == False:
            self.__do_write(genMap, is_new)
        else:
            write_th = threading.Thread(target=self.__do_write, args=(genMap, is_new))
            write_th.start()

    
    def __do_write(self, mapName, is_new):
        if is_new:
            self.write_yaml(mapName)
        
        self.write_pgm(mapName)


    def write_yaml(self, mapName):
        aproject = {"image":os.path.join(self.__map_dir, (mapName + ".pgm")),
                    "resolution":self.__resolution,
                    "origin":self.__origin,
                    "negate":self.__negate,
                    "occupied_thresh":self.__occupied_thresh,
                    "free_thresh":self.__free_thresh
        }
        yamp = os.path.join(self.__map_dir, (mapName + ".yaml"))
        with open(yamp, 'w') as f:
            yaml.dump(aproject, f, sort_keys=False)


    def write_pgm(self, mapName):
        # 将增加了障碍物的图片写入文件
        tmp = []
        obs_map = self.__obstalcle
        inh_map = self.__inherit_map
        height, width = [int(i) for i in self.__head[2].split()]
        path = os.path.join(self.__map_dir, (mapName + ".pgm"))
        rospy.loginfo(f"Map Server: new map: [{width}, {height}] [{path}]")

        # 把障碍物添加到raster中
        while mapName in inh_map:
            for (x, y), v in obs_map[mapName].items():
                tmp.append((x, y, self.__raster[y][x]))
                self.__raster[y][x] = v
            mapName = inh_map[mapName]

        pgmf = open(path, 'wb')

        # 写入头部信息
        for info in self.__head:
            pgmf.write(info)

        # 写入数据部分
        for y in range(height):
            for x in range(width):
                pgmf.write((self.__raster[y][x]).to_bytes(1, byteorder="little"))

        pgmf.close()

        # 恢复raster列表
        for x, y, v in tmp:
            self.__raster[y][x] = v

    
    def get_image(self):
        return self.__iamge


    def get_origin(self):
        return self.__origin


    def get_negate(self):
        return self.__negate


    def get_resolution(self):
        return self.__resolution


    def get_occupied_thresh(self):
        return self.__occupied_thresh


    def get_free_thresh(self):
        return self.__free_thresh


    def get_size(self):
        return [int(i) for i in self.__head[2].split()]


    def get_raster(self, mapName):
        if not mapName in self.__obstalcle:
            return self.__raster
        
        # 注意多级列表的深拷贝
        raster = []
        for i in range(len(self.__raster)):
            row = list(self.__raster[i])
            raster.append(row)

        print(self.__inherit_map)
        # 添加障碍物信息
        while mapName in self.__inherit_map:
            for (x, y), value in self.__obstalcle[mapName].items():
                raster[y][x] = value
            mapName = self.__inherit_map[mapName]

        return raster


class StaticMapPublisher():
    def __init__(self, robotMapServer) -> None:
        self.__context = robotMapServer
        self.__map_man = self.__context.get_map_manager()
        # 静态地图发布, latch参数保证后起节点能接收到地图
        self.map_pub = rospy.Publisher("map", OccupancyGrid, queue_size=1, latch=True)


    def publish_static_map(self, mapName, thread=False):
        if thread == False:
            self.do_publish(mapName)
        else:
            pub_th = threading.Thread(target=self.do_publish, args=(mapName, ))
            pub_th.setDaemon(True)
            pub_th.start()
        

    def do_publish(self, mapName):
        # 读取当前MapManager中图片的参数和raster，发布到/map话题
        rospy.loginfo(f"Map Server: 正在发布地图: {mapName}")
        width, height = self.__map_man.get_size()
        map_msg = OccupancyGrid()
        map_msg.header.stamp = rospy.Time.now()
        map_msg.header.frame_id = "map"

        map_msg.info.map_load_time = rospy.Time.now()
        map_msg.info.resolution = self.__map_man.get_resolution()
        map_msg.info.width = width
        map_msg.info.height = height
        origin = self.__map_man.get_origin()
        map_msg.info.origin.position.x = origin[0]
        map_msg.info.origin.position.y = origin[1]
        map_msg.info.origin.position.z = origin[2]
        qtn = tf.transformations.quaternion_from_euler(0, 0, 0)
        map_msg.info.origin.orientation.x = qtn[0]
        map_msg.info.origin.orientation.y = qtn[1]
        map_msg.info.origin.orientation.z = qtn[2]
        map_msg.info.origin.orientation.w = qtn[3]

        # 将gmapping保存的图片栅格数据转换成map_server发布的标准
        # 205:未知区域, 254:空白区域, 0:障碍物
        raster = list(self.__map_man.get_raster(mapName))
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


class RobotMaperServer():
    def __init__(self, nodeName) -> None:
        self.__map_list = []
        self.__curent_map = "aic"
        self.__namespace = "map_config"
        self.__map_dir = os.path.join(
                    os.path.split(os.path.realpath(__file__))[0], "../maps")
        
        # 读取map文件列表
        for filename in os.listdir(self.__map_dir):
            mapname, ending = os.path.splitext(filename)
            if ending == ".yaml":
                self.__map_list.append(mapname)

        # 初始化节点，创建服务对象
        rospy.init_node(nodeName, anonymous=False)
        rospy.Subscriber("/robot_upper_control/static_map", String, self.static_map_callback)
        rospy.on_shutdown(self.del_param)
        self.set_cur_map_param(self.__curent_map)
        self.set_map_list_param()
        self.__map_man = MapManager(self)  
        self.__map_editor = MapEditor(self)
        self.__map_pub = StaticMapPublisher(self)
        self.__map_man.update_config(self.__curent_map)
        self.__map_pub.publish_static_map(self.__curent_map, thread=False)


    def handle_map_changes(self, genMap):
        # 如果更改的是当前的地图则立刻再次发布
        if genMap == self.__curent_map:
            self.__map_pub.publish_static_map(genMap, thread=True)
        else:
            if not genMap in self.__map_list:
                self.__map_list.append(genMap)
                self.set_map_list_param()


    def static_map_callback(self, mapNameMsg):
        self.__curent_map = mapNameMsg.data
        self.set_cur_map_param(mapNameMsg.data)
        self.__map_man.update_config(mapNameMsg.data)
        self.__map_pub.publish_static_map(mapNameMsg.data, thread=False)


    def set_map_list_param(self):
        # 设置可用地图列表参数
        rospy.set_param(self.__namespace + "/map_list", self.__map_list)


    def set_cur_map_param(self, curMap):
        self.__curent_map = curMap
        rospy.set_param(self.__namespace + "/current_map", self.__curent_map)
        

    def del_param(self):
        try:
            rospy.delete_param(f"{self.__namespace}/current_map")
            rospy.delete_param(f"{self.__namespace}/map_list")
        except Exception as e:
            rospy.loginfo(e)


    def get_map_dir(self):
        return self.__map_dir


    def get_namespace(self):
        return self.__namespace


    def get_cur_map(self):
        return self.__curent_map


    def get_map_list(self):
        return self.__map_list


    def get_map_manager(self):
        return self.__map_man


    def get_map_editor(self):
        return self.__map_editor


    def get_static_map_publisher(self):
        return self.__map_pub


class MapEditor():
    def __init__(self, robotMapServer) -> None:
        self.__context = robotMapServer
        self.__map_man = self.__context.get_map_manager()
        # 创建服务
        self.add_obstacle_server = rospy.Service(
            "MapEditObstacle", MapEditObstacle, self.handle_map_edit_obs)
        # self.server = rospy.Service("MapCrop", MapCrop, self.map_crop_response)
        rospy.loginfo("Map Editor: 服务已经启动...")


    def handle_map_edit_obs(self, req):     
        rospy.loginfo("Map Editor: 开始处理...")
        
        try:
            assert req.mode == "add" or req.mode == "delete"
            assert req.gen_map_name != ""

        except AssertionError:
            rospy.logwarn("Map Editor: 请求信息有误！")
            return MapEditObstacleResponse(
                "failed: The requested information is incorrect!")

        # 获取生成的地图名称
        gen_map = req.gen_map_name

        # 获取mode
        mode = req.mode

        # mode对应的像素灰度值(添加/消除 对应 障碍/空白)
        # (205:未知区域, 254:空白区域, 0:障碍物)
        if mode == "delete":
            value = 254
        elif mode == "add":
            value = 0

        # 由于不需要索引，所以使用Set集合保存点集
        # Set还有一个优点是可以自动删除掉重复的点
        obs = set()

        # 获取障碍物数据
        # 将map_xy转换成raster_xy
        for p in req.points:
            obs.add(self.trans_map_to_raster(p.x, p.y))

        # 优化障碍物列表(加粗)
        self.optimize_obs(obs)
        # rospy.loginfo(f"Map Editor: {obs}")

        # 写入障碍物, 通知RobotMapServer地图参数变更
        flag = (gen_map != self.__context.get_cur_map())
        self.__map_man.generate_map(gen_map, obs, value, thread=flag)
        self.__context.handle_map_changes(gen_map)
        
        # 服务反馈
        if mode == "delete":
            rospy.loginfo("Map Editor: 障碍物消除成功")
        elif mode == "add":
            rospy.loginfo("Map Editor: 障碍物添加成功")
        return MapEditObstacleResponse("success")
            

    def optimize_obs(self, obstacle):
        # 加粗
        tmp = set()

        for x, y in obstacle:
            # tmp.add((x - 1, y - 1))
            # tmp.add((x - 1, y))
            # tmp.add((x - 1, y + 1))
            # tmp.add((x, y - 1))
            tmp.add((x, y + 1))
            # tmp.add((x + 1, y - 1))
            tmp.add((x + 1, y))
            tmp.add((x + 1, y + 1))

        obstacle.update(tmp)


    def handle_map_crop(self, req):
        # TODO: 未完成
        self.head, self.raster = self.crop_map(750, 750, 1250, 1250)


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
        origin = self.__map_man.get_origin()
        res = self.__map_man.get_resolution()

        # Transform
        raster_x = round((mapX - origin[0]) / res) 
        raster_y = height - round(((mapY - origin[1]) / res))

        if raster_x < 0  or raster_x > width or raster_y < 0 or raster_y > height:
            rospy.logwarn("Map Editor: 超出地图范围，将忽略此坐标")
            return (-1, -1)

        return raster_x, raster_y


if __name__ == '__main__':
    robot_map_server = RobotMaperServer("robot_map_server")
    rospy.spin()
