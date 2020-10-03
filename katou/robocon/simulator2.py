import matplotlib
#matplotlib.use('nbagg')
import matplotlib.animation as anm
import matplotlib.pyplot as plt
import math
import matplotlib.patches as patches
import numpy as np

class World:        ### fig:world_init_add_timespan (1-5行目)
    def __init__(self, time_span, time_interval, debug=False):
        self.objects = [] 
        self.debug = debug
        self.time_span = time_span                  # 追加
        self.time_interval = time_interval          # 追加
        
    def append(self,obj):             # オブジェクトを登録するための関数
        self.objects.append(obj)
    
    def draw(self):            ### fig:world_draw_with_timesapn (1, 10, 21-26, 28-34行目)
        fig = plt.figure(figsize=(4,4))                # 8x8 inchの図を準備
        ax = fig.add_subplot(111)                      # サブプロットを準備
        ax.set_aspect('equal')                         # 縦横比を座標の値と一致させる
        ax.set_xlim(-1.2,1.2)                              # X軸を-1.2m x 1.2mの範囲で描画
        ax.set_ylim(-1.2,1.2)                              # Y軸も同様に
        ax.set_xlabel("X",fontsize=10)                 # X軸にラベルを表示
        ax.set_ylabel("Y",fontsize=10)                 # 同じくY軸に
        
        elems = []

        if self.debug:        
            for i in range(int(self.time_span/self.time_interval)): self.one_step(i, elems, ax)
        else:
            self.ani = anm.FuncAnimation(fig, self.one_step, fargs=(elems, ax),
                                     frames=int(self.time_span/self.time_interval)+1, interval=int(self.time_interval*1000), repeat=False)
            plt.show()
        
    def one_step(self, i, elems, ax):
        while elems: elems.pop().remove()
        time_str = "t = %.2f[s]" % (self.time_interval*i)    # 時刻として表示する文字列
        elems.append(ax.text(-1, 1, time_str, fontsize=10))
        for obj in self.objects:
            obj.draw(ax, elems)#障害物関係の描画
            if hasattr(obj, "one_step"): obj.one_step(self.time_interval)#エージェント関係の描画                 # 変更



class IdealRobot:  
    def __init__(self, pose, agent=None, sensor=None, color="black"):    # 引数を追加
        self.pose = pose  
        self.r = 0.105
        self.color = color 
        self.agent = agent
        self.poses = [pose]
        self.shadow = pose
        self.sensor = sensor    # 追加
        self.xy1 = (self.pose[0]+self.r*math.cos(self.pose[2]),self.pose[1]+self.r*math.sin(self.pose[2]))
        self.xy2 = (self.pose[0]+self.r*math.cos(self.pose[2]+0.5*math.pi),self.pose[1]+self.r*math.sin(self.pose[2]+0.5*math.pi))
        self.xy3 = (self.pose[0]+self.r*np.sqrt(2)*math.cos(self.pose[2]+0.25*math.pi),self.pose[1]+self.r*np.sqrt(2)*math.sin(self.pose[2]+0.25*math.pi))
        self.X = (self.pose[0],self.xy1[0],self.xy2[0],self.xy3[0])
        self.Y = (self.pose[1],self.xy1[1],self.xy2[1],self.xy3[1])
    def draw(self, ax, elems):
        x, y, theta = self.pose
        x_ob, y_ob, theta_ob = self.shadow# 姿勢の変数を分解して3つの変数へ
        xn = x + self.r * math.cos(theta)         #  ロボットの鼻先のx座標 
        yn = y + self.r * math.sin(theta)         #  ロボットの鼻先のy座標 
        #xn = x         #  ロボットの鼻先のx座標 
        #yn = y         #  ロボットの鼻先のy座標
        elems += ax.plot([x,xn], [y,yn], color=self.color) # ロボットの向きを示す線分の描画
        c = ax.scatter(self.X, self.Y, s=100, marker="*", label="landmarks", color="orange")
        elems.append(c)
        r = patches.Rectangle(xy=(x, y), angle = int(theta*180 / math.pi),width = self.r,height = self.r,color=self.color,fill = False) 
        elems.append(ax.add_patch(r))   
        self.poses.append(self.pose)
        elems += ax.plot([e[0] for e in self.poses], [e[1] for e in self.poses], linewidth=0.5, color="black")
        if self.sensor and len(self.poses) > 1:
            self.sensor.draw(ax, elems, self.poses[-2])
            
    @classmethod 
    def state_transition(self, nu, omega, time, pose):
        t0 = pose[2]
        if math.fabs(omega) < 1e-10:
            return pose + np.array( [nu*math.cos(t0), 
                                     nu*math.sin(t0),
                                     omega ] ) * time
        else:
            return pose + np.array( [nu/omega*(math.sin(t0 + omega*time) - math.sin(t0)), 
                                     nu/omega*(-math.cos(t0 + omega*time) + math.cos(t0)),
                                     omega*time ] )

    def one_step(self, time_interval):
        if not self.agent: return        
        obs =self.sensor.data(self.pose) if self.sensor else None #追加
        nu, omega = self.agent.decision(obs) #引数追加
        self.shadow = self.pose
        self.pose = self.state_transition(nu, omega, time_interval, self.pose)
        self.xy1 = (self.pose[0]+self.r*math.cos(self.pose[2]),self.pose[1]+self.r*math.sin(self.pose[2]))
        self.xy2 = (self.pose[0]+self.r*math.cos(self.pose[2]+0.5*math.pi),self.pose[1]+self.r*math.sin(self.pose[2]+0.5*math.pi))
        self.xy3 = (self.pose[0]+self.r*np.sqrt(2)*math.cos(self.pose[2]+0.25*math.pi),self.pose[1]+self.r*np.sqrt(2)*math.sin(self.pose[2]+0.25*math.pi))
        self.X = (self.pose[0],self.xy1[0],self.xy2[0],self.xy3[0])
        self.Y = (self.pose[1],self.xy1[1],self.xy2[1],self.xy3[1])


class Agent:
    def __init__(self, nu, omega):
        self.nu = nu
        self.omega = omega
        
    def decision(self, observation=None):
        return self.nu, self.omega


class Landmark:
    def __init__(self, x, y,theta):
        self.pos = np.array([x, y,theta]).T
        self.id = None
        
    def draw(self, ax, elems):
        c = ax.scatter(self.pos[0], self.pos[1], s=100, marker="*", label="landmarks", color="orange")
        elems.append(c)
        elems.append(ax.text(self.pos[0], self.pos[1], "id:" + str(self.id), fontsize=10))


class Target:
    def __init__(self, pose, agent):
        self.pos = pose
        self.agent = agent
        self.id = None
        
    def draw(self, ax, elems):
        c = ax.scatter(self.pos[0], self.pos[1], s=100, marker="*", label="targets", color="orange")
        elems.append(c)
        #elems.append(ax.text(self.pos[0], self.pos[1], "id:" + str(self.id), fontsize=10))
    
    @classmethod 
    def state_transition(self, nu, omega, time, pose):
        t0 = pose[2]
        if math.fabs(omega) < 1e-10:
            return pose + np.array( [nu*math.cos(t0), 
                                     nu*math.sin(t0),
                                     omega ] ) * time
        else:
            return pose + np.array( [nu/omega*(math.sin(t0 + omega*time) - math.sin(t0)), 
                                     nu/omega*(-math.cos(t0 + omega*time) + math.cos(t0)),
                                     omega*time ] )
    
    def one_step(self, time_interval):
        nu, omega = self.agent.decision(obs) #引数追加
        self.pose = self.state_transition(nu, omega, time_interval, self.pose)


class Obstacle:#center障害物用のクラス
    def __init__(self,x,y,width,height):
        self.x=x#障害物の端点の座標
        self.y=y#障害物の端点の座標
        self.width = width#障害物の幅
        self.height = height#障害物の高さ
        self.id = None
        self.xy1 = (x-height/np.sqrt(2),y+height/np.sqrt(2)-0.5*(self.width**2 + self.height**2)**0.5)
        self.xy2 = (x+width/np.sqrt(2)-height/np.sqrt(2),y+height/np.sqrt(2)+width/np.sqrt(2)-0.5*(self.width**2 + self.height**2)**0.5)
        self.xy3 = (x+width/np.sqrt(2),y+width/np.sqrt(2)-0.5*(self.width**2 + self.height**2)**0.5)
        self.X = (self.x,self.xy1[0],self.xy2[0],self.xy3[0])
        self.Y = (self.y-0.5*(self.width**2 + self.height**2)**0.5,self.xy1[1],self.xy2[1],self.xy3[1])
    def draw(self, ax, elems):
        x1 = self.x
        y1 = self.y -0.5*(self.width**2 + self.height**2)**0.5
        r = patches.Rectangle(xy = (x1,y1),width = self.width,\
                              height = self.height,\
                              color="blue",angle = 45,alpha=0.2)
        c = ax.scatter(self.X, self.Y, s=100, marker="*", label="landmarks", color="orange")
        elems.append(c)
        elems.append(ax.add_patch(r))


class Obstacle2(Obstacle):#周辺4つの障害物用のクラス
    def draw(self,ax,elems):
        x1 = self.x -0.3*(self.width**2 + self.height**2)**0.5 
        y1 = self.y -0.4*(self.width**2 + self.height**2)**0.5
        r = patches.Rectangle(xy = (x1,y1),width = self.width,\
                              height = self.height,\
                              color="blue",angle = 45,alpha=0.2)
        elems.append(ax.add_patch(r))


        
class Map:
    def __init__(self):       # 空のランドマークのリストを準備
        self.landmarks = []
        self.obstacles = []
        self.obstacles2 = []
        self.shadows = []
        self.targets=[]
        
    def append_landmark(self, landmark):       # ランドマークを追加
        landmark.id = len(self.landmarks)           # 追加するランドマークにIDを与える
        self.landmarks.append(landmark)
    
    def append_target(self, target):       # ランドマークを追加
        target.id = len(self.targets)           # 追加するランドマークにIDを与える
        self.targets.append(target)
        
    def append_obstacle(self, obstacle):       # obstacleを追加
        obstacle.id = len(self.obstacles)           # 追加するランドマークにIDを与える
        self.obstacles.append(obstacle)
        
    def append_obstacle2(self, obstacle):       # obstacle2を追加
        obstacle.id = len(self.obstacles)           # 追加するランドマークにIDを与える
        self.obstacles.append(obstacle)

    def draw(self, ax, elems):                 # 描画（Landmarkのdrawを順に呼び出し）
        for lm in self.landmarks: lm.draw(ax, elems)
        for ob in self.obstacles: ob.draw(ax, elems)
        for ob2 in self.obstacles2: ob2.draw(ax, elems)
        for tar in self.targets:
            tar.draw(ax,elms)
            #tar.one_step(0.1)



class IdealCamera:                            ### fig:camera3
    def __init__(self, env_map, \
                 distance_range=(0.3, 1.2),
                 direction_range=(-math.pi/3, math.pi/3)):
        self.map = env_map
        self.lastdata = []
        
        self.distance_range = distance_range   #追加
        self.direction_range = direction_range  #追加
        
    def visible(self, polarpos):  # ランドマークが計測できる条件
        if polarpos is None:
            return False
        
        return self.distance_range[0] <= polarpos[0] <= self.distance_range[1] \
                and self.direction_range[0] <= polarpos[1] <= self.direction_range[1]
    
        
    def data(self, cam_pose):
        observed = []
        for lm in self.map.landmarks:
            z = self.observation_function(cam_pose, lm.pos)
            if self.visible(z):               # 条件を追加
                observed.append((z, lm.id))   # インデント
            
        self.lastdata = observed 
        return observed
    
    @classmethod
    def observation_function(cls, cam_pose, obj_pos):
        diff = obj_pos - cam_pose[0:2]
        phi = math.atan2(diff[1], diff[0]) - cam_pose[2]
        while phi >= np.pi: phi -= 2*np.pi
        while phi < -np.pi: phi += 2*np.pi
        return np.array( [np.hypot(*diff), phi ] ).T
    
    def draw(self, ax, elems, cam_pose): 
        for lm in self.lastdata:
            x, y, theta = cam_pose
            distance, direction = lm[0][0], lm[0][1]
            lx = x + distance * math.cos(direction + theta)
            ly = y + distance * math.sin(direction + theta)
            elems += ax.plot([x,lx], [y,ly], color="pink")

class TargetCamera:                            ### fig:camera3
    def __init__(self, target_agent, \
                 env_map,
                 distance_range=(0.5, 2*np.sqrt(2)),
                 direction_range=(-math.pi/4, math.pi/4)):
        self.target = target_agent
        self.map = env_map
        self.lastdata = []
        
        self.distance_range = distance_range   #追加
        self.direction_range = direction_range  #追加

    def visible(self, polarpos):  # 相手が計測できる条件
        if polarpos is None:
            return False
        
        return self.distance_range[0] <= polarpos[0] <= self.distance_range[1] \
                and self.direction_range[0] <= polarpos[1] <= self.direction_range[1]
        
    def rectangle_range(self,cam_pose,rectangle_X,rectangle_Y):
        max_ob_r = 0.0
        min_ob_r = 10.0
        max_ob_theta = -np.pi/2
        min_ob_theta = np.pi/2
        for x,y in zip(rectangle_X,rectangle_Y):
            pos = np.array([x, y,0]).T
            z = self.observation_function(cam_pose, pos)
            if max_ob_r < z[0]:
                max_ob_r = z[0]
            if min_ob_r > z[0]:
                min_ob_r = z[0]
            if max_ob_theta < z[1]:
                max_ob_theta = z[1]
            if min_ob_theta > z[1]:
                min_ob_theta = z[1]
        return np.array([max_ob_r,min_ob_r,max_ob_theta,min_ob_theta]).T 


    def data(self, cam_pose):
        observed = []
        #z = self.observation_function(cam_pose, self.target.shadow)
        z = self.rectangle_range(cam_pose,self.target.X,self.target.Y)
        flag = True
        #if self.visible(z):
        """
            for ob in self.map.obstacles:
                ob_z = self.rectangle_range(cam_pose,ob.X,ob.Y)
                if (ob_z[1] < z[1]) and\
                    (((ob_z[3]> z[3]) and (z[2] > ob_z[3])) or\
                    (( ob_z[2] < z[2]) and (z[3] < ob_z[2]))):
                    flag = False
            if flag == True:
                x = self.target.X[0]
                y = self.target.Y[0]
                pos = np.array([x, y,0]).T
                z1 = self.observation_function(cam_pose, pos)
                observed.append(z1)
        """
        #[max_ob_r,min_ob_r,max_ob_theta,min_ob_theta]
        for ob in self.map.obstacles:
            ob_z = self.rectangle_range(cam_pose,ob.X,ob.Y)
            if (ob_z[1] < z[1]) and\
                (((ob_z[3]> z[3]) and (z[2] > ob_z[3])) or\
                (( ob_z[2] < z[2]) and (z[3] < ob_z[2]))):
                flag = False
        if flag == True:
            x = self.target.X[0]
            y = self.target.Y[0]
            pos = np.array([x, y,0]).T
            z1 = self.observation_function(cam_pose, pos)
            observed.append(z1)
        self.lastdata = observed 
        return observed    
        
        
    @classmethod
    def observation_function(cls, cam_pose, obj_pos):
        diff = obj_pos[0:2] - cam_pose[0:2]
        phi = math.atan2(diff[1], diff[0]) - cam_pose[2]
        while phi >= np.pi: phi -= 2*np.pi
        while phi < -np.pi: phi += 2*np.pi
        return np.array( [np.hypot(*diff), phi ] ).T
        
    def draw(self, ax, elems, cam_pose): 
        for lm in self.lastdata:
            x, y, theta = cam_pose
            distance, direction = lm[0], lm[1]
            lx = x + distance * math.cos(direction + theta)
            ly = y + distance * math.sin(direction + theta)
            elems += ax.plot([x,lx], [y,ly], color="pink")




if __name__ == '__main__':
    world = World(30, 0.1, debug=False) 

    ### 地図を生成して3つランドマークを追加 ###
    m = Map()
    m.append_obstacle(Obstacle(0,0,0.35,0.35))
    m.append_obstacle2(Obstacle(0.53,0,0.2,0.15))
    m.append_obstacle2(Obstacle(0.0,0.53,0.2,0.15))
    m.append_obstacle2(Obstacle(-0.53,0,0.2,0.15))
    m.append_obstacle2(Obstacle(0.0,-0.53,0.2,0.15))
    world.append(m)
    straight = Agent(0.05, 0.0)    
    circling = Agent(0.05, 10.0/180*math.pi)  
    robot2 = IdealRobot(np.array([ -0.5, -0.5, 7*math.pi/6]).T,agent=circling,color="r" )
    learning_agent = IdealRobot(np.array([ 1, 1, 7*math.pi/6]).T,sensor = TargetCamera(robot2,m),agent=straight )             # 引数にcameraを追加、整理
    world.append(learning_agent)
    world.append(robot2)
    

    ### ロボットを作る ###
    

    ### アニメーション実行 ###
    world.draw()
