import matplotlib
#matplotlib.use('nbagg')
import matplotlib.animation as anm
import matplotlib.pyplot as plt
import math
import matplotlib.patches as patches
import numpy as np

class World:        ### fig:world_init_add_timespan (1-5�s��)
    def __init__(self, time_span, time_interval, debug=False):
        self.objects = [] 
        self.debug = debug
        self.time_span = time_span                  # �ǉ�
        self.time_interval = time_interval          # �ǉ�
        
    def append(self,obj):             # �I�u�W�F�N�g��o�^���邽�߂̊֐�
        self.objects.append(obj)
    
    def draw(self):            ### fig:world_draw_with_timesapn (1, 10, 21-26, 28-34�s��)
        fig = plt.figure(figsize=(4,4))                # 8x8 inch�̐}������
        ax = fig.add_subplot(111)                      # �T�u�v���b�g������
        ax.set_aspect('equal')                         # �c��������W�̒l�ƈ�v������
        ax.set_xlim(-1.2,1.2)                              # X����-1.2m x 1.2m�͈̔͂ŕ`��
        ax.set_ylim(-1.2,1.2)                              # Y�������l��
        ax.set_xlabel("X",fontsize=10)                 # X���Ƀ��x����\��
        ax.set_ylabel("Y",fontsize=10)                 # ������Y����
        
        elems = []

        if self.debug:        
            for i in range(int(self.time_span/self.time_interval)): self.one_step(i, elems, ax)
        else:
            self.ani = anm.FuncAnimation(fig, self.one_step, fargs=(elems, ax),
                                     frames=int(self.time_span/self.time_interval)+1, interval=int(self.time_interval*1000), repeat=False)
            plt.show()
        
    def one_step(self, i, elems, ax):
        while elems: elems.pop().remove()
        time_str = "t = %.2f[s]" % (self.time_interval*i)    # �����Ƃ��ĕ\�����镶����
        elems.append(ax.text(-1, 1, time_str, fontsize=10))
        for obj in self.objects:
            obj.draw(ax, elems)#��Q���֌W�̕`��
            if hasattr(obj, "one_step"): obj.one_step(self.time_interval)#�G�[�W�F���g�֌W�̕`��                 # �ύX



class IdealRobot:  
    def __init__(self, pose, agent=None, sensor=None, color="black"):    # ������ǉ�
        self.pose = pose  
        self.r = 0.105
        self.color = color 
        self.agent = agent
        self.poses = [pose]
        self.shadow = pose
        self.sensor = sensor    # �ǉ�
        self.xy1 = (self.pose[0]+self.r*math.cos(self.pose[2]),self.pose[1]+self.r*math.sin(self.pose[2]))
        self.xy2 = (self.pose[0]+self.r*math.cos(self.pose[2]+0.5*math.pi),self.pose[1]+self.r*math.sin(self.pose[2]+0.5*math.pi))
        self.xy3 = (self.pose[0]+self.r*np.sqrt(2)*math.cos(self.pose[2]+0.25*math.pi),self.pose[1]+self.r*np.sqrt(2)*math.sin(self.pose[2]+0.25*math.pi))
        self.X = (self.pose[0],self.xy1[0],self.xy2[0],self.xy3[0])
        self.Y = (self.pose[1],self.xy1[1],self.xy2[1],self.xy3[1])
    def draw(self, ax, elems):
        x, y, theta = self.pose
        x_ob, y_ob, theta_ob = self.shadow# �p���̕ϐ��𕪉�����3�̕ϐ���
        xn = x + self.r * math.cos(theta)         #  ���{�b�g�̕@���x���W 
        yn = y + self.r * math.sin(theta)         #  ���{�b�g�̕@���y���W 
        #xn = x         #  ���{�b�g�̕@���x���W 
        #yn = y         #  ���{�b�g�̕@���y���W
        elems += ax.plot([x,xn], [y,yn], color=self.color) # ���{�b�g�̌��������������̕`��
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
        obs =self.sensor.data(self.pose) if self.sensor else None #�ǉ�
        nu, omega = self.agent.decision(obs) #�����ǉ�
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
        nu, omega = self.agent.decision(obs) #�����ǉ�
        self.pose = self.state_transition(nu, omega, time_interval, self.pose)


class Obstacle:#center��Q���p�̃N���X
    def __init__(self,x,y,width,height):
        self.x=x#��Q���̒[�_�̍��W
        self.y=y#��Q���̒[�_�̍��W
        self.width = width#��Q���̕�
        self.height = height#��Q���̍���
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


class Obstacle2(Obstacle):#����4�̏�Q���p�̃N���X
    def draw(self,ax,elems):
        x1 = self.x -0.3*(self.width**2 + self.height**2)**0.5 
        y1 = self.y -0.4*(self.width**2 + self.height**2)**0.5
        r = patches.Rectangle(xy = (x1,y1),width = self.width,\
                              height = self.height,\
                              color="blue",angle = 45,alpha=0.2)
        elems.append(ax.add_patch(r))


        
class Map:
    def __init__(self):       # ��̃����h�}�[�N�̃��X�g������
        self.landmarks = []
        self.obstacles = []
        self.obstacles2 = []
        self.shadows = []
        self.targets=[]
        
    def append_landmark(self, landmark):       # �����h�}�[�N��ǉ�
        landmark.id = len(self.landmarks)           # �ǉ����郉���h�}�[�N��ID��^����
        self.landmarks.append(landmark)
    
    def append_target(self, target):       # �����h�}�[�N��ǉ�
        target.id = len(self.targets)           # �ǉ����郉���h�}�[�N��ID��^����
        self.targets.append(target)
        
    def append_obstacle(self, obstacle):       # obstacle��ǉ�
        obstacle.id = len(self.obstacles)           # �ǉ����郉���h�}�[�N��ID��^����
        self.obstacles.append(obstacle)
        
    def append_obstacle2(self, obstacle):       # obstacle2��ǉ�
        obstacle.id = len(self.obstacles)           # �ǉ����郉���h�}�[�N��ID��^����
        self.obstacles.append(obstacle)

    def draw(self, ax, elems):                 # �`��iLandmark��draw�����ɌĂяo���j
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
        
        self.distance_range = distance_range   #�ǉ�
        self.direction_range = direction_range  #�ǉ�
        
    def visible(self, polarpos):  # �����h�}�[�N���v���ł������
        if polarpos is None:
            return False
        
        return self.distance_range[0] <= polarpos[0] <= self.distance_range[1] \
                and self.direction_range[0] <= polarpos[1] <= self.direction_range[1]
    
        
    def data(self, cam_pose):
        observed = []
        for lm in self.map.landmarks:
            z = self.observation_function(cam_pose, lm.pos)
            if self.visible(z):               # ������ǉ�
                observed.append((z, lm.id))   # �C���f���g
            
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
        
        self.distance_range = distance_range   #�ǉ�
        self.direction_range = direction_range  #�ǉ�

    def visible(self, polarpos):  # ���肪�v���ł������
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

    ### �n�}�𐶐�����3�����h�}�[�N��ǉ� ###
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
    learning_agent = IdealRobot(np.array([ 1, 1, 7*math.pi/6]).T,sensor = TargetCamera(robot2,m),agent=straight )             # ������camera��ǉ��A����
    world.append(learning_agent)
    world.append(robot2)
    

    ### ���{�b�g����� ###
    

    ### �A�j���[�V�������s ###
    world.draw()
