from ThymioControl import ThymioControl
from math import radians, fabs
import time


class ThymioControlM(ThymioControl):
    ''' Thymio Control Class for the Mobile Robots Lab -- Moves the robot by setting the Pose in simulation'''
    def forward(self,distance):
        '''Robot moves forward a given distance'''
        p1=self.sim.getObjectPose(self.handles[self.names['robot'][0]],self.sim.handle_world)
        pm=self.sim.buildPose([distance,0,0],[0,0,0],0)
        p2=self.sim.multiplyPoses(p1,pm)
        self.sim.setObjectPose(self.handles[self.names['robot'][0]],self.sim.handle_world,p2)               
        #print(f'distance: {math.sqrt((p2[1]-p1[1])*(p2[1]-p1[1])+(p2[0]-p1[0])*(p2[0]-p1[0]))}')
        #self.step()


    def turn(self,angle):
        p1=self.sim.getObjectPose(self.handles[self.names['robot'][0]],self.sim.handle_world)
        pm=self.sim.buildPose([0,0,0],[0,0,radians(angle)],0)
        p2=self.sim.multiplyPoses(p1,pm)
        self.sim.setObjectPose(self.handles[self.names['robot'][0]],self.sim.handle_world,p2)                       
        #self.step()

    def check_valid_state(self): 
        return not self.getContacts(body='/Thymio') and not self.out_of_bounds(body='/Thymio')
    
    def out_of_bounds(self,body='/Thymio'):
        ''' Checks if the robot is out of bounds'''
        p=self.getPose()
        #print(f'Pose: {p}')
        return p[0]<-1 or p[0]>1 or p[1]<-1 or p[1]>1
      

class ThymioControlC(ThymioControl):
    '''Thymio Control Class for the Mobile Robots Lab -- Moves the robot using the wheels'''
    def forward(self,distance):
        '''Robot moves forward a given distance'''
        speed=1.0
        fwd_duration=distance/speed*42.4
        self.setSpeeds(speed,speed)
        start=self.sim.getSimulationTime()
        while self.sim.getSimulationTime()-start<fwd_duration:
            time.sleep(self.timestep/100)
        self.setSpeeds(0,0)
        #self.step()


    def turn(self,angle):
        speed=1.0
        turn_duration=fabs(angle)/fabs(speed)*0.037
        start=self.sim.getSimulationTime()      
        if angle>0:
            self.setSpeeds(-speed,speed)
        else:
            self.setSpeeds(speed,-speed)
        while self.sim.getSimulationTime()-start<turn_duration:
            time.sleep(self.timestep/100)
        self.setSpeeds(0,0)


    def check_valid_state(self): 
        return not self.getContacts(body='/Thymio') and not self.out_of_bounds(body='/Thymio')
    
    def out_of_bounds(self,body='/Thymio'):
        ''' Checks if the robot is out of bounds'''
        p=self.getPose()
        #print(f'Pose: {p}')
        return p[0]<-1 or p[0]>1 or p[1]<-1 or p[1]>1

import gym
import stable_baselines3.common.env_checker
from stable_baselines3 import DQN
from ThymioEnv import ThymioEnv


env=ThymioEnv(robot=ThymioControlM(),goal=[0.75,0.25])

stable_baselines3.common.env_checker.check_env(env,warn=True)

env.robot.sim_speed=int(4096)
env.robot.display=True
env.robot.sim.setBoolParam(env.robot.sim.boolparam_display_enabled,env.robot.display)

#model = DQN("MlpPolicy", env, verbose=1)
#model.learn(total_timesteps=100000, log_interval=1)
#model.save("move_robot_c")
#del model # remove to demonstrate saving and loading
model = DQN.load("move_robot_c")

print("Modelo treinado com sucesso! Vamos lá testar o modelo treinado!")

env=ThymioEnv(robot=ThymioControlC(),goal=[0.75,0.25])
env.robot.sim_speed=int(1)
obs = env.reset()
env.robot.display=True
env.robot.sim.setBoolParam(env.robot.sim.boolparam_display_enabled,env.robot.display)

while True:
    action, _states = model.predict(obs, deterministic=True)
    print(_states)
    
    obs, reward, done, info = env.step(action)    
    print(f'Observation: {obs} Action: {action}, Reward: {reward}, Done: {done}, Info: {info}')
    #env.render()
    if done:
      obs = env.reset()
   