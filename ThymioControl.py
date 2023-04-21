from abc import ABC, abstractmethod
import coppeliasim_zmqremoteapi_client as coppelia
import math
import time

class ThymioControl(ABC):
    def __init__(self,real=False):
        self.client = coppelia.RemoteAPIClient()
        self.sim = self.client.getObject('sim')
        self.client.setStepping(True)
        self.timestep = self.sim.getSimulationTimeStep()
        self.sim_speed=1
        self.display=True
        self.handles={}
        self.names={'robot': ['/Thymio'], \
                'prox': ['/Thymio/ProximityLeft','/Thymio/ProximityCenterLeft','/Thymio/ProximityCenter','/Thymio/ProximityCenterRight','/Thymio/ProximityRight','/Thymio/ProximityRearLeft','/Thymio/ProximityRearRight'], \
                'motors': ['/Thymio/RightMotor','/Thymio/LeftMotor'], \
                'acc': ['/Thymio/Accelerometer/forceSensor'],\
                'coll_ignore': ['/Thymio','/Floor']}
        
        self._getObjects(self.names['robot'],self.handles)
        self._getObjects(self.names['prox'],self.handles)
        self._getObjects(self.names['motors'],self.handles)
        self._getObjects(self.names['acc'],self.handles)
        self._getObjects(self.names['coll_ignore'],self.handles)

    
    def _getObjects(self,list,handles={}):        
        for obj in list:
            handles[obj]=self.sim.getObject(obj)
        return handles
    
    def getContacts(self,body='/Thymio',):
        ''' Returns True if the robot is in collision with any of the bodies in the scene'''
        in_collision=False
        try:    
            collidingObjects,_,_,_=self.sim.getContactInfo(self.sim.handle_all,self.handles[body],0)        
            coll_ignore=[self.handles[x] for x in self.names['coll_ignore']]
            in_collision = not all(x in coll_ignore for x in collidingObjects)        
        except Exception as e:
            print("Error in collision check: ",e)
            return False
        return in_collision
    
    def getProximity(self):
        ''' Returns the proximity sensor readings as a list of 7 values'''
        prox = []
        for h in self.names['prox']:
            prox.append(self.sim.handleProximitySensor(self.handles[h])[1])
        return prox
    
    def setSpeeds(self,left,right):
        ''' Sets the left and right wheel speeds'''
        self.sim.setJointTargetVelocity(self.handles['/Thymio/LeftMotor'],left)
        self.sim.setJointTargetVelocity(self.handles['/Thymio/RightMotor'],right)

    def getAcc(self):
        ''' Returns the accelerometer readings as a list of 3 values'''
        return self.sim.readForceSensor(self.handles[self.names['acc'][0]])
    
    def getPose(self):
        ''' Returns the robot pose as a list of 3 values: x, y, theta'''
        self.step()
        matrix=self.sim.getObjectMatrix(self.handles[self.names['robot'][0]],self.sim.handle_world)
        o=self.sim.getEulerAnglesFromMatrix(matrix)
        p=[matrix[3],matrix[7],matrix[11]]

        o[2]= ( o[2] + math.pi) % (2 * math.pi ) - math.pi

        return [p[0],p[1],o[2]]
    
    def step(self):
        ''' Performs a simulation step'''
        self.client.step()
        return self.sim.getSimulationTime()

    def reset(self):
        ''' Resets the simulation'''
        self.setSpeeds(0,0)
        self.sim.stopSimulation()        
        time.sleep(0.1)
        self.sim.startSimulation()
        self.sim.setInt32Param(self.sim.intparam_speedmodifier,int(self.sim_speed))
        self.sim.setBoolParam(self.sim.boolparam_display_enabled,self.display)
        
    
    @abstractmethod
    def forward(self,distance):
        '''Robot moves forward a given distance'''
        pass

    @abstractmethod
    def turn(self,angle):
        '''Robot turns a given angle'''
        pass

    @abstractmethod
    def check_valid_state(self):        
        ''' Checks if the robot is in a valid state'''
        pass


