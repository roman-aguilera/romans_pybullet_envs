####DEBUGGING CODE (START)
#call this envrionment from the python interpreter by using these commands
#import gym; import pybullet_envs; 
#env_instance = gym.make("OctopusArmLockedJoints2DBulletEnv-v0", render=True, number_of_free_joints=3)
#env_instance.reset(); 
#optional #env_instance.env._p.setRealTimeSimulation(1) #optional 
#joint_test=7; action_sample_test=env_instance.env.action_space.sample()*1; action_sample_test[0:joint_test]=0; action_sample_test[joint_test]=10; action_sample_test[joint_test+1:8]=0; env_instance.step(action_sample_test); #input torque at single joint for one timestep
#### DEBUGGING CODE (END)

###TODO: so far i have the last 3 joints unlocked, and they implement the torques
'''
###TESTER
import gym; import pybullet_envs; env_instance = gym.make('OctopusArmLockedJoints1stUnlockedJoints2DBulletEnv-v0', render=True, number_of_links_urdf=8, number_of_joints_urdf=8, number_of_free_joints=3);env_instance.reset();env_instance.env._p.setRealTimeSimulation(1);



import gym; import pybullet_envs; env_instance = gym.make('OctopusArmLockedJoints1stUnlockedJoints2DBulletEnv-v0', render=True, number_of_links_urdf=8, number_of_joints_urdf=8, number_of_free_joints=3);env_instance.reset(); env_instance.step(env_instance.action_space.sample()); env_instance.env._p.setRealTimeSimulation(1);
'''

import gym, gym.spaces, gym.utils, gym.utils.seeding
import numpy as np
import pybullet
import os

from pybullet_utils import bullet_client #for physics client (lets you run multiple clients in parallel)

from pkg_resources import parse_version 

import einsteinpy.coordinates #used to convert between cartesian to spherical coordinates

import random #used for resetting goal target

import pybullet_data


try:
  if os.environ["PYBULLET_EGL"]:
    import pkgutil
except:
  pass

#import gym; import pybullet_envs; 
#env_instance = gym.make('OctopusArmLockedJointsNUnlockedJoints2DBulletEnv-v0', renders=True, number_of_links_urdf=8, number_of_joints_urdf=8, number_of_free_joints=3)
#env_instance.reset(); #optional #env_instance.env._p.setRealTimeSimulation(1) 
#joint_test=7; action_sample_test=env_instance.env.action_space.sample()*1; action_sample_test[0:joint_test]=0; action_sample_test[joint_test]=10; action_sample_test[joint_test+1:8]=0; env_instance.step(action_sample_test);
class OctopusEnv(gym.Env):
  """
	Base class for Bullet physics simulation loading MJCF (MuJoCo .xml) environments in a Scene.
	These environments create single-player scenes and behave like normal Gym environments, if
	you don't use multiplayer.
	"""

  metadata = {'render.modes': ['human', 'rgb_array'], 'video.frames_per_second': 60}

  def __init__(self, render=True, number_of_links_urdf=8, number_of_joints_urdf=8, number_of_free_joints=3): #def __init__(self, render=False):
    self.scene = None
    self.physicsClientId = -1 #indicates that we have not started a pybullet simulation, this number is changed when we do start a simulation
    self.ownsPhysicsClient = 0 #indicates that we have not started a pybullet simulation, this is changed to True when we do start a simulation (in the reset function)
    self.camera = Camera()
    #self.joints = Joint()
    #self.joints_and_links = robotJointsandLinks(number_of_joints=None, number_of_links=None, bodyUniqueId=self.o)
    self.isRender = render
    #self.robot = robot
    self.seed()
    self._cam_dist = 3
    self._cam_yaw = 0
    self._cam_pitch = -30
    self._render_width = 320
    self._render_height = 240
    
    self.number_of_links_urdf = number_of_links_urdf
    self.number_of_joints_urdf = number_of_joints_urdf
    self.number_of_torques_urdf  = number_of_links_urdf
    self.number_of_free_joints = number_of_free_joints
    
    #### LOCK/UNLOCK MEACHANISM (begin)
    #creates a list of constraints to use and update (needed to lock and unlock joints)
    print("creating joint lock/unlock mechanism... ")
    self.constraintUniqueIdsList = [None]*self.number_of_joints_urdf
      
    #creates list of masks for chossing which joints to unlock #modified from https://stackoverflow.com/a/1851138/14024439
    import itertools 
    def kbits(n, k):
      result = []
      for bits in itertools.combinations(range(n), k):
        s = ['0'] * n
        for bit in bits:
          s[bit] = '1'
        result.append(''.join(s))
      return result
    #list of masks (locked vs unlocked) in string format
    self.masks_unlock_as_list_of_strings=kbits(self.number_of_joints_urdf, self.number_of_free_joints) #each element is a binary string 
    #convert masks (locked vs unlocked) from string format to np array format
    self.masks_unlock_as_list_of_nparrays=[None]*len(self.masks_unlock_as_list_of_strings) #empty list where size = (total number of permutations)
    
    for i in range( len(self.masks_unlock_as_list_of_nparrays) ):   
      self.masks_unlock_as_list_of_nparrays[i] = np.array([int(x) for x in self.masks_unlock_as_list_of_strings[i]]) #now we have masks as a 2D nparray indexing=(masks_unlock_as_list_of_nparrays[mask_number][joint_number])
     
    self.number_of_combinations_unlock=len(self.masks_unlock_as_list_of_nparrays)
    #### LOCK/UNLOCK MEACHANISM (end)
    
    
      
    #### ACTUATE/UNACTUATE MECHANISHM (begin)
    print("creating joint actuate/unactuate mechnism...")
    #creates list of masks for choosing whether unlocked joint is actuated or underactuated (torque 0) 
    #modified from https://code.activestate.com/recipes/425303-generating-all-strings-of-some-length-of-a-given-a/
    def allstrings2(alphabet, length):
      """Find the list of all strings of 'alphabet' of length 'length'"""
      c = [ ]
      for i in range(length):
        c = [[x]+y for x in alphabet for y in c or [[]]]
      return c
        
    #list of masks (actuated vs unactuated) (0 = unactuate, 1 = actuate)
    masks_unactuated_as_np_array = []
    masks_unactuated_as_list_of_list_of_ints = []
    for i in range(self.number_of_joints_urdf+1): #between 0 unlocked joints to all joints unlocked
      masks_unactuated_as_list_of_list_of_ints.append(allstrings2([0,1], i) ) 
    masks_unactuated_as_np_array = np.array(masks_unactuated_as_list_of_list_of_ints)
    #for masks_unactuated_as_np_array: first index corresponds to number of unlockedjoints,  second index corresponds to decision of unactuation for whole arm, third index corresponds to joint index
      #masks_unactuated_as_list_of_np_array = np.array(allstrings2([0,1], self.number_of_unlocked_joints)) # first index corresponds to decision of unactuation for whole arm, second index corresponds to joint index 
       
    self.number_of_combinations_unactuate=len(masks_unactuated_as_np_array[self.number_of_free_joints]) #length of masks for (self.number_of_free_joints) number of uncatuated joints
        
    #### ACTUATE/UNACTUATE MECHANISM (end)
    
    
    self.target_x = 0  # this is 0 in 2D case
    self.target_y = 5  # ee y coordinate limits for 8 link arm are
    self.target_z = 5  # ee z coordniate limits for 8 link arm are
    
        
    [self.x_lower_limit, self.x_upper_limit] = [0.0, 0.0]  # reachable x coordinates (double check for each arm)
    [self.y_lower_limit, self.y_upper_limit] = [-15,15]  # reachable y coordinates
    [self.z_lower_limit, self.z_upper_limit] = [0,15]  # reachable z coordinates
    
    self.radius_to_goal_epsilon = 5 #10 #1e0 #9e-1 #1e-1 #1e-3
    self.joint_states=np.zeros(shape=(2*self.number_of_links_urdf,))
    self.time_stamp = 0 #time elapsed since last reset
    self.time_step = 1./240 #this is the default timestep in pybullet, to set other tipestep use the following code and place it in the reset() method:  self._p.setTimeStep(timeStep = self.time_step, physicsclientId=self.physicsClientId)
     
     
     
    ##### CREATE ACTION AND OBSERVATION SPACES (begin)
    
    #observation space (formats the types of inputs to NN)
    obs_dim = 22 # num_links (8) *2 + extra_states (self.radius_to_goal, self.theta_to_goal, self.phi_to_goal) + extra states ((self.end_effector_vx, self.end_effector_vy, self.end_effector_vz))
    high = np.inf * np.ones([obs_dim]); 
    self.observation_space = gym.spaces.Box(-high, high)  #self.observation_space = robot.observation_space
      
      
    #action space(formats the types of outputs of NN)
    action_dim = self.number_of_torques_urdf 
    high = np.ones([action_dim]); 
    self.action_space = gym.spaces.Dict({
      'torques' : gym.spaces.Box(-high, high) ,
      'unlocked_joints_combination' : gym.spaces.Discrete(self.number_of_free_joints), 
      'unactuated_joints_combination' : gym.spaces.Discrete(self.number_of_combinations_unactuate) }) #self.number_of_free_joints??? insteaf 
      
    #self.action_space = robot.action_space
    #self.reset()
    #### CREATE ACTION AND OBSERVATION SPACES (end)
    
  #def configure(self, args):
  #  self.robot.args = args

  def seed(self, seed=None):
    self.np_random, seed = gym.utils.seeding.np_random(seed)
  #  self.robot.np_random = self.np_random  # use the same np_randomizer for robot as for env
    return [seed]

#### RESET
  def reset(self):
    if (self.physicsClientId < 0): #if it is the first time we are loading the simulations 
      self.ownsPhysicsClient = True  #this

      if self.isRender:
        self._p = bullet_client.BulletClient(connection_mode=pybullet.GUI) # connect to physics server, and render through Graphical User Interface (GUI)
      else:
        self._p = bullet_client.BulletClient() #connect to physics server, and DO NOT render through graphical user interface
      self.physicsClientId = self._p._client # get the client ID from physics server, this makes self.physicsClientId become a value greater than 0 (this value indicates the server that this environement instance is connected to)
      self._p.resetSimulation() # reset physics server and remove all objects (urdf files, or mjcf, or )
      #self.number_of_links_urdf = number_of_links_urdf
      self.model_urdf = "romans_urdf_files/octopus_files/python_scripts_edit_urdf/octopus_generated_"+str(self.number_of_links_urdf)+"_links.urdf"
      #load URDF into pybullet physics simulator
      self.octopusBodyUniqueId = self._p.loadURDF( fileName=os.path.join(pybullet_data.getDataPath(), self.model_urdf), flags=self._p.URDF_USE_SELF_COLLISION | self._p.URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS)
      #turn off all motors so that joints are not stiff for the rest of the simulation
      self._p.setJointMotorControlArray(bodyUniqueId=self.octopusBodyUniqueId, jointIndices=list(range(8)), controlMode = self._p.POSITION_CONTROL, positionGains=[0.1]*self.number_of_links_urdf, velocityGains=[0.1]*self.number_of_links_urdf, forces=[0]*self.number_of_links_urdf) 
        
        
      #### CHOOSE COMBINATION OF JOINTS (RESET) (begin)
      #this loop unlocks all of the arm's joints
      for i in range(self.number_of_joints_urdf):
        if self.constraintUniqueIdsList[i] is not None:
          self._p.removeConstraint(self.constraintUniqueIdsList[i])
          self.constraintUniqueIdsList[i]=None    
       
      #this loop locks all of the arm's joints
      for i in range(self.number_of_joints_urdf):
         
        #lock single joint
        self.constraintUniqueIdsList[i] = self._p.createConstraint( parentBodyUniqueId=self.octopusBodyUniqueId, parentLinkIndex=i-1, childBodyUniqueId=self.octopusBodyUniqueId, childLinkIndex=i, jointType=self._p.JOINT_FIXED, jointAxis=[1,0,0], parentFramePosition= [0,0,1], childFramePosition=[0,0,-1], parentFrameOrientation=self._p.getJointInfo(bodyUniqueId=self.octopusBodyUniqueId, jointIndex=i, physicsClientId=self.physicsClientId)[15], childFrameOrientation=self._p.getQuaternionFromEuler(eulerAngles=[-self._p.getJointState(bodyUniqueId=self.octopusBodyUniqueId, jointIndex=i, physicsClientId=self.physicsClientId)[0],-0,-0]), physicsClientId=self.physicsClientId )
         
      #unlock first joints
      print("hello",self.masks_unlock_as_list_of_nparrays[3][-1])
      for i in range(self.number_of_joints_urdf-1, self.number_of_joints_urdf):
        if self.constraintUniqueIdsList[self.number_of_joints_urdf-1-i] is not None:
          self._p.removeConstraint( userConstraintUniqueId=self.constraintUniqueIdsList[self.number_of_joints_urdf-1-i] , physicsClientId=self.physicsClientId  )
          self.constraintUniqueIdsList[self.number_of_joints_urdf-1-i]=None 
      #### CHOOSE COMBINATION OF JOINTS (RESET) (end)
      
      
      
      #indicate urdf file for visualizing goal point and load it
      self.goal_point_urdf = "sphere8cube.urdf" 
      self.goalPointUniqueId = self._p.loadURDF( fileName=os.path.join(pybullet_data.getDataPath(),self.goal_point_urdf) , basePosition=[self.target_x, self.target_y, self.target_z], useFixedBase=1 ) #flags=self._p.URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS) 
      
      #self.goalPointUniqueId = self._p.createVisualShape(physicsClientId=self.physicsClientId, shapeType=self._p.GEOM_SPHERE, radius=4, specularColor=[0.5,0.5,0.5]) #secularcolor = [r,g,b]
      #self._p.resetBasePositionAndOrientation(bodyUniqueId=self.goalPointUniqueId, physicsClientId=self.physicsClientId, posObj=[self.target_x, self.target_y, self.target_z], ornObj=[0,0,0,1])

      #function usage example: 'bodyUniqueId = pybullet.loadURDF(fileName="path/to/file.urdf", basePosition=[0.,0.,0.], baseOrientation=[0.,0.,0.,1.], useMaximalCoordinates=0, useFixedBase=0, flags=0, globalScaling=1.0, physicsClientId=0)\nCreate a multibody by loading a URDF file.'
      
      #optionally enable EGL for faster headless rendering
      try:
        if os.environ["PYBULLET_EGL"]:
          con_mode = self._p.getConnectionInfo()['connectionMethod']
          if con_mode==self._p.DIRECT:
            egl = pkgutil.get_loader('eglRenderer')
            if (egl):
              self._p.loadPlugin(egl.get_filename(), "_eglRendererPlugin")
            else:
              self._p.loadPlugin("eglRendererPlugin")
      except:
        pass
      
      # get Physics Client Id
      self.physicsClientId = self._p._client 
      
      # enable gravity
      self._p.setGravity(gravX=0, gravY=0, gravZ=0, physicsClientId=self.physicsClientId)
      self._p.configureDebugVisualizer(pybullet.COV_ENABLE_GUI, 0)
    
      self.joints_and_links = robotJointsandLinks(number_of_joints=self.number_of_joints_urdf, number_of_links=self.number_of_links_urdf, bodyUniqueId=self.octopusBodyUniqueId, physicsClientId=self.physicsClientId) #make dictionaries of joints and links (refer to robotJointsandLinks() class) 
      #end of "if first loading pybullet client"
    
    
     
    #reset goal target to random one 
    self.target_x = 0 # no x comoponent for 2D case 
    self.target_y = random.uniform(self.y_lower_limit, self.y_upper_limit) # choose y coordinates such that arm can reach
    self.target_z = random.uniform(self.z_lower_limit, self.z_upper_limit) # choose z coordinates such that arm can reach
    #correspondingly move visual representation of goal target
    self._p.resetBasePositionAndOrientation( bodyUniqueId=self.goalPointUniqueId, posObj=[self.target_x, self.target_y, self.target_z], ornObj=[0,0,0,1], physicsClientId=self.physicsClientId )
    
    #reset joint positions and velocities
    for i in range(self.number_of_joints_urdf):
      
      #ROLLOUT1: all positions=0, all velocities=0 #(initially stretched and static) 
      self._p.resetJointState(bodyUniqueId = self.octopusBodyUniqueId, physicsClientId=self.physicsClientId , jointIndex=i, targetValue=0, targetVelocity=0 ) #tagetValue is angular (or xyz)  position #targetvelocity is angular (or xyz) veloity
      
     #ROLLOUT2: all positions=pi, all velocities=0 #(initially contracted and static)
     #self._p.resetJointState(bodyUniqueId = self.octopusBodyUniqueId, physicsClientId=self.physicsClientId , jointIndex=i, targetValue=np.pi, targetVelocity=0 )
      
      #ROLLOUT3: all positions=0, all velocities=(-pi*c, pi*c) # (initially stretched and dynamic) 
      #self._p.resetJointState(bodyUniqueId = self.octopusBodyUniqueId, physicsClientId=self.physicsClientId , jointIndex=i, targetValue=0, targetVelocity=random.uniform(-np.pi*0.5, np.pi*0.5) )
      
      #ROLLOUT4: all positions = pi (IDEA: try uniform sampling around it?), all velocities=(-pi*c,*pi*c) (initially contracted and dynamic)
      #self._p.resetJointState(bodyUniqueId = self.octopusBodyUniqueId, physicsClientId=self.physicsClientId , jointIndex=i, targetValue=np.pi, targetVelocity=random.uniform(-np.pi*2, np.pi*2) )
     
      #TRAINING1: random initial positions=0, velocities=0 #essentialltially arm is reset to contracted and dynamic
      #self._p.resetJointState(bodyUniqueId = self.octopusBodyUniqueId, physicsClientId=self.physicsClientId , jointIndex=i, targetValue=random.uniform(-np.pi, np.pi), targetVelocity=random.uniform(-np.pi*0.5*0, np.pi*0.5*0) ) #tagetValue is angular (or xyz)  position #targetvelocity is angular (or xyz) veloity      
 
      #TRAINING2: random initial positions=(-pi, pi), velocities=0
      #self._p.resetJointState(bodyUniqueId = self.octopusBodyUniqueId, physicsClientId=self.physicsClientId , jointIndex=i, targetValue=random.uniform(-np.pi, np.pi), targetVelocity=random.uniform(-np.pi*0.5*0, np.pi*0.5*0) ) #tagetValue is angular (or xyz)  position #targetvelocity is angular (or xyz) veloity

      #LAST LEFT OFF HERE
      #TRAINING3: random initial positions=(-pi, pi), velocities=(-pi*c, pi*c)
      #self._p.resetJointState(bodyUniqueId = self.octopusBodyUniqueId, physicsClientId=self.physicsClientId , jointIndex=i, targetValue=random.uniform(-np.pi, np.pi), targetVelocity=random.uniform(-np.pi/4, np.pi/4) ) #tagetValue is angular (or xyz)  position #targetvelocity is angular (or xyz) veloity
      
      #TRAINING4: random initial positions=0, velocities=(-pi*c, pi*c)
      #self._p.resetJointState(bodyUniqueId = self.octopusBodyUniqueId, physicsClientId=self.physicsClientId , jointIndex=i, targetValue=random.uniform(-np.pi, np.pi), targetVelocity=random.uniform(-np.pi*0.5*0, np.pi*0.5*0) ) #tagetValue is angular (or xyz)  position #targetvelocity is angular (or xyz) veloity
      
      #TRAINING5: random initial positions=(-pi*c, pi*c), velocities=0
      #self._p.resetJointState(bodyUniqueId = self.octopusBodyUniqueId, physicsClientId=self.physicsClientId , jointIndex=i, targetValue=random.uniform(-np.pi/4, np.pi/4), targetVelocity=random.uniform(-np.pi*0.5*0, np.pi*0.5*0) ) #tagetValue is angular (or xyz)  position #targetvelocity is angular (or xyz) veloity
   
      #TRAINING6: random initial positions=(-pi*c, pi*c), velocities=(-pi*c, pi*c)
      #self._p.resetJointState(bodyUniqueId = self.octopusBodyUniqueId, physicsClientId=self.physicsClientId , jointIndex=i, targetValue=random.uniform(-np.pi/4, np.pi/4), targetVelocity=random.uniform(-np.pi*0.5*0, np.pi*0.5*0) ) #tagetValue is angular (or xyz)  position #targetvelocity is angular (or xyz) veloity

      #TRAINING7: all initial positions=0, velocities=(pi*c,pi*c) #essentially arm is is reset to stretched and dynamic

      #TRAINING8: all initial positions=pi, velocities=(pi*c,pi*c) #initiall arm is reset to contracted and dynamic 
      
    self.time_stamp=0
    
    #make first link angle face downward
    #self._p.resetJointState(bodyUniqueId = self.octopusBodyUniqueId, physicsClientId=self.physicsClientId , jointIndex=0, targetValue=random.uniform(np.pi/2*2, np.pi/2*2), targetVelocity=random.uniform(-np.pi*0, np.pi*0) ) #reset base link
    
    #self._p.resetJointState(bodyUniqueId = self.octopusBodyUniqueId, physicsClientId=self.physicsClientId , jointIndex=0, targetValue=random.uniform(-np.pi/2*1, np.pi/2*1), targetVelocity=random.uniform(-np.pi*0, np.pi*0) ) #reset base link
    
    #randomize joint positions and velocities
    #for i in range(self.number_of_joints_urdf):
    #  pass
    #self._p.resetJointState(bodyUniqueId = self.octopusBodyUniqueId, physicsClientId=self.physicsClientId , jointIndex=i, targetValue=random.uniform(-np.pi/2,np.pi/2), targetVelocity=0 ) #tagetValue is angular (or xyz)  position #targetvelocity is angular (or xyz) veloity
    
    #if self.scene is None:
    #  self.scene = self.create_single_player_scene(self._p)
    #if not self.scene.multiplayer and self.ownsPhysicsClient:
    #  self.scene.episode_restart(self._p)

    #self.robot.scene = self.scene

    self.frame = 0
    self.done = 0
    self.reward = 0
    self.states = None
    self.step_observations = self.states
    dump = 0
    
     
    #s = self.robot.reset(self._p)
    #self.potential = self.robot.calc_potential()
    
    self.states = self.get_states()
    self.step_observations = self.states
    return self.step_observations


#### GET STATES    
  def get_states(self):
    #### get tuple of link states
    self.link_states = self._p.getLinkStates(bodyUniqueId=self.octopusBodyUniqueId, linkIndices = list( range(self.number_of_links_urdf) ), computeLinkVelocity=True, computeForwardKinematics=True  ) # [self.number_of_links_urdf-1] )#linkIndices = list(range(self.number_of_links_urdf))) #linkIndices=[0,..., self.number_of_links_urdf-1]
    ### indexing tuple: link_states[index of link, index of state_variable tuple, index of state variable sub-variable]
    #linkWorldPosition = np.zeros(shape=(number_of_links,3))
      
    # get end effector x,y,z coordinates
    #( linkWorldPosition, linkWorldOrientation, localInertialFramePosition, localInertialFrameOrientation, worldLinkFramePosition, worldLinkFrameOrientation, worldLinkLinearVelocity, worldLinkAngularVelocity) = self.link_states[self.number_of_links-1]
    #linkWorldPosition == self.link_states[link_index][0] 
    (self.end_effector_x, self.end_effector_y, self.end_effector_z) = self.link_states[-1][0] #xyz position of end effector
    (self.end_effector_vx, self.end_effector_vy, self.end_effector_vz) = self.link_states[-1][6] #xyz velocity of end effector
      
    #get end effector x,y,z distances to goal 
    [self.x_to_goal, self.y_to_goal, self.z_to_goal]  = [self.target_x-self.end_effector_x, self.target_y-self.end_effector_y, self.target_z-self.end_effector_z]
      
    #get end effector distance and angles to goal
    (self.radius_to_goal, self.theta_to_goal, self.phi_to_goal) = einsteinpy.coordinates.utils.cartesian_to_spherical_novel(self.x_to_goal, self.y_to_goal, self.z_to_goal)
    self.polar_vector_to_goal_states = np.array((self.radius_to_goal, self.theta_to_goal, self.phi_to_goal)) #self.polar_vector_to_goal_states.shape==(3,) 
      
    ####TODO: append angle to goal and distance to goal to state vector
      
    #### observe jointstates (add jointstates to state vector)
    self.all_joint_states = self._p.getJointStates(bodyUniqueId=self.octopusBodyUniqueId, jointIndices = list(range(self.number_of_joints_urdf)))  # [self.number_of_joints_urdf] )#linkIndices = list(range(self.number_of_links_urdf))) #linkIndices=[0,..., slef.number_of_links_urdf-1]
              
    #for i in range(self.number_of_joints_urdf):
    #  (joint_pos, joint_vel, (joint_reaction_forces), applied_joint_motor_torque) = self.all_joint_states[i]
      ######## TODO: joint_pos, joint_vel (append these to joint state vector)
      
    #update_joint_states #self.joint_states.shape==(number_of_links_urdf, )
    #import pdb; pdb.set_trace();
    for i in range(self.number_of_joints_urdf):
      self.joint_states[i] = self.all_joint_states[i][0] #angular position of joint i
      self.joint_states[i+self.number_of_joints_urdf] = self.all_joint_states[i][1] #angular velocity of joint i
      
    #concatenate states
    states_as_list = list(self.joint_states) + list((self.radius_to_goal, self.theta_to_goal, self.phi_to_goal)) + list((self.end_effector_vx, self.end_effector_vy, self.end_effector_vz)) 
    self.states = np.array(states_as_list) 
    self.step_observations = self.states
    
    #get speed of end effector (used to reward low end effector speeds when goal is near)
    self.ee_speed = np.sqrt( sum( np.square([self.end_effector_vx, self.end_effector_vy, self.end_effector_vz]) ) ) # speed = 2-norm of the xyz velocity vector 
    
    return self.step_observations
#### STEP
  def step(self, actions):
    #self.link_states = self._p.getLinkStates(bodyUniqueId=self.octopusBodyUniqueId, linkIndices = list(range(self.number_of_links_urdf))) #linkIndices=[0,..., slef.number_of_links_urdf-1]
    #self.joint_states = self._p.getLinkStates(bodyUniqueId=self.octopusBodyUniqueId, linkIndices = list(range(self.number_of_links_urdf)))
    
    #set torques
    self.torques = actions['torques']*10 #100*actions #1000*actions
    #self.torques[7] = 240 #set last link's torque to zero
    #self.torques[7] =  self.torques[7]/10 #self.torques[7] = 0 
    #self.torques = np.clip(a=self.torques, a_min=np.array([-3000, -450, -400, -400, -400, -300, -300, -241]), a_max=np.array([3000, 450, 400, 400, 400, 300, 300, 241])) #min and max were determined with eye test 
    #note #clips for lower minimun torque to move joint a_min=np.array([240, 242, 242, 242, 242, 242, 240, 240])
    #self.torques[7] = 240.2
    
    #TODO: lock joints
    #for i in range(self.number_of_joints_urdf-1):  
      #this doesnt work #self._p.createConstraint(parentBodyUniqueId=self.octopusBodyUniqueId , parentLinkIndex=i childBodyUniqueId=self.octopusBodyUniqueId , childLinkIndex=i+1 , jointType=JOINT_FIXED , jointAxis=[0,0,1], parentFramePosition= [0,0,1], childFramePosition=[0,0,1], parentFrameOrientation=[0,0,1], childFrameOrientation=[0,0,1], physicsClientId=self.physicsClientId )
    
    #TODO: unlock joints
    
    #### CHOOSE COMBINATION OF UNLOCKED JOINTS (STEP) (begin)
    #this loop unlocks all of the arm's joints
    for i in range(self.number_of_joints_urdf):
      if self.constraintUniqueIdsList[i] is not None:
        self._p.removeConstraint(self.constraintUniqueIdsList[i])
        self.constraintUniqueIdsList[i]=None

    #this loop locks all of the arm's joints
    for i in range(self.number_of_joints_urdf):         
      #lock single joint
      self.constraintUniqueIdsList[i] = self._p.createConstraint( parentBodyUniqueId=self.octopusBodyUniqueId, parentLinkIndex=i-1, childBodyUniqueId=self.octopusBodyUniqueId, childLinkIndex=i, jointType=self._p.JOINT_FIXED, jointAxis=[1,0,0], parentFramePosition= [0,0,1], childFramePosition=[0,0,-1], parentFrameOrientation=self._p.getJointInfo(bodyUniqueId=self.octopusBodyUniqueId, jointIndex=i, physicsClientId=self.physicsClientId)[15], childFrameOrientation=self._p.getQuaternionFromEuler(eulerAngles=[-self._p.getJointState(bodyUniqueId=self.octopusBodyUniqueId, jointIndex=i, physicsClientId=self.physicsClientId)[0],-0,-0]), physicsClientId=self.physicsClientId )
    
    
    #unlock desired joints
    unlock_decision = actions['unlocked_joints_combination'] #is this an int
    
    print(self.masks_unlock_as_list_of_nparrays[1][-1]) 
    self.masks_unlock_as_list_of_nparrays[self.number_of_free_joints][-1] #three unlocked joints #last 3 unlocked
    unlock_decision = actions['unlocked_joints_combination']
     
    for i in range(self.number_of_joints_urdf-1, self.number_of_joints_urdf): #for i in range(3):
      if self.constraintUniqueIdsList[self.number_of_joints_urdf-1-i] is not None:
        self._p.removeConstraint( userConstraintUniqueId=self.constraintUniqueIdsList[self.number_of_joints_urdf-1-i] , physicsClientId=self.physicsClientId  )
        self.constraintUniqueIdsList[self.number_of_joints_urdf-1-i]=None
    #### CHOOSE COMBINATION OF UNLOCKED JOINTS (STEP) (end)
    
    unactuate_decision = actions['unactuated_joints_combination']

    
    self.torques = actions['torques']
    
    #set torques 
    self._p.setJointMotorControlArray(physicsClientId=self.physicsClientId, bodyUniqueId=self.octopusBodyUniqueId, jointIndices= list(range(self.number_of_torques_urdf)) , controlMode=self._p.TORQUE_CONTROL, forces=list( self.torques ) )
    
    #step simulation TODO: step 3 times?
    self._p.stepSimulation(physicsClientId=self.physicsClientId)
    
    
    self.time_stamp += self.time_step
    self.step_observations = self.get_states()
    self.ee_link_index = 7
    self.reward_for_being_close_to_goal = 0 # 1/(abs(0.05*self.radius_to_goal)**2)  #+ 1/(abs(np.clip(a=self.joint_states[self.ee_link_index+self.number_of_joints_urdf], a_min=1e-1, a_max=None) )) # + 1/abs(actions[self.ee_link_index]) # 1/abs(self.joint_states[self.ee_link_index+self.number_of_joints_urdf])) #reward low ee angular velocity #- (1/500)*sum(abs(self.torques)) # - (self.time_stamp)**2 #-electricity_cost #+1/(fractal_dimensionality_number)
    #self.reward += self.step_reward
    #print("radius to goal : ", self.radius_to_goal) #debugging
    

    #rewards
    self.reward_for_reaching_goal = 1 if self.radius_to_goal <= self.radius_to_goal_epsilon else 0 # 1/abs(self.radius_to_goal**4) if self.radius_to_goal <= 2*self.radius_to_goal_epsilon else 0 #reward for reaching goal
    self.reward_for_being_close_to_goal = 0# 1/(abs(0.05*self.radius_to_goal)**2)  #+ 1/(abs(np.clip(a=self.joint_states[self.ee_link_index+self.number_of_joints_urdf], a_min=1e-1, a_max=None) )) # + 1/abs(actions[self.ee_link_index]) # 1/abs(self.joint_states[self.ee_link_index+self.number_of_joints_urdf])) #reward low ee angular velocity #- (1/500)*sum(abs(self.torques)) # - (self.time_stamp)**2 #-electricity_cost #+1/(fractal_dimensionality_number)  
    
    #costs
    self.num_joints_at_limit = 0
    #self.joints_at_limit_array = np.logical_or(self.joints_and_links.joints_above_upper_limit_array, self.joints_and_links.joints_below_lower_limit_array) #MAKE THIS OBJECT IN JOINTS AND LINKS CLASS
    #self.num_joints_at_limit = np.count_nonzero(self.joints_at_limit_array)
    #self.joints_at_limit_cost = self.num_joints_at_limit * self.num_joints_at_limit
    
    
      #add reward that slows down the end effector near goal
      #self.reward_for_low_ee_speed = 1000*1/(abs(self.ee_speed)) 
      #self.step_reward += self.reward_for_low_ee_speed 
      #self.reward += self.reward_for_low_ee_speed
     
    #else: 
      #self.step_reward += self.reward_for_reaching_goal
      #self.reward += self.reward_for_reaching_goal
    self.step_reward = self.reward_for_being_close_to_goal + self.reward_for_reaching_goal  #+ self.joints_at_limit_cost
    self.reward += self.step_reward
    
    #stopping criteria
    self.done = True if self.radius_to_goal <= self.radius_to_goal_epsilon else self.done # update done criteria if ee is in epsilon ball  
    if self.time_stamp >= 8:  #originally >=20
      self.done=True
    if self.radius_to_goal <= self.radius_to_goal_epsilon: #within epsilon ball
      self.done=True
    return self.step_observations, self.step_reward, self.done, {}

#### RENDER
  def render(self, mode='human', close=False):
    if mode == "human":
      self.isRender = True
    if mode != "rgb_array":
      return np.array([])

    base_pos = [0, 0, 0]
    if (hasattr(self, 'robot')):
      if (hasattr(self.robot, 'body_xyz')):
        base_pos = self.robot.body_xyz
    if (self.physicsClientId>=0):
      view_matrix = self._p.computeViewMatrixFromYawPitchRoll(cameraTargetPosition=base_pos,
                                                            distance=self._cam_dist,
                                                            yaw=self._cam_yaw,
                                                            pitch=self._cam_pitch,
                                                            roll=0,
                                                            upAxisIndex=2)
      proj_matrix = self._p.computeProjectionMatrixFOV(fov=60,
                                                     aspect=float(self._render_width) /
                                                     self._render_height,
                                                     nearVal=0.1,
                                                     farVal=100.0)
      (_, _, px, _, _) = self._p.getCameraImage(width=self._render_width,
                                              height=self._render_height,
                                              viewMatrix=view_matrix,
                                              projectionMatrix=proj_matrix,
                                              renderer=pybullet.ER_BULLET_HARDWARE_OPENGL)
      try:
        # Keep the previous orientation of the camera set by the user.
        con_mode = self._p.getConnectionInfo()['connectionMethod']
        if con_mode==self._p.SHARED_MEMORY or con_mode == self._p.GUI:
          [yaw, pitch, dist] = self._p.getDebugVisualizerCamera()[8:11]
          self._p.resetDebugVisualizerCamera(dist, yaw, pitch, base_pos)
      except:
        pass

    else:
      px = np.array([[[255,255,255,255]]*self._render_width]*self._render_height, dtype=np.uint8)
    rgb_array = np.array(px, dtype=np.uint8)
    rgb_array = np.reshape(np.array(px), (self._render_height, self._render_width, -1))
    rgb_array = rgb_array[:, :, :3]
    return rgb_array

  def close(self):
    if (self.ownsPhysicsClient):
      if (self.physicsClientId >= 0):
        self._p.disconnect()
    self.physicsClientId = -1

  def HUD(self, state, a, done):
    pass

  # def step(self, *args, **kwargs):
  # 	if self.isRender:
  # 		base_pos=[0,0,0]
  # 		if (hasattr(self,'robot')):
  # 			if (hasattr(self.robot,'body_xyz')):
  # 				base_pos = self.robot.body_xyz
  # 				# Keep the previous orientation of the camera set by the user.
  # 				#[yaw, pitch, dist] = self._p.getDebugVisualizerCamera()[8:11]
  # 				self._p.resetDebugVisualizerCamera(3,0,0, base_pos)
  #
  #
  # 	return self.step(*args, **kwargs)
  if parse_version(gym.__version__) < parse_version('0.9.6'):
    _render = render
    _reset = reset
    _seed = seed


class Camera:

  def __init__(self):
    pass

  def move_and_look_at(self, i, j, k, x, y, z):
    lookat = [x, y, z]
    distance = 10
    yaw = 10
    self._p.resetDebugVisualizerCamera(distance, yaw, -20, lookat)


class robotJointsandLinks:
  def __init__(self, number_of_joints=None, number_of_links=None, bodyUniqueId=None, physicsClientId=None):
    self.joint_dictionary = dict() #create empty dictionary
    self.link_dictionary = dict() 
    
    for index in range(number_of_joints):
      self.joint_dictionary["joint_" + str(index)] = Joint( bodyUniqueId=bodyUniqueId, physicsClientId=physicsClientId, joint_index=index ) # add joint to a dictionary
    
    for index in range(number_of_links):
      self.link_dictionary["link_" + str(index)] = Link( bodyUniqueId=bodyUniqueId , link_index=index ) #add link to a dictionary
    
    #np.logical_or(self.joints_and_links.joints_above_upper_limit_array, self.joints_and_links.joints_below_lower_limit_array) #MAKE THIS OBJECT IN JOINTS AND LINKS CLASS
  
class Joint:  
  def __init__(self, bodyUniqueId=None, physicsClientId=None, joint_index=None, jointLowerLimit=-np.pi/2, jointUpperLimit=+np.pi/2 ):
    self.bodyUniqueId = bodyUniqueId
    self.jointIndex = joint_index 
    self.JointLimitLow = jointLowerLimit
    self.JointLimitUpper = jointUpperLimit 
    
  def check_if_at_limit(self):
    pass #self._p.get
    
  def joint_limit_reached(self):
    pass
       
  def update_upper_joint_limit(self, new_joint_lower_limit):
    pass
     
  def update_lower_joint_limit(self, new_joint_upper_limit):
    pass 
       
  def get_joint_info(self):
    pass
  
  def get_maximal_joint_state(self, index):
    pass
    
class Link:
  def __init__(self, bodyUniqueId=None, physicsCLientId=None, link_index=None ):
    self.bodyUniqueId = bodyUniqueId
    self.linkIndex = link_index
    
  def get_link_length(self):
    #return selflink_mass
    pass
    
  def get_link_mass(self):
    pass
   
  def getLinkPosition(self):
    pass
    
  def getLinkAngularVelocity(self):
    pass



#TODO: line 238 cost comment, add joints_at_limit_reward, mess with urdf joint parameters in order to change torque needed to move arm link
