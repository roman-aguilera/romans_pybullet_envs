


#this python scrpt is inteded to help with streamlining the creation of the octopus gym environments development
#it (1) loads the octopus urdf and (2) allows for for testing of different pybullet APIs
#refer to the pybullet quickstart guide found in "Pybullet Quickstart Guide.pdf" withinin this repository or the online one found in https://pybullet.org/wordpress/ . The online doc is usually very slow due to heavy online traffic 

import pybullet as p

physicsClientId = p.connect(p.GUI)# connect to a physics simulation server
p.setRealTimesimulation(1)
import os
import pybullet_data

number_of_joints_urdf=8
number_of_links_urdf=8

#load URDF
octopusBodyUniqueId = p.loadURDF( fileName=os.path.join(pybullet_data.getDataPath(), "romans_urdf_files/octopus_files/python_scripts_edit_urdf/octopus_generated_8_links.urdf") , flags=p.URDF_USE_SELF_COLLISION | p.URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS)

#turn off all motorrs so that joints are not stiff for the rest of the simulation
p.setJointMotorControlArray(bodyUniqueId=octopusBodyUniqueId, jointIndices=list(range(8)), controlMode=p.POSITION_CONTROL, positionGains=[0.1]*number_of_links_urdf, velocityGains=[0.1]*number_of_links_urdf, forces=[0]*number_of_links_urdf)

#query the joint info
p.getJointInfo(bodyUniqueId=octopusBodyUniqueId, jointIndex=0, physicsClientId=physicsClientId) #query all joint info
p.getJointInfo(bodyUniqueId=octopusBodyUniqueId,jointIndex=0)[12] # joint's child link name (helps with debugging)
#p.getJointInfos() doesnt exist !

#query the joint state(s)
p.getJointState(bodyUniqueId=octopusBodyUniqueId, jointIndex=0, physicsClientId=physicsClientId)  
p.getJointStates(bodyUniqueId=octopusBodyUniqueId, jointIndices=list(range(number_of_links_urdf)), physicsClientId=physicsClientId)

#query the link info
#p.getLinkInfo() doesnt exist !
#p.getLinkInfos() doesnt exist !

#query the link state
(linkWorldPosition, linkWorldOrientation, localInertialFramePosition, localInertialFrameOrientation, worldLinkFramePosition, worldLinkFrameOrientation, worldLinkLinearVelocity, worldLinkAngularVelocity) = p.getLinkState(bodyUniqueId=octopusBodyUniqueId, linkIndex=0, computeLinkVelocity=1, computeForwardKinematics=1, physicsClientId=physicsClientId) #list items 0,1 are same as list items 4,5 for octopus 8-link case #CoM frame position = inertial Frame position and orientation  # for 8-link octopusarms list items [2,3] are localInertialFramePosition=[0,0,0] and localInertialFrameOrientation=[0,0,0,1]   

 = p.getLinkStates( ) #list items 0,1 are same as list items 4,5 for octopus 8-link case #CoM frame position = inertial Frame position and orientation, localInertianFramePosition=[0,0,0], localInertialFrameOrientation=[0,0,0,0]   

#LOCK A JOINT

#get joint's child link name (for sanity check to ensure that we are indexing correct values from the list)
jointChildLinkName=jointParentFramePosition=p.getJointInfo(self.octopusBodyUniqueId, jointIndex=0, physicsClientId=self.physicsClientId)[12]

#get joint frame position, relative to parent link frame
jointFramePosition_WRT_ParentLinkFrame=p.getJointInfo(octopusBodyUniqueId, jointIndex=0, physicsClientId=physicsClientId)[14]

#get joint frame orientation, relative to parent CoM link frame
jointFrameOrientation_WRT_ParentLinkFrame=p.getJointInfo(octopusBodyUniqueId, jointIndex=0, physicsClientId=physicsClientId)[15]

#get position of the joint frame, relative to a given child CoM Coordidinate Frame, (other posibility: get world origin (0,0,0) if no child specified for this joint)  
jointsChildLinkFramePosition_WRT_WorldFramePosition = p.getJointState() 
jointsFramePosition_WRT_WorldFramePosition =  
jointsFramePosition_WRT_jointsChildLinkFramePosition = 

#get position of the joint frame, relative to a given child center of mass coordinate frame, (or get the world origin if no child is specified for this joint)
jointsChildLinkFrameOrientation_WRT_WorldOrientation=p.getLinkState()
jointFrameOrientation_WRT_WorldFrameOrientation=
jointFrameOrientation_WRT_jointsChildFrameOrientation=

#add constraint (lockjoint)
constraintId = p.createConstraint(
	parentBodyUniqueId=octopusBodyUniqueId , 
	parentLinkIndex=6, 
	childBodyUniqueId=octopusBodyUniqueId , 
	childLinkIndex=6+1 , 
	jointType=p.JOINT_FIXED , 
	jointAxis=[1,0,0], 
	parentFramePosition=[0,0,1], 
	childFramePosition=[0,0,-1], 
	parentFrameOrientation= [0,0,0,1] , #orientation of joint frame, relative to parent center of mass frame #parentFrameOrientation=p.getJointInfo(bodyUniqueId=octopusBodyUniqueId, jointIndex=7, physicsClientId=physicsClientId)[15] 
	childFrameOrientation=[0,0,0,1], #orientation of joint frame, relative to child center of mass frame # childFrameOrientation=
	physicsClientId=physicsClientId )

constraintId = p.createConstraint(
	parentBodyUniqueId=octopusBodyUniqueId, 
	parentLinkIndex=6, 
	childBodyUniqueId=octopusBodyUniqueId, 
	childLinkIndex=6+1, 
	jointType=p.JOINT_FIXED, 
	jointAxis=[1,0,0], 
	parentFramePosition=p.getJointInfo(bodyUniqueId=octopusBodyUniqueId, jointIndex=7, physicsClientId=physicsClientId)[14],  #jointFramePosition_WRT_ParentCoMFramePosition 
	childFramePosition=[0,0,-1], #jointFramePosition_WRT_ChildCoMFramePosition 
	parentFrameOrientation=p.getJointInfo(bodyUniqueId=octopusBodyUniqueId, jointIndex=7, physicsClientId=physicsClientId)[15], #jointFrameOrientation_WRT_ParentCoMOrientation 
	childFrameOrientation=p.getLinkState(bodyUniqueId=octopusBodyUniqueId, linkIndex=7, computeLinkVelocity=1, computeForwardKinematics=1, physicsClientId=physicsClientId)[4], physicsClientId=physicsClientId )

constraintId = p.createConstraint(
        parentBodyUniqueId=octopusBodyUniqueId,
        parentLinkIndex=6,
        childBodyUniqueId=octopusBodyUniqueId,
        childLinkIndex=6+1,
        jointType=p.JOINT_FIXED,
        jointAxis=[1,0,0],
        parentFramePosition= [0,0,1], #THIS WORKS
        childFramePosition=[0,0,-1], #THIS WORKS
        parentFrameOrientation=p.getJointInfo(bodyUniqueId=octopusBodyUniqueId, jointIndex=7, physicsClientId=physicsClientId)[15], #jointFrameOrientation_WRT_ParentCoMOrientation #THIS WORKS
        childFrameOrientation= p.getQuaternionFromEuler(eulerAngles=[np.pi,0,0]), #p.getLinkState(bodyUniqueId=octopusBodyUniqueId, linkIndex=7, computeLinkVelocity=1, computeForwardKinematics=1, physicsClientId=physicsClientId)[4], # 
        physicsClientId=physicsClientId )

# went from euler angles [angle,0,0] to quaternion [a,b,c,d] to set constraint
constraintId = p.createConstraint( parentBodyUniqueId=octopusBodyUniqueId, parentLinkIndex=6, childBodyUniqueId=octopusBodyUniqueId, childLinkIndex=6+1, jointType=p.JOINT_FIXED, jointAxis=[1,0,0], parentFramePosition= [0,0,1], childFramePosition=[0,0,-1], parentFrameOrientation=p.getJointInfo(bodyUniqueId=octopusBodyUniqueId, jointIndex=7, physicsClientId=physicsClientId)[15], childFrameOrientation= p.getQuaternionFromEuler(eulerAngles=[np.pi,0,0]), physicsClientId=physicsClientId )

constraintId = p.createConstraint( parentBodyUniqueId=octopusBodyUniqueId, parentLinkIndex=6, childBodyUniqueId=octopusBodyUniqueId, childLinkIndex=6+1, jointType=p.JOINT_FIXED, jointAxis=[1,0,0], parentFramePosition= [0,0,1], childFramePosition=[0,0,-1], parentFrameOrientation=p.getJointInfo(bodyUniqueId=octopusBodyUniqueId, jointIndex=7, physicsClientId=physicsClientId)[15], childFrameOrientation= p.getQuaternionFromEuler(eulerAngles=-[p.getJointState(bodyUniqueId=octopusBodyUniqueId, jointIndex=7, physicsClientId=physicsClientId)[0],0,0]), physicsClientId=physicsClientId )


while(1):
    print("running...")  

#remove constraint
p.removeConstraint(constraintId)


#position of the joint frame relative to parent center of mass frame.
p.getJointInfo(bodyUniqueId=octopusBodyUniqueId, jointIndex=7, physicsClientId=physicsClientId)[14]


#position of the joint frame relative to a given child center of mass frame (or world origin if no child specified)
   #joint position wrt to child = joint position wrt world - child position wrt to world = (joint position wrt to parent + parent position wrt world) - (child wrt world)
 
p.getJointInfo(bodyUniqueId=octopusBodyUniqueId, jointIndex=7, physicsClientId=physicsClientId)[14]) #joint position wrt to parent

p.getLinkState(bodyUniqueId=octopusBodyUniqueId, linkIndex=7, computeLinkVelocity=1, computeForwardKinematics=1, physicsClientId=physicsClientId)[0?][?] #parent position wrt to world

p.getLinkInfo() #child position wrt world
 
 # joint wrt child = 


#joint frame orientation wrt world frame
p.getJointInfo(bodyUniqueId=octopusBodyUniqueId, jointIndex=7, physicsClientId=physicsClientId)[15])

#orientation
p.getLinkState(bodyUniqueId=octopusBodyUniqueId, linkIndex=7, computeLinkVelocity=1, computeForwardKinematics=1, physicsClientId=physicsClientId)[4] #link frame orientation, offset to local inertial frame







