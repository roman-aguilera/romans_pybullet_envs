


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
p.getLinkState(bodyUniqueId=octopusBodyUniqueId, linkIndex=0, computeLinkVelocity=1, computeForwardKinematics=1, physicsClientId=physicsClientId)
p.getLinkStates() 

#LOCK A JOINT

#get joint's child link name (for sanity check to ensure that we are indexing correct values from the list)
jointChildLinkName=jointParentFramePosition=p.getJointInfo(self.octopusBodyUniqueId, jointIndex=0, physicsClientId=self.physicsClientId)[12]

#get joint frame position, relative to parent link frame
jointFramePositionWRTParentLinkFrame=p.getJointInfo(octopusBodyUniqueId, jointIndex=0, physicsClientId=physicsClientId)[14]

#get joint frame orientation, relative to parent CoM link frame
jointFrameOrientationWRTParentLinkFrame=p.getJointInfo(octopusBodyUniqueId, jointIndex=0, physicsClientId=physicsClientId)[15]

#get position of the joint frame, relative to a given child CoM Coordidinate Frame, (other posibility: get world origin (0,0,0) if no child specified for this joint)  
jointsChildLinkFramePositionWRTWorld = p.getJointState() 
jointFramePosition_WRT_WorldFramePosition =  
jointFramePosition_WRT_jointsChildLinkFramePosition = 

#get position of the joint frame, relative to a given child center of mass coordinate frame, (or get the world origin if no child is specified for this joint)

jointsChildLinkFrameOrientationWRTWorld=p.getLinkState()
jointFramePositionWRTWorld=
jointFramePosition

#add constraint (lockjoint)
constraintId = p.createConstraint(
	parentBodyUniqueId=octopusBodyUniqueId , 
	parentLinkIndex=6, 
	childBodyUniqueId=octopusBodyUniqueId , 
	childLinkIndex=6+1 , 
	jointType=p.JOINT_FIXED , 
	jointAxis=[1,0,0], 
	parentFramePosition=[0,0,1], 
	childFramePosition=[0,0,-2], 
	parentFrameOrientation= [0,0,0,1] , #orientation of joint frame, relative to parent center of mass frame 
	childFrameOrientation=[0,0,0,1], #orientation of joint frame, relative to child center of mass frame
	physicsClientId=physicsClientId )

#remove constraint
p.removeConstraint(constraintId)







