#!/usr/bin/env python

import pypot.dynamixel 
import time
import itertools
import numpy as np
import json
from signal import pause
from bluedot import BlueDot
import rospy
from std_msgs.msg import String


#--------------------------------------------------------------OFFSETS------------------------------------------------------------------------------
darwin = {1: 90, 2: -90, 3: 67.5, 4: -67.5, 7: 45, 8: -45, 9: 'i', 10: 'i', 13: 'i', 14: 'i', 17: 'i', 18: 'i'}
abmath = {11: 15, 12: -15, 13: -10, 14: 10, 15: -5, 16: 5}
hand = {5: 60, 6: -60}
#---------------------------------------------------------------------------------------------------------------------------------------------------

path = "/home/ali/Race/super.json"

class Dynamixel(object) :
	def __init__(self,Dxl,default_id=0) :
		ports = pypot.dynamixel.get_available_ports()
		if not ports :
			raise IOError("No ports found ")

		print "Connecting to ",ports[default_id]

		self.dxl = pypot.dynamixel.DxlIO(ports[default_id])
		self.ids = self.dxl.scan(range(25))
		print self.ids
		self.dxl.enable_torque(self.ids)
		if len(self.ids)<Dxl :
			raise RuntimeError("all the motors were not detected")

		self.dxl.set_moving_speed(dict(zip(self.ids,itertools.repeat(1024))))


	def posWrite(self,pose) :
		pos = {ids:angle for ids,angle in pose.items()}
		self.dxl.set_goal_position(pos)


	def listWrite(self,list) :
		pos = dict(zip(self.ids,list))
		self.dxl.set_goal_position(pos)


	def angleWrite(self,ids,pose) :
		self.dxl.set_goal_position({ids:pose})
		
	def returnPos(self,ids) :

		return self.dxl.get_present_position((ids,))	


class JSON(object) :
	def __init__(self,file) :
		try :
			with open(file,"r") as f :
				self.data = json.load(f)
		except :
			raise RuntimeError("File not found")

	'''def parse(self,motion) :
		find = "PageRoot/Page[@name='" +motion+ "']/steps/step"
		try :
			steps = [x for x in self.root.findall(find)]
		except :
			raise RuntimeError("Motion not found")

		p_frame = str()
		p_pose = str()
		write = []
		for step in steps :
			write.append(Motion(step.attrib['frame'],step.attrib['pose'],p_frame,p_pose))
			p_frame = step.attrib['frame']
			p_pose = step.attrib['pose']

		return write'''
		
	def parse(self,motion) :
		p_frame = str()
		p_pose = str()
		write = []
		js = self.data["Root"]["PageRoot"]["Page"]
		for j in js :
			try :
				 if motion in j["name"] :
					for step in j["steps"]["step"] :
						write.append(Motion(step["frame"],step["pose"],p_frame,p_pose))
						p_frame = step["frame"]
						p_pose = step["pose"]
			except :
				raise RuntimeError("Motion not found")
		return write
			
	def setparse(self,motion,offset=[]) :
		js = self.data["Root"]["FlowRoot"]["Flow"]
		motionset = []
		for j in js :
			try : 
				if motion in j["name"] :
					for unit in j["units"]["unit"] :
						motionset.append(Motionset(json.parse(motion=unit["main"]),speed=float(unit["mainSpeed"])/2.0,offset=offset))
			except :
				raise RuntimeError("Motionset not found")

		return motionset

	
	
class Motion(object) :
	def __init__(self,frame,pose,p_frame,p_pose) :
		self.frame = int(frame)
		self.begin = {}
		self.end = {}
	
		if not(p_pose) :
			self.frame_diff = 1
			p_pose = pose
		else :
			self.frame_diff = self.frame-int(p_frame) 

			
		for ids,pos in enumerate(map(float,p_pose.split())) :
			self.end[ids+1] = pos	

		for ids,pos in enumerate(map(float,pose.split())) :
			self.begin[ids+1] = pos


	def setoffset(self,offset={},darwin=True) :
		if not(darwin) :
			pass

		else :
			for key in offset.keys() :
				if offset[key] == 'i' :
					self.end[key] = -self.end[key]
					self.begin[key] = -self.begin[key]
				else :
					self.end[key] += offset[key]
					self.begin[key] += offset[key]		


	def motion(self,speed=1.0) :
		
		print self.begin
		print self.end
		write = []
		ids = []
		for key in self.end.keys() :
			linp = np.linspace(self.end[key],self.begin[key],self.frame_diff)
			write.append(linp)
			ids.append(key)	

		for pose in zip(*write) :
			print pose
			dxl.posWrite(dict(zip(ids,pose)))
			time.sleep(0.008/speed)



class Motionset(object) :
	def __init__(self,motion,speed=1,offset=[]) :
		self.motion = motion
		self.offset = offset
		self.speed = speed
		self.init = False

	def run(self,speed=1) :
		speed = self.speed
		
		if self.init :
			self.exe(speed)

		else :
			self.init = True
			for motion in self.motion :
				for offset in self.offset :
					#for m in motion :
					motion.setoffset(offset)
				motion.motion(speed)
			
	def exe(self,speed) :
		for motion in self.motion :
			motion.motion(speed)	
								

class Custom(object) :
	def __init__(self,motionset) :
		self.motionset = motionset

	def run(self,spd=None) :
		#prev_motionset = str()
		speed = spd
		for motionset in self.motionset :
			if not(spd) :
				speed = motionset.speed
  
			motionset.run(speed)

		
#--------------------------------------------------------------MOTIONS--------------------------------------------------------------------------------
json = JSON(path)
balance = Motionset(json.parse(motion="152 Balance"),offset=[darwin,hand])
w1 = Motionset(json.parse(motion="32 F_S_L"),speed=2.1,offset=[darwin])
w2 = Motionset(json.parse(motion="33 "),speed=2.1,offset=[darwin])
w3 = Motionset(json.parse(motion="38 F_M_R"),speed=2.7,offset=[darwin])
w4 = Motionset(json.parse(motion="39 "),speed=2.1,offset=[darwin])
w5 = Motionset(json.parse(motion="36 F_M_L"),speed=2.7,offset=[darwin])
w6 = Motionset(json.parse(motion="37 "),speed=2.1,offset=[darwin])
back_left = Motionset(json.parse(motion="17 B_R_E"),speed=1,offset=[darwin])
back_right = Motionset(json.parse(motion="18 B_L_E"),speed=1,offset=[darwin])
back_walk = Custom(json.setparse(motion="11 B_L_S",offset=[darwin]))
walk_init = Custom(motionset=[w1,w2])
walk_motion = Custom(motionset=[w3,w4,w5,w6])				
fast_left = Motionset(json.parse(motion="9 ff_r_l"),speed=1.5,offset=[darwin])
fast_right = Motionset(json.parse(motion="10 ff_l_r"),speed=1.5,offset=[darwin])
fast_walk = Custom(motionset=[fast_left,fast_right])
r_turn = Motionset(json.parse(motion="27 RT"),speed=1.2,offset=[darwin])
l_turn = Motionset(json.parse(motion="28 LT"),speed=1.2,offset=[darwin])
#-----------------------------------------------------------------------------------------------------------------------------------------------------

def dpad(pos) :
	if pos.top :
		fast_walk.run()
	elif pos.bottom :
		pass
	elif pos.left :
		l_turn.run()
	elif pos.right :
		r_turn.run()
	elif pos.middle :
		balance.run()
						
if __name__ == "__main__" :
	dxl = Dynamixel(Dxl=20)
	balance.run()	
	raw_input("Proceed?")	
	#walk_init.run()
	while True :
		walk_motion.run()
	'''bd = BlueDot()
	bd.when_pressed = dpad
	pause()'''
	
