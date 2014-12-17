import visual, ode
import operator, math
import numpy as np
import main
from main import GRAV_CONSTANT, FPS, DT

class Sphere:
	r = 0.5
	m = 2
	def vis(self):
		return visual.sphere(radius=self.r)
	def mass(self,m):
		m.setSphereTotal(self.m,self.r) # (density,radius)
	def geom(self,space):
		return ode.GeomSphere(space,radius=self.r)
class Box:
	dx = 1
	dy = 1
	dz = 0.5
	m = 2
	def vis(self):
		return visual.box(length=self.dx,height=self.dz,width=self.dy)
	def mass(self,m):
		m.setBoxTotal(self.m,self.dx,self.dy,self.dz)
	def geom(self,space):
		return ode.GeomBox(space,lengths=(self.dx,self.dy,self.dz))

class Quad(ode.Body):
	nextid = 0
	shape = Box()
	MAX_POWER = 100
	MAX_TORQUE = 5
	MAX_FORCE = 1000
	MAX_POWER_FORWARD = 5
	def F_ANTI_GRAVITY(self):
		return (0,0,-GRAV_CONSTANT*self.getMass().mass)
	def __init__(self, world):
		super(self.__class__,self).__init__(world)
		self.id = Quad.nextid
		Quad.nextid +=1
		self.vis = self.shape.vis()
		self.vis.color = visual.color.cyan
		self.goalvis = self.shape.vis()
		self.goalvis.color = visual.color.red
		self.goalvis.opacity = 0.3
		self.goalvis.axis = (1,0,0)
		self.goalvis.up = (0,0,1)
		self.showGoal(False)
		m = ode.Mass()
		self.shape.mass(m)
		self.setMass(m)
		self.energy = 0.
		self.aim = None
		self.theta = None
		self.omega = None
	def setSpace(self,space):
		self.geom = self.shape.geom(space)
		self.geom.setBody(self)
	#def __del__(self):
		#self.setVisible(False)
		#super(self.__class__,self).__del__()
	def setVisible(self,visible):
		self.vis.visible = visible
	def showGoal(self,visible):
		self.goalvis.opacity = 1
		self.goalvis.visible = visible
	def setAim(self,pos):
		self.aim = pos
		self.goalvis.pos = pos
	def setOrientation(self,theta):
		self.theta = theta
		self.goalvis.axis = (math.cos(theta),math.sin(theta),0)
	def setOrientationVel(self,omega):
		self.omega = omega
		rot = np.array(self.getRotation()).reshape(3,3)
		front = np.dot(rot,(1,0,0))
		front = math.atan2(front[1],front[0])
		theta = front+DT*omega
		self.goalvis.axis = (math.cos(theta),math.sin(theta),0)
	
	def thrust(self):
		rot = np.array(self.getRotation()).reshape(3,3)
		rotinv = np.linalg.inv(rot)
		top = np.dot(rot,(0,0,1))
		currAngVel = np.dot(rotinv,self.getAngularVel())
		currVel = np.dot(rotinv,self.getLinearVel())
		
		K_rotchVel = 1 # rotch is roll/pitch
		K_rotchAcc = 1
		K_yawVel = 1
		K_yawAcc = 1
		yawVel = 0
		if self.theta is not None:
			front = np.dot(rot,(1,0,0))
			front = math.atan2(front[1],front[0])
			theta = (self.theta-front) % (2*math.pi) - math.pi
			yawVel = K_yawVel*theta
		elif self.omega is not None:
			yawVel = self.omega
		omega = (K_rotchVel*top[1], -K_rotchVel*top[0], yawVel) - currAngVel
		alpha = np.multiply((K_rotchAcc,K_rotchAcc,K_yawAcc), omega)
		torque = np.dot(self.getMass().I,alpha)
		if np.linalg.norm(torque) > self.MAX_TORQUE:
			torque *= kelf.MAX_TORQUE/np.linalg.norm(torque) # multiplication with array
		# not currently worrying about a max rotational power
		
		K_linVel = 1
		K_linAcc = 1
		force = (0,0,0)
		if self.aim is not None:
			power_grav = -sum(map(operator.mul,self.F_ANTI_GRAVITY(),self.getLinearVel()))
			if power_grav > self.MAX_POWER: # if falling fast, fix it
				force = (0,0,self.MAX_POWER/self.getLinearVel()[2])
				force = np.dot(rotinv,force)
			else:
				dest = self.getPosRelPoint(self.aim)
				vel = K_linVel*np.array(dest)
				acc = K_linAcc*(vel-currVel)
				force = self.getMass().mass*acc
				power = abs(np.dot(force,currVel)) 
				if power > self.MAX_POWER-power_grav:
					force *= (self.MAX_POWER-power_grav)/power
				force += np.dot(rotinv,self.F_ANTI_GRAVITY())
			if force[2]<0: # can't thrust down
				force[2] = 0
			power_forward = np.dot(force[:2],currVel[:2])
			if power_forward > self.MAX_POWER_FORWARD:
				force[:2] *= self.MAX_POWER_FORWARD/power_forward
			power = np.dot(force,currVel)
			if power > self.MAX_FORCE:
				force *= self.MAX_FORCE/power
		
		self.addRelTorque(torque)
		self.addRelForce(force)
		self.energy += DT*abs(np.dot(torque,currAngVel))
		self.energy += DT*abs(np.dot(force,currVel))
		# this vastly undercounts energy use. Does not account for inefficiency and doesn't account for work against gravity
		
	def render(self):
		self.vis.pos = self.getPosition()
		rot = np.array(self.getRotation()).reshape(3,3)
		self.vis.axis = tuple(np.dot(rot,np.array((1,0,0))))
		self.vis.up = tuple(np.dot(rot,np.array((0,0,1))))

class Team(set):
	def __init__(self,world):
		super(self.__class__,self).__init__()
		self.world = world
	def thrust(self):
		for obj in self:
			obj.thrust()
	def render(self):
		for obj in self:
			obj.render()

contacts_count = 0 
def near_callback((world,contactgroup),geom1,geom2):
	"""Handles collisions from the collide() function."""
	global contacts_count
	obj1 = geom1.getBody()
	if not obj1:
		obj1 = main.floor
	obj2 = geom2.getBody()
	if not obj2:
		obj2 = main.floor
	if ode.areConnected(geom1.getBody(),geom2.getBody()):
		return
	contacts = ode.collide(geom1,geom2)
	for contact in contacts:
		contacts_count += 1
		print contacts_count,'Contact between object',obj1.id,'and object',obj2.id
		contact.setBounce(.3)
		contact.setMu(20)
		joint = ode.ContactJoint(world,contactgroup,contact)
		joint.attach(geom1.getBody(),geom2.getBody())
