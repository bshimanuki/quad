import sys, os, random, time, math
import visual, ode
from quad import *

GRAV_CONSTANT = -9.8
FPS = 50
DT = 1./FPS
REAL_TIME = False # things could break if this is True

world = ode.World()
world.setGravity((0,0,GRAV_CONSTANT))
world.setContactSurfaceLayer(1e-5)
#world.setERP(0.8)
#world.setCFM(1e-9)
space = ode.Space()
class dummy: pass
floor = dummy()
floor.geom = ode.GeomPlane(space,(0,0,1),0)
floor.vis = visual.box(pos=(0,0,0),size=(1000,1000,0.1),color=visual.color.green)
floor.id = -1
contactgroup = ode.JointGroup()
visual.scene.forward = (0,1,0)
visual.scene.up = (0,0,1)
visual.scene.center = (0,0,5)
visual.scene.range = 10
#visual.scene.userzoom = 0
#visual.scene.userspin = 0

if __name__ == '__main__':
	global last_time
	running = True
	total_time = 0.
	team = Team(world)

	for i in xrange(-9,10,2):
		quad = Quad(world)
		quad.setSpace(space)
		quad.setPosition((i,0,5))
		team.add(quad)
		quad.showGoal(True)
		#quad.setAim((0,i,2))
		#quad.setOrientationVel(-1)

	last_time = time.time()
	while running:
		dt = DT
		visual.rate(FPS)
		
		for quad in team:
			theta = 2*math.pi/10*quad.id+total_time
			quad.setAim((4*math.cos(theta),4*math.sin(theta),3))
		
		team.thrust()
		space.collide((world,contactgroup),near_callback)
		if REAL_TIME:
			now = time.time()
			dt = now-last_time
			last_time = now
		world.step(dt)
		contactgroup.empty()
		team.render()
		total_time += dt
