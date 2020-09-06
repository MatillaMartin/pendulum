# Online Python compiler (interpreter) to run Python online.
# Write Python 3 code in this online editor and run it.

# Inverted Pendulum PID

import numpy as np
import math
import pygame
import time

gravity = 9.81 # m/s

# create a surface on screen
screen_size = (1400, 700)
screen = pygame.display.set_mode(screen_size)
view_scale = 200

class Entity:
	def __init__(self):
		self.position = np.array([0,0]).astype('f')
		self.vel = np.array([0,0]).astype('f')
		self.acc = np.array([0,0]).astype('f')
		self.friction = 0
		self.mass = 1
	
	# integration step
	def update(self, delta_s):
		self.vel += self.acc * float(delta_s)
		self.vel -= self.vel * self.friction * float(delta_s)
		self.position += self.vel * float(delta_s)
		# print("Acc:", self.acc)
		# print("Vel:", self.vel)
		# print("Position:", self.position)
	
class Pendulum:
	def __init__(self):
		self.bob = Entity()
		self.base = Entity()
		self.base.friction = 2
		self.length = 1
		self.setAngle(0) # CW angle wrt vertical 
	
	def update(self, delta_s):		
		self.base.update(delta_s)
		
		pole_vector = self.bob.position - self.base.position
		pole_direction = pole_vector/np.linalg.norm(pole_vector)
		
		self.bob.acc = np.array([0.0,0.0])
		#gravity acceleration
		self.bob.acc += np.array([0,-gravity])
		# base acceleration through rigid rod
		self.bob.acc += pole_vector * np.dot(self.base.acc, pole_vector)
		
		# resolve constraints
		pole_ppd = np.array([pole_direction[1], -pole_direction[0]])
		self.bob.acc = pole_ppd * np.dot(self.bob.acc, pole_ppd)
		self.bob.vel = pole_ppd * np.dot(self.bob.vel, pole_ppd)
		
		# print("Direction:", pole_direction)
		# print("Perpendicular:", pole_ppd)
		# print("Acc:", self.bob.acc)
		# print("Vel:", self.bob.vel)
		self.bob.update(delta_s)
		
		# force length
		pole_vector = self.bob.position - self.base.position
		pole_direction = pole_vector/np.linalg.norm(pole_vector)
		self.bob.position = self.base.position + pole_direction * self.length
		#update angle
		self.angle = math.acos( pole_direction[1] / self.length )
		
	def setAngle(self, angle):
		self.angle = angle
		self.bob.position = self.length * np.array([math.sin(angle), math.cos(angle)])
		
	def accelerate(self, amount):
		self.base.acc = amout
		# transfer acc through rod

class PendulumRender:
	@staticmethod
	def render(screen, pendulum):
		rects = []
		base_start = pendulum.base.position-np.array([ sizeToScreen(0.001),0])
		base_end = pendulum.base.position+np.array([ sizeToScreen(0.001),0])
		rects.append(pygame.draw.line(screen, [255, 255, 255], pointToScreen(base_start), pointToScreen(base_end)))
		rects.append(pygame.draw.line(screen, [255, 255, 255], pointToScreen(pendulum.base.position), pointToScreen(pendulum.bob.position)))
		rects.append(pygame.draw.circle(screen, [255, 255, 255], pointToScreen(pendulum.bob.position), int(sizeToScreen(0.05))))
		
		# render debug too
		rects.append(pygame.draw.line(screen, [255, 0,0], pointToScreen(pendulum.bob.position), pointToScreen(pendulum.bob.position + pendulum.bob.acc)))
		#rects.append(pygame.draw.line(screen, [0,255,0], pointToScreen(pendulum.bob.position), pointToScreen(pendulum.bob.position + pendulum.bob.vel)))
		
		rects.append(pygame.draw.line(screen, [255, 0,0], pointToScreen(pendulum.base.position), pointToScreen(pendulum.base.position + pendulum.base.acc)))
		#rects.append(pygame.draw.line(screen, [0,255,0], pointToScreen(pendulum.base.position), pointToScreen(pendulum.base.position + pendulum.base.vel)))
		
		return rects
		
class Engine:
	step_time = 0.2 # simulate update every 0.1ms
	
	def __init__(self):
		self.entities = []
		self.left_over_time = 0
	
	def update(self, delta_ms): # ms
		elapsed_time = delta_ms
		# print("delta_ms:", delta_ms)
		elapsed_time += self.left_over_time;
		# print("Elapsed plus left over:", elapsed_time)
		n_steps = int(math.floor(elapsed_time / self.step_time))
		# print("Steps:", n_steps)
		self.left_over_time = elapsed_time - n_steps * self.step_time;
		# print("Left over time:", self.left_over_time)
		for step in range(n_steps):
			for entity in self.entities:
				entity.update(self.step_time / 1000)
		

class ManualControl:
	def update(self, pendulum):
		keys = pygame.key.get_pressed()
		acc = np.array([0,0])
		if keys[pygame.K_RIGHT]:
			acc[0] = 10
		if keys[pygame.K_LEFT]:
			acc[0] = -10
		if keys[pygame.K_UP]:
			acc[1] = 10
		if keys[pygame.K_DOWN]:
			acc[1] = -10
		pendulum.base.acc = acc

def sizeToScreen(size):
	return size * view_scale
def pointToScreen(world):
	return [int(screen_size[0]/2+ world[0]*view_scale), int(screen_size[1]/2 - world[1]*view_scale)]

# define a main function
def main():
	engine = Engine()
	control = ManualControl()
	pendulum = Pendulum()
	engine.entities.append(pendulum)

	# unstable penulum!
	pendulum.setAngle(math.pi / 20)
	
	pygame.init()
	pygame.display.set_caption("Inverted Pendulum PID")
	
	
	# define a variable to control the main loop
	running = True
	clock = pygame.time.Clock()
	rects = []
	lastVel = 0
	# main loop
	while running:
		# frame rate to 60, store last frame time in ms
		delta_ms = clock.tick(60)
		
		for event in pygame.event.get():
			if event.type == pygame.QUIT:
				running = False
				
		# update physics with constant steps
		engine.update(delta_ms)
		
		# update control
		control.update(pendulum)
		
		#render loop
		last_rects = rects
		rects = []
		#draw background
		for rect in last_rects:
			pygame.draw.rect(screen, [0, 0, 0], rect)
			
		rects = PendulumRender.render(screen, pendulum)
		pygame.display.update(last_rects)
		pygame.display.update(rects)
	 
# run the main function only if this module is executed as the main script
# (if you import this as a module then nothing is executed)

if __name__=="__main__":
	# call the main function
	main()

