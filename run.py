# Online Python compiler (interpreter) to run Python online.
# Write Python 3 code in this online editor and run it.

# Inverted Pendulum PID

import numpy as np
import math
import pygame
import time

gravity = 9.81 # m/s

pygame.font.init() 
text_font = pygame.font.SysFont('segoe-ui', 30)
	

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
		self.bob.friction = 0.0 #0.1
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
		#self.angle = math.acos( pole_direction[1] / self.length )
		self.angle = math.atan2( -pole_direction[0], pole_direction[1] );

		
	def setAngle(self, angle):
		self.angle = angle
		self.bob.position = self.length * np.array([math.sin(angle), math.cos(angle)])
		
	def accelerate(self, amount):
		self.base.acc = amout
		# transfer acc through rod

class PendulumRender:
	@staticmethod
	def render(camera, pendulum):
		rects = []
		base_size = [camera.sizeToScreen(0.1),camera.sizeToScreen(0.1)]
		base_start = camera.worldToScreen(pendulum.base.position) - np.array(base_size)/2.0
		rects.append(pygame.draw.rect(camera.screen, [255, 255, 255], pygame.Rect(tuple(base_start), tuple(base_size))))
		rects.append(pygame.draw.line(camera.screen, [255, 255, 255], camera.worldToScreen(pendulum.base.position), camera.worldToScreen(pendulum.bob.position)))
		rects.append(pygame.draw.circle(camera.screen, [255, 255, 255], camera.worldToScreen(pendulum.bob.position), int(camera.sizeToScreen(0.05))))
		
		# render debug too
		#rects.append(pygame.draw.line(camera.screen, [255, 0,0], camera.worldToScreen(pendulum.bob.position), camera.worldToScreen(pendulum.bob.position + pendulum.bob.acc)))
		#rects.append(pygame.draw.line(camera.screen, [0,255,0], camera.worldToScreen(pendulum.bob.position), camera.worldToScreen(pendulum.bob.position + pendulum.bob.vel)))
		
		#rects.append(pygame.draw.line(camera.screen, [255, 0,0], camera.worldToScreen(pendulum.base.position), camera.worldToScreen(pendulum.base.position + pendulum.base.acc)))
		#rects.append(pygame.draw.line(camera.screen, [0,255,0], camera.worldToScreen(pendulum.base.position), camera.worldToScreen(pendulum.base.position + pendulum.base.vel)))
		textsurface = text_font.render('Pos: ' + np.array_str(pendulum.base.position), True, (255, 255, 255))
		rects.append(camera.screen.blit(textsurface, camera.worldToScreen(pendulum.base.position)))
		textsurface = text_font.render('Acc: ' + np.array_str(pendulum.base.acc), True, (255, 255, 255))
		rects.append(camera.screen.blit(textsurface, camera.worldToScreen(pendulum.base.position) + np.array([0, 40])))
		textsurface = text_font.render('Angle: ' + str(pendulum.angle * 180 / math.pi), True, (255, 255, 255))
		rects.append(camera.screen.blit(textsurface, camera.worldToScreen(pendulum.base.position) + np.array([0, 80])))
		
		return rects
		
class GridRender:
	@staticmethod
	def render(camera):
		rects = []
		cellSize = 1
		# find min and max world in camera
		minWorld = camera.screenToWorld(np.array([0,0]))
		maxWorld = camera.screenToWorld(camera.screen_size)
		cameraSizeWorld = maxWorld - minWorld
		horCells = int(math.floor(cameraSizeWorld[0] / cellSize))
		verCells = int(math.floor(cameraSizeWorld[1] / cellSize))
		xStart = int(math.floor(minWorld[0] / cellSize) * cellSize)
		yStart = int(math.floor(minWorld[1] / cellSize) * cellSize)
		
		for x in range(horCells + 1):
			gridX = int(xStart + x * cellSize)
			rects.append(pygame.draw.line(camera.screen, [255, 255, 255], camera.worldToScreen([gridX, minWorld[1]]), camera.worldToScreen([gridX, maxWorld[1]])))
		for y in range(verCells + 1):
			gridY = int(yStart + y * cellSize)
			rects.append(pygame.draw.line(camera.screen, [255, 255, 255], camera.worldToScreen([minWorld[0], gridY]), camera.worldToScreen([maxWorld[0], gridY])))
		
		rects.append(pygame.draw.line(camera.screen, [255,0,0], camera.worldToScreen([0, 0]), camera.worldToScreen([1,0])))
		rects.append(pygame.draw.line(camera.screen, [0,255,0], camera.worldToScreen([0, 0]), camera.worldToScreen([0,1])))
		textsurface = text_font.render('X', True, (255,0,0))
		rects.append(camera.screen.blit(textsurface, camera.worldToScreen([1,0])))
		textsurface = text_font.render('Y', True, (0,255,0))
		rects.append(camera.screen.blit(textsurface, camera.worldToScreen([0,1])))
		
		
		return rects

class Engine:
	step_time = 1.0 / 1000 # simulate update every 0.2ms
	
	def __init__(self):
		self.entities = []
		self.left_over_time = 0
	
	def update(self, delta_s): # seconds
		elapsed_time = delta_s
		# print("delta_s:", delta_s)
		elapsed_time += self.left_over_time;
		# print("Elapsed plus left over:", elapsed_time)
		n_steps = int(math.floor(elapsed_time / self.step_time))
		# print("Steps:", n_steps)
		self.left_over_time = elapsed_time - n_steps * self.step_time;
		# print("Left over time:", self.left_over_time)
		for step in range(n_steps):
			for entity in self.entities:
				entity.update(self.step_time)
		

class ManualControl:
	def update(self, pendulum, delta_s):
		keys = pygame.key.get_pressed()
		if keys[pygame.K_RIGHT]:
			pendulum.base.acc[0] = 10
		if keys[pygame.K_LEFT]:
			pendulum.base.acc[0] = -10

class PIDControl:
# PID to minimize the pendulum angle
	def __init__(self):
		self.Kp = 100.0
		self.Ki = 1.0
		self.Kd = 10.0
		self.lastError = 0.0
		self.iv = 0.0 # integral
		
	def update(self, pendulum, delta_s):
		
		pv = pendulum.angle # sample pendulum angle
		sv = 0 # desired angle is zero
		
		error = sv - pv
		acc = np.array([0.0,0.0]).astype('f')
		
		# compute derivative
		dv = (error - self.lastError) / delta_s
		
		# compute integral
		self.iv += error * delta_s
		
		# only sideways for now
		acc[0] =  self.Kp * error + self.Ki*self.iv + self.Kd*dv
		
		pendulum.base.acc = acc
		# print(acc[0])
		self.lastError = error

class Camera:
	# create a surface on screen
	screen_size = np.array([1400, 700])
	screen = pygame.display.set_mode(screen_size)
	def __init__(self):
		self.view_scale = 100
		self.origin = np.array([0.0,0.0])
	def sizeToScreen(self, size):
		return size * self.view_scale
	def worldToScreen(self, world_pos):
		world = (self.screen_size/2.0 + (world_pos - self.origin)*self.view_scale).astype('int')
		# y up
		world[1] = self.screen_size[1] - world[1]
		return world
	def screenToWorld(self, screen_pos):
		return (screen_pos - self.screen_size/2.0)/self.view_scale + self.origin
	def track(self, position):
		# if abs(position[0] - self.origin[0]) > self.screen_size[0]/4.0:
			# self.origin[0] = math.copysign(position[0] - self.screen_size[0]/4.0, position[0] - self.origin[0])
		self.origin[0] = position[0]

# define a main function
def main():
	engine = Engine()
	manual_control = ManualControl()
	control = PIDControl()
	pendulum = Pendulum()
	camera = Camera()
	engine.entities.append(pendulum)

	# unstable penulum!
	pendulum.setAngle(-math.pi / 4)
	
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
		delta_s = clock.tick(60) / 1000.0
		
		for event in pygame.event.get():
			if event.type == pygame.QUIT:
				running = False
			elif event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
			   pygame.quit()
			   return
				
		# update physics with constant steps
		engine.update(delta_s)
		
		# update control
		control.update(pendulum, delta_s)
		manual_control.update(pendulum, delta_s)
		
		camera.track(pendulum.base.position)
		
		#render loop
		last_rects = rects
		rects = []
		#draw background
		for rect in last_rects:
			pygame.draw.rect(camera.screen, [0, 0, 0], rect)
			
		rects = []
		rects.extend(GridRender.render(camera))
		rects.extend(PendulumRender.render(camera, pendulum))
		
		pygame.display.update(last_rects)
		pygame.display.update(rects)
	
	pygame.quit()

# run the main function only if this module is executed as the main script
# (if you import this as a module then nothing is executed)

if __name__=="__main__":
	# call the main function
	main()

