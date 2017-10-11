# -*- coding: utf-8 -*-
"""
Created on Sat Jan 14 23:29:06 2017

@author: Daniel
"""

import random
import time
import math

import tkinter as tk
import numpy as np


BOIDS = 50
CHOOSE_FOCUS = False
OBS_TYPE = "line"
OBS_TYPE = "circle"
OBS_TYPE = "pass"


class Quadtree():
    '''Quadtree class which is designed to contain objects with a .rect attribute

    Attributes:
    max_objects: Maximum number of objects in a quad
    max_level: Maximum depth to recur the quads
    level: current level of the tree
    objects: actual objects in that quad
    bounds: the rectangle that bounds the top-most Quadtree
    nodes: all sub-Quadtrees
    '''
    def __init__(self, level, bounds):
        self.max_objects = 8
        self.max_level = 8
        self.level = level
        self.objects = []
        self.bounds = bounds
        self.nodes = []

    def clear2(self):
        '''Clears all objects recursively in the Quadtree'''
        self.objects.clear()
        for i in range(0, len(self.nodes)):
            if self.nodes[i]:
                self.nodes[i].clear2()
        self.nodes.clear()

    def split(self):
        '''Splits the quadtree into four sub trees'''
        subwidth = int((self.bounds[0]+self.bounds[2])/2)
        subheight = int((self.bounds[1]+self.bounds[3])/2)

        self.nodes.append(Quadtree(self.level+1,
                                   (subwidth, self.bounds[1],
                                    subwidth*2-self.bounds[0], subheight)))
        self.nodes.append(Quadtree(self.level+1,
                                   (self.bounds[0], self.bounds[1],
                                    subwidth, subheight)))
        self.nodes.append(Quadtree(self.level+1,
                                   (self.bounds[0], subheight,
                                    subwidth, 2*subheight-self.bounds[1])))
        self.nodes.append(Quadtree(self.level+1,
                                   (subwidth, subheight,
                                    2*subwidth - self.bounds[0], 2*subheight - self.bounds[1])))

    def getindex(self, rect):
        '''Returns an integer corresponding to where rect fits'''
        index = -1
        verticalmid = int((self.bounds[0]+self.bounds[2])/2)
        horizontalmid = int((self.bounds[1]+self.bounds[3])/2)

        topquad = (rect[3] < horizontalmid)
        botquad = (rect[1] > horizontalmid)

        if rect[2] < verticalmid:
            if topquad:
                index = 1
            elif botquad:
                index = 2
        elif rect[0] > verticalmid:
            if topquad:
                index = 0
            elif botquad:
                index = 3

        return index

    def getindex2(self, rect):
        '''Returns a list corresponding to where rect fits'''
        index = [0, 1, 2, 3]
        verticalmid = int((self.bounds[0]+self.bounds[2])/2)
        horizontalmid = int((self.bounds[1]+self.bounds[3])/2)

        topquad = (rect[3] < horizontalmid)
        botquad = (rect[1] > horizontalmid)

        if topquad:
            index.remove(2)
            index.remove(3)
        elif botquad:
            index.remove(1)
            index.remove(0)

        if rect[2] < verticalmid:
            try:
                index.remove(1)
                index.remove(2)
            except ValueError:
                pass
        elif rect[0] > verticalmid:
            try:
                index.remove(0)
                index.remove(3)
            except ValueError:
                pass

        return index

    def insert(self, boid):
        '''Puts an object into the right place in the tree, and splits as necessary'''
        if len(self. nodes) != 0:
            index = self.getindex(boid.rect)

            if index != -1:
                self.nodes[index].insert(boid)
                return()

        self.objects.append(boid)

        if len(self.objects) > self.max_objects and self.level < self.max_level:
            if len(self.nodes) == 0:
                self.split()

            i = 0
            while i < len(self.objects):
                index = self.getindex(self.objects[i].rect)
                if index != -1:
                    self.nodes[index].insert(self.objects.pop(i))
                else:
                    i += 1

    def retreive(self, returnobjects, boid):
        '''Retreives all objects that could collide with boid'''
        index = self.getindex(boid.rect)
        if index != -1 and len(self.nodes) != 0:
            self.nodes[index].retreive(returnobjects, boid)

        if index == -1:
            index_list = self.getindex2(boid.rect)
            to_test = []
            for i in index_list:
                try:
                    to_test.append(self.nodes[i])
                except IndexError:
                    pass
            for k in to_test:
                k.retreive(returnobjects, boid)


        for obj in self.objects:
            returnobjects.append(obj)

        return returnobjects

class World():
    '''A world containing Boids and Obstacles

    Attributes:
    width and height: width and height of the World
    boids and obstacles: lists of each type
    quad: a Quadtree that contains all the objects
    '''
    def __init__(self):
        self.width = 1000
        self.height = 1000
        self.boids = []
        self.obstacles = []
        self.quad = Quadtree(0, (0, 0, self.width, self.height))

    def setup_boids(self, num):
        '''Randomly generates n boids within 150 of the world bounds'''
        self.boids.clear()
        for i in range(num):
            k = Boid()
            k.rand_start(self.height, self.width)
            self.boids.append(k)

            if i == 0 and CHOOSE_FOCUS:
                k.foc()

    def setup_obstacles(self, obs_type):
        '''Sets up obstacles in different configurations'''
        self.obstacles.clear()

        if obs_type == "line": #Sets two vertical lines on the edges
            for i in range(int(self.height/50)+1):
                k = Obstacle()
                k.setxy(25, 50*i)
                j = Obstacle()
                j.setxy(self.width-25, 50*i)
                self.obstacles.append(k)
                self.obstacles.append(j)

        elif obs_type == "circle": #Sets a circle of obstacles
            for i in range(45):
                k = Obstacle()
                k_x = self.width/2 + 375* math.cos((i+1)*2*math.pi / 45)
                k_y = self.height/2 + 375* math.sin((i+1)*2*math.pi / 45)
                k.setxy(k_x, k_y)
                self.obstacles.append(k)

    def addboid(self, pos_x, pos_y):
        '''Adds a new Boid at x,y '''
        new_boid = Boid()
        new_boid.setxy(pos_x, pos_y)
        self.boids.append(new_boid)


    def step(self):
        '''Performs a deiscrete update step on all boids'''
        self.quad.clear2()

        for boid in self.boids:
            self.quad.insert(boid)

        for obstacle in self.obstacles:
            self.quad.insert(obstacle)

        returned = []
        for boid in self.boids:
            returned.clear()
            returned = self.quad.retreive(returned, boid)
            returned.remove(boid)

            boid.update(returned, self.height, self.width)


class Boid():
    '''The main object, a Boid

    Attributes:
    vel_x, vel_y, pos_x, pos_y: Position and speed components of the Boid
    theta: The angular direction it travels in
    rep, ali and att: Square of the distance to repulse, align and attract (resp)
    focus: Whether to focus on the boid or not
    speed, turn_speed: distance to travel or turn in one time step
    aware: list of all objects that the Boid is aware of
    obstacle: False if it is a Boid, true for an obstacle
    obstacles: Whether or not a boid sees an obstacle; allows for priority of movement
    rect: The rectangle which describes its attraction range
    '''

    # pylint: disable=too-many-instance-attributes
    # Boids just have a lot going on, ok?!

    def __init__(self):
        self.vel_x = 0
        self.vel_y = 0
        self.pos_x = 0
        self.pos_y = 0
        self.theta = 0

        self.rep = 900
        self.ali = 2200
        self.att = 3000
        self.rect = (0, 0, 0, 0)

        self.focus = False

        self.speed = 0.75
        self.turn_speed = 0.05
        self.aware = []         #all things it is aware of, colour purposes only

        self.obstacle = False
        self.obstacles = False

    def setxy(self, pos_x, pos_y):
        '''Set the x and y variables of the Boid, and calculate rect appropriately'''
        self.pos_x = pos_x
        self.pos_y = pos_y
        self.rect = (self.pos_x-int(self.att**(1/2)),
                     self.pos_y-int(self.att**(1/2)),
                     self.pos_x+int(self.att**(1/2)),
                     self.pos_y+int(self.att**(1/2)))

    def rand_start(self, height, width):
        '''Randomly generates a position to start the boid within the bounds of height and width'''
        self.pos_x = random.randint(150, width-150)
        self.pos_y = random.randint(150, height-150)

        self.theta = random.random()*2*math.pi
        self.vel_x = math.cos(self.theta)
        self.vel_y = math.sin(self.theta)

        self.rect = (self.pos_x-int(self.att**(1/2)),
                     self.pos_y-int(self.att**(1/2)),
                     self.pos_x+int(self.att**(1/2)),
                     self.pos_y+int(self.att**(1/2)))

    def foc(self):
        '''Change whether the boid is focussed or not'''
        self.focus = not self.focus

    def update(self, objects, height, width):
        '''The main method of a boid, includes all the logic for following the three rules of boids

        Repulse: Avoids nearby boids and preferentially avoids obstacless
        Align: Aligns to the direction of boids in a certain range
        Attract: Aligns to the average location of boids in the neighbourhood

        '''
        repulse = []
        align = []
        attract = []
        self.obstacles = False   #This allows boids to prioritise avoiding obstacles over all else

        velocity = np.array([self.vel_x, self.vel_y])
        for boid in objects:

            distance = (self.pos_x - boid.pos_x)**2 + (self.pos_y - boid.pos_y)**2

            if boid.obstacle and distance < self.att:
                repulse.append(boid)
                self.obstacles = True

            else:

                disp = np.array([boid.pos_x - self.pos_x, boid.pos_y-self.pos_y])
                dotted = np.dot(velocity, disp)
                vel_mag = np.sqrt(np.dot(velocity, velocity))
                disp_mag = np.sqrt(np.dot(disp, disp))
                if vel_mag > 0 and disp_mag > 0:
                    dotted /= (vel_mag * disp_mag)

                if dotted > -0.5:
                    if distance < self.rep:
                        repulse.append(boid)
                    elif distance < self.ali:
                        align.append(boid)
                    elif distance < self.att:
                        attract.append(boid)

        aim_theta = self.aim(repulse, align, attract)

        self.aware = repulse + align + attract

        self.steer(aim_theta)

        self.move(height, width)

    def aim(self, repulse, align, attract):
        ''' Takes in the three types of object and returns the new aiming angle'''
        aim_theta = self.theta
        if repulse and not self.obstacles:
            ave_x = sum(boid.pos_x-self.pos_x for boid in repulse)/len(repulse)
            ave_y = sum(boid.pos_y-self.pos_y for boid in repulse)/len(repulse)
            aim_theta = math.atan2(ave_y, ave_x) + math.pi
        elif repulse and self.obstacles:
            obs = []
            for obj in repulse:
                if obj.obstacle:
                    obs.append(obj)
            ave_x = sum(boid.pos_x-self.pos_x for boid in obs)/len(obs)
            ave_y = sum(boid.pos_y-self.pos_y for boid in obs)/len(obs)
            aim_theta = math.atan2(ave_y, ave_x) + math.pi
        else:

            if align and not self.obstacles:
                #new_theta = aim_theta
                new_theta = 0
                for boid in align:
                    new_theta += boid.theta
                #aim_theta = (new_theta/(len(align)+1))
                aim_theta = (new_theta/(len(align)))

            if attract and not self.obstacles:
                ave_x = sum(boid.pos_x-self.pos_x for boid in attract)/len(attract)
                ave_y = sum(boid.pos_y-self.pos_y for boid in attract)/len(attract)
                if align:
                    aim_theta = 0.5*(aim_theta + math.atan2(ave_y, ave_x))
                else:
                    aim_theta = math.atan2(ave_y, ave_x)

        if random.random() > 0.5 and not self.obstacles:
            if aim_theta > 0 and aim_theta < math.pi/2:
                aim_theta += random.uniform(-0.05, 0)
            elif math.pi/2 < aim_theta and aim_theta < math.pi:
                aim_theta += random.uniform(0, 0.05)

        if repulse and self.obstacles:
            closest = 1000000000
            for obj in repulse:
                if obj.obstacle:
                    dist = (obj.pos_y-self.pos_y)**2 + (obj.pos_x-self.pos_x)**2
                    if dist < closest:
                        closest = dist
                        aim_theta = math.atan2(obj.pos_y-self.pos_y, obj.pos_x-self.pos_x) + math.pi

        return aim_theta

    def steer(self, aim_theta):
        '''Steers towards the aimed for angle'''

        option1 = (self.theta - aim_theta)%(math.pi*2) < self.turn_speed
        option2 = (self.theta - aim_theta)%(math.pi*2) > 2*math.pi - self.turn_speed

        if option1 or option2:
            self.theta = aim_theta
        elif (aim_theta - self.theta)%(math.pi*2) < math.pi:
            self.theta = (self.theta + self.turn_speed)%(math.pi * 2)
        else:
            self.theta = (self.theta - self.turn_speed)%(math.pi * 2)

        self.vel_x = math.cos(self.theta)
        self.vel_y = math.sin(self.theta)

    def move(self, height, width):
        '''Moves the Boid, keeping it within the bounds and updates its rect'''
        self.pos_x = (self.pos_x + self.speed*self.vel_x)%width
        self.pos_y = (self.pos_y + self.speed*self.vel_y)%height
        self.rect = (self.pos_x-int(self.att**(1/2)),
                     self.pos_y-int(self.att**(1/2)),
                     self.pos_x+int(self.att**(1/2)),
                     self.pos_y+int(self.att**(1/2)))


class Obstacle(Boid):
    '''Extends the Boid class to make an Obstacle

    Attributes:
    x and y: The position of the Obstacle
    obstacle: True for Obstacles
    diameter: The width of the circle to be drawn
    oval: Rect for the circle to be drawn
    rect: The area that it could affect with att
    '''
    def __init__(self, *args, **kwargs):
        Boid.__init__(self, *args, **kwargs)
        self.obstacle = True
        self.diameter = 20
        self.oval = (0, 0, 0, 0)
        self.pos_x = 0
        self.pos_y = 0

    def setxy(self, x, y):
        '''Set the position of the Obstacle'''
        self.pos_x = x
        self.pos_y = y
        self.oval = (self.pos_x-int(0.5*self.diameter),
                     self.pos_y-int(0.5*self.diameter),
                     self.pos_x+int(0.5*self.diameter),
                     self.pos_y+int(0.5*self.diameter))

        self.rect = (self.pos_x-int(self.att**(1/2)),
                     self.pos_y-int(self.att**(1/2)),
                     self.pos_x+int(self.att**(1/2)),
                     self.pos_y+int(self.att**(1/2)))


class Flocking(tk.Tk):
    '''The main App class for the Flocking simulation

    Attributes:
    world: A world of boids and obstacles
    height and width: Inhereted from the World
    start: Used to time, in order to calculate FPS
    canvas: A tk canvas object
    '''
    def __init__(self, *args, **kwargs):
        tk.Tk.__init__(self, *args, **kwargs)

        self.world = World()
        self.height = self.world.height
        self.width = self.world.width
        self.start = time.time()
        self.draw_quad = tk.BooleanVar()
        self.draw_quad.set(False)
        self.running = tk.BooleanVar()
        self.running.set(True)

        self.canvas = tk.Canvas(self, width=self.width, height=self.height,
                                borderwidth=0, highlightthickness=0)

        self.canvas.bind("<Button-1>", self.addboid)

        self.canvas.pack(side="left", fill="both", expand="true")

        self.buttons = tk.Frame(self)
        self.create_buttons()
        self.buttons.pack(side="right", expand="true", fill="both")

        self.world.setup_boids(BOIDS)
        self.world.setup_obstacles(OBS_TYPE)
        self.update()

    def create_buttons(self):
        '''Creates all the buttons'''
        self.clear_button = tk.Button(self.buttons,
                                      text="Clear",
                                      command=self.kill_boids,
                                      width=18)
        self.clear_button.pack(side="top", pady=(10, 10))

        self.reset_button = tk.Button(self.buttons,
                                      text="Reset",
                                      command=lambda: self.world.setup_boids(BOIDS),
                                      width=18)
        self.reset_button.pack(side="top", pady=(10, 10))

        self.average_button = tk.Button(self.buttons,
                                        text="Average",
                                        command=self.averages,
                                        width=18)
        self.average_button.pack(side="top", pady=(10, 10))

        self.quad_button = tk.Checkbutton(self.buttons,
                                          text="Draw Quadtree",
                                          variable=self.draw_quad,
                                          width=18, onvalue=True, offvalue=False)
        self.quad_button.pack(side="top", pady=(10, 10))

        self.run_button = tk.Checkbutton(self.buttons,
                                         text="Pause",
                                         variable=self.running,
                                         width=18, onvalue=False, offvalue=True)
        self.run_button.pack(side="top", pady=(10, 10))


    def addboid(self, event):
        '''Adds a boid at the position of the mouse'''
        self.world.addboid(event.x, event.y)

    def kill_boids(self, event=None):
        '''Removes all boids'''
        self.world.boids.clear()

    def averages(self):
        '''Prints the mean angle of boids'''
        mean = sum(boid.theta for boid in self.world.boids)/len(self.world.boids)
        print(mean)

    def get_quads(self):
        '''Recursively gets all bounds of quads '''

        quad = self.world.quad
        new_nodes = set(quad.nodes)
        nodes = {quad}

        while new_nodes:
            nodes |= set(new_nodes)
            new_nodes.clear()
            for node in nodes:
                new_nodes |= set(node.nodes)
            new_nodes -= nodes

        bound_list = []
        for node in nodes:
            bound_list.append(node.bounds)

        return bound_list


    def update(self):
        '''Updates the world and draws everything per tick'''

        tick = time.time()-self.start
        self.start = time.time()
        if tick != 0:
            #print("\r", str(1/tick), end="")
            pass

        if self.running.get():
            self.world.step()
            self.canvas.delete("all")
            self.draw_boids()
            self.draw_obstacles()

        self.canvas.delete("quad")
        if self.draw_quad.get():
            self.draw_quadtree()

        self.canvas.pack(side="left", fill="both", expand="true")

        self.after(13, self.update)

    def draw_boids(self):
        '''Draws boids onto canvas'''

        for boid in self.world.boids:
            tip_x = boid.pos_x + (20*boid.vel_x)
            tip_y = boid.pos_y+ (20*boid.vel_y)

            if boid.focus:
                colour = "red"
            else:
                if boid.obstacles:
                    colour = "green"
                elif len(boid.aware) > 0:
                    colour = "blue"
                else:
                    colour = "orange"

            self.canvas.create_polygon((tip_x, tip_y,
                                        int(boid.pos_x-(5*boid.vel_y)),
                                        int(boid.pos_y+(5*boid.vel_x)),
                                        int(boid.pos_x+(5*boid.vel_y)),
                                        int(boid.pos_y-(5*boid.vel_x))),
                                       fill=colour, outline="black", tags="boid")

    def draw_obstacles(self):
        '''Draws the obstacles'''
        for obs in self.world.obstacles:
            self.canvas.create_oval(obs.oval, fill="red", outline="", tags="obstacle")
            #self.canvas.create_oval((obs.x-100,obs.y-100,obs.x+100,obs.y+100))

    def draw_quadtree(self):
        '''Draws the quadtree'''
        bound_list = self.get_quads()
        for bound in bound_list:
            self.canvas.create_rectangle(bound, tags="quad")


if __name__ == "__main__":
    APP = Flocking()
    APP.mainloop()
    APP.quit()
