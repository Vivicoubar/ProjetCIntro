import ctypes
import pathlib
import time
import os
import sys
from tkinter import *
from ctypes import *

##################################################
# Initialize C bindings
##################################################

# Load the shared library into ctypes
if sys.platform == 'win32':
  libname = os.path.join(pathlib.Path().absolute(), "libparticle.so")
else : 
  libname = os.path.join(pathlib.Path().absolute(), "libparticle.so")
c_lib = ctypes.CDLL(libname)

# Define python equivalent class

class PARTICLE(Structure):
    _fields_ = [("position", c_float*2),
                ("next_pos", c_float*2),
                ("velocity", c_float*2),
                ("inv_mass", c_float),
                ("radius",   c_float),
                ("solid_id", c_int),
                ("draw_id",  c_int)]

class CONTEXT(Structure):
    _fields_ = [("num_particles", c_int),
                ("capacity_particles", c_int),
                ("particles", POINTER(PARTICLE) ),
                ("num_ground_sphere", c_int) ]

# ("pos", c_float*2) => fixed size array of two float


# Declare proper return types for methods (otherwise considered as c_int)
c_lib.initializeContext.restype = POINTER(CONTEXT) # return type of initializeContext is Context*
c_lib.getParticle.restype = PARTICLE
# WARNING : python parameter should be explicitly converted to proper c_type of not integer.
# If we already have a c_type (including the one deriving from Structure above)
# then the parameter can be passed as is.

##################################################
# Class managing the UI
##################################################

class ParticleUI :
    def __init__(self) :
        # create drawing context
        self.context = c_lib.initializeContext(200)
        self.width = 1000
        self.height = 1000

        # physical simulation will work in [-world_x_max,world_x_max] x [-world_y_max,world_y_max]
        self.world_x_max = 10
        self.world_y_max = 10
        # WARNING : the mappings assume world bounding box and canvas have the same ratio !

        self.window = Tk()

        # create simulation context...
        self.canvas = Canvas(self.window,width=self.width,height=self.height)
        self.canvas.pack()

        # Initialize drawing, only needed if the context is initialized with elements,
        # otherwise, see addParticle
        for i in range(self.context.contents.num_particles):
            sphere = c_lib.getParticle(self.context, i)
            draw_id = self.canvas.create_oval(*self.worldToView( (sphere.position[0]-sphere.radius,sphere.position[1]-sphere.radius) ),
                                              *self.worldToView( (sphere.position[0]-sphere.radius,sphere.position[1]-sphere.radius) ),
                                              fill="orange")
            c_lib.setDrawId(self.context, i, draw_id) 
        
        # Initialize Mouse and Key events
        self.canvas.bind("<Button-1>", lambda event: self.mouseCallback(event))
        self.window.bind("<Key>", lambda event: self.keyCallback(event)) # bind all key
        self.window.bind("<Escape>", lambda event: self.enterCallback(event)) 
        # bind specific key overide default binding

    def launchSimulation(self) :
        # launch animation loop
        self.animate()
        # launch UI event loop
        self.window.mainloop()



    def animate(self) :
        """ animation loop """
        # APPLY PHYSICAL UPDATES HERE !
        for i in range(6) :
            c_lib.updatePhysicalSystem(self.context, c_float(0.016/float(6)), 1) #TODO can be called more than once..., just devise dt
            
        for i in range(self.context.contents.num_particles):
            sphere = c_lib.getParticle(self.context, i)
            self.canvas.coords(sphere.draw_id,
                               *self.worldToView( (sphere.position[0]-sphere.radius,sphere.position[1]-sphere.radius) ),
                               *self.worldToView( (sphere.position[0]+sphere.radius,sphere.position[1]+sphere.radius) ) )
        self.window.update()
        self.window.after(16, self.animate)

    # Conversion from worl space (simulation) to display space (tkinter canvas)
    def worldToView(self, world_pos) :
        return ( self.width *(0.5 + (world_pos[0]/self.world_x_max) * 0.5),
                 self.height *(0.5 - (world_pos[1]/self.world_y_max) * 0.5)) 
    def viewToWorld(self, view_pos) :
        return ( self.world_x_max * 2.0 * (view_pos[0]/self.width - 0.5) ,
                 self.world_y_max * 2.0 * (0.5-view_pos[1]/self.height))  

    def addParticle(self, screen_pos, radius, mass) :
        (x_world, y_world) = self.viewToWorld(screen_pos)
        # min max bounding box in view coordinates, will be propertly initialized 
        # in the canvas oval after the first call to animate
        #b_min = self.worldToView( (x_world-radius,y_world-radius) )
        #b_max = self.worldToView( (x_world+radius,y_world+radius) )
        draw_id = self.canvas.create_oval(0,0,0,0,fill="red")
        c_lib.addParticle(self.context, 
                        c_float(x_world), c_float(y_world), 
                        c_float(radius), c_float(mass),
                        draw_id)

    # All mouse and key callbacks

    def mouseCallback(self, event):
        self.addParticle((event.x,event.y), 0.2, 1.0)
    
    def keyCallback(self, event):
        print(repr(event.char))
    def enterCallback(self, event):
        self.window.destroy()


gui = ParticleUI()
gui.launchSimulation()

