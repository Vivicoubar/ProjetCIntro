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
    
class SPHERE_COLLIDER(Structure):
    _fields_ = [("center",c_float*2),
                ("radius", c_float)]
    

class CONSTRAINT(Structure):
    _fields_ = [("constraint", c_float*2),
                ("origin",c_int)]


class PLANE_COLLIDER(Structure):
    _fields_ = [("start_pos", c_float*2),
               ("director", c_float*2)]
    
class GROUND_CONSTRAINT(Structure):
    _fields_ = [("num_constraint", c_int),
                ("constraint", ctypes.POINTER(CONSTRAINT)),
                ("capacity_constraints",c_int)]

class PARTICLE_CONSTRAINT(Structure):
    _fields_ = [("num_constraint", c_int),
                ("constraint", ctypes.POINTER(CONSTRAINT)),
                ("capacity_constraints",c_int)]
    
class BOUNDS(Structure):
    _fields_ = [("particle1", c_int),
                ("particle2", c_int),
                ("target_distance", c_float),
                ("stiffness", c_float)]

    
class BOUNDS_CONSTRAINTS(Structure):
    _fields_ = [("num_bounds", c_int),
                ("bounds", ctypes.POINTER(BOUNDS)),
                ("capacity_bounds", c_int),
                ("num_constraints", c_int),
                ("constraints", ctypes.POINTER(CONSTRAINT)),
                ("capacity_constraints", c_int)]
    
    
class BOX_COLLIDER(Structure):
    _fields_ = [("center", c_float*2),
               ("director1", c_float*2),
               ("director2", c_float*2)]

class CONTEXT(Structure):
    _fields_ = [("num_particles", c_int),
                ("capacity_particles", c_int),
                ("particles", ctypes.POINTER(PARTICLE)),
                ("num_ground_sphere", c_int),
                ("ground_spheres", ctypes.POINTER(SPHERE_COLLIDER)),
                ("num_ground_plane", c_int),
                ("ground_planes", ctypes.POINTER(PLANE_COLLIDER)),
                ("num_boxes", c_int),
                ("box_collider", ctypes.POINTER(BOX_COLLIDER)),
                ("ground_constraints", ctypes.POINTER(GROUND_CONSTRAINT)),
                ("particle_constraints", ctypes.POINTER(PARTICLE_CONSTRAINT)),
                ("bounds_constraints", ctypes.POINTER(BOUNDS_CONSTRAINTS))]
    

# ("pos", c_float*2) => fixed size array of two float


# Declare proper return types for methods (otherwise considered as c_int)
c_lib.initializeContext.restype = POINTER(CONTEXT) # return type of initializeContext is Context*
c_lib.getParticle.restype = PARTICLE
c_lib.getGroundSphereCollider.restype = SPHERE_COLLIDER
c_lib.getGroundPlaneCollider.restype = PLANE_COLLIDER
c_lib.getBoxCollider.restype = BOX_COLLIDER
c_lib.initializeGroundConstraint.restype = POINTER(GROUND_CONSTRAINT)
c_lib.initializeBoundsConstraint.restype = POINTER(BOUNDS_CONSTRAINTS)
c_lib.initializeParticleConstraint.restype = POINTER(PARTICLE_CONSTRAINT)
# WARNING : python parameter should be explicitly converted to proper c_type of not integer.
# If we already have a c_type (including the one deriving from Structure above)
# then the parameter can be passed as is.

##################################################
# Class managing the UI
##################################################

class ParticleUI :
    def __init__(self) :
        # create drawing context
        self.context = c_lib.initializeContext(300)
        self.ground_constraint = c_lib.initializeGroundConstraint(300)
        self.particle_constraint = c_lib.initializeParticleConstraint(300)
        self.bounds_constraint = c_lib.initializeBoundsConstraint(300)
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

        # create grid
        self.createGridWithSubdivisions(1.0, 5)
        # Initialize drawing, only needed if the context is initialized with elements,
        for i in range(self.context.contents.num_ground_sphere):
            sphere = c_lib.getGroundSphereCollider(self.context, i)
            draw_id = self.canvas.create_oval(*self.worldToView( (sphere.center[0]-sphere.radius,sphere.center[1]-sphere.radius) ),
                                              *self.worldToView( (sphere.center[0]+sphere.radius,sphere.center[1]+sphere.radius) ),
                                              fill="blue") 
            
        for i in range(self.context.contents.num_ground_plane):
            plane = c_lib.getGroundPlaneCollider(self.context, i)
            x0 , y0 = plane.start_pos[0], plane.start_pos[1]
            x1 ,y1= x0+plane.director[0], y0+plane.director[1]
            draw_id = self.canvas.create_line(*self.worldToView( (x0,y0)),
                                              *self.worldToView((x1,y1)),
                                              fill="black", width=3) 

        for i in range(self.context.contents.num_boxes):
            box = c_lib.getBoxCollider(self.context, i)
            points = []
            points.append([*self.worldToView((box.center[0] + box.director1[0] - box.director2[0], box.center[1] + box.director1[1] - box.director2[1]))])
            points.append([*self.worldToView((box.center[0] - box.director1[0] - box.director2[0], box.center[1] - box.director1[1] - box.director2[1]))])
            points.append([*self.worldToView((box.center[0] - box.director1[0] + box.director2[0], box.center[1] - box.director1[1] + box.director2[1]))])
            points.append([*self.worldToView((box.center[0] + box.director1[0] + box.director2[0], box.center[1] + box.director1[1] + box.director2[1]))])
            draw_id = self.canvas.create_polygon(points, outline="black", fill="green")
            c_lib.setDrawId(self.context, i, draw_id) 

        # otherwise, see addParticle
        for i in range(self.context.contents.num_particles):
            sphere = c_lib.getParticle(self.context, i)
            draw_id = self.canvas.create_oval(*self.worldToView( (sphere.position[0]-sphere.radius,sphere.position[1]-sphere.radius) ),
                                              *self.worldToView( (sphere.position[0]+sphere.radius,sphere.position[1]+sphere.radius) ),
                                              fill="orange")
            c_lib.setDrawId(self.context, i, draw_id) 
        
        # Initialize Mouse and Key events
        self.canvas.bind("<Button-1>", lambda event: self.mouseCallback(event))
        self.canvas.bind("<Button-3>", lambda event: self.mouseCallback2(event))
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
            c_lib.updatePhysicalSystem(self.context, c_float(0.016/float(6)), 1)
            
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
    
    def addParticleWithBound(self, screen_pos, radius, mass) :
        (x_world, y_world) = self.viewToWorld(screen_pos)
        # min max bounding box in view coordinates, will be propertly initialized 
        # in the canvas oval after the first call to animate
        #b_min = self.worldToView( (x_world-radius,y_world-radius) )
        #b_max = self.worldToView( (x_world+radius,y_world+radius) )
        draw_id1 = self.canvas.create_oval(0,0,0,0,fill="yellow")
        draw_id2 = self.canvas.create_oval(0,0,0,0,fill="yellow")
        draw_id3 = self.canvas.create_oval(0,0,0,0,fill="yellow")
        draw_id4 = self.canvas.create_oval(0,0,0,0,fill="yellow")
        c_lib.addBound(self.context, 
                        c_float(x_world), c_float(y_world), 
                        c_float(radius), c_float(mass),
                        draw_id1, draw_id2, draw_id3, draw_id4)

    # All mouse and key callbacks

    def mouseCallback(self, event):
        self.addParticle((event.x,event.y), 0.2, 1.0)
    
    def mouseCallback2(self, event):
        self.addParticleWithBound((event.x,event.y), 0.2, 1.0)
    
    def keyCallback(self, event):
        if(event.char == "e"):
            for i in range(0,5):
                self.addParticle((100 + i*200, 0), 0.2, 1.0)
            
        print(repr(event.char))
    def enterCallback(self, event):
        self.window.destroy()

    def createGrid(self, cell_size, color):
        num_rows = int(2 * self.world_y_max / cell_size)
        num_cols = int(2 * self.world_x_max / cell_size)

        for row in range(num_rows + 1):
            x1_C = - self.world_x_max
            x2_C = self.world_x_max
            y_C = - self.world_y_max + row * cell_size
            self.canvas.create_line(*self.worldToView( (x1_C, y_C) ),
                                    *self.worldToView( (x2_C, y_C) ),
                                    fill=color)

        for col in range(num_cols + 1):
            x_C = - self.world_x_max + col * cell_size
            y1_C = - self.world_x_max
            y2_C = self.world_x_max
            self.canvas.create_line(*self.worldToView( (x_C, y1_C) ),
                                    *self.worldToView( (x_C, y2_C) ),
                                    fill=color)
    def createAxisLabels(self):
        x_labels = range(-10, 10, 2)
        for label in x_labels:
            x_C = label
            y_C = 0.0
            x, y = self.worldToView((x_C, y_C))
            y -= 10  # Position below the x-axis
            self.canvas.create_text(x, y, text=str(label), fill='dark gray')

        y_labels = range(-10, 10, 2)
        for label in y_labels:
            x_C = 0.0
            y_C = label
            x, y = self.worldToView((x_C, y_C))
            x -= 10  # Position to the right of the y-axis
            self.canvas.create_text(x, y, text=str(label), fill='dark gray')

    def createGridWithSubdivisions(self, cell_size, num_subdivisions):
        subdivision_size = cell_size / num_subdivisions
        self.createGrid(subdivision_size, 'light gray')
        self.createGrid(cell_size, 'dark gray')
        self.createAxisLabels()


gui = ParticleUI()
gui.launchSimulation()

