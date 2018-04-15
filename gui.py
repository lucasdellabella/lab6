import threading
from tkinter import *
import time
from setting import *
import random
random.seed(RANDOM_SEED)
import copy
import math

from PIL import ImageTk

from grid import *
from particle import Particle
from utils import *



# GUI class
class GUIWindow():
    def __init__(self, grid, show_camera=False):
        self.width = grid.width
        self.height = grid.height
        self.update_cnt = 0

        self.grid = grid
        self.running = threading.Event()
        self.updated = threading.Event()
        self.updated.clear()
        self.lock = threading.Lock()
        # grid info
        self.occupied = grid.occupied
        self.markers = grid.markers

        self.particles = []
        self.robot = None

        self.show_camera = show_camera
        self.camera_image = None

        print("Occupied: ")
        print(self.occupied)
        print("Markers: ")
        print(self.markers)


    """
    plot
    """
    def drawGrid(self):
        for y in range(1,self.grid.height):
            self.canvas.create_line(0, y * self.grid.scale, int(self.canvas.cget("width")) - 1, y * self.grid.scale)
        for x in range(1,self.grid.width):
            self.canvas.create_line(x * self.grid.scale, 0, x * self.grid.scale, int(self.canvas.cget("height")) - 1)

    def drawOccubpied(self):
        for block in self.occupied:
            self.colorCell(block, '#222222')

    def drawMarkers(self):
        for marker in self.markers:
            marker_x, marker_y, marker_h = parse_marker_info(marker[0], marker[1], marker[2]);

            arrow_head_x, arrow_head_y = rotate_point(0.8, 0, marker_h)
            self.colorLine((marker_x, marker_y), (marker_x + arrow_head_x, marker_y + arrow_head_y), \
                linewidth=2, color='#222222')
            c1x, c1y = rotate_point(0.2, -0.5, marker_h)
            c2x, c2y = rotate_point(0, 0.5, marker_h)
            self.colorRectangle((marker_x+c1x, marker_y+c1y), (marker_x+c2x, marker_y+c2y), '#00FFFF')

    def drawCameraImage(self):
        if self.camera_image:
            self.camera_canvas.image = ImageTk.PhotoImage(self.camera_image)
            self.camera_canvas.create_image(0, 0, anchor=NW, image=self.camera_canvas.image)

    def weight_to_color(self, weight):
        return "#%02x00%02x" % (int(weight * 255), int((1 - weight) * 255))

    def _show_mean(self, x, y, heading_deg, confident=False):
        if confident:
            color = "#00AA00"
        else:
            color = "#CCCCCC"
        location = (x,y)
        self.colorTriangle(location, heading_deg, color,tri_size=20)


    def _show_particles(self, particles):
        plot_cnt = PARTICLE_MAX_SHOW if len(particles) > PARTICLE_MAX_SHOW else len(particles)
        draw_skip = len(particles)/plot_cnt
        line_length = 0.3

        idx = 0
        while idx < len(particles):
            p = particles[int(idx)]
            coord = (p.x,p.y)
            # print((p.x,p.y))
            self.colorCircle(coord, '#FF0000', 2)
            ldx, ldy = rotate_point(line_length, 0, p.h)
            self.colorLine(coord, (coord[0]+ldx, coord[1]+ldy))
            idx += draw_skip

    def _show_robot(self, robot):
        coord = (robot.x, robot.y)
        self.colorTriangle(coord, robot.h, '#FF0000', tri_size=15)
        # plot fov
        fov_lx, fov_ly = rotate_point(8, 0, robot.h + ROBOT_CAMERA_FOV_DEG / 2)
        fov_rx, fov_ry = rotate_point(8, 0, robot.h - ROBOT_CAMERA_FOV_DEG / 2)
        self.colorLine(coord, (coord[0]+fov_lx, coord[1]+fov_ly), color='#222222', linewidth=2, dashed=True)
        self.colorLine(coord, (coord[0]+fov_rx, coord[1]+fov_ry), color='#222222', linewidth=2, dashed=True)

    def clean_world(self):
        #for eachparticle in self.dots:
        #    self.canvas.delete(eachparticle)
        self.canvas.delete("all")
        self.drawGrid()
        self.drawOccubpied()
        self.drawMarkers()

        if self.show_camera:
            self.drawCameraImage()

    """
    plot utils
    """

    # Draw a colored square at the specified grid coordinates
    def colorCell(self, location, color):
        coords = (location[0]*self.grid.scale, (self.height-location[1]-1)*self.grid.scale)
        self.canvas.create_rectangle(coords[0], coords[1], coords[0] + self.grid.scale, coords[1] + self.grid.scale, fill=color)

    def colorRectangle(self, corner1, corner2, color):
        coords1 =  (corner1[0]*self.grid.scale, (self.height-corner1[1])*self.grid.scale)
        coords2 =  (corner2[0]*self.grid.scale, (self.height-corner2[1])*self.grid.scale)
        self.canvas.create_rectangle(coords1[0], coords1[1], coords2[0], coords2[1], fill=color)

    def colorCircle(self,location, color, dot_size = 5):
        x0, y0 = location[0]*self.grid.scale - dot_size, (self.height-location[1])*self.grid.scale - dot_size
        x1, y1 = location[0]*self.grid.scale + dot_size, (self.height-location[1])*self.grid.scale + dot_size
        # print(x0,y0,x1,y1)
        return self.canvas.create_oval(x0, y0, x1, y1, fill=color)

    def colorLine(self, coord1, coord2, color='black', linewidth=1, dashed=False):
        if dashed:
            self.canvas.create_line(coord1[0] * self.grid.scale, (self.height-coord1[1])* self.grid.scale, \
                coord2[0] * self.grid.scale, (self.height-coord2[1]) * self.grid.scale,  \
                fill=color, width=linewidth, dash=(5,3))
        else:
            self.canvas.create_line(coord1[0] * self.grid.scale, (self.height-coord1[1])* self.grid.scale, \
                coord2[0] * self.grid.scale, (self.height-coord2[1]) * self.grid.scale,  \
                fill=color, width=linewidth)

    def colorTriangle(self, location, heading_deg, color, tri_size):
        hx, hy = rotate_point(tri_size, 0, heading_deg)
        lx, ly = rotate_point(-tri_size, tri_size, heading_deg)
        rx, ry = rotate_point(-tri_size, -tri_size, heading_deg)
        # reverse Y here since input to row, not Y
        hrot = (hx + location[0]*self.grid.scale, -hy + (self.height-location[1])*self.grid.scale)
        lrot = (lx + location[0]*self.grid.scale, -ly + (self.height-location[1])*self.grid.scale)
        rrot = (rx + location[0]*self.grid.scale, -ry + (self.height-location[1])*self.grid.scale)
        return self.canvas.create_polygon(hrot[0], hrot[1], lrot[0], lrot[1], rrot[0], rrot[1], \
            fill=color, outline='#000000',width=1)

    """
    Sync data to plot from other thread
    """
    def show_mean(self, x, y, heading_deg, confident=False):
        self.lock.acquire()
        self.mean_x = x
        self.mean_y = y
        self.mean_heading = heading_deg
        self.mean_confident = confident
        self.lock.release()

    def show_particles(self, particles):
        self.lock.acquire()
        self.particles = copy.deepcopy(particles)
        self.lock.release()

    def show_robot(self, robot):
        self.lock.acquire()
        self.robot = copy.deepcopy(robot)
        self.lock.release()

    def show_camera_image(self, image):
        self.lock.acquire()
        self.camera_image = image.resize((self.grid.width * self.grid.scale, self.grid.height * self.grid.scale))
        self.lock.release()

    def setupdate(self):
        self.updateflag = True

    def update(self):
        self.lock.acquire()
        self.clean_world()
        self._show_particles(self.particles)
        self._show_mean(self.mean_x, self.mean_y, self.mean_heading, self.mean_confident)
        if self.robot != None:
            self._show_robot(self.robot)
            # time.sleep(0.05)
        self.updated.clear()
        self.lock.release()
        # self.updateflag = False

    # start GUI thread
    def start(self):
        master = Tk()
        master.wm_title("Particle Filter: Grey/Green - estimated, Red - ground truth")

        self.canvas = Canvas(master, width = self.grid.width * self.grid.scale, height = self.grid.height * self.grid.scale, bd = 0, bg = '#FFFFFF')
        self.canvas.pack(side=LEFT)

        if self.show_camera:
            self.camera_canvas = Canvas(master, width=self.grid.width*self.grid.scale, height=self.grid.height*self.grid.scale, bd=0, bg='#ffffff')
            self.camera_canvas.pack(side=LEFT)
            self.drawCameraImage()

        self.drawGrid()
        self.drawOccubpied()
        self.drawMarkers()

        # Start mainloop and indicate that it is running
        self.running.set()
        while True:
            self.updated.wait()
            if self.updated.is_set():
                self.update()
            try:
                master.update_idletasks()
                master.update()
            except TclError:
                break

        # Indicate that main loop has finished
        self.running.clear()

class Visualizer():
    """Visualizer to display status of an associated CozMap instance
    """

    def __init__(self, cmap):
        self.cmap = cmap
        self.running = threading.Event()

    def draw_cmap(self):
        """Draw cmap lines
        """
        self.canvas.create_rectangle(0, 0, self.cmap.width, self.cmap.height)

    def draw_color_square(self, coord, color, size=20, bg=False, tags=''):
        """Draw a colored square centered at a given coord

            Arguments:
            coord -- coordinates of square
            color -- desired color, hexadecimal string (e.g.: '#C0FFEE')
            size -- size, in pixels
            bg -- draw square in background, default False
            tags -- tags to apply to square, list of strings or string
        """
        coords = (coord[0], (self.cmap.height - 1 - coord[1]))
        rect = self.canvas.create_rectangle(coords[0] - size / 2, coords[1] - size / 2, coords[0] + size / 2,
                                            coords[1] + size / 2,
                                            fill=color, tags=tags)
        if bg:
            self.canvas.tag_lower(rect)

    def draw_color_circle(self, coord, color, size=5, bg=False, tags=''):
        """Draw a colored circle centered at a given coord

            Arguments:
            coord -- coordinates of square
            color -- desired color, hexadecimal string (e.g.: '#C0FFEE')
            size -- size, in pixels
            bg -- draw square in background, default False
            tags -- tags to apply to square, list of strings or string
        """
        coords = (coord[0], (self.cmap.height - 1 - coord[1]))
        rect = self.canvas.create_oval(coords[0] - size / 2.0, coords[1] - size / 2.0, coords[0] + size / 2,
                                       coords[1] + size / 2,
                                       fill=color, tags=tags)
        if bg:
            self.canvas.tag_lower(rect)

    def draw_color_poly(self, coords, color, bg=False, tags=''):
        """Draw a colored polygon at a given coord

            Arguments:
            coords -- coordinates of vertices
            color -- desired color, hexadecimal string (e.g.: '#C0FFEE')
            bg -- draw square in background, default False
            tags -- tags to apply to square, list of strings or string
        """
        coords_flipped = [(coord[0], (self.cmap.height - 1 - coord[1])) for coord in coords]
        rect = self.canvas.create_polygon(coords_flipped, fill=color, tags=tags)
        if bg:
            self.canvas.tag_lower(rect)

    def draw_edge(self, start, end, color, width=1.5, tags=''):
        """Draw an edge segment between two cells

            Arguments:
            start -- starting coordinate
            end -- end coordinate
            color -- desired color, hexadecimal string (e.g.: '#C0FFEE')
            width -- desired width of edge
            tags -- tags to apply to edge
        """
        startcoords = ((start[0] + 0.5), (self.cmap.height - (start[1] + 0.5)))
        endcoords = ((end[0] + 0.5), (self.cmap.height - (end[1] + 0.5)))
        self.canvas.create_line(startcoords[0], startcoords[1], endcoords[0], endcoords[1], fill=color, width=width,
                                arrow=LAST, tags=tags)

    def draw_start(self):
        """Redraw start square
            Color is green by default
        """
        self.canvas.delete('start')
        if self.cmap._start != None:
            self.draw_color_circle(self.cmap._start, '#00DD00', size=15, bg=True, tags='start')

    def draw_goals(self):
        """Redraw all goal cells
            Color is blue by default
        """
        self.canvas.delete('goal')
        for goal in self.cmap._goals:
            self.draw_color_circle(goal, '#0000DD', size=15, bg=True, tags='goal')

    def draw_obstacles(self):
        """Redraw all obstacles
            Color is dark gray by default
        """
        self.canvas.delete('obstacle')
        for obstacle in self.cmap._obstacles:
            self.draw_color_poly(obstacle, '#222222', bg=True, tags='obstacle')

    def draw_nodes(self):
        """"Redraw all nodes, these nodes are in RRT
        """
        self.canvas.delete('nodes')
        for node in self.cmap._nodes:
            self.draw_color_circle(node, '#CCCCCC', bg=True, tags='nodes')

    def draw_node_path(self):
        """"Redraw all node paths
        """
        self.canvas.delete('node_paths')
        for node_path in self.cmap._node_paths:
            self.draw_edge(node_path[0], node_path[1], color='#DD0000', width=2, tags='node_paths')

    def draw_solution(self):
        """"Redraw one solution from start to goal
        """
        path = self.cmap.get_smooth_path()
        for p in range(len(path)-1):
            self.draw_edge(path[p], path[p+1], color='#DDDD00', width=5, tags='smoothed')

    def erase_solution(self):
        self.canvas.delete('smoothed')

    def update(self, *args):
        """Redraw any updated cmap elements
        """

        self.cmap.lock.acquire()
        self.running.clear()
        self.cmap.updated.clear()

        if 'start' in self.cmap.changes:
            self.draw_start()
        if 'goals' in self.cmap.changes:
            self.draw_goals()
        if 'obstacles' in self.cmap.changes:
            self.draw_obstacles()
        if 'nodes' in self.cmap.changes:
            self.draw_nodes()
            self.erase_solution()
        if 'node_paths' in self.cmap.changes:
            self.draw_node_path()
            self.erase_solution()
        if 'smoothed' in self.cmap.changes:
            self.draw_solution()

        self.cmap.changes = []
        self.running.set()
        self.cmap.lock.release()

    def setup(self):
        """Do initial drawing of cmap, start, goals, and obstacles
        """
        self.cmap.lock.acquire()
        self.draw_cmap()
        self.draw_start()
        self.draw_goals()
        self.draw_obstacles()
        self.cmap.lock.release()

    def start(self):
        """Start the visualizer, must be done in main thread to avoid issues on macs
            Blocks until spawned window is closed
        """
        # Set up Tk stuff
        master = Tk()
        master.title("CS 3630 Lab 4 RRT")
        self.canvas = Canvas(master, width=self.cmap.width, height=self.cmap.height, bd=0, bg='#FFFFFF')
        self.canvas.pack()

        # Draw cmap and any initial items
        self.setup()

        # Start mainloop and indicate that it is running
        self.running.set()

        while True:
            if self.cmap.updated.is_set():
                self.update()
            try:
                master.update_idletasks()
                master.update()
            except TclError:
                break

        # Indicate that main loop has finished
        self.running.clear()

