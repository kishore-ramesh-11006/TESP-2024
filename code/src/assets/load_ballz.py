import os
import time
import socket
import pickle
import threading
import numpy as np
import mujoco
import mujoco.viewer
import keyboard
from PIL import Image
from scipy import ndimage

class Main:
    def __init__(self):
        # Existing initialization code...

        self.maze_image = Image.open("Maze_123.png").convert("L")
        self.maze_array = np.array(self.maze_image)
        self.path_positions = self.get_path_centerline()
        self.spawned_objects = []

    def get_path_centerline(self):
        # Invert the image so that the paths are white (1) and walls are black (0)
        inverted_maze = 1 - (self.maze_array // 255)
        
        # Use distance transform to get the distance of each path pixel to the nearest wall
        distance = ndimage.distance_transform_edt(inverted_maze)
        
        # Use the peak_local_max function to find local maxima, which will be the centerline
        from scipy.ndimage import maximum_filter
        neighborhood_size = 10  # Adjust this value based on your maze's path width
        local_max = maximum_filter(distance, footprint=np.ones((neighborhood_size, neighborhood_size))) == distance
        
        # Get the coordinates of the centerline pixels
        centerline = np.argwhere(local_max & (distance > 0))
        
        return centerline

    def spawn_object(self, object_name):
        if not self.path_positions.size:
            print("No available positions to spawn objects.")
            return

        spawn_index = np.random.randint(0, len(self.path_positions))
        y, x = self.path_positions[spawn_index]
        
        # Convert image coordinates to MuJoCo coordinates
        mj_x = x / self.maze_image.width * 2 - 1
        mj_y = 1 - y / self.maze_image.height * 2

        new_body = mujoco.MjModel.body_add(self.model, self.model.body('spawned_objects').id)
        mujoco.MjModel.body_pos(self.model)[new_body.id] = [mj_x, mj_y, 0.1]  # Adjust z-coordinate as needed
        
        new_geom = mujoco.MjModel.geom_add(self.model, self.model.geom(object_name).id)
        mujoco.MjModel.geom_bodyid(self.model)[new_geom.id] = new_body.id

        self.spawned_objects.append((new_body.id, new_geom.id))
        print(f"Spawned {object_name} at ({mj_x}, {mj_y})")

    def remove_object(self):
        if not self.spawned_objects:
            print("No objects to remove.")
            return

        body_id, geom_id = self.spawned_objects.pop()
        mujoco.MjModel.body_remove(self.model, body_id)
        mujoco.MjModel.geom_remove(self.model, geom_id)
        print(f"Removed object (body_id: {body_id}, geom_id: {geom_id})")

    def run(self):
        # Existing run method code...

        with mujoco.viewer.launch_passive(self.model, data) as viewer:
            # Existing viewer setup...

            while viewer.is_running():
                # Existing loop code...

                if keyboard.is_pressed('s'):
                    self.spawn_object('red_ball')
                elif keyboard.is_pressed('r'):
                    self.remove_object()

                # Rest of the existing loop...

# Rest of the existing code...
