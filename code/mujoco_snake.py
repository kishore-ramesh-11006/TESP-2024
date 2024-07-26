import os
import time
import socket
import pickle
import threading
import numpy as np
import mujoco
import mujoco.viewer
import keyboard
#import the live_plot function from the live_plot.py file
from utils.live_plot import live_plot as lp
from PIL import Image
from scipy import ndimage


class Main:
    def __init__(self):
        self.data = [0,0,0]
        self.start_time = time.time()
        self.theta = 0
        self.script_dir = os.path.dirname(os.path.abspath(__file__))

        self.model= mujoco.MjModel.from_xml_path(f"{self.script_dir}/src/scene_maze.xml")
        self.timestep = self.model.opt.timestep

        self.plotting = False
        self.time_data = []
        self.omega_data = []
        self.lock = threading.Lock()

        self.maze_image = Image.open("code/src/assets/Maze_123.png").convert("L")
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
        script_dir = self.script_dir
        model = self.model
        data = mujoco.MjData(model)
        with mujoco.viewer.launch_passive(model, data) as viewer:
            viewer.cam.type = 1
            viewer.cam.trackbodyid = 0
            viewer.cam.distance = 2
        
            #make the camera look from a different angle
            viewer.cam.elevation = -90
            viewer.cam.azimuth = 0
            #zoom out 
            viewer.cam.distance += 1
            
            if keyboard.is_pressed('s'):
                self.spawn_object('red_ball')
            elif keyboard.is_pressed('r'):
                self.remove_object()

            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(2)
             
            self.prev_time = time.time()
            while viewer.is_running():

                #get the value of th keyBoard buttons that are pressed
                step_start = time.time()
                #if the up arrow key is pressed, the snake will move forward
                

                while self.data != [0,0,0]:
                    step_start = time.time()
                    if keyboard.is_pressed('i') and not self.plotting:
                        self.plotting = True
                        #self.plot_thread = threading.Thread(target=lp, args=(self.time_data, self.omega_data, self.lock))
                        #self.plot_thread.start()

                    x = self.data[0]
                    y = self.data[1]
                    data.ctrl[:] = self.get_target_q(x,y)
                    
                    mujoco.mj_step(model, data)
                    viewer.sync()
                    
                    time_until_next_step = model.opt.timestep - (time.time() - step_start)
                    if time_until_next_step > 0:
                        time.sleep(time_until_next_step) 

    
    def get_target_q(self,x,y,amp=1,alpha=0.1):
        '''
        This function returns the target joint angles for the snake robot

        Description:
        Defines the following parameters to compute the target joint angles for the snake robot:
        - theta: float, angle of the sine wave
        - omeg: float, angular velocity of the sine wave
        - B: float, offset of the sine wave
        - phi: float, phase of the sine

        Parameters:
        x: the x coordinate of the right wrist
        y: the y coordinate of the right wrist
        amp: float, amplitude of the sine wave

        Returns:
        target_q: list of floats, target joint angles for the snake robot
        '''
    
        target_q=np.zeros(12)
        is_hand_in_frame = self.data[2]
        if is_hand_in_frame:
            omeg=(6*self.data[1])-3
            B=0.3-(0.6*self.data[0]) #this linear combinations guarantees that the offset

            '''
            make th snake go faster if it is going forward and slower if it is going backwards
            
               
            This function returns the exponential linear unit (ELU) activation function
            https://paperswithcode.com/method/elu
            '''
            if omeg >= 0:
                elu = 0.1*omeg
                B=-B
            else:
                elu = alpha * (np.exp(omeg) - 1)
            omeg = omeg + elu
            

            self.theta=self.theta+omeg*self.timestep 

            for i in range(0,12):
                phi=np.pi/4
                target_q[i]=amp*np.sin(self.theta+phi*i) + B
        else:
            #stop the snake robot if the right wrist is not in the frame
            omeg=0
            self.theta=0
            B=0.3
            for i in range(0,12):
                phi=np.pi/4
                target_q[i]=amp*np.sin(self.theta+phi*i) + B
        with self.lock:
            self.omega_data.append(omeg)
            self.time_data.append(time.time() - self.start_time)
       
        return target_q

    

    def server(self, use_socket=False, ip="127.0.0.1", port=8000):
        if use_socket:
            server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            server_socket.bind((ip, port))
            server_socket.listen(1)

            client_socket, _ = server_socket.accept()

            while True:
                serialized_data = client_socket.recv(4096)
                if not serialized_data:
                    break
                self.data = pickle.loads(serialized_data)
            
            client_socket.close()
            server_socket.close()

if __name__ == "__main__":
    use_socket = True
    ip = "127.0.0.1"
    port = 8000

    main = Main()

    server_thread = threading.Thread(target=main.server, kwargs={"use_socket": use_socket, "ip": ip, "port": port})
    run_thread = threading.Thread(target=main.run)

    server_thread.start()
    run_thread.start()

    while True:
        time.sleep(1)

