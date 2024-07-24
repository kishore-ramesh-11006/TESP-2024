import os
import time
import socket
import pickle
import threading
import numpy as np
import mujoco
import mujoco.viewer
import keyboard

class Main:

    def __init__(self):
        self.data = None
        self.start_time = None
        self.turn_left_done = False
        self.turn_right_done = False
        self.turn_start_time = None
        self.turn_duration = 5  # Duration for turning (in seconds)
        self.turn_direction = None  # 'left' or 'right'
        self.start_time = time.time()
        self.theta = 0
        self.prev_time = time.time()
  
    def run(self):
        script_dir = os.path.dirname(os.path.abspath(__file__))
        model = mujoco.MjModel.from_xml_path(f"{script_dir}/src/scene.xml")
        data = mujoco.MjData(model)
        

        with mujoco.viewer.launch_passive(model, data) as viewer:
            viewer.cam.type = 1
            viewer.cam.trackbodyid = 0
            viewer.cam.distance = 2

            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(2)
             
            self.prev_time = time.time()
            while viewer.is_running():

                #get the value of th keyBoard buttons that are pressed
                step_start = time.time()
                #if the up arrow key is pressed, the snake will move forward
                while self.data != None:
                    while self.data[0]=='forward':
                        print(self.data)
                        self.turn_direction = None
                        data.ctrl[:] = self.get_target_q(self.data[1])
                        self.prev_time = time.time()
                        mujoco.mj_step(model, data)
                        viewer.sync()
                        time_until_next_step = model.opt.timestep - (time.time() - step_start)
                        if time_until_next_step > 0:
                            time.sleep(time_until_next_step)
                    #if the left arrow key is pressed, the snake will turn left
                    while self.data[0]=='left':
                        print(self.data)
                        self.turn_direction = 'left'
                        data.ctrl[:] = self.get_target_q(self.data[1])
                        self.prev_time = time.time()
                        mujoco.mj_step(model, data)
                        viewer.sync()
                        time_until_next_step = model.opt.timestep - (time.time() - step_start)
                        if time_until_next_step > 0:
                            time.sleep(time_until_next_step)

                    #if the right arrow key is pressed, the snake will turn right
                    while self.data[0]=='right':
                        print(self.data)
                        self.turn_direction = 'right'
                        data.ctrl[:] = self.get_target_q(self.data[1])
                        self.prev_time = time.time()
                        mujoco.mj_step(model, data)
                        viewer.sync()
                        time_until_next_step = model.opt.timestep - (time.time() - step_start)
                        if time_until_next_step > 0:
                            time.sleep(time_until_next_step)
                
                #if it returns none, the snake will stop
                while self.data=='none':
                    print(self.data)
                    viewer.sync()
                    time_until_next_step = model.opt.timestep - (time.time() - step_start)
                    if time_until_next_step > 0:
                        time.sleep(time_until_next_step)
                '''
                #create three while loops, the first one lasts 10 seconds, the second one lasts 5 seconds, and the third one lasts 5 seconds
                #the first one is the initial movement, the second one is the left turn, and the third one is the right turn
                #the first while loop is the initial movement
                step_start = time.time()
                self.turn_direction = None
                while time.time() - self.start_time < 20:
                    data.ctrl[:] = self.get_target_q()
                    mujoco.mj_step(model, data)
                    viewer.sync()

                    time_until_next_step = model.opt.timestep - (time.time() - step_start)
                    if time_until_next_step > 0:
                        time.sleep(time_until_next_step)
                #the second while loop is the left turn
                self.turn_direction = 'left'
                while time.time() - self.start_time < 25:
                    data.ctrl[:] = self.get_target_q()
                    mujoco.mj_step(model, data)
                    viewer.sync()

                    time_until_next_step = model.opt.timestep - (time.time() - step_start)
                    if time_until_next_step > 0:
                        time.sleep(time_until_next_step)
                #the third while loop is the right turn
                self.turn_direction = 'right'
                while time.time() - self.start_time < 30:
                    data.ctrl[:] = self.get_target_q()
                    mujoco.mj_step(model, data)
                    viewer.sync()

                    time_until_next_step = model.opt.timestep - (time.time() - step_start)
                    if time_until_next_step > 0:
                        time.sleep(time_until_next_step)
                    '''
            


    def get_target_q(self,f=1):
        elapsed_time = time.time() - self.prev_time
        if self.turn_direction == 'left':
            return self.move_sinusoidally(elapsed_time,offset=0.1,frequency=f)
        elif self.turn_direction == 'right':
            return self.move_sinusoidally(elapsed_time,offset=-0.1,frequency=f)
        else:
            return self.move_sinusoidally(elapsed_time,frequency=f)


    def move_sinusoidally(self, elapsed_time,offset=0,amplitude=1,frequency=np.pi,phase=np.pi/6):
        # Generate sinusoidal movement pattern
        q = np.zeros(12)
        self.theta += 2* frequency * elapsed_time
        for i in range(12):
            q[i] = amplitude *  np.sin(self.theta + i * phase)+ offset
        return q


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

