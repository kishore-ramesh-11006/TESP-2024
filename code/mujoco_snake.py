import os
import time
import socket
import pickle
import threading
import numpy as np
import mujoco
import mujoco.viewer
import math

class Main:

  def __init__(self):
    self.data = [0,0]
    self.theta=0
    self.script_dir = os.path.dirname(os.path.abspath(__file__))
    self.model = mujoco.MjModel.from_xml_path(f"{self.script_dir}/src/scene.xml")
    self.mujoco_data = mujoco.MjData(self.model)
    self.timestep = self.model.opt.timestep
  
  def run(self):
    script_dir = self.script_dir
    model = self.model
    data=self.mujoco_data    

    with mujoco.viewer.launch_passive(model, data) as viewer:
      viewer.cam.type = 1
      viewer.cam.trackbodyid = 0
      viewer.cam.distance = 2

      mujoco.mj_step(model, data)
      viewer.sync()
      time.sleep(2)
      
      while viewer.is_running():
        step_start = time.time()

        data.ctrl[:] = self.get_target_q()
        
        mujoco.mj_step(model, data)
        viewer.sync()
        
        time_until_next_step = model.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
          time.sleep(time_until_next_step)    

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

  def get_target_q(self):
    target_q=np.zeros(12)
    amp=1
    omeg=(6*self.data[1])-3
    self.theta=self.theta+omeg*self.timestep
    B=0.3-(0.6*self.data[0])
    for i in range(0,12):
      phi=math.pi/4
      target_q[i]=amp*math.sin(self.theta+phi*i) + B
    print(self.data)
    print(self.theta)
    return target_q
      

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
