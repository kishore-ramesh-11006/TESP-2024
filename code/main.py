import subprocess
import time

def run_script_in_new_terminal(script_name):
    subprocess.Popen(['start', 'cmd', '/k', f'python {script_name}'], shell=True)

# Run the first script in a new terminal
run_script_in_new_terminal('mujoco_snake.py')

# Wait for 5 seconds
time.sleep(5)

# Run the second script in a new terminal
run_script_in_new_terminal('mediapipe_pose.py')

