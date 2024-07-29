import xml.etree.ElementTree as ET
import numpy as np

def make_maze_on_mujoco(load_file_path, maze, start_pos, goal_pos, box_size=[0.5, 0.5, 0.15], maze_rgba="0.5 0.5 0.5 1", goal_rgba="0.6 0.9 0.6 0.5", save_file_path=None):
  tree = ET.parse(load_file_path)
  root = tree.getroot()
  worldbody = root.find("./worldbody")

  for i in range(maze.shape[0]):
    for j in range(maze.shape[1]):
      if maze[i][j] == 0:
        continue
      else:
        pos_x = 2 * box_size[0] * (j - start_pos[0])
        pos_y = -2 * box_size[1] * (i - start_pos[1])
        name = f"box_{i}_{j}"

        body_attribs = {
          "name": name,
          "pos": f"{pos_x} {pos_y} {box_size[2]}"
        }
        body = ET.SubElement(worldbody, "body", attrib=body_attribs)
        geom_attribs = {
          "name": name,
          "type": "box",
          "size": f"{box_size[0]} {box_size[1]} {box_size[2]}",
          "rgba": maze_rgba
        }
        ET.SubElement(body, "geom", attrib=geom_attribs)

  pos_x = 2 * box_size[0] * (goal_pos[0] - start_pos[0])
  pos_y = -2 * box_size[1] * (goal_pos[1] - start_pos[1])

  body_attribs = {
    "name": "goal",
    "pos": f"{pos_x} {pos_y} {box_size[2]}"
  }
  body = ET.SubElement(worldbody, "body", attrib=body_attribs)
  geom_attribs = {
    "name": "goal",
    "type": "box",
    "size": f"{box_size[0]} {box_size[1]} {box_size[2]}",
    "contype": "0",
    "conaffinity": "0",
    "rgba": goal_rgba
  }
  ET.SubElement(body, "geom", attrib=geom_attribs)

  if save_file_path:
    tree.write(save_file_path, encoding="utf-8", xml_declaration=True)        

def is_goal(model, data):
  snake_pos = data.body("frame_0-1").xpos

  goal_pos = model.body("goal").pos
  goal_size = model.geom("goal").size

  is_within_x_range = goal_pos[0] - goal_size[0] < snake_pos[0] < goal_pos[0] + goal_size[0]
  is_within_y_range = goal_pos[1] - goal_size[1] < snake_pos[1] < goal_pos[1] + goal_size[1]

  return is_within_x_range and is_within_y_range

if __name__ == "__main__":
  load_file_path = "src/scene.xml"
  maze = np.array([
    [1, 1, 1, 1, 1, 1, 1],
    [1, 0, 0, 0, 1, 0, 1],
    [1, 1, 1, 0, 1, 0, 1],
    [1, 0, 1, 0, 1, 0, 1],
    [1, 0, 1, 0, 1, 0, 1],
    [1, 0, 0, 0, 0, 0, 1],
    [1, 1, 1, 1, 1, 1, 1]
  ])

  start_pos = [1, 1]
  goal_pos = [5, 1]

  box_size = [0.5, 0.5, 0.15]
  maze_rgba = "0.5 0.5 0.5 1"
  goal_rgba = "0.6 0.9 0.6 0.5"
  save_file_path = "scene_maze.xml"

  
  
  make_maze_on_mujoco(load_file_path, maze, start_pos, goal_pos, box_size=box_size, maze_rgba=maze_rgba, goal_rgba=goal_rgba, save_file_path=save_file_path)
