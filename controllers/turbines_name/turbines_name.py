from controller import Supervisor
import sys
from pathlib import Path


file_path = Path(__file__).parents[2]

file_content = open(str(file_path)+"/turbines.txt", "r").read()


TIME_STEP = 32

supervisor = Supervisor()


coordinate_list = []

for i in range(int(file_content)):
    turbine_name = "turbine"+str(i+1)
    robot_node = supervisor.getFromDef(turbine_name)
    trans_field = robot_node.getField("translation")
    
    values = trans_field.getSFVec3f()
    coordinate_list.append(values[0:2:])

waypoint1 = [[i+2, j+2] for i, j in coordinate_list]
waypoint1.insert(len(coordinate_list),[26, 30])
waypoint1.insert(0,[26, 30])


waypoint2 = [[i-2, j-2] for i, j in coordinate_list]
waypoint2.insert(len(coordinate_list),[26, -24])
waypoint2.insert(0,[26, -24])


file1 = open(str(file_path)+"/mavic1.txt", "w").write(str(waypoint1))

file2 = open(str(file_path)+"/mavic2.txt", "w").write(str(waypoint2))