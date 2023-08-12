from controller import Robot

robot = Robot()

if __name__ == "__main__":

  # Get the hinge joint
  joint_name4 = "RM4"
  joint_name5 = "RM5"
  joint_name6 = "RM6"
  

  joint4 = robot.getDevice(joint_name4)
  joint5 = robot.getDevice(joint_name5)
  joint6 = robot.getDevice(joint_name6)

  # Rotate the joint
  joint4.setPosition(float('inf'))
  joint5.setPosition(float('inf'))
  joint6.setPosition(float('inf'))
