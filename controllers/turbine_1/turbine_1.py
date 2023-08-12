from controller import Robot

robot = Robot()

if __name__ == "__main__":

  # Get the hinge joint
  joint_name1 = "RM1"
  joint_name2 = "RM2"
  joint_name3 = "RM3"
  

  joint1 = robot.getDevice(joint_name1)
  joint2 = robot.getDevice(joint_name2)
  joint3 = robot.getDevice(joint_name3)

  # Rotate the joint
  joint1.setPosition(float('inf'))
  joint2.setPosition(float('inf'))
  joint3.setPosition(float('inf'))
