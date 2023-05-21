"""obtacle_avoidance controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, DistanceSensor, Motor

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)
# initialize devices
ds = []
dsNames = ['ds0', 'ds1', 'ds2', 'ds3', 'ds4', 'ds5', 'ds6', 'ds7']
for i in range(8):
    ds.append(robot.getDistanceSensor(dsNames[i]))
    ds[i].enable(timestep)
    
motorLeft = robot.getMotor('left wheel motor')
motorRight = robot.getMotor('right wheel motor')

motorLeft.setPosition(float('inf'))
motorRight.setPosition(float('inf'))

motorLeft.setVelocity(0.0)
motorRight.setVelocity(0.0)

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()
     # read sensors outputs
    dsValues = []
    for i in range(8):
        dsValues.append(ds[i].getValue())
        
    # initialize motor speeds at 50% of MAX_SPEED.
    leftSpeed = 0.5 * MAX_SPEED
    rightSpeed = 0.5 * MAX_SPEED
    
    # modify speeds according to obstacles
    for i in range(8):
        if dsValues[i] < 900.0:
            # when an obstacle is detected
            # the speed of the corresponding wheel is reduced
            if i < 3:
                # obstacle on the left, turn right
                leftSpeed = 0.5 * MAX_SPEED
                rightSpeed = -0.5 * MAX_SPEED
            elif i > 4:
                # obstacle on the right, turn left
                leftSpeed = -0.5 * MAX_SPEED
                rightSpeed = 0.5 * MAX_SPEED
            break

    # set wheel speeds
    motorLeft.setVelocity(leftSpeed)
    motorRight.setVelocity(rightSpeed)

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass

# Enter here exit cleanup code.
