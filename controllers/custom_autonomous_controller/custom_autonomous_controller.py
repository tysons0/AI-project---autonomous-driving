"""custom_autonomous_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot

# create the Robot instance.
robot = Robot()

def run_vehicle(vehicle):
    # get the time step of the current world.
    timestep = int(robot.getBasicTimeStep())
    max_speed= 10
    print("custom_autonomous_controller is running!")
    # You should insert a getDevice-like function in order to get the
    # instance of a device of the robot. Something like:
    #  motor = robot.getDevice('motorname')
    #  ds = robot.getDevice('dsname')
    #  ds.enable(timestep)
    left_motor = robot.getMotor('motor_1')
    right_motor = robot.getMotor('motor_2')
    
    left_motor.setPosition(float('int')
    right_motor.setPosition(float('int')
    
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)
    # Main loop:
    # - perform simulation steps until Webots is stopping the controller
    while robot.step(timestep) != -1:
        # Read the sensors:
        # Enter here functions to read sensor data, like:
        #  val = ds.getValue()
        left_motor.setVelocity(max_speed*.25)
        right_motor.setVelociity(max_speed*.25)
        # Process sensor data here.
    
        # Enter here functions to send actuator commands, like:
        #  motor.setPosition(10.0)
        pass


# Enter here exit cleanup code.
if __name__ == "__main__":
    my_vehicle = Robot()
    run_vehicle(my_vehicle)
    