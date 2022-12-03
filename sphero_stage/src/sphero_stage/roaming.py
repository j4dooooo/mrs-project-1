import numpy as np

#################################### SEEK Algorithm #############################################

def seek(position: np.array, target: np.array, max_speed: float, velocity: np.array) -> np.array:
    
    # Vector offset between the target and the current position of the robot
    offset = np.subtract(target, position) 
    
    # At which speed the robot should seek the target.
    desired_velocity = (offset/np.linalg.norm(offset))*max_speed  #First normlize the vector by dividing the vector by its length.
    print(desired_velocity)
    
    # Now if we subtract the desired velocity from the robot's current velocity, we will get the steering velocity.
    steering_velocity = np.subtract(desired_velocity, velocity) # 
    return steering_velocity

#################################### SEEK Algorithm #############################################


#################################### Arrive Algorithm #############################################

def arrival(position: np.array, target: np.array, max_speed: float, velocity: np.array, slowdown_speed: float, slowing_distance: float) -> np.array:
    
    # Vector offset between the target and the current position of the robot
    offset = np.subtract(target, position)
    
    #Distance from the target to the robot
    distance = np.linalg.norm(offset)
    
    # If we are not very close to the target 
    if distance > 1:
        
        # Normalize the desired velocity vector
        desired_velocity = (offset/np.linalg.norm(offset))
        
        # When the slowdown speed is not zero and we are close to the target
        if slowdown_speed > 0 and distance < slowing_distance:
            
            # We reduce the desired velocity
            desired_velocity = desired_velocity*max_speed*(distance/slowing_distance)
            
        # Otherwise desired velocity maintains the max speed    
        else:
            desired_velocity = desired_velocity*max_speed
        
        # Now if we subtract the desired velocity from the robot's current velocity, we will get the steering velocity.   
        steering_velocity = np.subtract(desired_velocity, velocity)
    
    
    # If we are close to the goal, we nullify the desired velocity 
    else:
        steering_velocity = np.zeros(2)
      
    return steering_velocity

#################################### Arrive Algorithm #############################################



#################################### Variables ####################################################

position = np.array([np.random.rand(),np.random.rand()])
target = np.array([np.random.rand(),np.random.rand()])
max_speed = 10
slowdown_speed = 1
slowing_distance = 100
velocity = np.array([1,1])

#################################### Variables ####################################################



#################################### Driver Code ##################################################

if __name__ == '__main__':
    print(seek(position, target, max_speed, velocity))
    print(arrival(position, target, max_speed, velocity, slowdown_speed, slowing_distance))
    
#################################### Driver Code ##################################################