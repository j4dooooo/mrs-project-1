import numpy as np

class Roaming:

    def __init__(self, max_speed, slowing_speed, slowing_distance) -> None:
        """
        This function initializes the Roaming class.
        """

        self.max_speed = max_speed
        self.slowdown_speed = slowing_speed
        self.slowing_distance = slowing_distance

    ################################ SEEK ROAMING ################################
    def seek(self, position: np.array, target: np.array, velocity: np.array) -> np.array:
        """
        This function returns the steering velocity to reach the target using the seek algorithm.
        
        :param position: The current position of the robots.
        :param target: The target position.
        :param velocity: The current velocity of the robots.
        :return: The steering velocity.
        """
        
        # vector offset between the target and the current position of the robot
        offset = np.subtract(target, position) 
        
        # at which speed the robot should seek the target, first normalize the vector by dividing the vector by its length
        desired_velocity = (offset/np.linalg.norm(offset, axis=1)[:,None])*self.max_speed
        
        # now if we subtract the desired velocity from the robot's current velocity, we get the steering velocity
        steering_velocity = np.subtract(desired_velocity, velocity)

        return steering_velocity

    ################################ ARRIVAL ROAMING ################################
    def arrival(self, position: np.array, target: np.array, velocity: np.array, slowdown_speed: float, slowing_distance: float) -> np.array:
        """
        This function returns the steering velocity to reach the target using the arrival algorithm.

        :param position: The current position of the robots.
        :param target: The target position.
        :param velocity: The current velocity of the robots.
        :param slowdown_speed: The speed at which the robot should start slowing down.
        :param slowing_distance: The distance at which the robot should start slowing down.
        :return: The steering velocity.
        """
        
        # vector offset between the target and the current position of the robot
        offset = np.subtract(target, position)
        
        # distance from the target to the robot
        distance = np.linalg.norm(offset)
        
        # if we are not very close to the target
        if distance > 3:
            
            # normalize the desired velocity vector
            desired_velocity = (offset/np.linalg.norm(offset, axis=1)[:,None])
            
            # when the slowdown speed is not zero and we are close to the target
            if slowdown_speed > 0 and distance < slowing_distance:
                # reduce the desired velocity
                desired_velocity = desired_velocity*self.max_speed*(distance/slowing_distance)
                
            # otherwise desired velocity maintains the max speed    
            else:
                desired_velocity = desired_velocity*self.max_speed
            
            # if we subtract the desired velocity from the robot's current velocity, we get the steering velocity
            steering_velocity = np.subtract(desired_velocity, velocity)
        
        # if we are close to the goal, we nullify the desired velocity 
        else:
            steering_velocity = np.zeros((position.shape[0], position.shape[1]))
        
        return steering_velocity