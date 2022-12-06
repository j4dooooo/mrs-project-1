import numpy as np

class ObstacleAvoidance:

    def __init__(self, r, distance=0.2, fov=np.pi/4, max_see_ahead=0.5):
        # initialize 2D array of integers which categorizes world occupancy
        self.map = None 
        # map sampling resolution, size of cell                          
        self.resolution = None
        # world position of cell (0, 0) in map
        self.origin = None
        self.there_is_map = False
        self.num_robots = r
        # radius around the robot to check for the occupancy of a given position              
        self.distance = distance                    
        self.ahead = np.zeros((r, 2))
        self.ahead_l = np.zeros((r, 2))
        self.ahead_r = np.zeros((r, 2))
        self.fov = np.deg2rad(fov)
        self.max_see_ahead = max_see_ahead
        self.obstacle_center = np.zeros((r, 2))
        self.avoidance_vector = np.zeros((r, 2))

    # set occupancy map, its resolution, and origin
    def set_map(self, data, resolution, origin):
        self.map = data
        self.resolution = resolution
        self.origin = np.array(origin)
        self.there_is_map = True

        # map output
        print("Map set:", self.map.shape)
        print(self.map[-2, -2])

    def look_ahead(self, pose, vel):
        """
        Given a pose and a velocity, predicts the position of the robot ahead and checks if it is obstacle free.        
        Args:
        - pose (np.array): 2D poses of the robots.
        - vel (np.array): 2D velocities of the robots.
        """

        # reset vector to zero
        self.avoidance_vector = np.zeros((self.num_robots, 2))

        # look ahead for each robot
        self.ahead[:,0] = pose[:,0] + vel[:,0]*self.max_see_ahead
        self.ahead[:,1] = pose[:,1] + vel[:,1]*self.max_see_ahead

        # line vector pointing to the front of the robot
        L = self.ahead - pose

        # get length of line for each robot
        L_norm = np.linalg.norm(self.ahead - pose, axis=1)

        # angle between the line vector and the x-axis
        alpha = np.arctan2(L[:,1], L[:,0])

        # rotate the line vector by the field of view
        self.ahead_l[:,0] = pose[:,0] + L_norm*np.cos(alpha + self.fov)
        self.ahead_l[:,1] = pose[:,1] + L_norm*np.sin(alpha + self.fov)

        # rotate the line vector by the negative of the field of view
        self.ahead_r[:,0] = pose[:,0] + L_norm*np.cos(alpha - self.fov)   
        self.ahead_r[:,1] = pose[:,1] + L_norm*np.sin(alpha - self.fov)

        # if map is present
        if self.there_is_map:

            for i in range(0, self.num_robots):

                if not self.is_valid(self.ahead[i,:]) or not self.is_valid(self.ahead_l[i,:]) or not self.is_valid(self.ahead_r[i,:]):

                    if not self.is_valid(self.ahead[i,:]) and not self.is_valid(self.ahead_l[i,:]) and not self.is_valid(self.ahead_r[i,:]):
                        self.avoidance_vector[i,0] =  pose[i,0] - self.ahead[i,0]
                        self.avoidance_vector[i,1] =  pose[i,1] - self.ahead[i,1]
                        self.avoidance_vector[i,:] = self.avoidance_vector[i,:]/np.linalg.norm(self.avoidance_vector[i,:]) if np.linalg.norm(self.avoidance_vector[i,:])!=0 else self.avoidance_vector[i,:]

                    elif not self.is_valid(self.ahead_l[i,:]) and self.is_valid(self.ahead_r[i,:]):
                        self.avoidance_vector[i,0] =  self.ahead_r[i,0] - self.ahead_l[i,0]
                        self.avoidance_vector[i,1] =  self.ahead_r[i,1] - self.ahead_l[i,1]
                        self.avoidance_vector[i,:] = self.avoidance_vector[i,:]/np.linalg.norm(self.avoidance_vector[i,:])  if np.linalg.norm(self.avoidance_vector[i,:])!=0 else self.avoidance_vector[i,:]

                    elif not self.is_valid(self.ahead_r[i,:]) and self.is_valid(self.ahead_l[i,:]):
                        self.avoidance_vector[i,0] =  self.ahead_l[i,0] - self.ahead_r[i,0]
                        self.avoidance_vector[i,1] =  self.ahead_l[i,1] - self.ahead_r[i,1]
                        self.avoidance_vector[i,:] = self.avoidance_vector[i,:]/np.linalg.norm(self.avoidance_vector[i,:])  if np.linalg.norm(self.avoidance_vector[i,:])!=0 else self.avoidance_vector[i,:]

                    else:
                        self.avoidance_vector = np.zeros((self.num_robots, 2))

            return self.avoidance_vector

    # given a pose, should return True if not in collision, otherwise it should return False
    def is_valid(self, pose):

        # convert world robot coordinates to map coordinates
        p = self.__position_to_map__(np.asarray((pose[0], pose[1])))

        vicinity = int(self.distance/self.resolution)
        x_width = self.map.shape[0]
        y_length = self.map.shape[1]
        
        # if p is outside the map return true, locations outside the map are considered free
        if len(p) == 2:
            u_min = p[0] - vicinity if p[0] - vicinity > 0 else 0
            v_min = p[1] - vicinity if p[1] - vicinity > 0 else 0
            u_max = p[0] + vicinity if p[0] + vicinity < x_width - 1 else x_width
            v_max = p[1] + vicinity if p[1] + vicinity < y_length - 1 else y_length

            # obstacle
            if np.any(self.map[u_min:u_max,v_min:v_max] > 0):
                return False
            
        return True

    def __position_to_map__(self, p):
        """
        Transform position with respect to the map's origin to cell coordinates.
        Args:
        - p (np.array): 2D position with respect to the map's origin.
        """
        # convert world position to map coordinates
        uv = (p - self.origin)/self.resolution

        # keep the position inside the map
        if uv[0] < 0 or uv[0] >= self.map.shape[0] or uv[1] < 0 or uv[1] >= self.map.shape[1]:
            return []

        return uv.astype(int)
