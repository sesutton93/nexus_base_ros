import numpy as np

class Odour():
    '''
    Represents a normally distributed odour plume
    '''

    def __init__(self, x, y, angle_m, angle_std, p=1, dt=1) -> None:
        self.x0 = x
        self.y0 = y
        self.angle_m = angle_m
        self.angle_std = angle_std
        self.p = p
        self.dt = dt

    def sample(self):
        self.update_angle()
        return self.get_vector()


    def update_angle(self):
        self.angle = np.random.normal(self.angle_m, self.angle_std)
        
    def get_vector(self, angle=None):
        if angle is None:
            angle = self.angle
        self.lx = 300
        self.ly = 0
        rad = (angle/360)*2*np.pi
        self.x1 = self.x0 + np.cos(rad)*self.lx - np.sin(rad)*self.ly
        self.y1 = self.y0 + np.sin(rad)*self.lx + np.cos(rad)*self.ly
        return ((self.x0, self.y0), (self.x1, self.y1))
    
    def check_proximity(self, xc, yc, radius=1):
        '''
        Check if a point is within a given radial proximity of the odour source
        Args:
            xc:
            yc:
            radius:

        Returns: Boolean

        '''
        return ((self.x - xc) ** 2 + (self.y - yc) ** 2) < radius ** 2
