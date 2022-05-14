from pathlib import Path
from matplotlib import interactive

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import numpy as np
np.random.seed(42)
from scipy.signal import lfilter
from scipy.stats import kde

from helpers import crosses

output_dir = Path("results")
output_dir.mkdir(parents=True, exist_ok=True)

class TurnFly():
    """
    A randomly walking fly that biases its walk by encounters, crossing between an odour vector and the fly's trajectory
    """
    def __init__(self, x=0, y=0, vx=0, vy=1, d=0, dt=1, p_t=1.0, W_f=1, tau=5., m_mean=30., m_std=10., alpha=1., ld_turn=1.3) -> None:
        self.x = x # x position
        self.y = y # y position
        self.x_last = x #  x position at previous timestep
        self.y_last = y # y position at previous timestep
        self.vx = vx # x velocity
        self.vy = vy # y velocity
        self.angle = d # direction
        self.dt = dt #
        self.W_f = W_f
        self.p_t = p_t
        self.tau = tau #time constant for moving average
        self._zi = 0
        self.ld_turn = ld_turn
        self.m_mean = m_mean
        self.m_std = m_std
        self.alpha = alpha
        self.update_p_angle()
    
    def _get_angle_to_wind(self):
        dx = self.vx# - self.x + 0.0000001
        dy = self.vy# - self.y + 0.0000001
        angle_to_wind = np.arcsin(dy/dx)
        # print(angle_to_wind)
        return angle_to_wind

    def is_left_upwind(self):
        angle_to_wind = self._get_angle_to_wind()
        # print(angle_to_wind)
        if  (angle_to_wind > 0) and (angle_to_wind <= np.pi):
            return 1
        else:
            return -1
        
    def get_angle(self, size=1):
        '''
        Gets a binomial
        Args:
            size:

        Returns:

        '''

        magnitude_abs = np.random.normal(self.m_mean, self.m_std, size=size)

        direction_binary = (np.random.binomial(n=1, p=self.p_angle,size=size)-0.5)*2*self.is_left_upwind()*-1
        if size==1:
            angle = magnitude_abs[0]*direction_binary[0]
        else:
            angle = magnitude_abs*direction_binary
            # print(self.is_left_upwind(), angle[0])
        return angle
        
    def get_n_turns(self):
        '''
        Sample number of turns from homogeneous Poisson distribution
        Returns:

        '''
        turns = np.random.poisson(self.ld_turn)
        return turns
        # self.p_t = self.p_t  # TODO
        
    def update_W_f(self, w_t):
        '''
        Updates
        Args:
            w_t:

        Returns:

        '''
        self.W_f = self.W_f + (w_t - self.W_f) / self.tau # Moving average. TODO: seems weird, check if correct
    
    def update_p_angle(self):
        '''
        Updates probability distribution
        Returns:

        '''
        # print(self.is_left_upwind())
        self.p_angle = 1./(1+np.exp(-self.alpha*self.W_f)) # TODO: remove tau if you want to be like the paper
        # print(self.p_angle)
        
    def update_angle(self, d=None):
        if d is None:
            self.angle = self.get_angle()
        else:
            self.angle = d
            
    def update_v(self):
        rad = (self.angle/360)*2*np.pi
        vx_temp = np.cos(rad)*self.vx - np.sin(rad)*self.vy
        vy_temp = np.sin(rad)*self.vx + np.cos(rad)*self.vy
        self.vx = vx_temp
        self.vy = vy_temp

    def update_pos(self):
        self.x_last = self.x
        self.y_last = self.y
        
        self.x += self.vx/self.dt
        self.y += self.vy/self.dt

    def check_encounter(self, odour_vector):
        fly_vector = ((self.x_last, self.y_last), (self.x, self.y))
        encounter = crosses(fly_vector, odour_vector)
        return encounter


