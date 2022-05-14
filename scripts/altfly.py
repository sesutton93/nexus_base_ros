# A sketch of a simplified fly model
import numpy as np
from helpers import crosses
np.random.seed(42)
import time

class Fly():
    '''
    A fly that is modelled as a set of states with transition probabilities between them that might me more amenable to robots
    
                  l_s_w                 l_w_t
     +----------------------+  +-----------------+
     |                      |  |                 |
     |                      v  |                 v
+----+-------+       +---------+---+    +--------------+
| stopped (0)|       | walking (1) |    | turning (2)  |
+------------+       +------+------+    +-------+------+
     ^                      |  ^                |
     |                      |  |                |
     +----------------------+  +----------------+
              l_w_s                   l_t_w
    
    The state transitions are defined by probabilities in the transition matrix that are updated by sensing and time
    state: integer from {0:'stopped', 1:'walking, 2:'turning'}
    '''

    def __init__(self, x=0, y=0, vx=1, vy=1, phi_abs_m=30, phi_abs_std=10, p_angle=0.5, timestep=0.001):

        self.timestep = timestep  # The timestep in seconds

        self.x, self.y = x, y # The position of the fly
        self.x_history, self.y_history = [], []
        self.x_history.append(self.x)
        self.y_history.append(self.y)

        self.vx = vx #x velocity
        self.vy = vy #y velocity
        self.angle = 0

        self.phi_abs_m = phi_abs_m
        self.phi_abs_std = phi_abs_std
        self.p_angle = p_angle


        self.state = 0  # starts stopped
        self.tm = np.zeros((3, 3))  # transition matrix
        self.setup_tm() # Sets up the transition matrix
        self.turn_distribution = None

        self.satisfied=False #Has the Fly has found the odourant


    def setup_tm(self, l_s_w=1000, l_w_t=100, l_w_s=10):
        l_t_w = 1/self.timestep
        # Convert Poisson rates to probabilities
        self.tm[0, 1] = l_s_w * self.timestep
        self.tm[1, 0] = l_w_s * self.timestep
        self.tm[1, 2] = l_w_t * self.timestep
        self.tm[2, 1] = l_t_w * self.timestep
        self.tm[2, 2] = 0 # Never stay in the turning state            pass
        self.tm[0, 0] = 1 - np.sum(self.tm[0, :])
        self.tm[1, 1] = 1 - np.sum(self.tm[1, :])
        assert np.sum(self.tm) == self.tm.shape[0]

    def set_satisfied(self):
        self.satisfied = True #The fly has found the odourant

    def setup_turn_distribution(self):
        # TODO set up turn distribution

        pass

    def update_turn_distribution_on_whiff(self):
        # Do everything that is needed to update the turn distribution on a whiff
        #TODO implement this
        pass

    def update_turn_distribution_on_timestep(self):
        # Do everything that is needed to update the turn distribution on a timestep
        #TODO implement this
        pass

    def turn(self):
        #Draw from the turn distribution
        self.angle = self.draw_from_turn_distribution()

    def check_encounter(self, odour):
        fly_vector = ((self.x_last, self.y_last), (self.x, self.y))
        encounter = crosses(fly_vector, odour.sample())
        return encounter

    def update_state(self):
        # Rolls the die to find the next state
        self.state = np.random.choice(np.arange(3), size=1, p=self.tm[self.state,:])[0]

    def _get_angle_to_wind(self):
        #TODO I feel that there might be a more efficient way of doing this
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

    def draw_from_turn_distribution(self, size=1):

        magnitude_abs = np.random.normal(self.phi_abs_m, self.phi_abs_std, size=size)
        direction_binary = np.random.choice([1,-1], p=[self.p_angle, 1-self.p_angle])
        angle = magnitude_abs * direction_binary
        return angle


    def turn(self):
        # Turn the fly
        self.angle += self.draw_from_turn_distribution()

    def move(self):
        self.x = self.x + (np.cos(np.radians(self.angle))*self.vx * self.timestep)
        self.y = self.y + (np.sin(np.radians(self.angle))*self.vy * self.timestep)
        self.x_history.append(self.x)
        self.y_history.append(self.y)

    def wait(self):
        time.sleep(self.timestep)

    def step(self):

        self.update_turn_distribution_on_timestep()
        # if self.check_encounter(odour):
        #     self.update_turn_distribution_on_whiff()
        self.update_state()

        print(self.state)

        if self.state == 0:
            self.wait()
        if self.state == 1:
            self.move()
        if self.state == 2:
            self.turn()


if __name__=='__main__':

    #TODO move this all into SimulationEnvironment

    import matplotlib.pyplot as plt


    def setup_plot(fly):
        fig = plt.figure()
        ax = fig.add_subplot(111)
        ax.set_xlim((-0.5, 0.5))
        ax.set_ylim((-0.5, 0.5))
        point, = ax.plot(fly.x, fly.y, 'o')
        line, = ax.plot(fly.x_history, fly.y_history)
        plt.ion()
        plt.show()
        return fig, point, line


    def update_plot(fig, point, line, fly):
        point.set_data(fly.x, fly.y)
        line.set_data(fly.x_history, fly.y_history)
        fig.canvas.draw()
        fig.canvas.flush_events()


    fly = Fly()

    fig, point, line = setup_plot(fly)

    for i in range(10000):
        fly.step()
        update_plot(fig, point, line, fly)



