from altfly import Fly
import numpy as np
seed_tweak = 2
np.random.seed(42+seed_tweak)
import time

class RoboFly(Fly):
    '''
    A class represent a fly model controlling an omnibot
    '''

    def __init__(self, timescale=1):
        super().__init__(timestep=3, vx=0.05, vy=0.05, p_angle=0.3)
        self.timescale = timescale
        self.state = 1
        #TODO add omnibot controller details

    def setup_tm(self):
        # This is hardcoded for expedience. A problem arises when timestep is too big, probabilities go over 1.
        self.tm[0, 1] = 0.1
        self.tm[1, 0] = 0.01
        self.tm[1, 2] = 0.95
        self.tm[2, 1] = 1
        self.tm[2, 2] = 0 # Never stay in the turning state
        self.tm[0, 0] = 1 - np.sum(self.tm[0, :])
        self.tm[1, 1] = 1 - np.sum(self.tm[1, :])
        assert np.sum(self.tm) == self.tm.shape[0]

    def move(self):

        #Just for fun, jitter the distance
        vel_x = np.random.normal(loc=self.vx, scale=0.01)
        vel_y = np.random.normal(loc=self.vy, scale=0.01)

        self.x = self.x + (np.cos(np.radians(self.angle))*vel_x * self.timestep)
        self.y = self.y + (np.sin(np.radians(self.angle))*vel_y * self.timestep)
        self.x_history.append(self.x)
        self.y_history.append(self.y)

        #TODO convert to robot-centric
        # TODO robot control

        time.sleep(self.timestep/self.timescale)

    def turn(self):
        # Turn the fly
        self.angle += self.draw_from_turn_distribution()

        #TODO convert to robot-centric
        #TODO robot control
        time.sleep(self.timestep/self.timescale)


    def wait(self):
        time.sleep(self.timestep/self.timescale)
        #pass
if __name__=='__main__':

    import matplotlib.pyplot as plt


    def setup_plot(fly):
        fig = plt.figure()
        ax = fig.add_subplot(111)
        scale = 2
        ax.set_xlim((-scale, scale))
        ax.set_ylim((-scale, scale))
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


    fly = RoboFly()

    fig, point, line = setup_plot(fly)

    for i in range(10000):
        fly.step()
        update_plot(fig, point, line, fly)