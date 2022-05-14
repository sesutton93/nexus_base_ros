from pathlib import Path
from tkinter import N
from matplotlib import interactive

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import numpy as np
from scipy.signal import lfilter
from scipy.stats import kde
np.random.seed(42)


from helpers import crosses
from strategies import TurnFly
from altfly import Fly
from odour import Odour

output_dir = Path("results")
output_dir.mkdir(parents=True, exist_ok=True)

class SimulationEnvironment():
    '''
    A representation of a simulated experiment that can inlcude a Fly and an Odour
    '''

    def __int__(self, odours, flies, timestep=0.001):
        self.odours = []
        self.flies = []
        self.timestep = timestep

    def add_odour(self, odour:Odour):
        self.odours.append(odour)

    def add_fly(self, fly:Fly):
        self.flies.append(fly)

    def step(self):
        '''
        Take a time step
        Returns:

        '''
        for fly in self.flies:
            if not fly.satisfied:
                fly.step()


    def ended(self, radius=1):
        #TODO be clever and speed this up
        for fly in self.flies:
            if fly.satisfied: # Ignore flies that have found things
                continue
            for odour in self.odours: # Got through all the odours
                if odour.check_proximity(fly.x, fly,y, radius):
                    fly.set_satified()
        if all([fly.satisfied for fly in self.flies]):
            return True

    def run(self, timesteps=1000):
        #TODO could be dumped in the simulation environment
        '''
        Run for a certain number of timesteps
        Args:
            timesteps:

        Returns:

        '''

        for i in range(timesteps):
            self.step()
            if self.ended():
                break

        return self.ended(), step

class MultipleTrials():
    '''
    Run multiple trials
    '''

    def __int__(self, N_trials=1000, timesteps=10000):
        self.N_trials= N_trials
        self.timesteps = timesteps

    def run_multiple(self):
        for trial in range(self.N_trials):
            print(f"trial {trial}")
            trial.run(self.timesteps)

if __name__ == "__main__":

    # Odour vector
    o_x0 = -50.
    o_x1 = 10  # 30.
    angle_m = 0  # 180+40
    angle_std = 20
    p = 1  # only works for p=1

    # Fly
    N_steps = 1000  # Maxiumal number of steps
    m_mean = 30
    m_std = 10
    vx0 = 1
    vy0 = 1
    f_x0 = 0
    f_y0 = 0
    W_f = 0.0
    tau = 15.  # Attention: this parameter is coupled with alpha through the exp dec moving average
    alpha = 200
    ld_turn = 1.3  # probability of taking a turn


    odour = Odour(x=o_x0, y=o_x1, angle_m=angle_m, angle_std=angle_std, p=p)
    fly = Fly(x=f_x0, y=f_y0, vx=vx0, vy=vy0, m_mean=m_mean, m_std=m_std)#, W_f=W_f, tau=tau, alpha=alpha)

    env = SimulationEnvironment(odours=[fly], flies=[odour])



    # Benchmarking parameters

    all_results = []
    all_durations = []
    plot_interactive = False#True#False#True



        result = 0

        fly.update_angle()

        past_x = []
        past_y = []
        past_turns = []
        encounters = []
        W_fs = []
        past_x.append(fly.x)
        past_y.append(fly.y)
        W_fs.append(W_f)
        encounters.append(False)

        if plot_interactive:
            # Initialise plots
            plt.ion()
            fig, ax = plt.subplots(ncols=2)

            # Initialise plot for histogram
            values = fly.get_angle(size=10000)
            prob_density = kde.gaussian_kde(values)
            x = np.linspace(-80,80,300)
            y = prob_density(x)
            l0, = ax[0].plot(x, y, c='k', zorder=10)
            l1, = ax[0].plot([values[0], values[0]], [0, prob_density(values[0])], c='r')
            ax[0].axvline(0, c='k', linestyle='--')
            ax[0].set_yticks([])
            ax[0].set_xlabel("Turn Angle (deg)")

            # Initialise plot for fly behaviour
            # Fly
            r0 = ax[1].scatter(x0, y0, c='b', zorder=100)
            r1, = ax[1].plot(past_x, past_y, c='k')
            r2 = ax[1].scatter(past_x, past_y, c=W_fs, cmap='viridis', s=2)
            # Odour
            r3, = ax[1].plot([o_x0, o_x1], [o_y0, o_y1], c='r')
            r4 = ax[1].scatter(odour.x0, odour.y0, c='r', zorder=10)
            r5, = ax[1].plot([o_x0, o_x1], [o_y0, o_y1], c='k', linestyle='--')
            ax[1].set_xlim([-60, 50])
            ax[1].set_ylim([-50, 50])
            ax[1].set_xlabel("pos x")
            ax[1].set_ylabel("pos y")
            ax[1].add_artist(circle)

        for step in range(N_steps):
            # Update pos
            fly.update_pos()
            past_x.append(fly.x)
            past_y.append(fly.y)

            # Check encounter
            encounter = fly.check_encounter(odour_vector=odour_vector)
            encounters.append(encounter)

            # Update encounter frequency
            fly.update_W_f(int(encounter)) # comment out this line if you want to disable sensing.
            W_fs.append(fly.W_f)

            if plot_interactive:
                # Update fly subplot
                r1.set_xdata(past_x)
                r1.set_ydata(past_y)
                r2.set_offsets(np.c_[past_x, past_y])

            # Get number of turns in this timestep
            turns = fly.get_n_turns() # Does this makes sense?
            past_turns.append(turns)

            # Turn n times
            for turn in range(turns):

                # Update probability distribution
                fly.update_p_angle()

                if not plot_interactive:
                    # Sample and update turn angle
                    values = fly.get_angle(size=1)
                    fly.update_angle(d=values)

                else:
                    # Sample and update turn angle
                    values = fly.get_angle(size=10000)
                    fly.update_angle(d=values[0])

                    # Update turn angle subplot
                    prob_density = kde.gaussian_kde(values)
                    y = prob_density(x)
                    l0.set_ydata(y)
                    l1.set_xdata([values[0], values[0]])
                    l1.set_ydata([0, prob_density(values[0])])
                    ax[0].relim()
                    ax[0].autoscale_view(True,True,True)
                    fig.canvas.draw()
                    # fig.canvas.draw_idle()
                    fig.canvas.flush_events()

            # Update velocity
            fly.update_v()

            # Update odour vector
            odour.update_angle()
            odour_vector = odour.get_vector()
            ((o_x0, o_y0), (o_x1, o_y1)) = odour_vector

            if plot_interactive:
                # Update fly subplot
                r3.set_xdata([o_x0, o_x1])
                r3.set_ydata([o_y0, o_y1])
                fig.canvas.draw()
                fig.canvas.flush_events()

            # Check if close enough
            circle = Circle((o_x0, o_y0), radius = r)
            close_enough = circle.contains_point((fly.x, fly.y))
            if close_enough:
                result = 1
                all_durations.append(step)
                break

        all_results.append(result)

    print(f"{np.mean(all_results)}")
    print(f"{np.mean(all_durations)} +- {np.std(all_durations)}")

            # exit()

    # else:
    #     for step in range(N_steps):
    #         # Update pos
    #         fly.update_pos()
    #         past_x.append(fly.x)
    #         past_y.append(fly.y)

    #         # Check encounter
    #         encounter = fly.check_encounter(odour_vector=odour_vector)
    #         encounters.append(encounter)

    #         # Update encounter frequency (TODO)
    #         fly.update_W_f(int(encounter))
    #         W_fs.append(fly.W_f)
            
    #         # Update turn probability
    #         fly.update_p_t()
                        
    #         turns = np.random.poisson(ld_turn)
    #         past_turns.append(turns)
    #         for turn in range(turns):        
    #             # Update direction
                
    #             if plot_hist:
    #                 values = fly._get_d(size=10000)
    #                 fly.update_d(d=values[0])
                    
    #                 fig, ax = plt.subplots()            
    #                 prob_density = kde.gaussian_kde(values)
    #                 x = np.linspace(-80,80,300)
    #                 y = prob_density(x)            
    #                 ax.plot(x, y, c='k', zorder=10)
    #                 ax.plot([values[0], values[0]], [0, prob_density(values[0])], c='r')
    #                 ax.axvline(0, c='k', linestyle='--')
    #                 ax.set_yticks([])
    #                 ax.set_xlabel("Turn Angle (deg)")
    #                 plt.show()
                    
    #             else:
    #                 fly.update_d()
                        
    #             # Update velocity
    #             fly.update_v()
                
    #         # Update odour vector
                
    #     fig, ax = plt.subplots()
    #     ax.scatter(x0, y0, c='b', zorder=100)
    #     ax.plot(past_x, past_y, c='k')
    #     # ax.scatter(past_x, past_y, c=encounters, cmap='viridis')
    #     sc = ax.scatter(past_x, past_y, c=W_fs, cmap='viridis')
    #     plt.colorbar(sc)
    #     # ax.scatter(past_x, past_y, c=past_turns, cmap='viridis')

    #     ((o_x0, o_y0), (o_x1, o_y1)) = odour_vector
    #     ax.plot([o_x0, o_x1], [o_y0, o_y1], c='r')
    #     ax.scatter(odour.x0, odour.y0, c='r', zorder=10)

    #     ax.set_xlabel("pos x")
    #     ax.set_ylabel("pos y")
    #     plt.savefig(output_dir.joinpath("agent_no_odour"), dpi=300)
    #     plt.show()


    # Turn Frequency
    # T = 50 # seconds
    # ld_turn = 1.3 # turns / s
    # t = np.linspace(0,T,T)
    # turns = np.random.poisson(ld_turn, size=T)

    # # fig, ax = plt.subplots()
    # # ax.scatter(t, turns, c='k')
    # # ax.set_ylabel(r"turns")
    # # ax.set_xlabel(r"time (s)")
    # # ax.set_title("Turn Frequency")
    # # plt.show()

    # # Turn absolute magnitude
    # N = 1000
    # m_mean = 30
    # m_std = 10
    # magnitudes_abs = np.random.normal(m_mean, m_std, N)

    # fig, ax = plt.subplots()
    # for n in range(100):
    #     odour.update_d()
    #     odour.get_vector()
    #     ax.plot([odour.x0, odour.x1], [odour.y0, odour.y1], c='k')
    # ax.scatter(odour.x0, odour.y0, c='r', zorder=10)
    # plt.show()
    # exit() 


    # fig, ax = plt.subplots()
    # ax.scatter(np.arange(N), magnitudes_abs, c='k')
    # ax.set_ylabel(r"Absolute Magnitude (deg)")
    # ax.set_xlabel(r"Sample")
    # ax.set_title("Turn Magnitude")
    # plt.show()

    # def get_W_f(t):
        
    # Turn probability p_T with respect to encounter frequency W_f
    # def p_T(W_f, alpha=1):
    #     return 1./(1+np.exp(-alpha*W_f))

    # W_f = np.linspace(0, 10, 100)
    # fig, ax = plt.subplots()
    # ax.scatter(W_f, np.array([p_T(w) for w in W_f]), c='k')
    # ax.set_ylabel(r"$p(W_f)$")
    # ax.set_xlabel(r"$W_f$")
    # ax.set_title("Turn Probability")
    # plt.show()

    # magnitude
    # p = 0.5
    # W_freqs = [0, 1,2,4,8]
    # fig, ax = plt.subplots(ncols=len(W_freqs), sharex=True, sharey=True)
    # for i, W_freq in enumerate(W_freqs):
    #     p = p_T(W_f=W_freq)
    #     direction = (np.random.binomial(n=1, p=p, size=N)-0.5)*2
    #     turn_magnitudes = magnitudes_abs*direction
    #     ax[i].scatter(turn_magnitudes, np.arange(N), c='k', s=1)
    #     ax[i].set_xlabel(r"Magnitude (deg)")
    #     ax[i].set_ylabel(r"Sample")
    #     ax[i].set_title(f"W_f={W_freq}")
    # plt.show()


    # N = 100000000
    # all_d = []
    # for N in range(1000):
    #     d = fly._get_d()
    #     all_d.append(d)
    # plt.hist(all_d, bins=100, color='k')
    # plt.axvline(0, c='k', linestyle='--')
    # plt.show()
    # exit()
