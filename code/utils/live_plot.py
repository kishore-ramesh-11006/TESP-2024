import matplotlib.pyplot as plt
import matplotlib.animation as animation

def live_plot(time_data, omega_data, lock):
    fig, ax = plt.subplots()
    line, = ax.plot([], [], 'r-')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Omega')

    def init():
        line.set_data([], [])
        return line,

    def update(frame):
        with lock:
            if len(time_data) > len(omega_data):
                time_data.pop()
            elif len(omega_data) > len(time_data):
                omega_data.pop()
            line.set_data(time_data, omega_data)
        ax.relim()
        ax.autoscale_view()
        return line,

    ani = animation.FuncAnimation(fig, update, init_func=init, blit=True, interval=100)
    plt.show()


# The rest of your code remains the same
