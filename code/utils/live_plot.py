import matplotlib.pyplot as plt
import time
import threading
lock = threading.Lock()

def live_plot(time_data, omega_data, plotting):
        plt.ion()
        fig, ax = plt.subplots()
        line, = ax.plot(time_data, omega_data, 'r-')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Omega')

        while plotting:
            #make time_data and omega_data same size
            if len(time_data) > len(omega_data):
                time_data = time_data[:len(omega_data)]
            elif len(omega_data) > len(time_data):
                omega_data = omega_data[:len(time_data)]
            with lock:
                line.set_xdata(time_data)
                line.set_ydata(omega_data)
            ax.relim()
            ax.autoscale_view()
            fig.canvas.draw()
            fig.canvas.flush_events()
            time.sleep(0.1)
        print("plotting")
        plt.ioff()
        plt.show()