import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib.pyplot as plt

import rclpy
from rclpy.time import Time
from rclpy.node import Node 
import sensor_msgs.msg
from rclpy.qos import  qos_profile_sensor_data
import logging
import numpy as np
import tf2_ros
from tf2_ros import Buffer
from tf2_ros import TransformListener
from rclpy.duration import Duration

from rclpy.executors import MultiThreadedExecutor
from rclpy.executors import SingleThreadedExecutor

class TransformLoggerNode(Node):
    def __init__(self, target_frame, source_frame):
        super().__init__("transform_logger_node")
        #self.mag_sub = self.create_subscription(sensor_msgs.msg.MagneticField,"imu/mag_data", self.mag_data,qos_profile_sensor_data)
        self.timer = self.create_timer(.3, self.transform_checker)
        self.counter = 0 

        self.target_frame = target_frame
        self.source_frame = source_frame

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.stats = {}
        self.stats['x'] = [] 
        self.stats['y'] = []
        self.stats['z'] = []
        self.fig = plt.figure()
        self.ax = self.fig.gca(projection='3d')
        plt.ion()
        self._old_transform = None

    def do_plot(self):
        self.plot_data() 

    def transform_checker(self):
        try:
            transform = self.tf_buffer.lookup_transform(target_frame=self.target_frame, 
                    source_frame=self.source_frame, 
                    time=Time(),
                    timeout=Duration(seconds=1.0))
            if self._old_transform is not None and self._old_transform.header.stamp == transform.header.stamp:
                #TODO: why does this counter need to exist
                if self.counter < 4: 
                    self.counter +=1
                else:
                    logging.info(f"no new measurements, ending{self._old_transform}\n{transform}")
                    self.timer.destroy()
                    plt.show(block=True)
                return
            self._old_transform = transform
            self.stats['x'].append(transform.transform.translation.x)
            self.stats['y'].append(transform.transform.translation.y)
            self.stats['z'].append(transform.transform.translation.z)
            #logging.info(f"latest transform is {transform}")

            self.plot_data()
            plt.pause(0.0001)
        except Exception as e:
            logging.warning(f"exception {e}")
            if len(self.stats['x']) > 0:
                #we have some data, do the plot
                self.timer.destoy()
                pass
    def plot_data(self):
        self.ax.clear()

        self.ax.set_title(f"transform of {self.target_frame} to {self.source_frame}")
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')
        self.ax.set_xlim3d([-10.0, 10.0])
        self.ax.set_ylim3d([-10.0, 10.0])
        self.ax.set_zlim3d([-10.0, 10.0])
        #self.ax.plot(self.stats['x'], self.stats['y'], self.stats['z'],
        #        label='parametric curve')
        self.ax.plot(self.stats['x'], self.stats['y'], self.stats['z'])
        #self.ax.legend()
        #plt.pause(0.0001)
        plt.show()



def _main():
    mpl.rcParams['legend.fontsize'] = 10

    fig = plt.figure()
    ax = fig.gca(projection='3d')
    theta = np.linspace(-4 * np.pi, 4 * np.pi, 100)
    z = np.linspace(-2, 2, 100)
    r = z**2 + 1
    x = r * np.sin(theta)
    y = r * np.cos(theta)
    ax.plot(x, y, z, label='parametric curve')
    ax.legend()
    plt.show()


def main(args=None): 
    logging.basicConfig(
            format='%(asctime)s %(levelname)-8s %(message)s',
            level=logging.INFO,
            datefmt='%Y-%m-%d %H:%M:%S')

    rclpy.init(args=args)

    odom_node = TransformLoggerNode('odom', 'base_link')
    map_node = TransformLoggerNode('map', 'base_link')
    executor = SingleThreadedExecutor()
    executor.add_node(odom_node)
    executor.add_node(map_node)
    try:
        #rclpy.spin(log_node)
        executor.spin()
    except:
        executor.shutdown()
        odom_node.destroy_node()
        map_node.destroy_node()

if __name__=='__main__':
    main()
