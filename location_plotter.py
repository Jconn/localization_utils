import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib.pyplot as plt

import mpl_toolkits.mplot3d.axes3d as p3
import mpl_toolkits
import matplotlib.animation as animation

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
import pdb

class TransformLoggerNode(Node):
    def __init__(self, target_frame, source_frame, executor):
        super().__init__("transform_logger_node")
        #self.mag_sub = self.create_subscription(sensor_msgs.msg.MagneticField,"imu/mag_data", self.mag_data,qos_profile_sensor_data)
        self.timer = self.create_timer(.3, self.transform_checker)
        self.counter = 0 

        self.executor = executor
        self.target_frame = target_frame
        self.source_frame = source_frame

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.stats = {}
        self.stats['x'] = [] 
        self.stats['y'] = []
        self.stats['z'] = []
        self.plotting = False
        self._old_transform = None
        #plt.ion()

    def transform_checker(self):
        try:
            transform = self.tf_buffer.lookup_transform(target_frame=self.target_frame, 
                    source_frame=self.source_frame, 
                    time=Time(),
                    timeout=Duration(seconds=1.0))
            if self._old_transform is not None and self._old_transform.header.stamp == transform.header.stamp:
                #TODO: why does this counter need to exist
                if self.counter < 10: 
                    self.counter +=1
                else:
                    #logging.info(f"no new measurements, ending{self._old_transform}\n{transform}")
                    logging.info(f"no new measurements, destroying node")
                    self.executor.remove_node(self)
                    #self.destroy_node()
                return

            self.counter = 0 
            self._old_transform = transform
            self.stats['x'].append(transform.transform.translation.x)
            self.stats['y'].append(transform.transform.translation.y)
            self.stats['z'].append(transform.transform.translation.z)
            #logging.info(f"latest transform is {transform}")

            #self.plot_data()
        except Exception as e:
            logging.warning(f"exception {e}")
            if len(self.stats['x']) > 0:
                #we have some data, do the plot
                self.timer.destoy()
                pass

    def update_line(self, num, data, lines):
        logging.warning(f"updating line {num}")
        #for line, data in zip(lines, dataline):
        logging.warning(f"{lines}")
        lines[0].set_data_3d(data[0][:num],
                         data[1][:num],
                         data[2][:num])
        return lines 

    def plot_data(self):
        if self.plotting:
            #plt.pause(0.0001)
            return

        self.fig = plt.figure()
        self.ax = p3.Axes3D(self.fig)

        self.plotting = True

        #self.ax.clear()

        lines = self.ax.plot(self.stats['x'][0:1],self.stats['y'][0:1],self.stats['z'][0:1])


        self.ax.set_title(f"transform of {self.target_frame} to {self.source_frame}")
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')
        self.ax.set_xlim3d([-7.0, 7.0])
        self.ax.set_ylim3d([-7.0, 7.0])
        self.ax.set_zlim3d([-2.0, 2.0])
        data = [self.stats['x'], self.stats['y'], self.stats['z']] 
        #pdb.set_trace()
        self.line_ani = animation.FuncAnimation(self.fig, self.update_line, len(self.stats['x']), 
                interval=50, blit=False, repeat_delay=5000, fargs=(data, lines))
        #self.ax.plot(self.stats['x'], self.stats['y'], self.stats['z'],
        #        label='parametric curve')
        #self.ax.plot(self.stats['x'], self.stats['y'], self.stats['z'])
        #self.ax.legend()
        #plt.show(block=False)




def main(args=None): 
    logging.basicConfig(
            format='%(asctime)s %(levelname)-8s %(message)s',
            level=logging.INFO,
            datefmt='%Y-%m-%d %H:%M:%S')

    rclpy.init(args=args)

    executor = SingleThreadedExecutor()
    odom_node = TransformLoggerNode('odom', 'base_link', executor=executor)
    map_node = TransformLoggerNode('map', 'base_link', executor=executor)
    #executor.add_node(odom_node)
    executor.add_node(map_node)
    try:
        #rclpy.spin(log_node)
        while True:
            executor.spin_once()
            if len(executor.get_nodes()) == 0:
                break

    except:
        logging.warning(f"hit exception")
        pass
        #executor.shutdown()
    finally:
        logging.warning(f"cleaning up")
        odom_node.plot_data()
        map_node.plot_data()
        plt.pause(1)
        plt.show(block=True)
        odom_node.destroy_node()
        map_node.destroy_node()


if __name__=='__main__':
    main()
