#!/usr/bin/env python
import rospy
from std_msgs.msg import Empty
import os
import csv
import time
import pandas as pd
import matplotlib
matplotlib.use('Agg')

class Plots:
    def __init__(self, path):
        self.path = path
        rospy.Subscriber("/csv/end", Empty, self.Plot)
        self.restart_pub = rospy.Publisher('/restart', Empty, queue_size=1)
        rospy.spin() 

    def Plot(self, data):
        odometry = pd.read_csv(os.path.join(self.path,'odometry.csv'))
        odometry.rename(columns={'X': r'$x$', 'Y': r'$y$', 'Z': r'$z$', 'Roll': r'$\phi$', 'Pitch': r'$\theta$', 'Yaw': r'$\psi$'}, inplace = True)
        reference = pd.read_csv(os.path.join(self.path,'setpoint.csv'))
        reference.rename(columns={'X': r'$x$', 'Y': r'$y$', 'Z': r'$z$', 'Roll': r'$\phi$', 'Pitch': r'$\theta$', 'Yaw': r'$\psi$'}, inplace = True)

        odometry_df = odometry.loc[:, ['Tiempo',r'$x$',r'$y$',r'$z$',r'$\psi$']]
        reference_df = reference.loc[:, ['Tiempo',r'$x$',r'$y$',r'$z$',r'$\psi$']]
        minV = odometry_df['Tiempo'].min()
        maxV = odometry_df['Tiempo'].max()

        ax = odometry_df.plot(x=0,grid=True,title='Odometry')
        ax.set_xlim(minV,maxV)
        ax.get_figure().savefig(os.path.join(self.path,'odometry.png'))
        ax = reference_df.plot(x=0,grid=True,title='Reference')
        ax.set_xlim(minV,maxV)
        ax.get_figure().savefig(os.path.join(self.path,'reference.png'))
        
        errors = {}

        for ax in [r'x',r'y',r'z',r'\psi']:
            ax_orig = r'$' + ax + r'$'
            ax_odom = r'$' + ax + r'_{odom}$'
            ax_ref = r'$' + ax + r'_{ref}$'
            ax_err = r'$e_{' + ax + r'}$'
            odometry_ = odometry_df.loc[:,['Tiempo',ax_orig]]
            odometry_.rename(columns={ax_orig: ax_odom},inplace = True)
            reference_ = reference_df.loc[:,['Tiempo',ax_orig]]
            reference_.rename(columns={ax_orig: ax_ref},inplace = True)
            df = pd.merge(odometry_, reference_, on='Tiempo', how='inner')
            df[ax_err] = df[ax_ref] - df[ax_odom]
            errors[ax] = df[ax_err]**2
            name = ax + '.png'
            fn = os.path.join(self.path,name)
            ax = df.plot(x=0,grid=True,title=ax_orig)
            ax.set_xlim(minV,maxV)
            ax.get_figure().savefig(fn)
        
        errors[r'xyz'] = errors[r'x'] + errors[r'y'] + errors[r'z']

        with open(os.path.join(self.path,'resultados.csv'), 'w') as archivo:
            writer = csv.writer(archivo)
            writer.writerow(["MSE", errors[r'xyz'].mean()])
            writer.writerow(["MSE_x", errors[r'x'].mean()])
            writer.writerow(["MSE_y", errors[r'y'].mean()])
            writer.writerow(["MSE_z", errors[r'z'].mean()])
            writer.writerow(["MSE_psi", errors[r'\psi'].mean()])
        self.restart_pub.publish(Empty())
        rospy.signal_shutdown("")

if __name__ == '__main__':
    rospy.init_node('plot')
    path = rospy.get_param('~Ruta')
    sub = rospy.get_param('/Subcarpeta')
    sub = sub[:-1]
    plt = Plots(os.path.join(path,sub))
    
    