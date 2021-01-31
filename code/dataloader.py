import os
import numpy as np
import sys
import math as m
import pandas as pd 

# import local modules
import map_maker, load_data

custom_modules = ["map_maker","load_data"]

# DELETE: assures up-to-date local modules are always used
for module_par in custom_modules:
    del sys.modules[module_par]

# RELOAD: assures up-to-date local modules are always used
import map_maker, load_data

def load_folder(folder_par):
    
    # prefix_list= ["Hokuyo","Encoders","imu"]
    # find location of period (where file extension starts) and extract unique filenames
    unique_files = list(set([i[i.find(".")-2:i.find(".")] for i in os.listdir(folder_par)]))

    # load all map attributes into classes
    '''NOTE: r prefix tells interpreter the backslash is to be take literally, as any other character.'''
    map_list = [map_object(\
                            id,\
                            load_data.get_lidar(folder_par+r"/Hokuyo{0}".format(id)),\
                            load_data.get_encoder(folder_par+r"/Encoders{0}".format(id)),\
                            load_data.get_imu(folder_par+r"/imu{0}".format(id)))\
                            for id in unique_files]  
    return map_list

class map_object:
    def __init__(self,id,lidar,encoder,imu):
        self.id = id
        self.lidar = lidar
        self.encoder = encoder
        self.imu = imu
        self.processed_encoder = self.process_encoder()
        self.side_averages = self.process_averages()
        self.step_angle,self.cummulative_angle = self.get_angles()
        
        self.step_displacement_local, self.step_displacement,\
        self.cummulative_displacement, self.mean_displament = self.get_displacements()
        # self.lidar_map = self.get_lidar_map()

    def process_encoder(self):
        encoder_data = np.transpose(np.asarray(self.encoder))
        time_array = encoder_data[:,4]
        # time_array = time_array-time_array[0]
        encoder_data = np.hstack((time_array[:,np.newaxis],encoder_data[:,:4]))
        return encoder_data

    def process_averages(self):
        
        # Wheel diameter in meters
        wheel_diameter = 0.254

        # Get timed right wheel data (time,front,rear)
        right_data = np.hstack((self.processed_encoder[:,0][:,np.newaxis],\
                                self.processed_encoder[:,1][:,np.newaxis],\
                                self.processed_encoder[:,3][:,np.newaxis]))
        
        # Get average right wheel data (no time included)
        delta_right = np.absolute(self.processed_encoder[:,1]-self.processed_encoder[:,3])
        min_right   = np.amin(self.processed_encoder[:,1]-self.processed_encoder[:,3])
        min_right   = np.amin(self.processed_encoder[:,1]-self.processed_encoder[:,3])

        
        right_averages = np.mean(right_data[:,-2:],axis=1)[:,np.newaxis]/360*m.pi*wheel_diameter

        # Get timed left wheel data (time,front,rear)
        left_data = np.hstack((self.processed_encoder[:,0][:,np.newaxis],\
                               self.processed_encoder[:,2][:,np.newaxis],\
                               self.processed_encoder[:,4][:,np.newaxis]))
        
        # Get average left wheel data (no time included)
        left_averages  = np.mean(left_data[:,-2:],axis=1)[:,np.newaxis]/360*m.pi*wheel_diameter

        return np.hstack((self.processed_encoder[:,0][:,np.newaxis],right_averages,left_averages))

    def get_angles(self):
        
        # This is the dimensional width. Hence, it has been commented. 
        # robot_width = (0.31115+0.47626)/2
        
        # This is the effective width of the robot, as computed by optimization
        robot_width = 0.731
        #robot_width = 0.680


        cummulative_angle = np.zeros([self.side_averages.shape[0]])
        cummulative_angle_WRONG = np.zeros([self.side_averages.shape[0]])
        
        step_angle = (self.side_averages[:,1]-self.side_averages[:,2])/(robot_width)
        
        indexer = np.arange(0,self.side_averages.shape[0])
        cummulative_angle_WRONG[0]     = 0 
        cummulative_angle_WRONG[1:]    = np.asarray([np.sum(step_angle[0:i+1]) for i in indexer[:-1]])
        
        for time_step in range (0,self.side_averages.shape[0]):
            cummulative_angle[time_step] = np.sum(step_angle[0:time_step+1])
#             print(cummulative_angle[time_step] ,cummulative_angle_WRONG[time_step])

#         print(np.linalg.norm(cummulative_angle_WRONG-cummulative_angle), "norm distance for two methods")
        
        return step_angle, cummulative_angle

    def get_displacements(self):
        
        step_displacement_local = np.empty([self.side_averages.shape[0],2])
        cummulative_displacement = np.empty([self.side_averages.shape[0],2])
        
        # compute average of left and right side displacement
        mean_displament   = np.mean(self.side_averages[:,1:],axis=1) [:,np.newaxis]


        x_steps_local = np.multiply(mean_displament ,np.cos(self.step_angle)[:,np.newaxis])
        y_steps_local = np.multiply(mean_displament ,np.sin(self.step_angle)[:,np.newaxis])
        
        step_displacement_local = np.hstack((x_steps_local,y_steps_local))

        x_steps_global = np.multiply(step_displacement_local[:,0] ,np.cos(self.cummulative_angle))\
                       - np.multiply(step_displacement_local[:,1] ,np.sin(self.cummulative_angle))

        y_steps_global = np.multiply(step_displacement_local[:,0] ,np.sin(self.cummulative_angle))\
                       + np.multiply(step_displacement_local[:,1] ,np.cos(self.cummulative_angle))\
        
        step_displacement_global = np.hstack((x_steps_global[:,np.newaxis],y_steps_global[:,np.newaxis]))

        
        for time_step in range (0,self.side_averages.shape[0]):
            cummulative_displacement[time_step] = np.sum(step_displacement_global[0:time_step+1],axis=0)

#         cummulative_displacement[0] = 0,0
#         for time_step in range (1,self.side_averages.shape[0]):
#             cummulative_displacement[time_step] = cummulative_displacement[time_step-1] +\
#                                                   step_displacement_global[time_step-1]

        return step_displacement_local,step_displacement_global,cummulative_displacement,mean_displament
    
    # def get_lidar_map(self):

        