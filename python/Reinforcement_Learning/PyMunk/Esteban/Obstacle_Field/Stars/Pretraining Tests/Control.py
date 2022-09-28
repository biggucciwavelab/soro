"""
A fork of the original control laws developed to pretrain the agent on navigating across an obstacle field in PyChrono.

Note that because the original laws were developed for PyChrono, some directions may be labeled in the z-direction. This is analogoues to the y-direction in PyMunk.
"""
import numpy as np
from collections import Counter
import os
import pdb
from statistics import mean
from math import sqrt

class Control:
    def __init__(self, R, numBots = 10, obsRadius = np.nan, forcex=1, forcey=1, env_run=None):
        """
        
        """
        self.forcex = forcex # Force applied into X-Direction
        self.forcey = forcey # Force applied into Z-direction

        self.num_bots= numBots
        self.obs_radius = obsRadius
        self.R = R
        self.buffer=0.01 # System can be slightly off of where the z-axis is
        
    def control_5(self, obs):
        """
        The holy grail of control law. Must be able to do the following:
            1) Move intentionally around an obstacle when it detects one, not ram into it and not care.
            2) Have an increasing z-force applied to each bot that increases as the system moves farther away from the z-axis.
        """
        # Step 0) Initiate the action by pushing all bots forward
        action = np.zeros(2*self.num_bots)
        for a in range(2*self.num_bots):
            if a%2==0: # Applying force in the x-direction on each bot
                action[a]=self.forcex
        
        # Step 1) Gather the Z-positions and X-Positions for the system
        bot_pos=obs[0:2*self.num_bots]
        z_pos=[]
        x_pos=[]
        for i in range(len(bot_pos)):
            if i%2 != 0: #Gathering the z_positions
                z_pos.append(bot_pos[i]*self.R*5) # Multiplied with 5R to counteract normalization in environment
            elif i%2==0:
                x_pos.append(bot_pos[i]*self.R*28)
        
        # Step 2) Find the average Z-position
        z_center=mean(z_pos)
        abs_z = abs(z_center)
        
        # Step 3) If too far away from z-axis, push bots towards z-axis
        if abs_z>self.obs_radius*2:
            for j in range(self.num_bots):
                z=z_pos[j]
                j+=1
                action[j*2-1]=-np.sign(z)*self.forcez*2
        
        # Step 4) Check for contact with an obstacle
        contacts=obs[6*self.num_bots:8*self.num_bots]
        contact=False
        if np.count_nonzero(contacts) != 0:
            contact=True
            
        # If contact is present...... 
        #Step 5) Count how many are in contact and which ones
        if contact==True:
            bots_contact = np.zeros(self.num_bots)
            for i in range(self.num_bots):
                i+=1
                if contacts[i*2-2]!=0 or contacts[i*2-1]!=0:
                    bots_contact[i-1]=1
        
        # Step 6) Check total z-contact sign
            j=0
            z_tot=0
            for i in bots_contact:
                if i==1:
                    z_tot=np.sign(z_pos[j])
                j+=1
                
            if z_tot==0:
                z_dir=1
            else:
                z_dir=np.sign(z_tot)
                
        # Step 7) If z_center is close to axis, move in direction of z_tot
            if abs_z<self.obs_radius:
                for i in range(self.num_bots):
                    i+=1
                    action[i*2-2] = 0
                    action[i*2-1] = z_dir*self.forcez
                    
        # Step 8) If z_center is far from the axis, move in the opposite direction of contact
            elif abs_z>self.obs_radius:
                for i in range(self.num_bots):
                    i+=1
                    action[i*2-2] = 0.1*self.forcex
                    action[i*2-1] = -z_dir*self.forcez
        
        # Step last) If the x_pos of anybot>0, shut off the forces of that bot! Or better yet, reverse it.
        for i in range(self.num_bots):
            if x_pos[i]>0:
                i+=1
                action[i*2-2]=-self.forcex
                
        return(action)
    
    def control_6(self, obs):
        """
        Control_5 was most certainly NOT the holy grail of contrl laws, I was still left with errors that need exploration.
        Control_6 will not use the quadrant (w.r.t. the system's center) to define where the collision is occuring
        """
        # Initiate the action by pushing all bots forward
        action = np.zeros(2*self.num_bots)
        
        # Gather the Z-positions and X-Positions for the system
        bot_pos=obs[0:2*self.num_bots]
        z_pos=[]
        x_pos=[]
        for i in range(len(bot_pos)):
            if i%2 != 0: #Gathering the z_positions
                z_pos.append(bot_pos[i]*self.R*5) # Multiplied with 5R to counteract normalization in environment
            elif i%2==0:
                x_pos.append(bot_pos[i]*self.R*28)
        
        # Find the average Z-position
        x_center=mean(x_pos)
        z_center=mean(z_pos)
        abs_z = abs(z_center)
        
        # Check for contact with an obstacle
        contacts=obs[6*self.num_bots:8*self.num_bots]
        contact=False
        if np.count_nonzero(contacts) != 0:
            contact=True
            
        if contact==True: # If contact is present...... 
            #Collects which bots are in contact with an obstacle and their quadrant
            bots_contact = np.zeros(self.num_bots)
            contact_quadrants=np.zeros(self.num_bots)
            for i in range(self.num_bots):
                i+=1
                if contacts[i*2-2]!=0 or contacts[i*2-1]!=0:
                    bots_contact[i-1]=1
                    contact_quadrants[i-1]=quadrant(x_pos[i-1]-x_center, z_pos[i-1]-z_center)
            quads=Counter(contact_quadrants)
            
            #Expected collisions:
            #Head on collision. The back of the system is not in contact with an obstacle.
            if quads[2]==0 and quads[3]==0:
                if quads[1]>quads[4]: # Move down (positive z)
                    for i in range(self.num_bots):
                        i+=1
                        action[2*i-2]=0.1*self.forcex #X-force
                        action[2*i-1]=0.2*self.forcez #Z-force
                elif quads[1]<quads[4]: # Move up (negative z)
                    for i in range(self.num_bots):
                        i+=1
                        action[2*i-2]=0.1*self.forcex #X-force
                        action[2*i-1]=-0.2*self.forcez #Z-force
                else: #Arbitrary, move down
                    for i in range(self.num_bots):
                        i+=1
                        action[2*i-2]=0.1*self.forcex
                        action[2*i-1]=0.2*self.forcez
                
            #Bottom skid. The POSITIVE z-side of the system is in contact with an obstacle. Negative z-side is NOT in contact with an obstacle.
            elif quads[3]==0 and quads[4]==0:
                for i in range(self.num_bots):
                    i+=1
                    action[2*i-2]=0.3*self.forcex
                    action[2*i-1]=-0.1*self.forcez
            
            #Top skid. The NEGATIVE z-side is in contact with an obstacle. Negative z-side is NOT in contact with an obstacle.
            elif quads[1]==0 and quads[2]==0:
                for i in range(self.num_bots):
                    i+=1
                    action[2*i-2]=0.3*self.forcex
                    action[2*i-1]=0.1*self.forcez
            
            #Turn off all forces for bots that are in collision
            bot_col_ids=[i for i in range(len(bots_contact)) if bots_contact[i]==1]
            for off in range(self.num_bots):
                if off in bot_col_ids:
                    off+=1
                    action[2*off-2]=0 # Turn off forcex
                    action[2*off-1]=0 # Turn off forcez
            
        elif contact==False: # No Contact
        # Apply z-force that is proportional to distance from the x-axis
            for i in range(self.num_bots):
                i+=1
                action[i*2-2] = 0.5*self.forcex
                z_dir=np.sign(z_pos[i-1])
                action[i*2-1] = -z_dir*0.1*self.forcez
        
        # If the x_pos of anybot>0, shut off the forces of that bot! Or better yet, reverse it.
        for i in range(self.num_bots):
            if x_pos[i]>0:
                i+=1
                action[i*2-2]=-0.5*self.forcex
                    
        return(action)
    
    def control_7(self, obs):
        """
        Control_7 will have the forces pointed towards the target, with the same magnitude when not in contact with an obstacle
        
        In this control law, the magnitude of forces at anytime does not exceed a threshold
        """
        # Initiate the action by pushing all bots forward
        action = np.zeros(2*self.num_bots)
        
        # Gather the Z-positions and X-Positions for the system
        bot_pos=obs[0:2*self.num_bots]
        self.z_pos=[]
        self.x_pos=[]
        for i in range(len(bot_pos)):
            if i%2 != 0: #Gathering the z_positions
                self.z_pos.append(bot_pos[i]) # Multiplied with 5R to counteract normalization in environment
            elif i%2==0:
                self.x_pos.append(bot_pos[i])
        
        # Find the average Z-position
        self.x_center=mean(self.x_pos)
        self.z_center=mean(self.z_pos)
        
        # Check for contact with an obstacle
        contacts=obs[6*self.num_bots:8*self.num_bots]
        self.contact=False
        if np.count_nonzero(contacts) != 0:
            self.contact=True
            
        if self.contact==True: # If contact is present...... 
            #Collects which bots are in contact with an obstacle and their quadrant
            bots_contact = np.zeros(self.num_bots)
            self.contact_quadrants=np.zeros(self.num_bots)
            for i in range(self.num_bots):
                i+=1
                if contacts[i*2-2]!=0 or contacts[i*2-1]!=0:
                    bots_contact[i-1]=1
                    self.contact_quadrants[i-1]=quadrant(self.x_pos[i-1]-self.x_center, self.z_pos[i-1]-self.z_center)
            quads=Counter(self.contact_quadrants)
            
            #Expected collisions:
            #Head on collision. The back of the system is not in contact with an obstacle.
            if quads[2]==0 and quads[3]==0:
                if quads[1]>=quads[4]: # Move down (positive z)
                    for i in range(self.num_bots):
                        i+=1
                        action[2*i-2]=0.5*self.forcex #X-force
                        action[2*i-1]=0.5*self.forcez #Z-force
                elif quads[1]<quads[4]: # Move up (negative z)
                    for i in range(self.num_bots):
                        i+=1
                        action[2*i-2]=0.5*self.forcex #X-force
                        action[2*i-1]=-0.5*self.forcez #Z-force
                
            #Bottom skid. The POSITIVE z-side of the system is in contact with an obstacle. Negative z-side is NOT in contact with an obstacle.
            elif quads[3]==0 and quads[4]==0:
                for i in range(self.num_bots):
                    i+=1
                    action[2*i-2]=0.5*self.forcex
                    action[2*i-1]=-0.25*self.forcez
            
            #Top skid. The NEGATIVE z-side is in contact with an obstacle. Positive z-side is NOT in contact with an obstacle.
            elif quads[1]==0 and quads[2]==0:
                for i in range(self.num_bots):
                    i+=1
                    action[2*i-2]=0.5*self.forcex
                    action[2*i-1]=0.25*self.forcez
            
            #If none of the above are true, just pretend that we are not in collision. Let's assume that the collision is on the butt of the system.
            elif quads[1]==0 and quads[4]==0: 
                    self.contact=False
                    
            #Turn off all forces for bots that are in collision
            bot_col_ids=[i for i in range(len(bots_contact)) if bots_contact[i]==1]
            for off in range(self.num_bots):
                if off in bot_col_ids:
                    off+=1
                    action[2*off-2]=0 # Turn off forcex
                    action[2*off-1]=0 # Turn off forcez
            
        elif self.contact==False: # No Contact
            force_mag=1 #Basically the gain.
            for i in range(self.num_bots):
                distance=sqrt((self.x_pos[i])**2 + (self.z_pos[i])**2)
                x_dir = -self.x_pos[i]/distance
                z_dir = -self.z_pos[i]/distance
                i+=1
                action[2*i-2]=force_mag*x_dir #x_force
                action[2*i-1]=force_mag*z_dir
                
        return(action)


    def control_8(self, obs):
        """
        Control_7 will have the forces pointed towards the target, with the same magnitude when not in contact with an obstacle
        
        In this control law, the magnitude of forces at anytime does not exceed a threshold
        """
        # Initiate the action by pushing all bots forward
        action = np.zeros(2*self.num_bots)
        
        # Gather the Z-positions and X-Positions for the system
        bot_pos=obs[0:2*self.num_bots]
        self.z_pos=[]
        self.x_pos=[]
        for i in range(len(bot_pos)):
            if i%2 != 0: #Gathering the z_positions
                self.z_pos.append(bot_pos[i])
            elif i%2==0:
                self.x_pos.append(bot_pos[i])
        
        # Find the average Z-position
        self.x_center=mean(self.x_pos)
        self.z_center=mean(self.z_pos)
        
        # Check for contact with an obstacle
        contacts=obs[6*self.num_bots:8*self.num_bots]
        self.contact=False
        if np.count_nonzero(contacts) != 0:
            self.contact=True
            
        if self.contact==True: # If contact is present...... 
            #Collects which bots are in contact with an obstacle and their quadrant
            bots_contact = np.zeros(self.num_bots)
            self.contact_quadrants=np.zeros(self.num_bots)
            for i in range(self.num_bots):
                i+=1
                if contacts[i*2-2]!=0 or contacts[i*2-1]!=0:
                    bots_contact[i-1]=1
                    self.contact_quadrants[i-1]=quadrant(self.x_pos[i-1]-self.x_center, self.z_pos[i-1]-self.z_center)
            quads=Counter(self.contact_quadrants)
            
            #Expected collisions:
            #Head on collision. The back of the system is not in contact with an obstacle.
            if (quads[4] + quads[1])>(quads[3]+quads[2]):
                if quads[1]>=quads[4]: # Move down (positive z)
                    for i in range(self.num_bots):
                        i+=1
                        # action[2*i-2]=0.5*self.forcex #X-force
                        action[2*i-1]=1*self.forcez #Z-force
                elif quads[1]<quads[4]: # Move up (negative z)
                    for i in range(self.num_bots):
                        i+=1
                        # action[2*i-2]=0.5*self.forcex #X-force
                        action[2*i-1]=-1*self.forcez #Z-force
                
            #Positive skid. The POSITIVE z-side of the system is in contact with an obstacle. Negative z-side is NOT in contact with an obstacle.
            elif (quads[3]+quads[4])>(quads[1]+quads[2]):
                for i in range(self.num_bots):
                    i+=1
                    # action[2*i-2]=0.5*self.forcex
                    action[2*i-1]=-1*self.forcez
            
            #Negative skid. The NEGATIVE z-side is in contact with an obstacle. Positive z-side is NOT in contact with an obstacle.
            elif (quads[1]+quads[2])>(quads[3]+quads[4]):
                for i in range(self.num_bots):
                    i+=1
                    # action[2*i-2]=0.5*self.forcex
                    action[2*i-1]=1*self.forcez
            
            #If collision is on the butt of the system.
            elif (quads[3]+quads[2])>(quads[4]+quads[1]): 
                    self.contact=False
            
            #Turn off all forces for bots that are in collision
            bot_col_ids=[i for i in range(len(bots_contact)) if bots_contact[i]==1]
            for off in range(self.num_bots):
                if off in bot_col_ids:
                    off+=1
                    action[2*off-2]=0 # Turn off forcex
                    action[2*off-1]=0 # Turn off forcez
            
        elif self.contact==False: # No Contact
            force_mag=1 #Basically the gain.
            for i in range(self.num_bots):
                distance=sqrt((self.x_pos[i])**2 + (self.z_pos[i])**2)
                x_dir = -self.x_pos[i]/distance
                z_dir = -self.z_pos[i]/distance
                i+=1
                action[2*i-2]=force_mag*x_dir #x_force
                action[2*i-1]=force_mag*z_dir #z_force
                
        return(action)
    
    
    def control_9(self, obs):
        """
        Changes from controller 8:
            - There is a weight term on the z-direction to counter the fact that the system starts on the z-axis. If this is not included, the system does not catch itself from deviations in the z-direction until it is too late.
            - An else statement that, if there is no other collision scenario that does not make sense, applied the standard control w/o collision
            - Small back-tracking term for forces to by applied on bots to move away from collision location.
        """
        # Initiate the action by pushing all bots forward
        action = np.zeros(2*self.num_bots)
        
        # Gather the Z-positions and X-Positions for the system
        bot_pos=obs[0:2*self.num_bots]
        self.z_pos=[]
        self.x_pos=[]
        for i in range(len(bot_pos)):
            if i%2 != 0: #Gathering the z_positions
                self.z_pos.append(bot_pos[i])
            elif i%2==0:
                self.x_pos.append(bot_pos[i])
        
        # Find the average Z-position
        self.x_center=mean(self.x_pos)
        self.z_center=mean(self.z_pos)
        
        # Check for contact with an obstacle
        contacts=obs[6*self.num_bots:8*self.num_bots]
        self.contact=False
        if np.count_nonzero(contacts) != 0:
            self.contact=True
            
        if self.contact==True: # If contact is present...... 
            #Collects which bots are in contact with an obstacle and their quadrant
            bots_contact = np.zeros(self.num_bots)
            self.contact_quadrants=np.zeros(self.num_bots)
            for i in range(self.num_bots):
                i+=1
                if contacts[i*2-2]!=0 or contacts[i*2-1]!=0:
                    bots_contact[i-1]=1
                    self.contact_quadrants[i-1]=quadrant(self.x_pos[i-1]-self.x_center, self.z_pos[i-1]-self.z_center)
            quads=Counter(self.contact_quadrants)
            
            #Expected collisions:
            #Head on collision. The back of the system is not in contact with an obstacle.
            if (quads[4] + quads[1])>(quads[3]+quads[2]):
                if quads[1]>=quads[4]: # Move down (positive z)
                    for i in range(self.num_bots):
                        i+=1
                        # action[2*i-2]=0.1*self.forcex #X-force
                        action[2*i-1]=1*self.forcez #Z-force
                elif quads[1]<quads[4]: # Move up (negative z)
                    for i in range(self.num_bots):
                        i+=1
                        # action[2*i-2]=0.1*self.forcex #X-force
                        action[2*i-1]=-1*self.forcez #Z-force
                
            #Positive skid. The POSITIVE z-side of the system is in contact with an obstacle. Negative z-side is NOT in contact with an obstacle.
            elif (quads[3]+quads[4])>(quads[1]+quads[2]):
                for i in range(self.num_bots):
                    i+=1
                    # action[2*i-2]=0.1*self.forcex
                    action[2*i-1]=-1*self.forcez
            
            #Negative skid. The NEGATIVE z-side is in contact with an obstacle. Positive z-side is NOT in contact with an obstacle.
            elif (quads[1]+quads[2])>(quads[3]+quads[4]):
                for i in range(self.num_bots):
                    i+=1
                    # action[2*i-2]=0.1*self.forcex
                    action[2*i-1]=1*self.forcez
            
            #If collision is on the butt of the system.
            elif (quads[3]+quads[2])>(quads[4]+quads[1]): 
                    self.contact=False
            
            #If NONE of the above scenarios are true, just pretend we are not in contact
            else:
                self.contact=False
                    
            #Turn off all forces for bots that are in collision
            bot_col_ids=[i for i in range(len(bots_contact)) if bots_contact[i]==1]
            for off in range(self.num_bots):
                if off in bot_col_ids:
                    off+=1
                    action[2*off-2]=0 # Turn off forcex
                    action[2*off-1]=0 # Turn off forcez
            
        elif self.contact==False: # No Contact
            force_mag=1 #Basically the gain.
            for i in range(self.num_bots):
                distance=sqrt((self.x_pos[i])**2 + 10*(self.z_pos[i])**2)
                x_dir = -self.x_pos[i]/distance
                z_dir = -self.z_pos[i]/distance
                i+=1
                action[2*i-2]=force_mag*x_dir #x_force
                action[2*i-1]=force_mag*z_dir #z_force
                
        return(action)
    




    def control_10(self, obs):
        """
        The premise of this control law is to do the following:
            - In the event that there is a head-on collision (in the form of more collisions in quadrants 1+4 than 2+3), then follow the recommendation of control law 8 and 9
            - If any other type of collision is occuring, then turn off forces of colliding bots and otherwise react as if there is no collision.
        """
        # Initiate the action by pushing all bots forward
        action = np.zeros(2*self.num_bots)
        
        # Gather the Z-positions and X-Positions for the system
        bot_pos=obs[0:2*self.num_bots]
        y_pos=[]
        x_pos=[]
        for i in range(len(bot_pos)):
            if i%2 != 0: #Gathering the y_positions
                y_pos.append(bot_pos[i])
            elif i%2==0:
                x_pos.append(bot_pos[i])
        
        # Find the average Z-position
        x_center = mean(x_pos)
        y_center = mean(y_pos)
        
        bot_vel = obs[2*self.num_bots: 4*self.num_bots]

        # Check for contact with an obstacle
        contacts=obs[4*self.num_bots:]
        contact_present = np.any(contacts!=0)

        # If there is contact with some external object
        if contact_present:
            # print('Collision!')
            # Count in what quadrants contacts are occuring
            contact_quadrants=np.zeros(self.num_bots)
            for i, c in enumerate(contacts):
                if c: # True if c==1
                    contact_quadrants[i] = quadrant(x_pos[i] + x_center, y_pos[i] + y_center)
            quads = Counter(contact_quadrants)

            #Head on collision. The back of the system is not in contact with an obstacle.
            if (quads[4] + quads[1])>(quads[3]+quads[2]):
                print('Frontal Cotact')
                if np.all(np.abs(bot_vel)<1e-2):
                    # pdb.set_trace()
                    pass
                if quads[1]>=quads[4]: # Move down (positive y)
                    for i in range(self.num_bots):
                        i+=1
                        action[2*i-1] = self.forcey #Y-force
                elif quads[1]<quads[4]: # Move up (negative y)
                    for i in range(self.num_bots):
                        i+=1
                        action[2*i-1] = self.forcey #Y-force
            
            # If we are not in a head-on collision, then use the next control law
            else:
                contact_present = False

        if not contact_present:
            # print('No Collision!')
            y_axis_gain = 10 # Promotes staying on the y-axis as much as possible.
            for i in range(self.num_bots):
                distance = sqrt((x_pos[i])**2 + y_axis_gain*(y_pos[i])**2)
                x_dir = x_pos[i]/distance
                y_dir = y_pos[i]/distance
                i += 1
                action[2*i-2] = x_dir #x_force
                action[2*i-1] = y_dir #z_force
            
            #Turn off all forces for bots that are in collision
            bot_col_ids = np.where(contacts)[0]
            for off in range(self.num_bots):
                if off in bot_col_ids:
                    off += 1
                    action[2*off-2] = 0 # Turn off forcex
                    action[2*off-1] = 0 # Turn off forcey
        
        return action
    



    def gap_push(self, obs):
        contacts=obs[6*self.num_bots:8*self.num_bots]
        
        if np.count_nonzero(contacts) == 0: 
            print('Control_8')
            return self.control_8(obs)
        else: 
            print('X_push')
            return self.x_push(obs)
    
    def large_push(self,obs):
        
        # Initiate the action by pushing all bots forward
        action = np.zeros(2*self.num_bots)
        
        # Gather the Z-positions and X-Positions for the system
        bot_pos=obs[0:2*self.num_bots]
        self.z_pos=[]
        self.x_pos=[]
        for i in range(len(bot_pos)):
            if i%2 != 0: #Gathering the z_positions
                self.z_pos.append(bot_pos[i]) # Multiplied with 5R to counteract normalization in environment
            elif i%2==0:
                self.x_pos.append(bot_pos[i])
        
        # Find the average Z-position
        self.x_center=mean(self.x_pos)
        self.z_center=mean(self.z_pos)
        contacts=obs[6*self.num_bots:8*self.num_bots]
        self.contact=False
        if np.count_nonzero(contacts) != 0:
            self.contact=True
            
        if self.contact:
            #Collects which bots are in contact with an obstacle and their quadrant
            bots_contact = np.zeros(self.num_bots)
            self.contact_quadrants=np.zeros(self.num_bots)
            for i in range(self.num_bots):
                i+=1
                if contacts[i*2-2]!=0 or contacts[i*2-1]!=0:
                    bots_contact[i-1]=1
                    self.contact_quadrants[i-1]=quadrant(self.x_pos[i-1]-self.x_center, self.z_pos[i-1]-self.z_center)
            quads=Counter(self.contact_quadrants)
        #Expected collisions:
        #Head on collision. The back of the system is not in contact with an obstacle.
            if (quads[4] + quads[1])>(quads[3]+quads[2]):
                if quads[1]>=quads[4]: # Move down (positive z)
                    for i in range(self.num_bots):
                        i+=1
                        # action[2*i-2]=0.5*self.forcex #X-force
                        action[2*i-1]=1*self.forcez #Z-force
                elif quads[1]<quads[4]: # Move up (negative z)
                    for i in range(self.num_bots):
                        i+=1
                        # action[2*i-2]=0.5*self.forcex #X-force
                        action[2*i-1]=-1*self.forcez #Z-force
            else:
                force_mag=1 #Basically the gain.
                for i in range(self.num_bots):
                    distance=sqrt((self.x_pos[i])**2 + (self.z_pos[i])**2)
                    x_dir = -self.x_pos[i]/distance
                    z_dir = -self.z_pos[i]/distance
                    i+=1
                    action[2*i-2]=force_mag*x_dir #x_force
                    action[2*i-1]=force_mag*z_dir #z_force
        else:
            force_mag=1 #Basically the gain.
            for i in range(self.num_bots):
                distance=sqrt((self.x_pos[i])**2 + (self.z_pos[i])**2)
                x_dir = -self.x_pos[i]/distance
                z_dir = -self.z_pos[i]/distance
                i+=1
                action[2*i-2]=force_mag*x_dir #x_force
                action[2*i-1]=force_mag*z_dir #z_force
                    
            return(action)
                    
                
    
    def print_info(self):
        #if self.x_center!=None and self.z_center!=None:
            # print('System Center: ('+str(self.x_center)+','+str(self.z_center)+')')
            # print('Bot 1 Position: ('+str(self.x_pos[0])+','+str(self.z_pos[0])+')')
        if self.contact==True:
            print('Bot 1 Contact Quad:',self.contact_quadrants[0])
            print('Bot 8 Contact Quad:', self.contact_quadrants[7])
            print('Bot 15 Contact Quad:', self.contact_quadrants[15])
            print('Bot 23 Contact Quad:', self.contact_quadrants[22])
        pass

def quadrant(x,y):
    """
    Takes a coordinate pair and determines which quadrant the coordiante pair are in.
    
    Note, this is meant to mimic the quadrants in chrono, not the actual careision plane. The positive z-direction is DOWN in videos
    
    Quad=0 indicates no contact on that bot

    Quadrant schematic:
        II  |  I
        __________
        
        III |  IV
        
    """
    if x>0 and y>0:
        quad=4
    elif x<0 and y>0:
        quad=3
    elif x<0 and y<0:
        quad=2
    elif x>0 and y<0:
        quad=1
    return(quad)