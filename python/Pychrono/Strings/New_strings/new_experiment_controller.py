# -*- coding: utf-8 -*-
"""
Created on Wed Feb  9 09:54:30 2022

@author: dmulr
"""

"""
This module contains many controllers for Sphero(s) Bolt to be used
with ROS.
"""

###################################################
from JamoebaGatt.SpherosGatt import SpheroBolt, MultiSpheroBolt
from pid_controller.pid import PID
from numpy import zeros, ndarray
from math import atan2, sqrt, pi
from time import time, sleep
import rospy
from std_msgs.msg import String
###################################################


###################################################
# ------------------- Classes ------------------- #
###################################################


###################################################
class SpheroController:
    """PID controller of a single Sphero.
    Parameters
    ----------
    single_sphero : SpheroMini or SpheroBolt or int
    The Sphero instance to be controlled. If an int,
    this is the controller for a simulation.
    kp : float, optional
    Proportional gain of the PID.
    ki : float, optional
    Integral gain of the PID.
    kd : float, optional
    Derivative gain of the PID.
    is_alone : bool, optional
    Set to False if the controller is used in a swarm (multiple controller).
    Attributes
    ----------
    sphero : SpheroMini or SpheroBolt or None
    The Sphero instance that is controlled.
    at_id : int
    The Apriltag ID of the Sphero.
    x : int
    Current position in x.
    y : int
    Current position in y.
    tar_x : int
    Target position in x.
    tar_y : int
    Target position in y.
    _ulim : int
    Ultimate speed limit for the Sphero.
    _freq : float
    Maximum frequency to send command to the Sphero via Bluetooth.
    speed_ctl : PID
    The speed PID controller.
    last : float
    The time at which data was last sent to the Sphero via Bluetooth.
    """




    def __init__(self, single_sphero, kp=1., ki=0., kd=0., is_alone=True, speed=60):
        """Constructor of the SpheroController class.
        Parameters
        ----------
        single_sphero : SpheroMini or SpheroBolt or int
        The Sphero instance to be controlled. If an int,
        this is the controller for a simulation.
        kp : float, optional
        Proportional gain of the PID.
        ki : float, optional
        Integral gain of the PID.
        kd : float, optional
        Derivative gain of the PID.
        is_alone : bool, optional
        Set to False if the controller is used in a swarm (multiple controller).
        Yields
        ------
        SpheroController
        A PID controller for a single Sphero Mini/Bolt.
        """
        
        # Check if this is a simulated Sphero.
    if type(single_sphero) is int:
        # simulation
        self.sphero = None
        self.at_id = single_sphero
    else:
        # experiment
        self.sphero = single_sphero
        self.at_id = single_sphero.id
        
        # Initialize the PID speed controller.
        self.speed_ctl = PID(p=kp, i=ki, d=kd)
        self.speed_ctl.target = 0
        
        # Set the last data sending time as now.
        self.last = time()
        
        # If this is not a part of a swarm, start ROS
        # In MultiControlNode.py, is_alone = False
    if is_alone:
        rospy.init_node("Sphero_" + str(self.at_id), anonymous=False)
        # Each time the topic is updated, subscribe it and pass the topic msg into the callback func \
        # then do something as it does
        rospy.Subscriber("target", String, self.ros_set_target, queue_size=1)
        rospy.Subscriber("state", String, self.ros_callback, queue_size=1)
        rospy.spin()        
    self.x = self.y = 0
    self.tar_x = self.tar_y = 0
    self._ulim = 255
    self._freq = 2
    self.my_speed = speed
    self.m = 1
    self.control_mode = "shape_formation"
    self.geometry = 'circle'
    self.xc = 0
    self.yc = 0
    self.radius = 1

        
    class controller:
        def __init__(self,m,control_mode,geometry,xc,yc,radius):
    
            self.m = m
            self.control_mode = control_mode
            self.geometry = geometry
            self.xc = xc
            self.yc = yc
            self.radius = radius       
        
        
        
        
        ######### SHAPE FORMATION #########
        if self.control_mode=="shape_formation":
        
            if self.geometry=='circle':
                self.segments = 0
                self.a = self.xc
                self.b = self.yc
                self.R = self.radius
            
            if self.geometry=='pacman':
                self.a = self.xc
                self.b = self.yc
                self.R = self.radius
                self.segments = np.array([[self.a,self.b,self.R*np.cos(np.pi/4),self.R*np.sin(np.pi/4)],[self.a,self.b,self.R*np.cos(-np.pi/4),self.R*np.sin(-np.pi/4)]])
                theta=np.linspace(np.pi/4,7*np.pi/4,100)
                self.xp=[]
                self.yp=[]
                for i in range(len(theta)):
                    self.xp.append(self.R*np.cos(theta[i])+self.a)
                    self.yp.append(self.R*np.sin(theta[i])+self.b)
                
                self.xp.append(self.R*np.cos(-np.pi/4))
                self.xp.append(self.a)
                self.xp.append(self.R*np.cos(np.pi/4))
                
            
                self.yp.append(self.R*np.sin(-np.pi/4))
                self.yp.append(self.a)
                self.yp.append(self.R*np.sin(np.pi/4))     
                   
            if self.geometry=='square':
                w=1.5
                h=1
                self.xp=[w,-w,-w,w,w]
                self.yp=[h,h,-h,-h,h]
                (self.segments)=create_segment(self.xp,self.yp)
        

    def create_segment(self,x,y):
         """ Create segment matrix for points of R-function """
         seglen=len(x)
         segments=np.zeros((seglen-1,4))
         for i in range(seglen-1):
         #[x1,y1,x2,y2]
         #[x2,y2,x3,y3]
             segments[i,0]=x[i]
             segments[i,1]=y[i]
             segments[i,2]=x[i+1]
             segments[i,3]=y[i+1]
         return(segments)  

    def phi_circle(self,x,y,a,b,R):
        """ Normalized distance function of a circle """
        phi = (R**2 - (x-a)**2 - (y-b)**2)
        return(abs(phi))
     
        
    def dphix_circle(self,x,y,a,b,R):
        """ Derivative of the distance function of cirlce wrt x """
        return(R*(x-a)*np.sign((R/2)*(- R**2 +(a-x)**2 + (b-y)**2)))
    
    
    def dphiy_circle(self,x,y,a,b,R):
        """ Derivative of the distance function of cirlce wrt y """
        return(R*(y-b)*np.sign((R/2)*(- R**2 +(a-x)**2 + (b-y)**2)))
    
    
    def phi_quarter_circle(self,x,y,a,b,R):
        """ Distance function for partial circles """
        theta1=np.pi/4
        theta2=-np.pi/4
        x1=R*np.cos(theta1)
        y1=R*np.sin(theta1)

        x2=R*np.cos(theta2)
        y2=R*np.sin(theta2)

        T=self.phi_line_(x,y,x1,y1,x2,y2)
        f=self.phi_circle(x,y,self.a,self.b,self.R)
        
        phi1=self.Trim(f,T)
        
        return(phi1)
    
    
    def dphix_quarter_circle(self,x,y,a,b,R):
        """ Distance function for partial circles wrt x  """
        theta1=np.pi/4
        theta2=-np.pi/4
        x1=R*np.cos(theta1)
        y1=R*np.sin(theta1)

        x2=R*np.cos(theta2)
        y2=R*np.sin(theta2)
        
        T=self.phi_line_(x,y,x1,y1,x2,y2)
        Tx=self.dphix_line_(x,y,x1,y1,x2,y2)
        
        
        f=self.phi_circle(x,y,self.a,self.b,self.R)
        fx=self.dphix_circle(x,y,self.a,self.b,self.R)

        
        phi1=self.Trimx(f,T,fx,Tx)
        
        return(phi1)    
    

    def dphiy_quarter_circle(self,x,y,a,b,R):
        """ Distance function for partial circles wrt y  """
        theta1=np.pi/4
        theta2=-np.pi/4
        x1=R*np.cos(theta1)
        y1=R*np.sin(theta1)

        x2=R*np.cos(theta2)
        y2=R*np.sin(theta2)
        
        T=self.phi_line_(x,y,x1,y1,x2,y2)
        Ty=self.dphiy_line_(x,y,x1,y1,x2,y2)
        
        
        f=self.phi_circle(x,y,self.a,self.b,self.R)
        fy=self.dphiy_circle(x,y,self.a,self.b,self.R)

        
        phi1=self.Trimy(f,T,fy,Ty)
        
        return(phi1)


    
    
    def Trim(self,f,t):
        """ Trim function for two functions  """
        phi=np.sqrt(t**2 + f**4)
        return(np.sqrt(f**2 + ((phi-t)/2)**2))

    def Trimx(self,f,t,fx,tx):
        """Derivative Trim function for two functions wrt x  """
        term1 = (2*(f**3)*fx + tx*t)/(np.sqrt(f**4 + t**2)) - tx
        term2 = np.sqrt(f**4 + t**2)/2 - t/2
        term3 = f*fx
        term4 = np.sqrt((np.sqrt(f**4 + t**2)/2 - t/2)**2 + f**2)
        
        return((0.5*term1*term2 +term3)/term4)
        
    def Trimy(self,f,t,fy,ty):
        """Derivative Trim function for two functions wrt y """
        term1 = (2*(f**3)*fy + ty*t)/(np.sqrt(f**4 + t**2)) - ty
        term2 = np.sqrt(f**4 + t**2)/2 - t/2
        term3 = f*fy
        term4 = np.sqrt((np.sqrt(f**4 + t**2)/2 - t/2)**2 + f**2)
        
        return((0.5*term1*term2 +term3)/term4)    
    
##############################################################################   

    def phi_line_(self,x,y,x1,y1,x2,y2):
        """ Distance function of a line """
        L = np.sqrt((x2-x1)**2 + (y2-y1)**2)
        return(((x-x1)*(y2-y1)-(y-y1)*(x2-x1))/L)


    def dphix_line_(self,x,y,x1,y1,x2,y2):
        """ Derivative distance function of a line wrt x """
        L = np.sqrt((x2-x1)**2 + (y2-y1)**2)
        return((-y1+y2)/L)


    def dphiy_line_(self,x,y,x1,y1,x2,y2):
        """ Derivative distance function of a line wrt y """
        L = np.sqrt((x2-x1)**2 + (y2-y1)**2)
        return((x1-x2)/L)


    def trim(self,x,y,x1,y1,x2,y2):
        """ Trim function """
        L = np.sqrt((x2-x1)**2 + (y2-y1)**2)
        xc = np.array([(x2+x1)/2,(y2+y1)/2])
        t = (1/L)*((L/2)**2 - ((x-xc[0])**2 + (y-xc[1])**2))    
        return(t)


    def dtrimx(self,x,y,x1,y1,x2,y2):
        """ Derivative of Trim function wrt x """
        xc = np.array([(x2+x1)/2,(y2+y1)/2]) 
        L = np.sqrt((x2-x1)**2 + (y2-y1)**2)
        return(-(x-xc[0])/ (L*np.sqrt((x-xc[0])**2 + (y-xc[1])**2)))

    def dtrimy(self,x,y,x1,y1,x2,y2):
        """ Derivative of Trim function wrt y """
        xc = np.array([(x2+x1)/2,(y2+y1)/2]) 
        L = np.sqrt((x2-x1)**2 + (y2-y1)**2)
        return(-(y-xc[1])/ (L*np.sqrt((x-xc[0])**2 + (y-xc[1])**2)))

    def phi_line(self,x,y,x1,y1,x2,y2):
        """ Trimmed line segment"""
        t = self.trim(x,y,x1,y1,x2,y2)
        f = self.phi_line_(x,y,x1,y1,x2,y2)
        rho = np.sqrt(t**2 + f**4)
        return(np.sqrt(f**2 + ((rho-t)/2)**2))

    def dphix_line(self,x,y,x1,x2,y1,y2):
        """ Derivative of trimmed line segment wrt x"""
        ff = self.phi_line_(x,y,x1,x2,y1,y2)
        dfx = self.dphix_line_(x,y,x1,x2,y1,y2)
        tf = self.trim(x,y,x1,x2,y1,y2)
        dtx = self.dtrimx(x,y,x1,x2,y1,y2)
        term1 = 0.5*((2 * ff**3 * dfx + tf*dtx)/(np.sqrt(ff**4 + tf**2)) - dtx)*((np.sqrt(ff**4 + tf**2)-tf)/2)
        term2 = ff*dfx
        term3 = np.sqrt((((np.sqrt(ff**4 + tf**2)-tf)/2)**2) + ff**2)  
        return((term1+term2)/term3)

    def dphiy_line(self,x,y,x1,x2,y1,y2):
        """ Derivative of trimmed line segment wrt y"""
        ff = self.phi_line_(x,y,x1,x2,y1,y2)
        dfy = self.dphiy_line_(x,y,x1,x2,y1,y2)
        tf = self.trim(x,y,x1,x2,y1,y2)
        dty = self.dtrimy(x,y,x1,x2,y1,y2)
        term1 = 0.5*((2 * ff**3 * dfy + tf*dty)/(np.sqrt(ff**4 + tf**2)) - dty)*((np.sqrt(ff**4 + tf**2)-tf)/2)
        term2 = ff*dfy
        term3 = np.sqrt((((np.sqrt(ff**4 + tf**2)-tf)/2)**2) + ff**2)    
        return((term1+term2)/term3)   


    def phi_segments(self,x,y,segments):
        """ R equivelent of trimmed line segments"""
        R=0
        for i in range(len(segments[:,0])):
            R = R + 1/self.phi_line(x,y,segments[i,0],segments[i,1],segments[i,2],segments[i,3])**self.m
        R = 1/R**(1/self.m)
        return(R)

    def dphix_segments(self,x,y,segments):
        """ Derivative of R equivelent of trimmed line segments wrt x"""

        term1=0
        term2=0
        term3=0
        for i in range(len(segments[:,0])):
            term1=-self.m*(self.phi_line(x,y,segments[i,0],segments[i,1],segments[i,2],segments[i,3])**-self.m) * self.dphix_line(x,y,segments[i,0],segments[i,1],segments[i,2],segments[i,3]) + term1
            term2=(self.phi_line(x,y,segments[i,0],segments[i,1],segments[i,2],segments[i,3])**-self.m)**(-1/self.m) + term2
            term3=self.m*(self.phi_line(x,y,segments[i,0],segments[i,1],segments[i,2],segments[i,3])**-self.m) + term3      
        R=(-term1*term2/term3)
        return(R)

    def dphiy_segments(self,x,y,segments):
        """ Derivative of R equivelent of trimmed line segments wrt y"""

        term1=0
        term2=0
        term3=0
        for i in range(len(segments[:,0])):
            term1=-self.m*(self.phi_line(x,y,segments[i,0],segments[i,1],segments[i,2],segments[i,3])**(-self.m)) * self.dphiy_line(x,y,segments[i,0],segments[i,1],segments[i,2],segments[i,3]) + term1
            term2=(self.phi_line(x,y,segments[i,0],segments[i,1],segments[i,2],segments[i,3])**(-self.m))**(-1/self.m) + term2
            term3=self.m*(self.phi_line(x,y,segments[i,0],segments[i,1],segments[i,2],segments[i,3])**(-self.m)) + term3      
        R=(-term1*term2/term3)
        return(R)
    
    
    
    
    
    def FX(self,x,y):
        """ Single function to call on derivative wrt x"""
        if self.geometry=='circle':
            Fx = self.dphix_circle(x,y,self.a,self.b,self.R)
            
            
        if self.geometry=='pacman':
            f1 = self.phi_quarter_circle(x,y,self.a,self.b,self.R)
            f1x = self.dphix_quarter_circle(x,y,self.a,self.b,self.R)
            
            f2 = self.phi_segments(x,y,self.segments)
            f2x = self.dphix_segments(x,y,self.segments)
            
            Fx = (f1**self.m + f2**self.m)**(-1/self.m)*f1*f2x + (f1**self.m + f2**self.m)**(-1/self.m)*f2*f1x - ((self.m*(f2**self.m)*f2x)/f2 + (self.m*(f1**self.m)*f1x)/f1)*((f1**self.m) +f2**self.m)**(-1/self.m)*f1*f2/(self.m*(f1**self.m + f2**self.m))
            
        else:
            Fx = self.dphix_segments(x,y,self.segments)
            
        return(Fx)


    def FY(self,x,y):
        """ Single function to call on derivative wrt y"""
        if self.geometry=='circle':
            Fy = self.dphiy_circle(x,y,self.a,self.b,self.R)
            
        if self.geometry=='pacman':
            f1 = self.phi_quarter_circle(x,y,self.a,self.b,self.R)
            f1y = self.dphiy_quarter_circle(x,y,self.a,self.b,self.R)
        
            f2 = self.phi_segments(x,y,self.segments)
            f2y = self.dphiy_segments(x,y,self.segments)
            
            Fy = (f1**self.m + f2**self.m)**(-1/self.m)*f1*f2y + (f1**self.m + f2**self.m)**(-1/self.m)*f2*f1y - ((self.m*(f2**self.m)*f2y)/f2 + (self.m*(f1**self.m)*f1y)/f1)*((f1**self.m) +f2**self.m)**(-1/self.m)*f1*f2/(self.m*(f1**self.m +f2**self.m))
            
        else:
            Fy = self.dphiy_segments(x,y,self.segments)
    
        return(Fy)    
    
    
    def F(self,x,y):
       """ Single function to call field"""
       if self.geometry=='circle':
           F = self.phi_circle(x,y,self.a,self.b,self.R)
            
       if self.geometry=='pacman':
           f = self.phi_quarter_circle(x,y,self.a,self.b,self.R)
           f2=self.phi_segments(x,y,self.segments)
           F = (f*f2)/((f**self.m +f2**self.m)**(1/self.m))
           
         
       else:
           F = self.phi_segments(x,y,self.segments)
           
       return(F)

		# Check if this is a simulated Sphero.
		if type(single_sphero) is int:
			# simulation
			self.sphero = None
			self.at_id = single_sphero
		else:
			# experiment
			self.sphero = single_sphero
			self.at_id = single_sphero.id

		# Initialize the PID speed controller.
		self.speed_ctl = PID(p=kp, i=ki, d=kd)
		self.speed_ctl.target = 0

		# Set the last data sending time as now.
		self.last = time()

		# If this is not a part of a swarm, start ROS
		# In MultiControlNode.py, is_alone = False
		if is_alone:
			rospy.init_node("Sphero_" + str(self.at_id), anonymous=False)
			# Each time the topic is updated, subscribe it and pass the topic msg into the callback func \
			# then do something as it does
			rospy.Subscriber("target", String, self.ros_set_target, queue_size=1)
			rospy.Subscriber("state", String, self.ros_callback, queue_size=1)
			rospy.spin()



	def get_error_length(self):
		"""Method to get the length between the current position and the target position.
		Returns
		-------
		float
			The length between the current position and the target position.
		"""

		dx = self.tar_x - self.x
		dy = self.tar_y - self.y
		return sqrt(dx**2 + dy**2)

	def get_error_angle(self):
		"""Method to get the orientation of the target relative to the current position.
		Returns
		-------
		float
			The orientation of the target relative to the current position.
		"""

		dx = self.tar_x - self.x
		dy = self.tar_y - self.y
		ang = 180*atan2(dy, dx)/pi
		return ang % 360

	def ros_set_target(self, data):
		"""Method used to update the closed loop target positions. Used with a ROS Subscriber.
		Parameters
		----------
		data : String
			Message published on the /target topic containing the position of the target tag.
		"""

		data_dict = eval(data.data)

		# self.tar_x = data_dict[str(self.at_id)][0]
		# self.tar_y = data_dict[str(self.at_id)][1]

		for key in data_dict:
			self.tar_x = data_dict[key][0]
			self.tar_y = data_dict[key][1]



	def ros_callback(self, data):
		"""Method used to run an iteration of the control loop. Used with a ROS Subscriber.
		Parameters
		----------
		data : String
			Message published on the /state topic containing the position of all tags.
		"""

		data_dict = eval(data.data)
        # Set the current positions
        self.x = data_dict[str(self.at_id)][0]
        self.y = data_dict[str(self.at_id)][1]
        self.t_k_1 = time()
        self.PHI = self.controller(self.m,self.control_mode,self.geometry,0,0,self.radius)
        
        VY=self.PHI.FY(self.x,self.y)
        VX=self.PHI.FX(self.x,self.y)  
        mag = np.sqrt(VY ** 2 + VX ** 2)  # magnitude
        VY = VY / mag  # normalize  Y
        VX = VX / mag  # normalize X
        Vx = -self.alpha * VX # - self.beta * vx
        Vy = -self.alpha * VY  # - self.beta * vy
        V = np.sqrt(Vx ** 2 + Vy ** 2)
        theta = np.nan_to_num(180 * np.arctan2(Vy, Vx) / np.pi) % 360  # set heading angle
        speed = V
        ang = theta

		# Send it to the sphero
		freq = 1.0/(time() - self.last)
		if freq <= self.freq:
			self.last = time()

			if self.sphero is None:
				print("\r[{0}] ---> {1:3d}° at {2} % speed.".format(self.at_id, head, int(100 * speed / 255)))
			else:
				self.sphero.send(speed=speed, heading=head)
				print("\r[{0}] ---> {1:3d}° at {2} % speed.".format(self.at_id, head, int(100 * speed / 255)))




	@property
	def ulim(self):
		return self._ulim

	@ulim.setter
	def ulim(self, lim):
		self._ulim = max(0, min(lim, 255))

	@property
	def freq(self):
		return self._freq

	@freq.setter
	def freq(self, f):
		self._freq = max(0., min(f, 20.))
###################################################


###################################################
class MultiSpheroController:
	"""PID controller of a multiple Spheros.
	Parameters
	----------
	multi_sphero : MultiSpheroMini or MultiSpheroBolt or tuple of int
		The Spheros swarm instance to be controlled. If a tuple,
		this is the controllers for a simulation.
	kp : float, optional
		Proportional gain of the PID.
	ki : float, optional
		Integral gain of the PID.
	kd : float, optional
		Derivative gain of the PID.
	Attributes
	----------
	spheros : ndarray of SpheroMini or ndarray of SpheroBolt or tuple of int
		An numpy array of the SpheroMini/SpheroBolt or of the Apriltag IDs if this is a simulation.
	num : int
		Number of Sphero in the swarm.
	ids : tuple of int
		A tuple containing the Apriltag ID of the Spheros.
	swarm : MultiSpheroMini or MultiSpheroBolt or None
		The Sphero swarm instance or None if this is a simulation.
	controllers : ndarray of SpheroController
		An numpy array of the SpheroController.
	last : float
		The time at which data was last received from the /state topic.
	last_tar : float
		The time at which data was last received from the /target topic.
	freq_tar : float
		Frequency at which the /target topic receives messages.
	"""

	def __init__(self, multi_sphero, kp=1., ki=0., kd=0., start_accel=False, start_sensor=False, speed=60):
		"""Constructor of the SpheroController class.
		Parameters
		----------
		multi_sphero : MultiSpheroMini or MultiSpheroBolt or tuple of int
			The Spheros swarm instance to be controlled. If a tuple,
			this is the controllers for a simulation.
		kp : float, optional
			Proportional gain of the PID.
		ki : float, optional
			Integral gain of the PID.
		kd : float, optional
			Derivative gain of the PID.
		Yields
		------
		MultiSpheroController
			A PID controller for multiple Sphero Mini/Bolt.
		"""

		if type(multi_sphero) is tuple:
			# simulation
			self.spheros = self.ids = multi_sphero
			self.num = len(multi_sphero)
			self.swarm = None
		else:
			# experiment
			self.swarm = multi_sphero
			self.spheros = self.swarm.spheros
			self.num = self.swarm.num
			self.ids = self.swarm.ids

		# create the template as zeros, data type=SpheroController
		# Later, substitute SpheroController instances
		self.controllers = zeros(self.num, dtype=SpheroController)
		# Here each SpheroController instances are created for each Sphero
		# is_alone=False, so NO ROS functions in SpheroController at this time and no control command is sent
		for i in range(self.num):
			self.controllers[i] = SpheroController(self.spheros[i], kp, ki, kd, is_alone=False, speed=speed)

		self.last = self.last_tar = time()
		self.freq_tar = 0
		print("Frequencies :\n| Apriltags |  Target  | Timeout Error |")

		# Here ROS starts fro multi sphero control
		node_name = "Spheros" + '_'.join(str(param) for param in self.ids)
		rospy.init_node(node_name, anonymous=False)
		# Start publishing accel
		if start_accel:
			for s in self.spheros:
				s.stream_accel()
		# Start publishing sensor data
		if start_sensor:
			for s in self.spheros:
				s.stream_sensors()
		# You can cancel notify and still sensor data is streaming...
		if start_accel or start_sensor:
			for s in self.spheros:
				s.cancel_sensor()
				# print('Thread closed.')
		sleep(3)
		rospy.Subscriber("target", String, self.ros_set_target, queue_size=1)
		rospy.Subscriber("state", String, self.ros_callback, queue_size=1)
		rospy.spin()

	def __del__(self):
		""" Destructor of the SpheroController class. """

		print("\b\b  \n")

	def ros_set_target(self, data):
		"""Method used to update the closed loop target positions. Used with a ROS Subscriber.
		Parameters
		----------
		data : String
			Message published on the /target topic containing the position of the target tag.
		"""

		self.freq_tar = 1./(time() - self.last_tar)
		self.last_tar = time()
		for ctl in self.controllers:
			ctl.ros_set_target(data)

	def ros_callback(self, data):
		"""Method used to run an iteration of the control loop. Used with a ROS Subscriber.
		Parameters
		----------
		data : String
			Message published on the /state topic containing the position of all tags.
		"""

		freq = 1./(time() - self.last)
		self.last = time()
		print("\r| {0:2.2f} Hz  | {1:2.2f} Hz | {2} Errors    |      ".format(freq, self.freq_tar, self.get_sum_error()), end='')

		for ctl in self.controllers:
			ctl.ros_callback(data)
		# Wait for a sec
		# sleep(1)
        
        
