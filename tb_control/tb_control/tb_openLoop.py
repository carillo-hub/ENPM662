import sys
from   datetime          import datetime
import matplotlib.pyplot as plt
import rclpy
from   rclpy.node        import Node
from   geometry_msgs.msg import Twist, Pose
from rclpy.qos import QoSProfile

class MyNode(Node):
    
    def __init__(self, arg1):
                        
        # Initialize Node
        super().__init__(arg1)
        print("\n")        
        self.get_logger().info(f'ROS2 Node is running with argument {arg1}\n')
        self.goal_reached = False
        
        # Set scenario
        if arg1.lower() == "scenario_1":
            self.scenario_1 = True
            self.scenario_2 = False
        elif arg1.lower() == "scenario_2":
            self.scenario_1 = False
            self.scenario_2 = True
        else: 
            print("\n")
            print("****************************************************")
            print("** Must specify CLI 'Scenario_1' or 'Scenario_2'! **")
            print("**     Invalid argument entered. Exiting...       **")
            print("****************************************************")
            print("\n")
            self.destroy_node()
            return
        
        
        # Initialize control parameters
        self.init_time    = datetime.now()
        self.time_elapsed = 0
        self.dist_trav    = 0     # meters
        self.moving_      = True  # control flag
        self.exit_ctr     = 0
        
        if self.scenario_1:
            self.max_time   = 10 # sec
            self.max_dist   = 1  # meters
            self.velocity   = self.max_dist / self.max_time #constant velocity for scenario 1
        elif self.scenario_2:
            self.accel      = 0.1 # m/s2   
            self.velocity   = 0.1 # m/s - initial velocity
            self.max_vel    = 0.5 # m/s  
            self.dist_x2    = 2   # meters
            self.op_mode    = "accelerate"   # accel, const, or decel mode
            self.start_x2   = 0   # to mark current dist once in 'constant' op_mode
        
        # Initialize Publisher
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pub_rate   = 1.0
        self.timer_     = self.create_timer(self.pub_rate, self.publish_message)
        self.msg_       = Twist()

        # Initialize Pose Plot tools
        self.x_pose     = [0]
        self.time_step  = [0]
        
    # Callback Function
    def publish_message(self):
        self.msg_.linear.x = self.velocity
        self.publisher_.publish(self.msg_)
        self.get_logger().info(f'     Current Speed: {self.msg_.linear.x} [m/s]')              
        self.time_elapsed = (datetime.now() - self.init_time).total_seconds()
        self.get_logger().info(f'      Time elapsed: {self.time_elapsed} [sec]')              
                
        if self.moving_:                                      
            # Scenario 1: constant velocity travelling 1 meter for ~10-sec
            if self.scenario_1:                                       
                self.dist_trav = self.velocity * self.time_elapsed #controller eq x = v*t         
                if self.dist_trav >= self.max_dist:
                    self.goal_reached = True
            # Scenario 2: accelerate until max velocity, then decelerate to 0
            elif self.scenario_2:
                self.get_logger().info(f'           Op Mode: {self.op_mode}')  
                dt = self.pub_rate
                if self.op_mode != "constant": # accel or decel              
                    self.dist_trav   += (self.velocity * dt) + (0.5*self.accel*dt**2) #dx = v*dt + (1/2)a*dt^2
                    self.velocity    += (self.accel    * dt)                          #v1 = v0 + a*dt                    
                    if self.velocity >= self.max_vel:
                        self.op_mode  = "constant"
                        self.start_x2 = self.dist_trav 
                    elif self.velocity <= 0:
                        self.goal_reached = True
                else: # constant vel mode 
                    self.dist_trav += (self.velocity * dt)
                    if self.dist_trav - self.start_x2 >= self.dist_x2:
                        self.op_mode  = "decelerate"
                        self.accel    = -1 * self.accel                    
            
            # Report distance travelled
            self.get_logger().info(f'Distance travelled: {self.dist_trav} [meters]')
            print("\n")
                    
            if self.goal_reached:
                print("*******************")
                print("** Goal Reached! **")
                print("*******************")
                print("\n")
                self.velocity = 0.0
                self.moving_  = False
                
        
        if self.velocity == 0.0:
            self.exit_ctr += 1
            if self.exit_ctr >= 10:
                print("\nRobot has been idle for >10 seconds. Exiting control node and plotting results.")
                self.destroy_node()
                self.plot_pose()
                     
                                        
        # Add current position to pose tracker        
        self.x_pose.append(self.dist_trav)
        self.time_step.append(self.time_elapsed)  
            
    
    def plot_pose(self):
        plt.figure()
        plt.plot(self.time_step, self.x_pose, marker='o')
        plt.title('Turtlebot Pose over Time')
        plt.xlabel('Time in Simulation [sec]')
        plt.ylabel('X Pose (m)')
        plt.grid(True)
        plt.xlim(left=0)
        plt.ylim(bottom=0)
        plt.show()


def main(arg1=None):

    if len(sys.argv) > 1:
        arg1 = sys.argv[1]
    else:
        print("\n")
        print("***********************************************")
        print("** No CLI entered. Defaulting to Scenario_1! **")
        print("***********************************************")        
        print("\n")
        
        arg1 = "Scenario_1"
    
    rclpy.init()
         
    node = MyNode(arg1)

    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == '__main__':
    
    main()
