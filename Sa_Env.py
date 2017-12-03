import rospy
import time
import numpy as np



from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from tf.transformations import euler_from_quaternion


class Sa_Env():
    def __init__(self):
        self.vel_pub=rospy.Publisher('cmd_vel_command',Twist,queue_size=5)
        self.values_list=[4, 4, 4, 4]
        self.done=False
        self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self._reset()
        self.action_space=['f','l','r']
        self.action_n=len(self.action_space)
        self.min_range=0.5
        rospy.init_node('gym',anonymous=True)




    def _step(self,action):
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            self.unpause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/unpause_physics service call failed")

        twist = Twist()

        if action==0:#forward
            twist.linear.x = 1
            twist.angular.z = 0.0
            self.vel_pub.publish(twist)
        elif action == 1:#left
            twist.linear.x = 0
            twist.angular.z = 0.785*4
            self.vel_pub.publish(twist)
        elif action == 2:
            twist.linear.x = 0
            twist.angular.z = -0.785*4
            self.vel_pub.publish(twist)
        rospy.sleep(1)
        twist.linear.x = 0
        twist.angular.z = 0
        self.vel_pub.publish(twist)


        data = None
        while data is None:
            try:
                self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callbacks)
                data = self.values_list
            except:
                pass
        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            #resp_pause = pause.call()
            self.pause()
        except (rospy.ServiceException) as e:
            pass

        # done = self.values_list[0] < self.min_range or self.values_list[1] < \
        #                            self.min_range or \
        #       self.values_list[2] < self.min_range or self.values_list[3] < self.min_range \
        #       or self.values_list[4] < self.min_range
        state = self.values_list
        #rospy.sleep(1.)

        if not self.done:
            if action == 0:
                reward = 5
            else:
                reward = 0
        else:
            reward = -300
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            self.unpause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/unpause_physics service call failed")

        return reward, self.done, state

    def laser_callbacks(self,msg):
        min_real_distance=4
        for i in range(900,299,-30):
            if msg.ranges[i] < self.min_range:
                    self.done=True
                    break
            
            if msg.ranges[i] < min_real_distance:
                min_real_distance=msg.ranges[i]
            if i % 150 == 0:
                
                if min_real_distance >= 4:
                    self.values_list[5-i/150]=4
                    min_real_distance = 4
                else:
                    self.values_list[5-i/150]=((int)(min_real_distance))
                    min_real_distance = 4




        #self.values_list = [msg.ranges[300], msg.ranges[450], msg.ranges[600], msg.ranges[750],msg.ranges[900]]
        #print(self.values_list)


    def _reset(self):
        Set_model = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        State = ModelState()
        State.model_name = "summitXl"
        State.pose.position.x = 5*np.random.randn()
        State.pose.position.y = 4*np.random.randn()
        State.pose.position.z = 0

        State.pose.orientation.z = 3.14*np.random.randn()
        

        State.reference_frame = "world"
        Set_done=Set_model(State)
        print(Set_done)

        # Unpause simulation to make observation
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            #resp_pause = pause.call()
            self.unpause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/unpause_physics service call failed")


        data = None
        while data is None:
            try:
                self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callbacks)
                data = self.values_list
                state=data
            except:
                pass
        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            #resp_pause = pause.call()
            self.pause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/pause_physics service call failed")
        self.done=False




        return state

    #def _check_

'''if __name__=="__main__":
    a = Sa_Env()
    sum_reward = 0
    while not rospy.is_shutdown():
        state_now=a._reset()
        print('state_now:',state_now)
        rospy.sleep(1.)

    while not rospy.is_shutdown():

        ireward,done,state=a._step(0)

        sum_reward=sum_reward+ireward
        print(sum_reward)
        print('state:',state)
        rospy.sleep(1.)'''



