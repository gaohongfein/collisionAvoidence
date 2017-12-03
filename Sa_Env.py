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
from rosgraph_msgs.msg import Clock


class Sa_Env():
    def __init__(self):
        self.vel_pub=rospy.Publisher('cmd_vel_command',Twist,queue_size=5)
        self.values_list=[0, 0]
        self.done=False
        self.test = 0
        self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        # self._reset()
        self.action_space=['f','l','r']
        self.action_n=len(self.action_space)
        self.min_range=0.5
        rospy.init_node('gym',anonymous=True)

    def _deal_scan_data(self, scan_date):
        data = scan_date.ranges
        state = [0, 0, 0, 0]
        done = False
        dis_data = []
        safe_range = 5
        reward = 20
        for i in range(0, 1300, 100):
            if data[i]>=safe_range:
                dis_data.append(safe_range)
            else:
                dis_data.append((int)(data[i]))
                state[3] = 1

            if data[i] < self.min_range:
                done = True
                reward = -500
            if data[i] < safe_range:
                reward = -20
            

        
        if dis_data[4] + dis_data[5] > dis_data[7] + dis_data[8]:
            state[0] = 1
        elif dis_data[4] + dis_data[5] < dis_data[7] + dis_data[8]:
            state[0] = 2
        else:
            state[0] = 0

        if data[600] < safe_range or data[620] < safe_range or data[580] < safe_range:
            state[1] = 1
        else:
            state[1] = 0

        if data[1200] < safe_range or data[20] < safe_range or data[1080] < safe_range:
            state[2] = 1
        else:
            state[2] = 0

        return state, done, reward












    def _step(self,action):
        # rospy.sleep(1.)
        print("test now",self.test)

        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            #resp_pause = pause.call()
            self.unpause()
        except (rospy.ServiceException) as e:
            pass

        twist = Twist()
        forward_v = 2
        if action==0:#forward
            twist.linear.x = forward_v
            twist.angular.z = 0.0
            self.vel_pub.publish(twist)
        elif action == 1:#left
            twist.linear.x = forward_v
            twist.angular.z = 0.785*4
            self.vel_pub.publish(twist)
        elif action == 2:
            twist.linear.x = forward_v
            twist.angular.z = -0.785*4
            self.vel_pub.publish(twist)
        # elif action == 3:
        #     twist.linear.x = -forward_v
        #     twist.angular.z = 0
        #     self.vel_pub.publish(twist)
        rospy.sleep(1)
        data = rospy.wait_for_message('/scan', LaserScan, timeout=5)
        # data_gazebo_state = rospy.wait_for_message('odom_diff', Odometry, timeout= 5)
        # print('odom_diff', int(data_gazebo_state.pose.pose.position.x), int(data_gazebo_state.pose.pose.position.y))
        print("data",len(data.ranges))

        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            self.pause()
        except (rospy.ServiceException) as e:
            pass

        state, self.done, reward = self._deal_scan_data(data)
        # if 15<int(data_gazebo_state.pose.pose.position.x)<18 and  0<int(data_gazebo_state.pose.pose.position.y)<2:
        #     reward = 100

        # if action == 1 or action == 2 or action == 3:
        #     reward = 0

        # if not self.done:
        #     if action == 0:
        #         reward = 5
        #     else:
        #         reward = 0
        # else:
        #     reward = -100
        return reward, self.done, state


    def _reset(self):
        Set_model = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        State = ModelState()
        State.model_name = "summitXl"
        State.pose.position.x = 5*np.random.random()
        State.pose.position.y = 5*np.random.random()-10
        State.pose.position.z = 0
        State.pose.orientation.z = 3.14*np.random.randn()
        State.reference_frame = "world"
        Set_done=Set_model(State)
        print(Set_done)

        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            self.unpause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/unpause_physics service call failed")


        data = rospy.wait_for_message('/scan', LaserScan, timeout=5)
        time = rospy.wait_for_message('/clock', Clock, timeout=5)

        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            self.pause()
        except (rospy.ServiceException) as e:
            pass

        state, self.done, reward= self._deal_scan_data(data)
        time = time.clock.secs





        return state, time

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



