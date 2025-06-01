#!/usr/bin/env python3
import rospy
import numpy as np
import random
import os
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState

class QLearningFastLidar:
    def __init__(self):
        rospy.init_node('q_learning_fast_lidar')

        self.cmd_pub = rospy.Publisher('/robot/cmd_vel', Twist, queue_size=10)
        self.goal_pub = rospy.Publisher('/goal_reached', Float32, queue_size=10)

        rospy.Subscriber('/fast_lidar/sectors', Float32MultiArray, self.lidar_callback)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)

        self.reward_pub = rospy.Publisher('/episode_reward', Float32, queue_size=10)

        self.lidar_data = None
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.last_distance_to_goal = None

        self.q_table_file = os.path.expanduser('~/.ros/q_table.npy')
        self.q_table = np.load(self.q_table_file) if os.path.exists(self.q_table_file) else np.zeros((27, 3))

        self.learning_rate = 0.1
        self.discount = 0.9
        self.epsilon = 0.2
        self.epsilon_min = 0.01
        self.epsilon_decay = 0.995

        self.max_steps = 300  # aumentamos os passos por episódio
        self.episode_count = 0

        self.goal_position = (6.0, -7.0)
        self.goal_tolerance = 0.5

    def lidar_callback(self, msg):
        self.lidar_data = msg.data

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

    def reached_goal(self):
        dx = self.goal_position[0] - self.robot_x
        dy = self.goal_position[1] - self.robot_y
        return np.hypot(dx, dy) < self.goal_tolerance

    def get_distance_to_goal(self):
        dx = self.goal_position[0] - self.robot_x
        dy = self.goal_position[1] - self.robot_y
        return np.hypot(dx, dy)

    def get_state(self, sectors):
        right = min(sectors[0:10])
        front = min(sectors[10:20])
        left = min(sectors[20:30])

        def discretize(d):
            if d < 0.5:
                return 0
            elif d < 1.0:
                return 1
            else:
                return 2

        r, f, l = discretize(right), discretize(front), discretize(left)
        return l * 9 + f * 3 + r

    def select_action(self, state):
        if random.random() < self.epsilon:
            return random.randint(0, 2)
        return int(np.argmax(self.q_table[state]))

    def execute_action(self, action):
        cmd = Twist()
        if action == 0:
            cmd.angular.z = 0.5  # virar à esquerda
        elif action == 1:
            cmd.linear.x = 0.3  # ir para frente
        elif action == 2:
            cmd.angular.z = -0.5  # virar à direita
        self.cmd_pub.publish(cmd)

    def compute_reward(self, state, action):
        r = state % 3
        f = (state // 3) % 3
        l = state // 9

        reward = 0.0

        if f == 0 and action == 1:
            self.reset_robot_position()
            return -10.0  # colisão
        elif f == 1 and action == 1:
            reward = -3.0  # perto do obstáculo, mas ainda não colidiu
        elif f == 2 and action == 1:
            reward = 1 # seguro para avançar
        elif (l == 0 and action == 0) or (r == 0 and action == 2):
            reward = -1.0  # tentativa de virar para lugar bloqueado

        dist_now = self.get_distance_to_goal()
        if self.last_distance_to_goal is not None:
            if dist_now < self.last_distance_to_goal:
                reward += 1  # está se aproximando da meta
            else:
                reward -= 1  # está se afastando da meta

        self.last_distance_to_goal = dist_now
        return reward

    def reset_robot_position(self):
        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            state = ModelState()
            state.model_name = 'mobile_rob_dev_sim'
            state.pose.position.x = 3.0
            state.pose.position.y = 0.0
            state.pose.position.z = 3.0
            state.pose.orientation.w = 1.0
            state.reference_frame = 'world'
            set_state(state)
            rospy.loginfo("Resetando posição do robô.")
        except rospy.ServiceException as e:
            rospy.logerr(f"Erro ao resetar: {e}")

    def run_episode(self):
        step = 0
        done = False
        last_state = None
        last_action = None
        self.last_distance_to_goal = self.get_distance_to_goal()
        total_reward = 0.0

        while step < self.max_steps and not rospy.is_shutdown():
            if self.lidar_data is None:
                rospy.sleep(0.1)
                continue

            state = self.get_state(self.lidar_data)
            action = self.select_action(state)
            self.execute_action(action)

            if self.reached_goal():
                reward = 10.0
                done = True
                rospy.loginfo("Meta atingida!")
                self.reset_robot_position()
            else:
                reward = self.compute_reward(state, action)

            total_reward += reward

            if last_state is not None:
                old_q = self.q_table[last_state, last_action]
                max_q = np.max(self.q_table[state])
                new_q = (1 - self.learning_rate) * old_q + self.learning_rate * (reward + self.discount * max_q)
                self.q_table[last_state, last_action] = new_q

            last_state = state
            last_action = action
            step += 1
            rospy.sleep(0.2)

            goal_reached = 1.0 if self.reached_goal() else 0.0
            self.goal_pub.publish(Float32(data=goal_reached))

            if done or reward <= -10.0:
                break

        self.reward_pub.publish(Float32(data=total_reward))  # Publica recompensa total no RQt
        self.epsilon = max(self.epsilon_min, self.epsilon * self.epsilon_decay)
        np.save(self.q_table_file, self.q_table)
        self.episode_count += 1
        rospy.loginfo(f"Episódio {self.episode_count} finalizado | ε: {self.epsilon:.3f}")

        if self.episode_count % 10 == 0:
            rospy.logwarn("Reinicializando.")
            
            if total_reward < 0:
                self.q_table[last_state, last_action] *= 0.8  
            
            self.reset_robot_position()



    def run(self):
        while not rospy.is_shutdown():
            self.run_episode()

if __name__ == '__main__':
    try:
        node = QLearningFastLidar()
        node.run()
    except rospy.ROSInterruptException:
        pass
