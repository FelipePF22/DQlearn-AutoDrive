#!/usr/bin/env python3
import rospy
import numpy as np
import random
import os
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class QLearningFastLidar:
    def __init__(self):
        rospy.init_node('q_learning_fast_lidar')
        self.cmd_pub = rospy.Publisher('/robot/cmd_vel', Twist, queue_size=10)
        self.lidar_data = None

        rospy.Subscriber('/fast_lidar/sectors', Float32MultiArray, self.lidar_callback)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)

        # Q-learning setup
        self.q_table_file = os.path.expanduser('~/.ros/q_table.npy')
        self.q_table = np.load(self.q_table_file) if os.path.exists(self.q_table_file) else np.zeros((27, 3))

        self.learning_rate = 0.1
        self.discount = 0.9
        self.epsilon = 0.2
        self.epsilon_min = 0.01
        self.epsilon_decay = 0.995

        self.max_steps = 100
        self.episode_count = 0

        # Meta (Goal)
        self.goal_position = (5.0, 3.0)  # ajuste conforme o mapa do Gazebo
        self.goal_tolerance = 0.5  # distância mínima para considerar atingido

        self.robot_x = 0.0
        self.robot_y = 0.0

    def lidar_callback(self, msg):
        self.lidar_data = msg.data

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

    def reached_goal(self):
        dx = self.goal_position[0] - self.robot_x
        dy = self.goal_position[1] - self.robot_y
        distance = np.hypot(dx, dy)
        return distance < self.goal_tolerance

    def get_state(self, sectors):
        right = min(sectors[0:10])
        front = min(sectors[10:20])
        left = min(sectors[20:30])

        def discretize(d):
            if d < 0.7: return 0
            elif d < 1.5: return 1
            else: return 2

        r, f, l = discretize(right), discretize(front), discretize(left)
        return l * 9 + f * 3 + r

    def select_action(self, state):
        if random.random() < self.epsilon:
            return random.randint(0, 2)
        return int(np.argmax(self.q_table[state]))

    def execute_action(self, action):
        twist = Twist()
        if action == 0:
            twist.angular.z = 0.5  # esquerda
        elif action == 1:
            twist.linear.x = 0.3   # frente
        elif action == 2:
            twist.angular.z = -0.5  # direita
        self.cmd_pub.publish(twist)

    def compute_reward(self, state, action):
        r = state % 3
        f = (state // 3) % 3
        l = state // 9

        if f == 0 and action == 1:
            return -1.0
        elif f == 2 and action == 1:
            return 1.0
        elif f == 1 and action == 1:
            return 0.3
        elif (l == 0 and action == 0) or (r == 0 and action == 2):
            return -0.5
        else:
            return 0.0

    def run_episode(self):
        step = 0
        done = False
        last_state = None
        last_action = None

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
                rospy.loginfo("🎯 Meta atingida!")
            else:
                reward = self.compute_reward(state, action)

            if last_state is not None:
                old_q = self.q_table[last_state, last_action]
                max_next_q = np.max(self.q_table[state])
                new_q = (1 - self.learning_rate) * old_q + self.learning_rate * (reward + self.discount * max_next_q)
                self.q_table[last_state, last_action] = new_q

            last_state = state
            last_action = action

            step += 1
            rospy.sleep(0.2)

            if done:
                break

        self.epsilon = max(self.epsilon_min, self.epsilon * self.epsilon_decay)
        np.save(self.q_table_file, self.q_table)
        self.episode_count += 1
        rospy.loginfo(f"🏁 Episódio {self.episode_count} concluído | Eps: {self.epsilon:.3f}")

    def run(self):
        while not rospy.is_shutdown():
            self.run_episode()

if __name__ == '__main__':
    try:
        node = QLearningFastLidar()
        node.run()
    except rospy.ROSInterruptException:
        pass
