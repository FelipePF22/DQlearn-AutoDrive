#!/usr/bin/env python3
import rospy
import torch
import torch.nn as nn
import numpy as np
from std_msgs.msg import Float32MultiArray, Float32
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
import os
import time


class QNetwork(nn.Module):
    def __init__(self, input_dim=5, output_dim=3):
        super(QNetwork, self).__init__()
        self.model = nn.Sequential(
            nn.Linear(input_dim, 64),
            nn.ReLU(),
            nn.Linear(64, 64),
            nn.ReLU(),
            nn.Linear(64, output_dim)
        )

    def forward(self, x):
        return self.model(x)


class DQNValidator:
    def __init__(self):
        rospy.init_node('dqn_validator')
        self.cmd_pub = rospy.Publisher('/robot/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/fast_lidar/sectors', Float32MultiArray, self.lidar_callback)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)

        self.lidar_data = None
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.goal_position = (12.0, 4.0)
        self.goal_tolerance = 0.5

        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.q_net = QNetwork().to(self.device)

        model_path = os.path.expanduser('~/ros_ws/src/Qlearn-AutoDrive/Autodriver/q_learning_control/network/q_net.pth')
        self.q_net.load_state_dict(torch.load(model_path, map_location=self.device))
        self.q_net.eval()

    def lidar_callback(self, msg):
        self.lidar_data = msg.data

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

    def get_state(self):
        if self.lidar_data is None:
            return None
        right = min(self.lidar_data[0:10])
        front = min(self.lidar_data[10:20])
        left = min(self.lidar_data[20:30])
        dx = self.goal_position[0] - self.robot_x
        dy = self.goal_position[1] - self.robot_y
        return np.array([right, front, left, dx, dy], dtype=np.float32)

    def reached_goal(self):
        dx = self.goal_position[0] - self.robot_x
        dy = self.goal_position[1] - self.robot_y
        return np.hypot(dx, dy) < self.goal_tolerance

    def reset_position(self):
        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            state = ModelState()
            state.model_name = 'mobile_rob_dev_sim'
            state.pose.position.x = 3.0
            state.pose.position.y = 0.0
            state.pose.position.z = 0.0
            state.pose.orientation.w = 1.0
            state.reference_frame = 'world'
            set_state(state)
            rospy.loginfo("Robô reposicionado para validação.")
        except rospy.ServiceException as e:
            rospy.logerr(f"Erro ao resetar posição: {e}")

    def select_best_action(self, state):
        state_tensor = torch.FloatTensor(state).unsqueeze(0).to(self.device)
        with torch.no_grad():
            q_values = self.q_net(state_tensor)
        return int(torch.argmax(q_values).item())

    def execute_action(self, action):
        cmd = Twist()
        if action == 0:
            cmd.angular.z = 0.5
        elif action == 1:
            cmd.linear.x = 0.3
        elif action == 2:
            cmd.angular.z = -0.5
        self.cmd_pub.publish(cmd)

    def validate_episode(self, episode_num, max_steps=500):
        self.reset_position()
        rospy.sleep(1.0)

        steps = 0
        start_time = time.time()
        reached = False

        while not rospy.is_shutdown() and steps < max_steps:
            if self.lidar_data is None:
                rospy.sleep(0.1)
                continue

            state = self.get_state()
            if state is None:
                continue

            action = self.select_best_action(state)
            self.execute_action(action)

            if self.reached_goal():
                reached = True
                break

            steps += 1
            rospy.sleep(0.2)

        duration = time.time() - start_time
        result_str = "Sucesso" if reached else "Falha"

        # Posição final
        final_x = self.robot_x
        final_y = self.robot_y

        # Salva os dados no arquivo
        with open("validacao.txt", "a") as f:
            if reached:
                f.write(f"Episódio {episode_num}: {result_str} | Passos: {steps} | Tempo: {duration:.2f} s\n")
            else:
                f.write(f"Episódio {episode_num}: {result_str} | Passos: {steps} | Tempo: {duration:.2f} s | Posição final: x={final_x:.2f}, y={final_y:.2f}\n")

        if reached:
            rospy.loginfo(f"🏁 Objetivo atingido em {steps} passos | Tempo: {duration:.2f}s")
        else:
            rospy.logwarn(f"⚠️ Episódio finalizado sem atingir o objetivo. Passos: {steps} | Tempo: {duration:.2f}s | Posição final: x={final_x:.2f}, y={final_y:.2f}")

        return reached

    def validate(self, episodes=20):
        success_count = 0
        for i in range(episodes):
            rospy.loginfo(f"=== Validação Episódio {i+1}/{episodes} ===")
            success = self.validate_episode(i+1)
            if success:
                success_count += 1
        rospy.loginfo(f"✅ Validação concluída: {success_count}/{episodes} objetivos atingidos.")


if __name__ == '__main__':
    try:
        validator = DQNValidator()
        validator.validate(episodes=20)  # Ajuste o número de episódios conforme necessário
    except rospy.ROSInterruptException:
        pass
