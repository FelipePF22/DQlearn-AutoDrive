#!/usr/bin/env python3
import rospy
import numpy as np
import random
import os
import torch
import torch.nn as nn
import torch.optim as optim
from std_msgs.msg import Float32, Float32MultiArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
import json
from datetime import datetime
from collections import deque

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


class QLearningFastLidar:
    def __init__(self):
        rospy.init_node('q_learning_fast_lidar')

        self.replay_buffer = deque(maxlen=5000)
        self.batch_size = 32

        self.cmd_pub = rospy.Publisher('/robot/cmd_vel', Twist, queue_size=10)
        self.goal_pub = rospy.Publisher('/goal_reached', Float32, queue_size=10)
        self.reward_pub = rospy.Publisher('/episode_reward', Float32, queue_size=10)

        rospy.Subscriber('/fast_lidar/sectors', Float32MultiArray, self.lidar_callback)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)

        self.lidar_data = None
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.last_distance_to_goal = None
        self.current_trajectory = []

        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.q_net = QNetwork().to(self.device)
        self.optimizer = optim.Adam(self.q_net.parameters(), lr=0.001)
        self.loss_fn = nn.MSELoss()

        model_path = os.path.expanduser('~/ros_ws/src/Qlearn-AutoDrive/Autodriver/q_learning_control/network/q_net.pth')
        if os.path.exists(model_path):
            self.q_net.load_state_dict(torch.load(model_path))

        self.discount = 0.9
        self.epsilon = 0.6
        self.epsilon_min = 0.01
        self.epsilon_decay = 0.995

        self.max_steps = 500
        self.episode_count = 0

        self.goal_position = (12.0, 4.0)
        self.goal_tolerance = 0.5

        log_file = os.path.expanduser('~/ros_ws/src/Qlearn-AutoDrive/Autodriver/q_learning_control/network/episode_log.txt')
        with open(log_file, 'w') as f:
            f.write("REGISTRO DE EPISÓDIOS - APRENDIZADO POR REFORÇO\n")
            f.write("=================================================\n")
            f.write("Formato: Episódio | Resultado | Tempo (s) | Distância (m) | Pontos | Epsilon\n")
            f.write("-------------------------------------------------\n")

        self.trajectory_dir = os.path.expanduser('~/ros_ws/src/Qlearn-AutoDrive/Autodriver/q_learning_control/trajectories')
        os.makedirs(self.trajectory_dir, exist_ok=True)

    def save_trajectory(self, episode_num, outcome):
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"trajectory_ep{episode_num:04d}_{outcome}_{timestamp}.json"
        filepath = os.path.join(self.trajectory_dir, filename)
        trajectory_data = {
            'episode': episode_num,
            'outcome': outcome,
            'timestamp': timestamp,
            'goal_position': self.goal_position,
            'trajectory': self.current_trajectory
        }
        with open(filepath, 'w') as f:
            json.dump(trajectory_data, f, indent=2)
        rospy.loginfo(f"Trajetória do episódio {episode_num} salva em {filename}")

    def log_episode_data(self, episode_num, outcome, duration, distance, total_points, epsilon):
        log_file = os.path.expanduser('~/ros_ws/src/Qlearn-AutoDrive/Autodriver/q_learning_control/network/episode_log.txt')
        with open(log_file, 'a') as f:
            f.write(f"{episode_num:8d} | {outcome:16s} | {duration:8.2f} | {distance:12.2f} | {total_points:8.2f} |{epsilon:8.3f}\n")

    def lidar_callback(self, msg):
        self.lidar_data = msg.data

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.current_trajectory.append({
            'x': float(self.robot_x),
            'y': float(self.robot_y),
            'time': float(rospy.get_time())
        })

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
        dx = self.goal_position[0] - self.robot_x
        dy = self.goal_position[1] - self.robot_y
        return np.array([right, front, left, dx, dy], dtype=np.float32)


    def select_action(self, state):
        if random.random() < self.epsilon:
            return random.randint(0, 2)
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

   

    def compute_reward(self, state, action):
        r, f, l = state[0], state[1], state[2]
        dx = self.goal_position[0] - self.robot_x
        dy = self.goal_position[1] - self.robot_y
        dist_now = self.get_distance_to_goal()
        reward = 0.0
        done = False

        # Colisão frontal
        if f < 0.5 and action == 1:
            self.reset_robot_position()
            done = True
            return -100.0, done

        # Evitar colisão lateral
        if (l < 0.4 and action == 0) or (r < 0.4 and action == 2):
            reward -= 10.0  # punição forte por virar em direção ao obstáculo

        # Recompensa por seguir em frente com espaço livre
        if f >= 1.0 and action == 1:
            reward += 5.0

        # Recompensa leve por se mover na direção da meta
        if abs(dx) > 0.3:
            if dx > 0 and action == 2:
                reward += 3.0
            elif dx < 0 and action == 0:
                reward += 3.0
            else:
                reward -= 2.0

        if abs(dy) > 0.3:
            if dy > 0 and action == 1:
                reward += 3.0
            elif dy < 0:
                reward -= 1.0

        # Considera se o afastamento é justificável por obstáculos
        obstacle_near = f < 0.6 or (dx > 0 and r < 0.4) or (dx < 0 and l < 0.4)
        if self.last_distance_to_goal is not None:
            delta_dist = self.last_distance_to_goal - dist_now
            if delta_dist > 0.05:
                reward += 5.0
            elif delta_dist > 0:
                reward += 1.0
            elif delta_dist < -0.05 and not obstacle_near:
                reward -= 5.0  # afastou sem motivo
            elif delta_dist < -0.05 and obstacle_near:
                reward -= 0.5  # afastou mas desviando

        self.last_distance_to_goal = dist_now
        return reward, done



    def reset_robot_position(self):
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
        start_time = rospy.get_time()
        self.current_trajectory = []
        outcome = "Seguiu o caminho"

        while step < self.max_steps and not rospy.is_shutdown():
            if self.lidar_data is None:
                rospy.sleep(0.1)
                continue
            state = self.get_state(self.lidar_data)
            action = self.select_action(state)
            self.execute_action(action)
            if self.reached_goal():
                reward = 100.0
                done = True
                outcome = "Atingiu o objetivo"
                rospy.loginfo("Meta atingida!")
                self.reset_robot_position()
            else:
                reward, done = self.compute_reward(state, action)
                if reward <= -100.0:
                    outcome = "Colidiu"
            total_reward += reward
            if last_state is not None:
                self.replay_buffer.append((last_state, last_action, reward, state, done))
            if len(self.replay_buffer) >= self.batch_size:
                batch = random.sample(self.replay_buffer, self.batch_size)
                states, actions, rewards, next_states, dones = zip(*batch)
                states_tensor = torch.FloatTensor(states).to(self.device)
                next_states_tensor = torch.FloatTensor(next_states).to(self.device)
                rewards_tensor = torch.FloatTensor(rewards).to(self.device)
                actions_tensor = torch.LongTensor(actions).unsqueeze(1).to(self.device)
                dones_tensor = torch.BoolTensor(dones).to(self.device)
                q_values = self.q_net(states_tensor).gather(1, actions_tensor)
                with torch.no_grad():
                    max_next_q_values = self.q_net(next_states_tensor).max(dim=1)[0]
                    target_q_values = rewards_tensor + self.discount * max_next_q_values * (~dones_tensor)
                loss = self.loss_fn(q_values.squeeze(), target_q_values)
                self.optimizer.zero_grad()
                loss.backward()
                self.optimizer.step()
            last_state = state
            last_action = action
            step += 1
            rospy.sleep(0.2)
            self.goal_pub.publish(Float32(data=1.0 if self.reached_goal() else 0.0))
            if done:
                break
        duration = rospy.get_time() - start_time
        distance = 0.0
        for i in range(1, len(self.current_trajectory)):
            x1, y1 = self.current_trajectory[i - 1]['x'], self.current_trajectory[i - 1]['y']
            x2, y2 = self.current_trajectory[i]['x'], self.current_trajectory[i]['y']
            distance += np.hypot(x2 - x1, y2 - y1)
        self.log_episode_data(self.episode_count, outcome, duration, distance, total_reward, self.epsilon)
        self.save_trajectory(self.episode_count, outcome)
        self.reward_pub.publish(Float32(data=total_reward))
        self.epsilon = max(self.epsilon_min, self.epsilon * self.epsilon_decay)
        self.episode_count += 1
        torch.save(self.q_net.state_dict(), os.path.expanduser('~/ros_ws/src/Qlearn-AutoDrive/Autodriver/q_learning_control/network/q_net.pth'))
        rospy.loginfo(f"Episódio {self.episode_count} finalizado | Resultado: {outcome} | ε: {self.epsilon:.3f}")
        if self.episode_count % 20 == 0:
            rospy.logwarn("Reinicializando.")
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