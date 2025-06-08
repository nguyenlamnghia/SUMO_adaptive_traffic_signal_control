import traci
import numpy as np
import tensorflow as tf
from collections import deque
import random
import os

# Thiết lập tham số
state_size = 6  # [queue_north, queue_south, queue_east, queue_west, waiting_time, phase]
action_size = 6  # 0: Bắc-Nam 10s, 1: Bắc-Nam 20s, 2: Bắc-Nam 30s, 3: Đông-Tây 10s, 4: Đông-Tây 20s, 5: Đông-Tây 30s
green_times = [10, 20, 30, 10, 20, 30]  # Thời gian xanh tương ứng với mỗi hành động
phases = [0, 0, 0, 2, 2, 2]  # Pha tín hiệu tương ứng (0: Bắc-Nam, 2: Đông-Tây)
gamma = 0.95  # Hệ số chiết khấu
epsilon = 1.0  # Tỷ lệ khám phá
epsilon_min = 0.01
epsilon_decay = 0.995
learning_rate = 0.001
batch_size = 32
memory = deque(maxlen=2000)

# Xây dựng mạng Quad-DQN
class QuadDQN:
    def __init__(self):
        self.model = self.build_model()
        self.target_model = self.build_model()
        self.update_target_model()

    def build_model(self):
        model = tf.keras.Sequential([
            tf.keras.layers.Dense(24, input_dim=state_size, activation='relu'),
            tf.keras.layers.Dense(24, activation='relu'),
            tf.keras.layers.Dense(action_size + 1, activation='linear')  # +1 cho giá trị trạng thái
        ])
        model.compile(loss='mse', optimizer=tf.keras.optimizers.Adam(learning_rate=learning_rate))
        return model

    def update_target_model(self):
        self.target_model.set_weights(self.model.get_weights())

    def get_action(self, state):
        if np.random.rand() <= epsilon:
            return random.randrange(action_size)
        q_values = self.model.predict(state, verbose=0)[0]
        return np.argmax(q_values[:action_size])  # Chỉ lấy Q-value của action

    def train(self):
        if len(memory) < batch_size:
            return
        minibatch = random.sample(memory, batch_size)
        states = np.zeros((batch_size, state_size))
        next_states = np.zeros((batch_size, state_size))
        actions, rewards, dones = [], [], []

        for i, (state, action, reward, next_state, done) in enumerate(minibatch):
            states[i] = state
            next_states[i] = next_state
            actions.append(action)
            rewards.append(reward)
            dones.append(done)

        targets = self.model.predict(states, verbose=0)
        target_next = self.target_model.predict(next_states, verbose=0)

        for i in range(batch_size):
            if dones[i]:
                targets[i][actions[i]] = rewards[i]
            else:
                state_value = target_next[i][-1]
                max_action_value = np.max(target_next[i][:action_size])
                targets[i][actions[i]] = rewards[i] + gamma * (max_action_value + state_value)

        self.model.fit(states, targets, epochs=1, verbose=0)

# Khởi tạo Quad-DQN
dqn = QuadDQN()

# Hàm lấy state từ SUMO
# EWSN: ĐÔNG-TÂY-NAM-BẮC
def get_state():
    queue_north = traci.lane.getLastStepVehicleNumber("-E1_0")
    queue_south = traci.lane.getLastStepVehicleNumber("-E2_0")
    queue_east = traci.lane.getLastStepVehicleNumber("-E3_0")
    queue_west = traci.lane.getLastStepVehicleNumber("E0_0")
    waiting_time = (traci.lane.getWaitingTime("-E1_0") +
                    traci.lane.getWaitingTime("-E2_0") +
                    traci.lane.getWaitingTime("-E3_0") +
                    traci.lane.getWaitingTime("E0_0")) / 4
    phase = 0 if traci.trafficlight.getPhase("J1") == 0 else 1
    return np.array([queue_north, queue_south, queue_east, queue_west, waiting_time, phase])

# Hàm tính phần thưởng
def calculate_reward(old_waiting_time, new_waiting_time, queue_length, green_time, action):
    # Phần thưởng cơ bản dựa trên thay đổi thời gian chờ
    reward = -(new_waiting_time - old_waiting_time) * 10
    # Phạt nếu thời gian xanh không phù hợp với hàng đợi
    if action < 3:  # Bắc-Nam
        queue = queue_north + queue_south
    else:  # Đông-Tây
        queue = queue_east + queue_west
    if queue > 15 and green_time < 20:  # Đông nhưng thời gian ngắn
        reward -= 10
    elif queue < 5 and green_time > 20:  # Ít xe nhưng thời gian dài
        reward -= 5
    return reward

# Chạy mô phỏng
def run_simulation():
    global epsilon
    sumo_cmd = ["sumo", "-c", "test.sumocfg"]
    traci.start(sumo_cmd)
    step = 0
    total_reward = 0
    max_steps = 3600  # 1 giờ mô phỏng

    while step < max_steps:
        traci.simulationStep()
        # Lấy state hiện tại
        state = get_state()
        state = np.reshape(state, [1, state_size])
        old_waiting_time = state[0][4]
        queue_north, queue_south, queue_east, queue_west = state[0][:4]

        # Chọn hành động
        action = dqn.get_action(state)
        green_time = green_times[action]
        phase = phases[action]
        traci.trafficlight.setPhase("J1", phase)

        # Chạy mô phỏng theo thời gian xanh
        for _ in range(green_time):
            traci.simulationStep()

        # Lấy state mới và tính phần thưởng
        new_state = get_state()
        new_state = np.reshape(new_state, [1, state_size])
        new_waiting_time = new_state[0][4]
        reward = calculate_reward(old_waiting_time, new_waiting_time,
                                queue_north + queue_south + queue_east + queue_west,
                                green_time, action)
        total_reward += reward

        # Lưu kinh nghiệm
        done = step >= max_steps - 1
        memory.append((state, action, reward, new_state, done))

        # Huấn luyện Quad-DQN
        dqn.train()

        # Cập nhật target model mỗi 10 bước
        if step % 10 == 0:
            dqn.update_target_model()

        # Giảm epsilon
        if epsilon > epsilon_min:
            epsilon *= epsilon_decay

        step += 1
        if step % 100 == 0:
            print(f"Step: {step}, Action: {action}, Green Time: {green_time}s, "
                  f"Total Reward: {total_reward}, Epsilon: {epsilon}")

    traci.close()
    print(f"Simulation ended. Final Total Reward: {total_reward}")

if __name__ == "__main__":
    run_simulation()