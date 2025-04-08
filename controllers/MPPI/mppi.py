import numpy as np
import matplotlib.pyplot as plt

class VehicleModel:
    def __init__(self):
        self.m = 200.0            # Mass (kg)
        self.tau_steering = 2     # Steering time constant
        self.tau_velocity = 3     # Velocity time constant
        self.max_vel = 5           # Maximum velocity

    def step(self, action, dt, state):
        x, y, phi, prev_vel, prev_steer = state
        vel, steer = action

        # Update velocity and steering
        vel = prev_vel + dt * (vel - prev_vel) / self.tau_velocity
        steer = prev_steer + dt * (steer - prev_steer) / self.tau_steering

        # Limit velocity
        if vel > self.max_vel:
            vel = self.max_vel

        # Update state
        x += vel * dt * np.cos(steer + phi)
        y += vel * dt * np.sin(steer + phi)

        phi += steer
        # vel_x = vel * np.cos(phi)
        # vel_y = vel * np.sin(phi)
        # x += (np.cos(phi) * vel_x - np.sin(phi) * vel_y) * dt
        # y += (np.sin(phi) * vel_x + np.cos(phi) * vel_y) * dt
        
        

        return np.array([x, y, phi, vel, steer])


class MPPIController:
    def __init__(self, lambda_, cov, nu, R, horizon, n_samples, model, dt, goal, obstacles):
        self.lambda_ = lambda_
        self.horizon = horizon
        self.n_samples = n_samples
        self.cov = cov
        self.nu = nu
        self.R = R
        self.model = model
        self.dt = dt
        self.goal = goal
        self.obstacles = obstacles

        self.control_sequence = np.zeros((2, self.horizon))  # 2 control inputs: velocity and steering
        self.optimized_control_sequence = None
        self.predicted_lines = []


        # Initialize n_states
        self.n_states = 5  # Assuming state vector has 5 elements: [x, y, phi, v, steer]

        self.rollouts_states = np.zeros((self.n_samples, self.horizon + 1, self.n_states))
        self.rollouts_costs = np.zeros(self.n_samples)
        self.rollouts_plot_handle = []

        self.max_vel = 5
        self.max_steer = 6.0

    def get_action(self, state):
        self.state = state
        init_state = state.copy()
        states = np.zeros((self.n_states, self.horizon + 1))  # Now self.n_states is defined
        S = np.zeros(self.n_samples)

        # Generate random control input disturbances
        delta_u_vel = np.random.normal(0, self.cov[0], (self.n_samples, self.horizon))
        delta_u_steer = np.random.normal(0, self.cov[1], (self.n_samples, self.horizon))

        # Clip steering disturbances
        delta_u_steer = np.clip(delta_u_steer, -0.5, 0.5)

        # Combine disturbances into a single array
        delta_u = np.array([delta_u_vel, delta_u_steer])  # Shape (2, n_samples, horizon)

        for k in range(self.n_samples):
            states[:, 0] = init_state
            for i in range(self.horizon):
                # Single trajectory step
                combined_action = self.control_sequence[:, i] + delta_u[:, k, i]  # Correct indexing
                states[:, i + 1] = self.model.step(combined_action, self.dt, states[:, i])

                # Compute cost of the state
                S[k] += self.cost_function(states[:, i + 1], self.control_sequence[:, i], delta_u[:, k, i])

            self.rollouts_states[k, :, :] = states.T
            self.rollouts_costs[k] = S[k]

        # Update control input
        S_normalized = S - np.min(S)
        for i in range(self.horizon):
            self.control_sequence[:, i] += self.total_entropy(delta_u[:, :, i], S_normalized)

        # Output saturation
        self.control_sequence[0, self.control_sequence[0, :] > self.model.max_vel] = self.model.max_vel
        self.control_sequence[0, self.control_sequence[0, :] < -self.model.max_vel] = -self.model.max_vel
        self.control_sequence[1, self.control_sequence[1, :] > self.max_steer] = self.max_steer
        self.control_sequence[1, self.control_sequence[1, :] < -self.max_steer] = -self.max_steer

        action = self.control_sequence[:, 0]
        self.control_sequence = np.roll(self.control_sequence, -1, axis=1)
        self.control_sequence[:, -1] = 0

        return action


    def cost_function(self, state, u, du):
        state_cost = self.state_cost_function(state)
        control_cost = self.control_cost_function(u, du)
        return state_cost + control_cost

    def state_cost_function(self, state):
        obstacle_cost = self.obstacle_cost_function(state)
        heading_cost = self.heading_cost_function(state)
        distance_cost = self.distance_cost_function(state)

        return distance_cost + heading_cost + obstacle_cost

    def distance_cost_function(self, state):
        weight = 100
        return weight * np.linalg.norm(self.goal[:2] - state[:2]) ** 2

    def heading_cost_function(self, state):
        weight = 50
        return weight * abs(self.get_angle_diff(self.goal[2], state[2])) ** 2

    def control_cost_function(self, u, du):
        return (1 - 1 / self.nu) / 2 * du.T @ self.R @ du + u.T @ self.R @ du + 1 / 2 * u.T @ self.R @ u

    def obstacle_cost_function(self, state):
        if self.obstacles.size == 0:
            return 0

        distance_to_obstacle = np.linalg.norm(state[:2] - self.obstacles[:, :2], axis=1)
        min_dist = np.min(distance_to_obstacle)
        min_dist_idx = np.argmin(distance_to_obstacle)

        hit = 1 if min_dist <= self.obstacles[min_dist_idx, 2] else 0
        return 550 * np.exp(-min_dist / 5) + 1e6 * hit
        # return 100 * abs(-min_dist)+1e6*hit

    def total_entropy(self, du, trajectory_cost):
        exponents = np.exp(-1 / self.lambda_ * trajectory_cost)
        return np.sum(exponents[:, np.newaxis] * du.T / np.sum(exponents), axis=0)

    @staticmethod
    def get_angle_diff(angle1, angle2):
        angle_diff = angle1 - angle2
        return (angle_diff + np.pi) % (2 * np.pi) - np.pi

    def plot_rollouts(self, fig):
        # Pokud existují staré trajektorie, smažte je
        if self.predicted_lines:
            for line in self.predicted_lines:
                line[0].remove()  # Odstraní vykreslenou čáru
            self.predicted_lines = []  # Vyprázdněte seznam

        # Nastavte aktuální graf
        plt.figure(fig)
        costs = (self.rollouts_costs - np.min(self.rollouts_costs)) / (np.max(self.rollouts_costs) - np.min(self.rollouts_costs))
        min_idx = np.argmin(costs)

        # Vykreslete všechny trajektorie
        for i in range(self.n_samples):
            color = [1, 1, 1] if i == min_idx else [1 - costs[i], 0.1, 0.8]  # Barva dle nákladů
            line_handle = plt.plot(self.rollouts_states[i, :, 0], self.rollouts_states[i, :, 1], '-', color=color)
            self.predicted_lines.append(line_handle)  # Uložte handle pro pozdější odstranění

        # Vykreslete optimalizovanou trajektorii
        states = np.zeros((self.n_states, self.horizon + 1))
        states[:, 0] = self.state

        for i in range(self.horizon):
            states[:, i + 1] = self.model.step(self.control_sequence[:, i], self.dt, states[:, i])

        # Vykreslete optimální trajektorii (zelená čára)
        line_handle = plt.plot(states[0, :], states[1, :], linewidth=3, linestyle="--", color=[0.2, 0.2, 0.2])
        self.predicted_lines.append(line_handle)  # Uložte handle optimální trajektorie



# Main simulation parameters
n_samples = 50        # Number of rollout trajectories
horizon = 25           # Prediction horizon
lambda_ = 10           # Temperature
nu = 100               # Exploration variance
R = np.diag([5, 5])    # Control weighting matrix
cov = [1, 0.4]         # Variance of control inputs disturbance
dt = 0.1               # Time step of controller and simulation

init_state = np.array([0, 3, 0.8, 0, 0])  # x, y, phi, v, steer
goal_state = np.array([10, 2, 0])

# Define environment - obstacles [x, y, radius]
n_obstacles = 30
obstacles = np.hstack((np.random.rand(n_obstacles, 2) * 4 + 1, 0.2 * np.ones((n_obstacles, 1))))

# Initialize models
car_real = VehicleModel()
car = VehicleModel()
controller = MPPIController(lambda_, cov, nu, R, horizon, n_samples, car, dt, goal_state, obstacles)

# Prepare visualization
fig = plt.figure()
plt.axis('equal')
plt.xlim([-0.5 + min(init_state[0], goal_state[0]), 0.5 + max(init_state[0], goal_state[0])])
plt.ylim([-0.5 + min(init_state[1], goal_state[1]), 0.5 + max(init_state[1], goal_state[1])])
plt.scatter(init_state[0], init_state[1], color='blue')   # Initial state
plt.scatter(goal_state[0], goal_state[1], color='red')    # Goal state

# Plot obstacles
for obstacle in obstacles:
    r = obstacle[2]
    circle = plt.Circle((obstacle[0], obstacle[1]), r, color='black', fill=True)
    plt.gca().add_artist(circle)

# Control loop
car_state = init_state.copy()


def generate_trajectory(n_points=100):
    x = np.linspace(0, 10, n_points)  # 100 points along x-axis
    y = np.sin(0.5*x)+3  # A sine wave
    return np.vstack((x, y)).T  # Return as Nx2 array

trajectory = generate_trajectory(300)
plt.plot(trajectory[:, 0], trajectory[:, 1], 'g--', label='Desired Trajectory')

for i in range(800):
    nearest_index = np.argmin(np.linalg.norm(trajectory - car_state[:2], axis=1))
    try:
        nearest_point = trajectory[nearest_index+2*horizon]
    except:
        nearest_point = trajectory[-1]
    print(nearest_point)
    controller.goal = np.array([nearest_point[0], nearest_point[1], 0])
    # plt.scatter(nearest_point[0], nearest_point[1], color='red')
    action = controller.get_action(car_state)
    controller.plot_rollouts(fig)
    
    car_state = car_real.step(action, dt, car_state)
    plt.scatter(car_state[0], car_state[1], marker="+", color='red')  # Current state
    
    plt.pause(0.1)  # Pause for a moment

plt.show()