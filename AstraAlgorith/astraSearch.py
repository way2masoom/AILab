import tkinter as tk
from tkinter import messagebox
import heapq
import math
from collections import deque

# Directions for movement (up, down, left, right)
DIRECTIONS = [(-1, 0), (1, 0), (0, -1), (0, 1)]  # No diagonal movement for Manhattan heuristic
DIAGONAL_DIRECTIONS = DIRECTIONS + [(-1, -1), (-1, 1), (1, -1), (1, 1)]  # Adding diagonal moves

class PathfindingApp:
    def __init__(self, root, grid_size=20):
        self.root = root
        self.grid_size = grid_size
        self.canvas = tk.Canvas(root, width=grid_size * 30, height=grid_size * 30)
        self.canvas.pack()

        self.grid = [[0 for _ in range(grid_size)] for _ in range(grid_size)]  # 0 = free, 1 = obstacle
        self.start = None
        self.goal = None
        self.path = []

        self.create_ui()
        self.draw_grid()

    def create_ui(self):
        self.start_button = tk.Button(self.root, text="Set Start", command=self.set_start)
        self.start_button.pack(side=tk.LEFT)

        self.goal_button = tk.Button(self.root, text="Set Goal", command=self.set_goal)
        self.goal_button.pack(side=tk.LEFT)

        self.obstacle_button = tk.Button(self.root, text="Toggle Obstacle", command=self.toggle_obstacle)
        self.obstacle_button.pack(side=tk.LEFT)

        self.clear_button = tk.Button(self.root, text="Clear Grid", command=self.clear_grid)
        self.clear_button.pack(side=tk.LEFT)

        self.a_star_button = tk.Button(self.root, text="Run A* (Manhattan)", command=lambda: self.run_algorithm('A*', heuristic='manhattan'))
        self.a_star_button.pack(side=tk.LEFT)

        self.bfs_button = tk.Button(self.root, text="Run BFS", command=lambda: self.run_algorithm('BFS'))
        self.bfs_button.pack(side=tk.LEFT)

        self.ucs_button = tk.Button(self.root, text="Run UCS", command=lambda: self.run_algorithm('UCS'))
        self.ucs_button.pack(side=tk.LEFT)

    def draw_grid(self):
        self.canvas.delete("all")  # Clears the canvas before redrawing
        for i in range(self.grid_size):
            for j in range(self.grid_size):
                color = "white" if self.grid[i][j] == 0 else "black"
                self.canvas.create_rectangle(j * 30, i * 30, (j + 1) * 30, (i + 1) * 30, fill=color, outline="gray")

        if self.start:
            self.canvas.create_oval(self.start[1] * 30 + 5, self.start[0] * 30 + 5, self.start[1] * 30 + 25, self.start[0] * 30 + 25, fill="green")
        if self.goal:
            self.canvas.create_oval(self.goal[1] * 30 + 5, self.goal[0] * 30 + 5, self.goal[1] * 30 + 25, self.goal[0] * 30 + 25, fill="red")
        for cell in self.path:
            self.canvas.create_rectangle(cell[1] * 30, cell[0] * 30, (cell[1] + 1) * 30, (cell[0] + 1) * 30, fill="blue")

    def select_cell(self, event):
        """ Helper function to get the selected cell from a mouse click """
        row, col = event.y // 30, event.x // 30
        return row, col

    def set_start(self):
        self.canvas.bind("<Button-1>", self.on_start_click)

    def set_goal(self):
        self.canvas.bind("<Button-1>", self.on_goal_click)

    def toggle_obstacle(self):
        self.canvas.bind("<Button-1>", self.on_obstacle_click)

    def clear_grid(self):
        self.grid = [[0 for _ in range(self.grid_size)] for _ in range(self.grid_size)]
        self.start = None
        self.goal = None
        self.path = []
        self.draw_grid()

    def on_start_click(self, event):
        self.start = self.select_cell(event)
        self.canvas.unbind("<Button-1>")
        self.draw_grid()

    def on_goal_click(self, event):
        self.goal = self.select_cell(event)
        self.canvas.unbind("<Button-1>")
        self.draw_grid()

    def on_obstacle_click(self, event):
        row, col = self.select_cell(event)
        if (row, col) != self.start and (row, col) != self.goal:  # Prevent blocking start or goal
            self.grid[row][col] = 1 if self.grid[row][col] == 0 else 0
        self.canvas.unbind("<Button-1>")
        self.draw_grid()

    def heuristic(self, node, goal, type='manhattan'):
        if type == 'manhattan':
            return abs(node[0] - goal[0]) + abs(node[1] - goal[1])
        elif type == 'euclidean':
            return math.sqrt((node[0] - goal[0]) ** 2 + (node[1] - goal[1]) ** 2)

    def get_neighbors(self, node, diagonal=False):
        directions = DIAGONAL_DIRECTIONS if diagonal else DIRECTIONS
        neighbors = []
        for dx, dy in directions:
            nx, ny = node[0] + dx, node[1] + dy
            if 0 <= nx < self.grid_size and 0 <= ny < self.grid_size and self.grid[nx][ny] != 1:
                neighbors.append((nx, ny))
        return neighbors

    def a_star_search(self, start, goal, heuristic_type='manhattan'):
        open_list = []
        heapq.heappush(open_list, (0, start))
        g_costs = {start: 0}
        came_from = {}

        while open_list:
            _, current = heapq.heappop(open_list)
            if current == goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.reverse()
                return path

            for neighbor in self.get_neighbors(current, diagonal=True):
                new_cost = g_costs[current] + 1
                if neighbor not in g_costs or new_cost < g_costs[neighbor]:
                    g_costs[neighbor] = new_cost
                    priority = new_cost + self.heuristic(neighbor, goal, heuristic_type)
                    heapq.heappush(open_list, (priority, neighbor))
                    came_from[neighbor] = current

        return []

    def bfs(self, start, goal):
        queue = deque([start])
        came_from = {start: None}

        while queue:
            current = queue.popleft()
            if current == goal:
                path = []
                while current is not None:
                    path.append(current)
                    current = came_from[current]
                path.reverse()
                return path

            for neighbor in self.get_neighbors(current):
                if neighbor not in came_from:
                    came_from[neighbor] = current
                    queue.append(neighbor)

        return []

    def uniform_cost_search(self, start, goal):
        open_list = []
        heapq.heappush(open_list, (0, start))
        g_costs = {start: 0}
        came_from = {}

        while open_list:
            cost, current = heapq.heappop(open_list)
            if current == goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.reverse()
                return path

            for neighbor in self.get_neighbors(current):
                new_cost = g_costs[current] + 1
                if neighbor not in g_costs or new_cost < g_costs[neighbor]:
                    g_costs[neighbor] = new_cost
                    heapq.heappush(open_list, (new_cost, neighbor))
                    came_from[neighbor] = current

        return []

    def run_algorithm(self, algorithm, heuristic='manhattan'):
        if not self.start or not self.goal:
            messagebox.showerror("Error", "Please set both start and goal points.")
            return

        self.path = self.a_star_search(self.start, self.goal, heuristic) if algorithm == 'A*' else \
                    self.bfs(self.start, self.goal) if algorithm == 'BFS' else \
                    self.uniform_cost_search(self.start, self.goal)
        self.draw_grid()

if __name__ == "__main__":
    root = tk.Tk()
    app = PathfindingApp(root)
    root.mainloop()
import tkinter as tk
from tkinter import messagebox
import heapq
import math
from collections import deque

# Directions for movement (up, down, left, right)
DIRECTIONS = [(-1, 0), (1, 0), (0, -1), (0, 1)]  # No diagonal movement for Manhattan heuristic
DIAGONAL_DIRECTIONS = DIRECTIONS + [(-1, -1), (-1, 1), (1, -1), (1, 1)]  # Adding diagonal moves

class PathfindingApp:
    def __init__(self, root, grid_size=20):
        self.root = root
        self.grid_size = grid_size
        self.canvas = tk.Canvas(root, width=grid_size * 30, height=grid_size * 30)
        self.canvas.pack()

        self.grid = [[0 for _ in range(grid_size)] for _ in range(grid_size)]  # 0 = free, 1 = obstacle
        self.start = None
        self.goal = None
        self.path = []

        self.create_ui()
        self.draw_grid()

    def create_ui(self):
        self.start_button = tk.Button(self.root, text="Set Start", command=self.set_start)
        self.start_button.pack(side=tk.LEFT)

        self.goal_button = tk.Button(self.root, text="Set Goal", command=self.set_goal)
        self.goal_button