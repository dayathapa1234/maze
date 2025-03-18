from window import Window
from point import Point
from line import Line
from cell import Cell
from maze import Maze
import sys
import time

import tkinter as tk

def launch_config():
    config_root = tk.Tk()
    config_root.title("Maze Configurations")

    tk.Label(config_root, text="Rows").grid(row=0, column=0)
    rows_entry = tk.Entry(config_root)
    rows_entry.insert(0, "20")
    rows_entry.grid(row=0, column=1)

    tk.Label(config_root, text="Columns").grid(row=1, column=0)
    cols_entry = tk.Entry(config_root)
    cols_entry.insert(0, "25")
    cols_entry.grid(row=1, column=1)

    tk.Label(config_root, text="Animation Speed (sec)").grid(row=2, column=0)
    speed_entry = tk.Entry(config_root)
    speed_entry.insert(0, "0.005")
    speed_entry.grid(row=2, column=1)

    algorithm_var = tk.StringVar(config_root)
    algorithm_var.set("DFS")  # default algorithm
    tk.Label(config_root, text="Algorithm").grid(row=3, column=0)
    tk.OptionMenu(config_root, algorithm_var, "DFS", "BFS", "A*").grid(row=3, column=1)

    def start_maze():
        # Retrieve configuration values
        rows = int(rows_entry.get())
        cols = int(cols_entry.get())
        speed = float(speed_entry.get())
        algorithm = algorithm_var.get()
        config_root.destroy()
        launch_maze(rows, cols, speed, algorithm)

    tk.Button(config_root, text="Start Maze", command=start_maze).grid(row=4, column=0, columnspan=2)
    config_root.mainloop()

def launch_maze(rows, cols, speed, algorithm):
    # Set up window and maze parameters
    from window import Window
    from maze import Maze
    screen_x, screen_y = 800, 600
    margin = 50
    cell_size_x = (screen_x - 2 * margin) / cols
    cell_size_y = (screen_y - 2 * margin) / rows
    win = Window(screen_x, screen_y)
    maze = Maze(margin, margin, rows, cols, cell_size_x, cell_size_y, win, seed=0)
    maze._animation_speed = speed  # for example
    start_time = time.time()
    if algorithm == "DFS":
        solved = maze.solve()
    elif algorithm == "BFS":
        solved = maze.solve(algo="breadth first search") 
    elif algorithm == "A*":
        solved = maze.solve(algo="A*") 
    end_time = time.time()
    print(f"{algorithm} solved the maze in {end_time - start_time:.3f} seconds")   
    print("Maze solved:" if solved else "Maze cannot be solved!")
    win.wait_for_close()

if __name__ == "__main__":
    launch_config()