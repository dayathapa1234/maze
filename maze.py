from cell import Cell
import random
import time
from collections import deque
import heapq

class Maze:
    def __init__(
        self,
        x1,
        y1,
        num_rows,
        num_cols,
        cell_size_x,
        cell_size_y,
        win=None,
        seed=None,
    ):
        self._cells = []
        self._x1 = x1
        self._y1 = y1
        self._num_rows = num_rows
        self._num_cols = num_cols
        self._cell_size_x = cell_size_x
        self._cell_size_y = cell_size_y
        self._win = win
        if seed is not None:
            random.seed(seed)

        self._create_cells()
        self._break_entrance_and_exit()
        self._break_walls_r(0, 0)
        self._reset_cells_visited()  # Clear visited flags for solving

    def _create_cells(self):
        # Create a grid of cells, stored as _cells[i][j] where i = column, j = row.
        for i in range(self._num_cols):
            col_cells = []
            for j in range(self._num_rows):
                col_cells.append(Cell(self._win))
            self._cells.append(col_cells)
        # Draw every cell (using _draw_cell to compute coordinates)
        for i in range(self._num_cols):
            for j in range(self._num_rows):
                self._draw_cell(i, j)

    def _draw_cell(self, i, j):
        if self._win is None:
            return
        # Calculate pixel coordinates for cell (i, j)
        x1 = self._x1 + i * self._cell_size_x
        y1 = self._y1 + j * self._cell_size_y
        x2 = x1 + self._cell_size_x
        y2 = y1 + self._cell_size_y
        self._cells[i][j].draw(x1, y1, x2, y2)
        self._animate()

    def _animate(self):
        if self._win is None:
            return
        self._win.redraw()
        time.sleep(0.005)  # Adjust this delay for faster/slower animation

    def _break_entrance_and_exit(self):
        # Break the entrance (top wall of start cell)
        self._cells[0][0].has_top_wall = False
        self._draw_cell(0, 0)
        # Break the exit (bottom wall of goal cell)
        self._cells[self._num_cols - 1][self._num_rows - 1].has_bottom_wall = False
        self._draw_cell(self._num_cols - 1, self._num_rows - 1)

    def _break_walls_r(self, i, j):
        self._cells[i][j].visited = True
        while True:
            next_index_list = []
            # Check left
            if i > 0 and not self._cells[i - 1][j].visited:
                next_index_list.append((i - 1, j))
            # Check right
            if i < self._num_cols - 1 and not self._cells[i + 1][j].visited:
                next_index_list.append((i + 1, j))
            # Check up
            if j > 0 and not self._cells[i][j - 1].visited:
                next_index_list.append((i, j - 1))
            # Check down
            if j < self._num_rows - 1 and not self._cells[i][j + 1].visited:
                next_index_list.append((i, j + 1))
            if len(next_index_list) == 0:
                self._draw_cell(i, j)
                return
            direction_index = random.randrange(len(next_index_list))
            next_index = next_index_list[direction_index]
            # Knock down walls between current and next cell.
            if next_index[0] == i + 1:
                self._cells[i][j].has_right_wall = False
                self._cells[i + 1][j].has_left_wall = False
            if next_index[0] == i - 1:
                self._cells[i][j].has_left_wall = False
                self._cells[i - 1][j].has_right_wall = False
            if next_index[1] == j + 1:
                self._cells[i][j].has_bottom_wall = False
                self._cells[i][j + 1].has_top_wall = False
            if next_index[1] == j - 1:
                self._cells[i][j].has_top_wall = False
                self._cells[i][j - 1].has_bottom_wall = False
            self._break_walls_r(next_index[0], next_index[1])

    def _reset_cells_visited(self):
        for col in self._cells:
            for cell in col:
                cell.visited = False

    # --- Depth-first search solver (DFS) using draw_move for animation ---
    def _solve_r(self, i, j):
        self._animate()
        self._cells[i][j].visited = True
        # If at goal, return True
        if i == self._num_cols - 1 and j == self._num_rows - 1:
            return True

        # Try moving left.
        if (
            i > 0
            and not self._cells[i][j].has_left_wall
            and not self._cells[i - 1][j].visited
        ):
            self._cells[i][j].draw_move(self._cells[i - 1][j])
            if self._solve_r(i - 1, j):
                return True
            else:
                self._cells[i][j].draw_move(self._cells[i - 1][j], undo=True)
        # Try moving right.
        if (
            i < self._num_cols - 1
            and not self._cells[i][j].has_right_wall
            and not self._cells[i + 1][j].visited
        ):
            self._cells[i][j].draw_move(self._cells[i + 1][j])
            if self._solve_r(i + 1, j):
                return True
            else:
                self._cells[i][j].draw_move(self._cells[i + 1][j], undo=True)
        # Try moving up.
        if (
            j > 0
            and not self._cells[i][j].has_top_wall
            and not self._cells[i][j - 1].visited
        ):
            self._cells[i][j].draw_move(self._cells[i][j - 1])
            if self._solve_r(i, j - 1):
                return True
            else:
                self._cells[i][j].draw_move(self._cells[i][j - 1], undo=True)
        # Try moving down.
        if (
            j < self._num_rows - 1
            and not self._cells[i][j].has_bottom_wall
            and not self._cells[i][j + 1].visited
        ):
            self._cells[i][j].draw_move(self._cells[i][j + 1])
            if self._solve_r(i, j + 1):
                return True
            else:
                self._cells[i][j].draw_move(self._cells[i][j + 1], undo=True)
        return False

    # --- Breadth-first search solver (BFS) using draw_move to animate path ---
    def _solve_bfs(self):
        self._reset_cells_visited()
        queue = deque()
        parent = {}
        start = (0, 0)
        queue.append(start)
        parent[start] = None
        self._cells[0][0].visited = True

        while queue:
            i, j = queue.popleft()
            if i == self._num_cols - 1 and j == self._num_rows - 1:
                # Reconstruct the path.
                path = []
                cur = (i, j)
                while cur is not None:
                    path.append(cur)
                    cur = parent[cur]
                path.reverse()
                # Animate the solution path using draw_move between consecutive cells.
                for idx in range(len(path) - 1):
                    current_cell = self._cells[path[idx][0]][path[idx][1]]
                    next_cell = self._cells[path[idx+1][0]][path[idx+1][1]]
                    current_cell.draw_move(next_cell, undo=False)
                    self._animate()
                return True

            current = self._cells[i][j]
            # Determine neighbors based on available passages.
            neighbors = []
            if i > 0 and not current.has_left_wall:
                neighbors.append((i - 1, j))
            if i < self._num_cols - 1 and not current.has_right_wall:
                neighbors.append((i + 1, j))
            if j > 0 and not current.has_top_wall:
                neighbors.append((i, j - 1))
            if j < self._num_rows - 1 and not current.has_bottom_wall:
                neighbors.append((i, j + 1))

            for ni, nj in neighbors:
                if not self._cells[ni][nj].visited:
                    self._cells[ni][nj].visited = True
                    parent[(ni, nj)] = (i, j)
                    queue.append((ni, nj))
        return False

    # --- A* solver using draw_move for animation ---
    def _heuristic(self, cell, goal):
        # Manhattan distance.
        return abs(cell[0] - goal[0]) + abs(cell[1] - goal[1])

    def _solve_a_star(self):
        self._reset_cells_visited()
        open_set = []
        parent = {}
        start = (0, 0)
        goal = (self._num_cols - 1, self._num_rows - 1)
        # Initialize scores.
        g_score = { (i, j): float('inf') for i in range(self._num_cols) for j in range(self._num_rows) }
        f_score = { (i, j): float('inf') for i in range(self._num_cols) for j in range(self._num_rows) }
        g_score[start] = 0
        f_score[start] = self._heuristic(start, goal)
        heapq.heappush(open_set, (f_score[start], start))
        self._cells[start[0]][start[1]].visited = True

        while open_set:
            current_f, (i, j) = heapq.heappop(open_set)
            if (i, j) == goal:
                # Reconstruct the path.
                path = []
                cur = (i, j)
                while cur is not None:
                    path.append(cur)
                    cur = parent.get(cur)
                path.reverse()
                # Animate the solution path using draw_move.
                for idx in range(len(path) - 1):
                    current_cell = self._cells[path[idx][0]][path[idx][1]]
                    next_cell = self._cells[path[idx+1][0]][path[idx+1][1]]
                    current_cell.draw_move(next_cell, undo=False)
                    self._animate()
                return True

            current = self._cells[i][j]
            # Determine neighbors.
            neighbors = []
            if i > 0 and not current.has_left_wall:
                neighbors.append((i - 1, j))
            if i < self._num_cols - 1 and not current.has_right_wall:
                neighbors.append((i + 1, j))
            if j > 0 and not current.has_top_wall:
                neighbors.append((i, j - 1))
            if j < self._num_rows - 1 and not current.has_bottom_wall:
                neighbors.append((i, j + 1))

            for (ni, nj) in neighbors:
                tentative_g = g_score[(i, j)] + 1
                if tentative_g < g_score[(ni, nj)]:
                    parent[(ni, nj)] = (i, j)
                    g_score[(ni, nj)] = tentative_g
                    f_score[(ni, nj)] = tentative_g + self._heuristic((ni, nj), goal)
                    if not self._cells[ni][nj].visited:
                        heapq.heappush(open_set, (f_score[(ni, nj)], (ni, nj)))
                        self._cells[ni][nj].visited = True
        return False

    # Public solver interface.
    def solve(self, algo="depth first search"):
        algo = algo.lower()
        if algo in ["depth first search", "dfs"]:
            return self._solve_r(0, 0)
        elif algo in ["breadth first search", "bfs"]:
            return self._solve_bfs()
        elif algo in ["a*", "astar", "a star"]:
            return self._solve_a_star()
        else:
            return False
