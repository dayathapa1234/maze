import unittest
from maze import Maze
from window import Window

class Tests(unittest.TestCase):
    def test_maze_create_cells(self):
        num_cols = 12
        num_rows = 10
        m1 = Maze(0, 0, num_rows, num_cols, 10, 10)
        self.assertEqual(len(m1._cells), num_cols)
        self.assertEqual(len(m1._cells[0]), num_rows)

    def test_maze_with_different_sizes(self):
        m2 = Maze(0, 0, 5, 5, 20, 20)
        self.assertEqual(len(m2._cells), 5)
        self.assertEqual(len(m2._cells[0]), 5)

    def test_maze_large(self):
        m3 = Maze(10, 10, 20, 15, 15, 15)
        self.assertEqual(len(m3._cells), 15)
        self.assertEqual(len(m3._cells[0]), 20)

    def test_break_entrance_and_exit(self):
            num_cols = 10
            num_rows = 8
            maze = Maze(0, 0, num_rows, num_cols, 10, 10)
            self.assertFalse(maze._cells[0][0].has_top_wall)
            last_col = len(maze._cells) - 1
            last_row = len(maze._cells[0]) - 1
            self.assertFalse(maze._cells[last_col][last_row].has_bottom_wall)
            
    def test_reset_cells_visited(self):
        maze = Maze(0, 0, 5, 5, 20, 20)
        maze._reset_cells_visited()
        for col in maze._cells:
            for cell in col:
                self.assertFalse(cell.visited)
if __name__ == "__main__":
    unittest.main()
