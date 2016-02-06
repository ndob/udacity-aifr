# -----------
# User Instructions:
#
# Modify the the search function so that it becomes
# an A* search algorithm as defined in the previous
# lectures.
#
# Your function should return the expanded grid
# which shows, for each element, the count when
# it was expanded or -1 if the element was never expanded.
# 
# If there is no path from init to goal,
# the function should return the string 'fail'
# ----------


grid = [[0, 1, 0, 1, 0, 0],
        [0, 1, 0, 1, 0, 0],
        [0, 1, 0, 1, 0, 0],
        [0, 1, 0, 1, 0, 0],
        [0, 0, 0, 1, 0, 0,]]
heuristic = [[9, 8, 7, 6, 5, 4],
             [8, 7, 6, 5, 4, 3],
             [7, 6, 5, 4, 3, 2],
             [6, 5, 4, 3, 2, 1],
             [5, 4, 3, 2, 1, 0]]

init = [0, 0]
goal = [len(grid)-1, len(grid[0])-1]
cost = 1

delta = [[-1, 0], # go up
         [ 0,-1], # go left
         [ 1, 0], # go down
         [ 0, 1]] # go right

delta_name = ['^', '<', 'v', '>']

class node:
    def __init__(self, x, y, gvalue, hvalue):
        self.x = x
        self.y = y
        self.gvalue = gvalue
        self.hvalue = hvalue
        self.fvalue = gvalue + hvalue
    
    def __repr__(self):
        return str(self.x) + "," + str(self.y) + " g:" + str(self.gvalue) + " f:" + str(self.fvalue)

def search(grid, init, goal, cost, heuristic):
    open_list = []
    closed_list = []
    grid_width = len(grid[0])
    grid_height = len(grid)
    expand = [[-1 for row in range(grid_width)] for col in range(grid_height)]
   
    open_list.append(node(init[1], init[0], 0, heuristic[init[1]][init[0]]))

    while True:
        if len(open_list) == 0:
            return "fail"
            
        open_list.sort(key=lambda item: item.fvalue)
        open_list.reverse()
        smallest = open_list.pop()

        expand[smallest.y][smallest.x] = len(closed_list)

        if smallest.x == goal[1] and smallest.y == goal[0]:
            path = [smallest.gvalue, smallest.y, smallest.x]
            break

        closed_list.append(smallest)

        for i in range(len(delta)):
            new_x = smallest.x + delta[i][1]
            new_y = smallest.y + delta[i][0]

            # has the position already been visited?
            if any(elem.x == new_x and elem.y == new_y for elem in closed_list):
                continue

            # is out of bounds?
            if new_x < 0 or new_x >= grid_width or new_y < 0 or new_y >= grid_height:
                continue

            # is occupied?
            if grid[new_y][new_x] == 1:
                continue

            # is the position already in open list?
            if any(elem.x == new_x and elem.y == new_y for elem in open_list):
                continue

            new_elem = node(
                new_x,
                new_y,
                smallest.gvalue + cost,
                heuristic[new_y][new_x])
            open_list.append(new_elem)
            
    return expand

print search(grid, init, goal, cost, heuristic)
