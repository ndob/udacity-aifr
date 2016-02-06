# ----------
# User Instructions:
# 
# Define a function, search() that returns a list
# in the form of [optimal path length, row, col]. For
# the grid shown below, your function should output
# [11, 4, 5].
#
# If there is no valid path from the start point
# to the goal, your function should return the string
# 'fail'
# ----------

# Grid format:
#   0 = Navigable space
#   1 = Occupied space

grid = [[0, 0, 1, 0, 0, 0],
        [0, 0, 1, 0, 0, 0],
        [0, 0, 0, 0, 1, 0],
        [0, 0, 1, 1, 1, 0],
        [0, 0, 0, 0, 1, 0]]
init = [0, 0]
goal = [len(grid)-1, len(grid[0])-1]
cost = 1

delta = [[-1, 0], # go up
         [ 0,-1], # go left
         [ 1, 0], # go down
         [ 0, 1]] # go right

delta_name = ['^', '<', 'v', '>']

class node:
    def __init__(self, x, y, gvalue):
        self.x = x
        self.y = y
        self.gvalue = gvalue
    
    def __repr__(self):
        return str(self.x) + "," + str(self.y) + ":" + str(self.gvalue)

def search(grid,init,goal,cost):
    open_list = []
    closed_list = []
    grid_width = len(grid[0])
    grid_height = len(grid)
   
    open_list.append(node(init[1], init[0], 0))

    while True:
        if len(open_list) == 0:
            return "fail"
            
        open_list.sort(key=lambda item: item.gvalue)
        open_list.reverse()
        smallest = open_list.pop()

        if smallest.x == goal[1] and smallest.y == goal[0]:
            path = [smallest.gvalue, smallest.y, smallest.x]
            break

        closed_list.append(smallest)
        
        for i in range(len(delta)):
            new_elem = node(
                smallest.x + delta[i][1],
                smallest.y + delta[i][0], 
                smallest.gvalue + cost)

            # has the position already been visited?
            if any(elem.x == new_elem.x and elem.y == new_elem.y for elem in closed_list):
                continue

            # is out of bounds?
            if new_elem.x < 0 or new_elem.x >= grid_width or new_elem.y < 0 or new_elem.y >= grid_height:
                continue

            # is occupied?
            if grid[new_elem.y][new_elem.x] == 1:
                continue

            # is the position already in open list?
            if any(elem.x == new_elem.x and elem.y == new_elem.y for elem in open_list):
                continue

            open_list.append(new_elem)
            
    return path

print search(grid, init, goal, cost)
