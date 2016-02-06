# -----------
# User Instructions:
#
# Modify the the search function so that it returns
# a shortest path as follows:
# 
# [['>', 'v', ' ', ' ', ' ', ' '],
#  [' ', '>', '>', '>', '>', 'v'],
#  [' ', ' ', ' ', ' ', ' ', 'v'],
#  [' ', ' ', ' ', ' ', ' ', 'v'],
#  [' ', ' ', ' ', ' ', ' ', '*']]
#
# Where '>', '<', '^', and 'v' refer to right, left, 
# up, and down motions. Note that the 'v' should be 
# lowercase. '*' should mark the goal cell.
#
# You may assume that all test cases for this function
# will have a path from init to goal.
# ----------

grid = [[0, 0, 1, 0, 0, 0],
        [0, 0, 0, 0, 0, 0],
        [0, 0, 1, 0, 1, 0],
        [0, 0, 1, 0, 1, 0],
        [0, 0, 1, 0, 1, 0]]

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

def visualize(grid, init, goal, closed_list):
    visualized = [[" " for row in range(len(grid[0]))] for col in range(len(grid))]
    visualized[goal[0]][goal[1]] = "*"
    
    x = goal[1]
    y = goal[0]

    def find_elem(x, y, closed_list):
        for i in range(len(closed_list)):
            if closed_list[i].x == x and closed_list[i].y == y:
                return closed_list[i]
        return None

    while not (x == init[1] and y == init[0]):
        possible_next = []
        for i in range(len(delta)):
            to_x = x + delta[i][1]
            to_y = y + delta[i][0]
            elem = find_elem(to_x, to_y, closed_list)
            if elem != None and visualized[to_y][to_x] == " ":
                possible_next.append(elem)

        possible_next.sort(key=lambda item: item.gvalue)

        new_x = possible_next[0].x
        new_y = possible_next[0].y

        if new_x - x == -1:
            visualized[new_y][new_x] = ">"
        elif new_x - x == 1:
            visualized[new_y][new_x] = "<"
        if new_y - y == -1:
            visualized[new_y][new_x] = "v"
        elif new_y - y == 1:
            visualized[new_y][new_x] = "^"            

        x = new_x
        y = new_y

    # pretty print
    #for i in range(len(visualized)):
    #    print visualized[i]
    
    return visualized

def search(grid, init, goal, cost):
    open_list = []
    closed_list = []
    grid_width = len(grid[0])
    grid_height = len(grid)
    expand = [[-1 for row in range(grid_width)] for col in range(grid_height)]
   
    open_list.append(node(init[1], init[0], 0))

    while True:
        if len(open_list) == 0:
            break
            
        open_list.sort(key=lambda item: item.gvalue)
        open_list.reverse()
        smallest = open_list.pop()

        expand[smallest.y][smallest.x] = len(closed_list)
        
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
            
    return visualize(grid, init, goal, closed_list)

print search(grid, init, goal, cost)
