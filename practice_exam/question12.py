# -------------------
# Background Information
#
# In this problem, you will build a planner that helps a robot
# find the shortest way in a warehouse filled with boxes
# that he has to pick up and deliver to a drop zone.
# 
# For example:
#
# warehouse = [[ 1, 2, 3],
#              [ 0, 0, 0],
#              [ 0, 0, 0]]
# dropzone = [2,0] 
# todo = [2, 1]
# 
# The robot starts at the dropzone.
# The dropzone can be in any free corner of the warehouse map.
# todo is a list of boxes to be picked up and delivered to the dropzone.
#
# Robot can move diagonally, but the cost of a diagonal move is 1.5.
# The cost of moving one step horizontally or vertically is 1.
# So if the dropzone is at [2, 0], the cost to deliver box number 2
# would be 5.

# To pick up a box, the robot has to move into the same cell as the box.
# When the robot picks up a box, that cell becomes passable (marked 0)
# The robot can pick up only one box at a time and once picked up 
# it has to return the box to the dropzone by moving onto the dropzone cell.
# Once the robot has stepped on the dropzone, the box is taken away, 
# and it is free to continue with its todo list.
# Tasks must be executed in the order that they are given in the todo list.
# You may assume that in all warehouse maps, all boxes are
# reachable from beginning (the robot is not boxed in).

# -------------------
# User Instructions
#
# Design a planner (any kind you like, so long as it works!)
# in a function named plan() that takes as input three parameters: 
# warehouse, dropzone, and todo. See parameter info below.
#
# Your function should RETURN the final, accumulated cost to do
# all tasks in the todo list in the given order, which should
# match with our answer. You may include print statements to show 
# the optimum path, but that will have no effect on grading.
#
# Your solution must work for a variety of warehouse layouts and
# any length of todo list.
# 
# Add your code at line 76.
# 
# --------------------
# Parameter Info
#
# warehouse - a grid of values, where 0 means that the cell is passable,
# and a number 1 <= n <= 99 means that box n is located at that cell.
# dropzone - determines the robot's start location and the place to return boxes 
# todo - list of tasks, containing box numbers that have to be picked up
#
# --------------------
# Testing
#
# You may use our test function below, solution_check(),
# to test your code for a variety of input parameters. 

warehouse = [[ 1, 2, 3],
             [ 0, 0, 0],
             [ 0, 0, 0]]
dropzone = [2,0] 
todo = [2, 1]

# ------------------------------------------
# Astar - implements a*-search algorithm
# ----------------------------------------

class Astar:

    # represents single node on map
    # values f,g and h correspond to notation in a* definition
    class Node:
        def __init__(self, x, y, walkable):
            self.x = x
            self.y = y
            self.g = 0.0
            self.h = 0.0
            self.walkable = walkable
            self.parent = None

        def get_f(self):
            return self.g + self.h

        def __repr__(self):
            return str(self.x) + "," + str(self.y) + ":" + str(self.get_f())


    def __init__(self, warehouse_map):
        # initialize map with nodes
        self.nav_map = []
        for y in range(len(warehouse_map)):
            self.nav_map.append([])
            for x in range(len(warehouse_map[y])):
                self.nav_map[y].append(self.Node(x, y, warehouse_map[y][x] == 0))

    def get_heuristic_value(self, start_x, start_y, end_x, end_y):
        return abs(end_x - start_x) + abs(end_y - start_y)

    def is_diagonal(self, start_x, start_y, end_x, end_y):
        return abs(end_x - start_x) + abs(end_y - start_y) > 1

    def is_on_map(self, x, y):
        return y >= 0 and y < len(self.nav_map) and x >= 0 and x < len(self.nav_map[y])

    def get_cost(self, from_point, to_point):
        cost = 999999
        open_list = []
        closed_list = []
        open_list.append(self.nav_map[from_point[0]][from_point[1]])

        # make target walkable
        self.nav_map[to_point[0]][to_point[1]].walkable = True

        while len(open_list) > 0:
            # sort by smallest cost
            open_list = sorted(open_list, key=lambda x: x.get_f(), reverse=True)
            current = open_list.pop()
            closed_list.append(current)

            # found target
            if current.x == to_point[1] and current.y == to_point[0]:
                cost = current.get_f()
                break

            # iterate neighbours
            for x in [-1, 0, 1]:
                for y in [-1, 0, 1]:
                    if x == 0 and y == 0:
                        continue

                    neighbour_x = current.x + x
                    neighbour_y = current.y + y

                    if not self.is_on_map(neighbour_x, neighbour_y):
                        continue

                    neighbour_node = self.nav_map[neighbour_y][neighbour_x]

                    if not neighbour_node.walkable:
                        continue

                    # check if node is already on closed list
                    if len([node for node in closed_list if node.x == neighbour_x and node.y == neighbour_y]) > 0:
                        continue

                    if self.is_diagonal(current.x, current.y, neighbour_x, neighbour_y):
                        move_cost = 1.5
                    else:
                        move_cost = 1

                    # if node is in the open list -> check if current path is better
                    already_in_open_list = [node for node in open_list if node.x == neighbour_x and node.y == neighbour_y]
                    if len(already_in_open_list) > 0:
                        if current.g + move_cost < already_in_open_list[0].g:
                            already_in_open_list[0].parent = current
                            already_in_open_list[0].g = current.g + move_cost
                            already_in_open_list[0].h = self.get_heuristic_value(neighbour_x, neighbour_y, to_point[1], to_point[0])
                    else:
                        neighbour_node.parent = current
                        neighbour_node.g = current.g + move_cost
                        neighbour_node.h = self.get_heuristic_value(neighbour_x, neighbour_y, to_point[1], to_point[0])
                        open_list.append(neighbour_node)
        
        return cost

# ------------------------------------------
# find_item - Returns the coordinates for an item
# ----------------------------------------

def find_item(warehouse, item_id):
    for y in range(len(warehouse)):
        for x in range(len(warehouse[y])):
            if warehouse[y][x] == item_id:
                return [y,x]

# ------------------------------------------
# plan - Returns cost to take all boxes in the todo list to dropzone
# ----------------------------------------

def plan(warehouse, dropzone, todo):
    cumulative_cost = 0
    for item_id in todo:
        item_position = find_item(warehouse, item_id)
        if item_position != None:
            pathfinder = Astar(warehouse)
            # multiply by 2 to get cost for "there and back"
            cumulative_cost += pathfinder.get_cost(dropzone, item_position) * 2
            # remove the item from map
            warehouse[item_position[0]][item_position[1]] = 0
            print warehouse
    
    return cumulative_cost
    
################# TESTING ##################
       
# ------------------------------------------
# solution check - Checks your plan function using
# data from list called test[]. Uncomment the call
# to solution_check to test your code.
#
def solution_check(test, epsilon = 0.00001):
    answer_list = []
    
    import time
    start = time.clock()
    correct_answers = 0
    for i in range(len(test[0])):
        user_cost = plan(test[0][i], test[1][i], test[2][i])
        true_cost = test[3][i]
        if abs(user_cost - true_cost) < epsilon:
            print "\nTest case", i+1, "passed!"
            answer_list.append(1)
            correct_answers += 1
            #print "#############################################"
        else:
            print "\nTest case ", i+1, "unsuccessful. Your answer ", user_cost, "was not within ", epsilon, "of ", true_cost 
            answer_list.append(0)
    runtime =  time.clock() - start
    if runtime > 1:
        print "Your code is too slow, try to optimize it! Running time was: ", runtime
        return False
    if correct_answers == len(answer_list):
        print "\nYou passed all test cases!"
        return True
    else:
        print "\nYou passed", correct_answers, "of", len(answer_list), "test cases. Try to get them all!"
        return False
#Testing environment
# Test Case 1 
warehouse1 = [[ 1, 2, 3],
             [ 0, 0, 0],
             [ 0, 0, 0]]
dropzone1 = [2,0] 
todo1 = [2, 1]
true_cost1 = 9
# Test Case 2
warehouse2 = [[   1, 2, 3, 4],
              [   0, 0, 0, 0],
              [   5, 6, 7, 0],
              [ 'x', 0, 0, 8]] 
dropzone2 = [3,0] 
todo2 = [2, 5, 1]
true_cost2 = 21

# Test Case 3
warehouse3 = [[   1, 2,  3,  4, 5, 6,  7],
              [   0, 0,  0,  0, 0, 0,  0],
              [   8, 9, 10, 11, 0, 0,  0],
              [ 'x', 0,  0,  0, 0, 0, 12]] 
dropzone3 = [3,0] 
todo3 = [5, 10]
true_cost3 = 18

# Test Case 4
warehouse4 = [[ 1, 17, 5, 18,  9, 19,  13],
              [ 2,  0, 6,  0, 10,  0,  14],
              [ 3,  0, 7,  0, 11,  0,  15],
              [ 4,  0, 8,  0, 12,  0,  16],
              [ 0,  0, 0,  0,  0,  0, 'x']] 
dropzone4 = [4,6]
todo4 = [13, 11, 6, 17]
true_cost4 = 41

testing_suite = [[warehouse1, warehouse2, warehouse3, warehouse4],
                 [dropzone1, dropzone2, dropzone3, dropzone4],
                 [todo1, todo2, todo3, todo4],
                 [true_cost1, true_cost2, true_cost3, true_cost4]]

solution_check(testing_suite) #UNCOMMENT THIS LINE TO TEST YOUR CODE
