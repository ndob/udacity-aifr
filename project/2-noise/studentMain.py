# ----------
# Part Two
#
# Now we'll make the scenario a bit more realistic. Now Traxbot's
# sensor measurements are a bit noisy (though its motions are still
# completetly noise-free and it still moves in an almost-circle).
# You'll have to write a function that takes as input the next
# noisy (x, y) sensor measurement and outputs the best guess 
# for the robot's next position.
#
# ----------
# YOUR JOB
#
# Complete the function estimate_next_pos. You will be considered 
# correct if your estimate is within 0.01 stepsizes of Traxbot's next
# true position. 
#
# ----------
# GRADING
# 
# We will make repeated calls to your estimate_next_pos function. After
# each call, we will compare your estimated position to the robot's true
# position. As soon as you are within 0.01 stepsizes of the true position,
# you will be marked correct and we will tell you how many steps it took
# before your function successfully located the target bot.

# These import steps give you access to libraries which you may (or may
# not) want to use.

from robot import *  # Check the robot.py tab to see how this works.
from math import *
from matrix import * # Check the matrix.py tab to see how this works.
import random

class ExtendedKalmanFilter:

    # x: initial state
    # u: external motion
    # P: initial uncertainty
    # R: measurement uncertainty
    def __init__(self, x, u, P, R):
        self.x = x
        self.u = u
        self.P = P
        self.R = R

        # Identity matrix should have the same dimensionality as state space
        self.I = matrix([[]])
        self.I.identity(x.dimx)

    def predict(self):      
        # predict (a priori)
        self.x = self.g()
        self.P = self.G * self.P * self.G.transpose()

        return self.x
    
    def measure(self, measurement):

        # a posteriori
        Z = matrix([measurement])        
        y = Z.transpose() - self.h()
        S = self.H * self.P * self.H.transpose() + self.R

        K = self.P * self.H.transpose() * S.inverse()
        self.x = self.x + (K * y)
        self.P = (self.I - (K * self.H)) * self.P

        return self.x

    def h(self):
        raise NotImplementedError("Implement h-function.")

    def g(self):
        raise NotImplementedError("Implement g-function.")

class RobotFilter(ExtendedKalmanFilter):
    def h(self):
        # answers: what would be the next measurement according to current state?
        newx = matrix([[]])
        newx.zero(2,1)

        newx.value[0][0] = self.x.value[0][0]
        newx.value[1][0] = self.x.value[1][0]

        # jacobian
        self.H = matrix([
            [1., 0., 0., 0., 0.],
            [0., 1., 0., 0., 0.]
        ])
        return newx

    def g(self):
        distancediff = self.x.value[2][0]
        anglediff = self.x.value[3][0]
        theta = self.x.value[4][0] + anglediff

        x = self.x.value[0][0] + cos(theta) * distancediff
        y = self.x.value[1][0] + sin(theta) * distancediff

        # jacobian
        self.G = matrix([
            [1., 0., cos(self.x.value[4][0]), 0., -1 * sin(self.x.value[4][0]) * distancediff],
            [0., 1., sin(self.x.value[4][0]), 0., cos(self.x.value[4][0]) * distancediff],
            [0., 0., 1., 0., 0.],
            [0., 0., 0., 1., 0.],
            [0., 0., 0., 1., 1.]
        ])

        moved_angle = atan2(y - self.x.value[1][0], x - self.x.value[0][0])
        new_angle_diff = (moved_angle - self.x.value[4][0]) % (pi * 2)
        new_distance_diff = distance_between([x,y], [self.x.value[0][0],self.x.value[1][0]])

        self.x.value[0][0] = x
        self.x.value[1][0] = y
        self.x.value[2][0] = new_distance_diff
        self.x.value[3][0] = new_angle_diff
        self.x.value[4][0] = theta
        return self.x


# This is the function you have to write. Note that measurement is a 
# single (x, y) point. This function will have to be called multiple
# times before you have enough information to accurately predict the
# next position. The OTHER variable that your function returns will be 
# passed back to your function the next time it is called. You can use
# this to keep track of important information over time.
def estimate_next_pos(measurement, OTHER = None):
    xy_estimate = measurement

    if OTHER == None or not OTHER.has_key("initialized"):
        xy_estimate = measurement

        if OTHER == None:
            OTHER = dict()
        if OTHER.has_key("measurement"):
            OTHER["initialized"] = True
            OTHER["angle"] = atan2(measurement[1] - OTHER["measurement"][1], measurement[0] - OTHER["measurement"][0])
        OTHER["measurement"] = measurement
    elif not OTHER.has_key("kalman"):
        moved_angle = atan2(measurement[1] - OTHER["measurement"][1], measurement[0] - OTHER["measurement"][0])
        angle_diff = moved_angle - OTHER["angle"]
        distance_diff = distance_between(measurement, OTHER["measurement"])

        x = matrix([[measurement[0]], [measurement[1]], [distance_diff], [angle_diff], [moved_angle]])
        u = matrix([[0.], [0.], [0.], [0.], [0.]])

        # initial uncerties
        P = matrix([
            [0.05, 0., 0., 0., 0.],
            [0., 0.05, 0., 0., 0.],
            [0., 0., 0.05, 0., 0.],
            [0., 0., 0., 0.05, 0.],
            [0., 0., 0., 0., 0.05]          
        ])

        # measurement uncertainty
        R = matrix([
            [0.1, 0.],
            [0., 0.1], 
        ])

        OTHER["kalman"] = RobotFilter(x, u, P, R)
        OTHER["kalman"].measure([measurement[0], measurement[1]])
        OTHER["kalman"].predict()
    else:
        OTHER["kalman"].measure([measurement[0], measurement[1]])
        estimate = OTHER["kalman"].predict()
        xy_estimate = (estimate.value[0][0], estimate.value[1][0])

    return xy_estimate, OTHER

# A helper function you may find useful.
def distance_between(point1, point2):
    """Computes distance between point1 and point2. Points are (x, y) pairs."""
    x1, y1 = point1
    x2, y2 = point2
    return sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

# This is here to give you a sense for how we will be running and grading
# your code. Note that the OTHER variable allows you to store any 
# information that you want. 
def demo_grading(estimate_next_pos_fcn, target_bot, OTHER = None):
    localized = False
    distance_tolerance = 0.01 * target_bot.distance
    ctr = 0
    # if you haven't localized the target bot, make a guess about the next
    # position, then we move the bot and compare your guess to the true
    # next position. When you are close enough, we stop checking.
    while not localized and ctr <= 1000:
        ctr += 1
        measurement = target_bot.sense()
        position_guess, OTHER = estimate_next_pos_fcn(measurement, OTHER)
        target_bot.move_in_circle()
        true_position = (target_bot.x, target_bot.y)
        error = distance_between(position_guess, true_position)
        if error <= distance_tolerance:
            print "You got it right! It took you ", ctr, " steps to localize."
            localized = True
        if ctr == 1000:
            print "Sorry, it took you too many steps to localize the target."
    return localized

def demo_grading_visual(estimate_next_pos_fcn, target_bot, OTHER = None):
    localized = False
    distance_tolerance = 0.01 * target_bot.distance
    ctr = 0
    # if you haven't localized the target bot, make a guess about the next
    # position, then we move the bot and compare your guess to the true
    # next position. When you are close enough, we stop checking.
    #For Visualization
    import turtle    #You need to run this locally to use the turtle module
    window = turtle.Screen()
    #window.delay(50)
    window.bgcolor('white')
    size_multiplier= 25.0  #change Size of animation
    broken_robot = turtle.Turtle()
    broken_robot.shape('turtle')
    broken_robot.color('green')
    broken_robot.resizemode('user')
    broken_robot.shapesize(0.1, 0.1, 0.1)
    measured_broken_robot = turtle.Turtle()
    measured_broken_robot.shape('circle')
    measured_broken_robot.color('red')
    measured_broken_robot.resizemode('user')
    measured_broken_robot.shapesize(0.1, 0.1, 0.1)
    prediction = turtle.Turtle()
    prediction.shape('arrow')
    prediction.color('blue')
    prediction.resizemode('user')
    prediction.shapesize(0.1, 0.1, 0.1)
    prediction.penup()
    broken_robot.penup()
    measured_broken_robot.penup()
    #End of Visualization
    while not localized and ctr <= 1000:
        ctr += 1
        measurement = target_bot.sense()
        position_guess, OTHER = estimate_next_pos_fcn(measurement, OTHER)
        target_bot.move_in_circle()
        true_position = (target_bot.x, target_bot.y)
        error = distance_between(position_guess, true_position)
        print "diff:", position_guess, true_position
        print "error:", error
        if error <= distance_tolerance:
            print "You got it right! It took you ", ctr, " steps to localize."
            localized = True
        if ctr == 1000:
            print "Sorry, it took you too many steps to localize the target."
        #More Visualization
        measured_broken_robot.setheading(target_bot.heading*180/pi)
        measured_broken_robot.goto(measurement[0]*size_multiplier, measurement[1]*size_multiplier-200)
        measured_broken_robot.stamp()
        broken_robot.setheading(target_bot.heading*180/pi)
        broken_robot.goto(target_bot.x*size_multiplier, target_bot.y*size_multiplier-200)
        broken_robot.stamp()
        prediction.setheading(target_bot.heading*180/pi)
        prediction.goto(position_guess[0]*size_multiplier, position_guess[1]*size_multiplier-200)
        prediction.stamp()
        #End of Visualization
    return localized


# This is a demo for what a strategy could look like. This one isn't very good.
def naive_next_pos(measurement, OTHER = None):
    """This strategy records the first reported position of the target and
    assumes that eventually the target bot will eventually return to that 
    position, so it always guesses that the first position will be the next."""
    if not OTHER: # this is the first measurement
        OTHER = measurement
    xy_estimate = OTHER 
    return xy_estimate, OTHER

# This is how we create a target bot. Check the robot.py file to understand
# How the robot class behaves.
test_target = robot(2.1, 4.3, 0.5, 2*pi / 34.0, 1.5)
measurement_noise = 0.05 * test_target.distance
test_target.set_noise(0.0, 0.0, measurement_noise)

demo_grading(estimate_next_pos, test_target)




