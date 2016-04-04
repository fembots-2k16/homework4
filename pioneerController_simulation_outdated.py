#!/usr/bin/env python

#definitely using this as a big reference
#http://www.laurentluce.com/posts/solving-mazes-using-python-simple-recursivity-and-a-search/

import rospy
import heapq
import math
from nav_msgs.msg import Odometry
from nav_msgs.msg import MapMetaData
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from p2os_msgs.msg import MotorState
from geometry_msgs.msg import Twist
from p2os_msgs.msg import SonarArray

####################################################
####################GOAL COORDINATES###############
####################################################
start = {'x': 11, 'y': 25, 'theta': 0.00}
end = {'x':48, 'y':25}
#end = {'x':12, 'y':38}

#end = {'x': 24, 'y': 13}
#end = {'x': 21, 'y': 26}
move_amt = 0.38
move_speed = 0.38
grid_size = 5 #math.sqrt(pixels per cell) (width & height)
####################################################
##################################################3

class Cell(object):
    def __init__(self, x, y, reachable, occupied):
        self.x = x
        self.y = y
        self.reachable = reachable
        self.occupied = occupied
        self.parent = None
        self.g = 0
        self.h = 0
        self.f = 0

class Robot(object):
    def __init__(self):
        self.queue = []
        self.prev_queue = []

        self.angle = 0
        self.pose = None
        self.prev_pose = None
        self.initial_pose = None
        self.laser = ()

        self.angle_sensitivity = 40
        self.proximity_sensitivity = 0.4
        self.is_spinning = 0
        self.is_stuck = 0
        self.moving_forward = 0
        self.count = 0

        self.was_i_turning_left = False
        self.get_off_the_wall = False
        self.reverse_flag = False
        self.flip_flag = False
        self.just_started_turning = False
        self.looking_for_right = False

        self.total_dist = 0

        self.map = None
        self.map_metadata = None

        self.grid = None
        self.start = None
        self.end = None

        #more stuff for A* search
        self.opened = []
        heapq.heapify(self.opened)
        self.closed = set()

    def setPose(self, pose):
        if self.initial_pose == None:
            self.initial_pose = pose

        if (self.prev_pose != None):
            dist = self.getDistanceTraveledDelta()
            if dist < 0.001:
                self.is_stuck += 1
            else: self.is_stuck = 0

        self.prev_pose = self.pose
        self.pose = pose

        #print self.pose

    def getDistanceTraveled(self):
        x1 = self.initial_pose.pose.position.x
        y1 = self.initial_pose.pose.position.y

        x2 = self.pose.pose.position.x
        y2 = self.pose.pose.position.y

        return self.dist(x1, y1, x2, y2)

    def getDistanceTraveledDelta(self):
        x1 = self.prev_pose.pose.position.x
        y1 = self.prev_pose.pose.position.y

        x2 = self.pose.pose.position.x
        y2 = self.pose.pose.position.y

        #print "x1:",x1,",y1:",y1,",x2:",x2,",y2:",y2

        dist = self.dist(x1, y1, x2, y2)
        self.total_dist += dist
        return dist

    def dist(self, x1, y1, x2, y2):
        return math.sqrt(math.pow(x2-x1, 2) + math.pow(y2-y1, 2))

    def hasFrontObstacle(self):
        return self.hasObstacle(90, self.angle_sensitivity, self.proximity_sensitivity)
    def hasLeftObstacle(self):
        return self.hasObstacle(180, 45, self.proximity_sensitivity)
    def hasRightObstacle(self):
        return self.hasObstacle(0, 45, self.proximity_sensitivity)
    def hasObstacle(self, angle, offset, s):
        has_obstacle = False
        i = angle - offset
        while i < angle + offset:
            if i < 0:
                i += 1
                continue
            if i >= len(self.laser.ranges): break
            has_obstacle = has_obstacle or (self.laser.ranges[i] < s)
            i += 1
        return has_obstacle

    def orientationToAngle(self, orient):
        angel = 2*math.degrees(math.asin(orient))
        if angel < 0:
            angel += 360
        return angel

    def angleToOrientation(self, angle):
        #angle = angle % 360
        mult = 1
        if angle > 180: mult = -1
        orient = math.sin(math.radians(angle/2))
        if mult < 0:
            orient = mult*orient
        return orient

    def turnToward(self, goal):
        return ["turnToward", True, goal]

    def turn(self, amt, dir):
        self.just_started_turning = True
        #print "turn,",amt,self.angle,self.angle+amt
        return ["turn", False, self.angle+amt, dir]
    def leftTurn(self):
        return self.turn(90, 1)
    def rightTurn(self):
        return self.turn(-90, -1)
    def properDiffAngle(self, curr_angle, desired_angle, dir):
        #angels
        if dir > 0:
            if self.count < 0:
                if curr_angle > 270:
                    curr_angle -= 360
                else:
                    self.count = 0
            elif curr_angle < 0 or (self.count > 0 and curr_angle < desired_angle):
                curr_angle += 360
        else:
            if (curr_angle > 330 or self.count > 0):
                curr_angle -= 360
                if self.count >= 0:
                    self.count += 1

        return curr_angle - desired_angle


    def moveForward(self, dist):
        return ["move", False, dist, None, None]
    def properDiffDist(self, x1, y1, x2, y2):
        if self.angle == 0:
            return x1 - x2
        if self.angle == 90:
            return y1 - y2
        if self.angle == 180:
            return x2 - x1
        if self.angle == 270:
            return y2 - y1

    def stop(self):
        return ["stop", False, 5]

    def isQueueEmpty(self):
        return len(self.queue) == 0

    def navigate(self, twist):
        if self.isQueueEmpty():
            return

        item = self.queue[0]
        command = item[0]
        if command == "turnToward":
            goal = item[2]

            queue_tail = self.queue[:]
            self.queue = []
            if goal == 0:
                if self.angle == 270:
                    self.queue.append(self.leftTurn())
                if self.angle == 90:
                    self.queue.append(self.rightTurn())
                if self.angle == 180:
                    self.queue.append(self.leftTurn())
                    self.queue.append(self.leftTurn())
            if goal == 90:
                if self.angle == 0:
                    self.queue.append(self.leftTurn())
                if self.angle == 180:
                    self.queue.append(self.rightTurn())
                if self.angle == 270:
                    self.queue.append(self.leftTurn())
                    self.queue.append(self.leftTurn())
            if goal == 180:
                if self.angle == 90:
                    self.queue.append(self.leftTurn())
                if self.angle == 270:
                    self.queue.append(self.rightTurn())
                if self.angle == 0:
                    self.queue.append(self.leftTurn())
                    self.queue.append(self.leftTurn())
            if goal == 270:
                if self.angle == 180:
                    self.queue.append(self.leftTurn())
                if self.angle == 0:
                    self.queue.append(self.rightTurn())
                if self.angle == 90:
                    self.queue.append(self.leftTurn())
                    self.queue.append(self.leftTurn())
            self.queue = self.queue + queue_tail

        #print "command:",command
        if command == "move":
            x1 = self.pose.pose.position.x
            y1 = self.pose.pose.position.y

            if item[3] == None:
                item[3] = self.pose.pose.position.x
            if item[4] == None:
                item[4] = self.pose.pose.position.y
            x2 = item[3]
            if self.angle == 0: x2 += item[2]
            elif self.angle == 180: x2 -= item[2]
            y2 = item[4]
            if self.angle == 90: y2 += item[2]
            elif self.angle == 270: y2 -= item[2]

            diff = self.properDiffDist(x1, y1, x2, y2)

            global move_speed
            twist.linear.x = move_speed

            if diff > 0:
                twist.linear.x = 0
                self.queue[0][1] = True

            # just wait...
            if self.hasFrontObstacle():
                twist.linear.x = 0

        if command == "turn":
            sensitivity = 0.9
            dr = self.queue[0][3]
            twist.angular.z = dr*0.9
            curr_angle = self.orientationToAngle(self.pose.pose.orientation.z)
            if curr_angle > 270:
                if self.count >= 0:
                    self.flip_flag = True
                if self.just_started_turning and dr > 0:
                    self.flip_flag = False
                    self.count = -1
            if curr_angle < 45 and self.flip_flag:
                self.count += 1
            desired_angle = self.queue[0][2]
            diff = self.properDiffAngle(curr_angle, desired_angle, dr)
            if self.reverse_flag and dr > 0: self.count += 1
            if dr > 0:
                if diff > sensitivity:
                    self.reverse_flag = True
                    twist.angular.z = -0.1
                elif diff > 0:
                    twist.angular.z = 0
                    self.queue[0][1] = True
                    self.angle = desired_angle % 360
            if dr < 0:
                #print curr_angle, desired_angle, diff, self.count
                if diff < -1*sensitivity:
                    self.reverse_flag = True
                    twist.angular.z = 0.1
                elif diff < 0:
                    twist.angular.z = 0
                    self.queue[0][1] = True
                    self.angle = desired_angle % 360
            self.just_started_turning = False

        if command == "stop":
            self.queue[0][2] -= 1
            if self.queue[0][2] <= 0:
                self.queue[0][1] = True

        if self.queue[0][1]:
            self.queue = self.queue[1:]
            self.count = 0
            self.reverse_flag = False
            self.flip_flag = False
            self.just_started_turning = False
            #print "UNQUEUE"
        return twist


    def hasMap(self):
        return self.map != None and self.map_metadata != None

    def get_heuristic(self, cell):
        #distance between this cell and the ending cell multiplied by 10
        return 10 * (abs(cell.x - self.end.x) + abs(cell.y - self.end.y))

    def get_cell(self, x, y):
        return self.grid[y][x]

    def get_adjacent_cells(self, cell):
        cells = []
        if cell.x < len(self.grid[cell.y])-1:
            cells.append(self.get_cell(cell.x+1, cell.y))
        if cell.y > 0:
            cells.append(self.get_cell(cell.x, cell.y-1))
        if cell.x > 0:
            cells.append(self.get_cell(cell.x-1, cell.y))
        if cell.y < len(self.grid)-1:
            cells.append(self.get_cell(cell.x, cell.y+1))
        return cells

    def update_cell(self, adj, cell):
        adj.g = cell.g + 10
        adj.h = self.get_heuristic(adj)
        adj.parent = cell
        adj.f = adj.h + adj.g

    def createCommandsFromPath(self):
        #this queues up the commands in reverse order
        #   (since we're working from the end cell to the start cell)
        #and then reverses the queue so that it's correctly ordered
        self.queue = []

        cell = self.end
        looping = True
        print "path: cell: %d,%d" % (cell.x, cell.y)
        while looping:
            #queue up a move forward command
            global move_amt
            self.queue.append(self.moveForward(move_amt))

            prev_cell = cell
            cell = cell.parent
            if cell is self.start: looping = False

            print "path: cell: %d,%d" % (cell.x, cell.y)
            # parent is cell one step back in the path
            # so we need to set the cell's angle relative
            # to where the prev_cell was (aka above, below, left or right)
            if prev_cell.x < cell.x:
                #need to move in 180 degrees direction
                self.queue.append(self.turnToward(180))

            if prev_cell.x > cell.x:
                #need to move in 0 degrees direction
                self.queue.append(self.turnToward(0))

            if prev_cell.y < cell.y:
                #need to move in 90 degrees direction
                self.queue.append(self.turnToward(90))
            if prev_cell.y > cell.y:
                #need to move in 270 degrees direction
                self.queue.append(self.turnToward(270))

        self.queue.reverse()
        self.queue.append(self.stop())

        for item in self.queue:
            print item

    def createPathFromGrid(self):
        # add starting cell to open heap queue
        heapq.heappush(self.opened, (self.start.f, self.start))
        while len(self.opened):
            # pop cell from heap queue
            f, cell = heapq.heappop(self.opened)
            # add cell to closed list so we don't process it twice
            self.closed.add(cell)
            # if ending cell, display found path
            if cell is self.end:
                print "THERE IS A PATH!"
                break
            # get adjacent cells for cell
            adj_cells = self.get_adjacent_cells(cell)
            for adj_cell in adj_cells:
                #if not adj_cell.reachable:
                #    print adj_cell.x, adj_cell.y, adj_cell.reachable, adj_cell.occupied
                if adj_cell.reachable and adj_cell not in self.closed:
                    if (adj_cell.f, adj_cell) in self.opened:
                        # if adj cell in open list, check if current path is
                        # better than the one previously found for this adj cell
                        if adj_cell.g > cell.g + 10:
                            self.update_cell(adj_cell, cell)
                    else:
                        self.update_cell(adj_cell, cell)
                        # add adj cell to open list
                        heapq.heappush(self.opened, (adj_cell.f, adj_cell))

    def createGridFromMap(self):
        width = int(self.map_metadata.width)

        global grid_size
        tile_size = grid_size
        i = 0
        self.grid = [[]]
        last_mod = 0
        x = 0
        y = 0
        while i < len(self.map.data):
            occupied = 0
            for yi in range(i, i+(width*(tile_size-1))+1, width):
                #we've gone too far off the bottom edge!
                if yi >= len(self.map.data):
                    break
                for xi in range(yi, yi+tile_size, 1):
                    #if we're at the rightmost edge, just skip
                    #(won't count it as occupied, doesn't matter since it's
                    #   just grey area on the edges)
                    if xi % width < last_mod or xi >= len(self.map.data):
                        break
                    pixel = self.map.data[xi]
                    if pixel >= 50:
                        occupied += 1

            # reachable = occupied < 2 TODO:: tweak this?
            cell = Cell(x, y, occupied < 2, occupied)

            # for j in xrange(13, 30):
            #     should_print = False
            #     if y == j and (x >= 11 and x <= 13):
            #         print x, y, occupied,
            #         should_print = True
            #     if should_print: print

            #push cell into the most recent row
            self.grid[len(self.grid)-1].append(cell)

            # is this a start position?
            global start
            startx = int(round(start['x']))
            starty = int(round(start['y']))
            #print x, y, startx, starty
            if startx == x and starty == y:
                self.start = cell

            #is this an end position?
            global end
            endx = int(round(end['x']))
            endy = int(round(end['y']))
            if endx == x and endy == y:
                self.end = cell

            #increment by tile_size
            i += tile_size
            x += 1

            #fix it if we've bled over onto the next "row"
            mod = i % width
            while mod < last_mod and mod != 0:
                i -= 1
                mod = i % width
            last_mod = mod

            #if we've reached the end of this row (on the new row)
            # we need to jump down to the next row
            #   and actually that needs to be the next one a tile size down
            if mod == 0:
                i += width * (tile_size-1)
                self.grid.append([])
                x = 0
                y += 1

        #get rid of the last empty row
        self.grid = self.grid[0:len(self.grid)-1]
        #flip it from normal y axis to graphics y axis
        #self.grid.reverse()


robot = Robot()
robot.angle = start['theta']

def processOdometry(odoMsg):
    global robot
    robot.setPose(odoMsg.pose)
    #linear = odoMsg.pose.pose.position
    #angular = odoMsg.pose.pose.orientation
    #print "linear x=%0.2f, y=%0.2f, z=%0.2f" %(linear.x, linear.y, linear.z)
    #print "angulr x=%0.2f, y=%0.2f, z=%0.2f, w=%0.2f" %(angular.x, angular.y, angular.z, angular.w)

def processLaser(laserMsg):
    global robot
    robot.laser = laserMsg

def processSonar(sonMsg, sonLock):
    #sonMsg.ranges (didn't work in simulation for hw2??)
    pass

def processMapMetaData(map_metadataMsg):
    global robot
    if robot.map_metadata == None:
        robot.map_metadata = map_metadataMsg

def processMap(mapMsg):
    global robot
    if robot.map == None:
        robot.map = mapMsg

###########################MAIN###################################
if __name__ == "__main__":
    #TODO:: somehow to get these from command lines
    # or can just hard code them i guess...
    # see top of code
    rospy.init_node('pioneerController')

    ns = "pioneer"
    sonSub = rospy.Subscriber("/sonar", SonarArray, processSonar)
    #TODO:: DEBUG:: /pose instead of /odom ???
    odoSub = rospy.Subscriber("/odom", Odometry, processOdometry)
    #TODO:: DEBUG:: /scan instead of /base_scan_1
    laserSub = rospy.Subscriber("/base_scan_1", LaserScan, processLaser)

    map_metadataSub = rospy.Subscriber("/map_metadata", MapMetaData, processMapMetaData)
    mapSub = rospy.Subscriber("/map", OccupancyGrid, processMap)

    #publishers
    velPub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    motPub = rospy.Publisher("/cmd_motor_state", MotorState, queue_size=10)

    rate = rospy.Rate(10)
    count = 0

    motor = MotorState()
    motor.state = 1

    #wait for us to have map data...
    print "waiting for map... and waiting for pose position"
    while not rospy.is_shutdown() and (not robot.hasMap() or robot.initial_pose == None):
        rate.sleep()

    #print robot.map.data


    print "starting motors..."
    for i in xrange(30):
        motPub.publish(motor)
        rate.sleep()

    print "hey! i've got the map! let's path plan..."
    robot.createGridFromMap()
    robot.createPathFromGrid()
    robot.createCommandsFromPath()

    print "starting navigation (from start point???)"
    while not rospy.is_shutdown() and not robot.isQueueEmpty():
        vel = Twist()

        #   TODO:: how does relocalizing ourself work with the grid??
        vel = robot.navigate(vel)
        velPub.publish(vel)
        rate.sleep()

    print "i've (probably) reached the goal!"
    print "stopping robot"

    vel = Twist()
    motor.state = 0
    for i in xrange(30):
        velPub.publish(vel)
        motPub.publish(motor)
