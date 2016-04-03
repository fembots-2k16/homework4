#!/usr/bin/env python

#definitely using this as a big reference
#http://www.laurentluce.com/posts/solving-mazes-using-python-simple-recursivity-and-a-search/

import rospy
import heapq
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
start = {'x': 12, 'y': 26, 'theta': 0.00}
end = {'x': 20, 'y': 30}
####################################################
##################################################3

class Cell(object):
    def __init__(self, x, y, reachable):
        self.x = x
        self.y = y
        self.reachable = reachable
        self.parent = None
        self.g = 0
        self.h = 0
        self.f = 0

class Robot(object):
    def __init__(self):
        self.map = None
        self.map_metadata = None

        self.grid = None
        self.start = None
        self.end = None

        #more stuff for A* search
        self.opened = []
        heapq.heapify(self.opened)
        self.closed = set()

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

    #TODO:: do we need this?
    def display_path(self):
        cell = self.end
        while cell.parent is not self.start:
            cell = cell.parent
            print "path: cell: %d,%d" % (cell.x, cell.y)

    def update_cell(self, adj, cell):
        adj.g = cell.g + 10
        adj.h = self.get_heuristic(adj)
        adj.parent = cell
        adj.f = adj.h + adj.g

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
                self.display_path()
                break
            # get adjacent cells for cell
            adj_cells = self.get_adjacent_cells(cell)
            for adj_cell in adj_cells:
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

        tile_size = 5
        i = 0
        self.grid = [[]]
        last_mod = 0
        x = 0
        y = 0
        while i < len(self.map.data):
            occupied = 0
            for yi in range(i, i+(width*(tile_size-1))+1, width):
                #we've gone too far off the bottom edge!
                if yi < len(self.map.data):
                    break
                for xi in range(yi, yi+tile_size, 1):
                    #if we're at the rightmost edge, just skip
                    #(won't count it as occupied, doesn't matter since it's
                    #   just grey area on the edges)
                    if xi % width < last_mod or xi < len(self.map.data):
                        break
                    pixel = self.map.data[xi]
                    if pixel >= 0.5:
                        occupied += 1

            # reachable = occupied < 2 TODO:: tweak this?
            cell = Cell(x, y, occupied < 2)

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


robot = Robot()

def processOdometry(odoMsg):
    linear = odoMsg.pose.pose.position
    angular = odoMsg.pose.pose.orientation
    #print "linear x=%0.2f, y=%0.2f, z=%0.2f" %(linear.x, linear.y, linear.z)
    #print "angulr x=%0.2f, y=%0.2f, z=%0.2f, w=%0.2f" %(angular.x, angular.y, angular.z, angular.w)

def processLaser(laserMsg):
    #laserMsg.ranges
    pass

def processSonar(sonMsg, sonLock):
    #sonMsg.ranges (didn't work in simulation for hw2??)
    pass

def processMapMetaData(map_metadataMsg):
    global robot
    robot.map_metadata = map_metadataMsg

def processMap(mapMsg):
    global robot
    robot.map = mapMsg

###########################MAIN###################################
if __name__ == "__main__":
    #TODO:: somehow to get these from command lines
    # or can just hard code them i guess...
    # see top of code
    rospy.init_node('pioneerController')

    ns = "pioneer"
    sonSub = rospy.Subscriber("/sonar", SonarArray, processSonar)
    odoSub = rospy.Subscriber("/pose", Odometry, processOdometry)
    laserSub = rospy.Subscriber("/scan", LaserScan, processLaser)

    map_metadataSub = rospy.Subscriber("/map_metadata", MapMetaData, processMapMetaData)
    mapSub = rospy.Subscriber("/map", OccupancyGrid, processMap)

    #publishers
    velPub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    motPub = rospy.Publisher("/cmd_motor_state", MotorState, queue_size=10)

    rate = rospy.Rate(1)
    count = 0

    motor = MotorState()
    motor.state = 1

    #wait for us to have map data...
    print "waiting for map..."
    while not rospy.is_shutdown() and not robot.hasMap():
        rate.sleep()

    print "hey! i've got the map! let's path plan..."
    robot.createGridFromMap()
    robot.createPathFromGrid()

    #TODO:: debug delete
    print robot.map_metadata

    print "starting navigation (from start point???)"
    while not rospy.is_shutdown():
        vel = Twist()

        motPub.publish(motor)
        velPub.publish(vel)
        rate.sleep()

    print "i've (probably) reached the goal!"
