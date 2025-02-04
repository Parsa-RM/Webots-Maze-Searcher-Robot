from controller import Robot

# create the Robot instance .
robot = Robot ()

# get the time step of the current world .
timestep = int( robot.getBasicTimeStep ())
MAX_SPEED = 6.28

def delay(ms):
    initTime = robot.getTime()      # Store starting time (in seconds)
    while robot.step(timestep) != -1:
        if (robot.getTime() - initTime) * 1000.0 > ms: # If time elapsed (converted into ms) is greater than value passed in
            break


ps = []
psNames = [
    'ps0', 'ps1', 'ps2', 'ps3',
    'ps4', 'ps5', 'ps6', 'ps7'
]

for i in range(8):
    ps.append(robot.getDevice(psNames[i]))
    ps[i].enable(timestep)

leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0 * MAX_SPEED)
rightMotor.setVelocity(0 * MAX_SPEED)

Tile_block = 120

class Node:
    def __init__(self, parent = None) :
        flag_f, wall_distance_f = measureTheWallDistance()
        turnBack(wall_distance_f)
        rotate(90)
        flag_r, wall_distance_r = measureTheWallDistance()
        turnBack(wall_distance_r)
        rotate(-90)
        rotate(-90)
        flag_l, wall_distance_l = measureTheWallDistance()
        turnBack(wall_distance_l)
        rotate(90)
        self.Walls = {"front":(flag_f, wall_distance_f),"right": (flag_r, wall_distance_r), "left":(flag_l,wall_distance_l)}
        self.parent = parent
        self.path = []
        self.children = []

    def grow(self):
        walls = self.Walls
        new_nodes = []
        for key, value in walls.items():
            if not walls[key][0]: # if there is no wall in a position
                if key == "left":
                    rotate(-90)
                elif key == "right":
                    rotate(90)
                # lets goStraight to the new node's position
                goStraight(Tile_block)
                # create node by its surrounding walls
                new_node = Node(self) # setting this node as its parent node
                # append the history of it to all the new children
                for step in self.path:
                    new_node.path.append(step)
                
                # adding the new node relative path from its parent(node) to its path
                new_node.path.append(key) 
                # append the new node to the new nodes list:
                new_nodes.append(new_node)           
                turnBack(Tile_block)
                if key == "left":
                    rotate(90)
                elif key == "right":
                    rotate(-90)
            else:
                new_nodes.append(None)

        self.children = new_nodes
    
    
    def goToNode(self, destination_node):
        current_path = self.path
        destination_path = destination_node.path
        common_steps_index = min(len(destination_path), len(current_path)) - 1
        for i in range(min(len(destination_path), len(current_path))):
            if current_path[i] != destination_path[i]:
                common_steps_index = i - 1
                break
        if common_steps_index == -1:
            # one of the pathes were empty meaning root path
            if not current_path:
                # if the current path is root simply go straight to the destination path step by step
                moveForwardStepByStep(destination_path)
            else:
                # destination is the root simply return all the steps from the current absolute path
                moveBackwardsStepByStep(current_path)
            return True
        temp_list = []
        for i in range(len(current_path)):
            if i > common_steps_index:
                temp_list.append(current_path[i])
        moveBackwardsStepByStep(temp_list)

        temp_list = []
        for i in range(len(destination_path)):
            if i > common_steps_index:
                temp_list.append(destination_path[i])
        moveForwardStepByStep(temp_list)
    
    
    def get_children(self):
        return self.children
    
    def console(self):
        rst = "Path from root: " + str(self.path) + "\n wall detection results: " + str(self.Walls)
        print(rst)
    

# not to be modified 
##################################################
def read_ps_values():
    rst = []
    for i in range(8):
        rst.append(ps[i].getValue())
    return rst

##################################################
def detect_wall():
    THRESHOLD = 90
    ps_values = read_ps_values()
    is_wall = ps_values[7] > THRESHOLD or ps_values[0] > THRESHOLD
    return is_wall
##################################################
def measureTheWallDistance():
    distance_measured = 0
    is_any_wall = True
    HALF_TILE_WIDTH = 100 # width of tiles
    while(not detect_wall()):
        if distance_measured > HALF_TILE_WIDTH:
            is_any_wall = False
            break
        leftMotor.setVelocity(0.5 * MAX_SPEED)
        rightMotor.setVelocity(0.5 * MAX_SPEED)
        delay(1)
        distance_measured += 1
    leftMotor.setVelocity(0 * MAX_SPEED)
    rightMotor.setVelocity(0 * MAX_SPEED)
    return is_any_wall, distance_measured
##################################################
# to rotate the robot dg degrees + for right and - for negative
DelayPerDegree = (750)/90
def rotate(dg) :
    if dg < 0:
        # turn left
        lVelocity = -0.5
        rVelocity = 0.5
    else:
        # turn right
        lVelocity = 0.5
        rVelocity = -0.5
    leftMotor.setVelocity(lVelocity * MAX_SPEED)
    rightMotor.setVelocity(rVelocity * MAX_SPEED)
    delay(DelayPerDegree * abs(dg))
    leftMotor.setVelocity(0 * MAX_SPEED)
    rightMotor.setVelocity(0 * MAX_SPEED)
##################################################
def turnBack(distance):
    leftMotor.setVelocity(-0.5 * MAX_SPEED)
    rightMotor.setVelocity(-0.5 * MAX_SPEED)
    for i in range(distance):
        delay(1)
    
    leftMotor.setVelocity(0 * MAX_SPEED)
    rightMotor.setVelocity(0 * MAX_SPEED)
##################################################
def goStraight(distance):
    leftMotor.setVelocity(0.5 * MAX_SPEED)
    rightMotor.setVelocity(0.5 * MAX_SPEED)
    for i in range(distance):
        delay(1)
    
    leftMotor.setVelocity(0 * MAX_SPEED)
    rightMotor.setVelocity(0 * MAX_SPEED)
#####################################################
def moveForwardStepByStep(steps):
    for step in steps:
        if step == "left":
            rotate(-90)
        elif step == "right":
            rotate(90)
        goStraight(Tile_block)

def moveBackwardsStepByStep(steps):
    steps.reverse()
    for step in steps:
            turnBack(Tile_block)
            if step == "left":
                rotate(90)
            elif step == "right":
                rotate(-90)
#####################################################


def dfs(call_back_function):
    maze = list()
    stack = list()
    # detecting walls of the current location
    stack.insert(0,Node())
    while(stack):
        node = stack.pop(0)
        maze.append(node)
        call_back_function(node)
        for child in node.get_children():
            if child != None:
                stack.insert(0, child)
        if len(stack) > 0:
            # checks if there exist a node which we have not already covered
            # on the other hand checks if this is the last iteration 
            # because in the last iteration no next node exists at all!
            next_node = stack[0]
            node.goToNode(next_node)

    return maze

def bfs(call_back_function):
    maze = list()
    queue = list()
    # detecting walls of the current location
    queue.append(Node())
    while(queue):
        node = queue.pop(0)
        call_back_function(node)
        maze.append(node)
        for child in node.get_children():
            if child != None:
                queue.append(child)
        if len(queue) > 0:
            # checks if there exist a node which we have not already covered
            # on the other hand checks if this is the last iteration 
            # because in the last iteration no next node exists at all!
            next_node = queue[0]
            node.goToNode(next_node)
    
    return maze






# feedback loop: step simulation until receiving an exit event
def map(node):
    node.grow()
flag = True
while robot.step(timestep) != -1:
    if flag:
        rotate(90)
        Maze = dfs(map)
        root = Maze[0]
        last_node = Maze[-1]
        last_node.goToNode(root)
        # uncomment the lines below to map the maze based on BFS
        #################
        # Maze = bfs(map)
        # root = Maze[0]
        # last_node = Maze[-1]
        # last_node.goToNode(root)
        #################
        flag = False
        break
print("---------------------------------------- DFS Results ----------------------------------------")
for node in Maze:
    node.console()