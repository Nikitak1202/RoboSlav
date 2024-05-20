from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import math
import random
import traceback

# Connect to the CoppeliaSim Remote API
client = RemoteAPIClient()
sim = client.require('sim')
sim.loadScene('/Users/mavri/Documents/Механика/АОвП/first_try.ttt')

# Run a simulation in stepping mode
sim.setStepping(True)
sim.startSimulation()

global course, turning, state, diff, targetHeading
global vehicleHandle, cat1, cat2
global sensor_front, sound, sound2, sensor_left, sensor_right
global leftDynamicMotors, rightDynamicMotors
global leftVelocity, rightVelocity, acceleration, leftTargetVelocity, rightTargetVelocity
global dist_fr, dist_fr2, count_sides, count_sides_turn
global targetHeading, state, turning, course, wheelRadius, interTrackDistance, dynamicSimulation


# Global variables
vehicleHandle = None
cat1 = None
cat2 = None
sensor_front = None
sound = None
sound2 = None
sensor_left = None
sensor_right = None
diff = None

leftDynamicMotors = [0, 0, 0, 0]
rightDynamicMotors = [0, 0, 0, 0]

leftVelocity = 0.0
rightVelocity = 0.0
acceleration = 0.0
leftTargetVelocity = 0.0
rightTargetVelocity = 0.0

dist_fr = 100.0
dist_fr2 = 100.0
count_sides = 0
count_sides_turn = 0

targetHeading = math.radians(0)

state = 'done'
turning = 'turn'
course = 'nice'

wheelRadius = 0.0704
interTrackDistance = 0.24
dynamicSimulation = True


# Initialize the simulation
def sysCall_init():
    vehicleHandle = sim.getObject('./caterpillar')
    cat1 = sim.getObject('./caterpillar/cat1')
    cat2 = sim.getObject('./caterpillar/cat2')

    for i in range(4):
        leftDynamicMotors[i] = sim.getObject(f'./dynamicLeftJoint{i + 1}')
        rightDynamicMotors[i] = sim.getObject(f'./dynamicRightJoint{i + 1}')

    sensor_front = sim.getObject('./sensorFR')

    sound = sim.getObject('./sound')
    sound2 = sim.getObject('./sound2')

    sensor_left = sim.getObject('./sensorL')
    sensor_right = sim.getObject('./sensorR')

    p = sim.getModelProperty(vehicleHandle) | sim.modelproperty_not_dynamic
    if dynamicSimulation:
        sim.setModelProperty(vehicleHandle, p - sim.modelproperty_not_dynamic)
    else:
        sim.setModelProperty(vehicleHandle, p)


# Function to calculate the difference between two headings
def headingDiff(a, b):
    res = (a - b) % (2 * math.pi)
    if res > math.pi:
        return 2 * math.pi - res
    else:
        return res


# Function to turn the robot towards the target heading
def turnToHeading():
    currentHeading = sim.getObjectOrientation(vehicleHandle, sim.handle_world)[2]
    diff = headingDiff(currentHeading, targetHeading)

    if math.degrees(targetHeading) < 0:
        leftTargetVelocity = -0.05
        rightTargetVelocity = 0.05
    else:
        leftTargetVelocity = 0.05
        rightTargetVelocity = -0.05

    if math.degrees(abs(diff)) > 2:
        acceleration = diff * (-0.01)
    else:
        state = 'done'
        leftVelocity = 0.03
        rightVelocity = 0.03
        leftTargetVelocity = 0.03
        rightTargetVelocity = 0.03
        if abs(targetHeading) < math.radians(5):
            targetHeading = math.radians(0)


# Main logic function
def go():
    if (dist_fr != 0.0 and dist_fr <= 0.3) and (dist_fr2 != 0.0 and dist_fr2 <= 0.3) or state == 'progress' or course != 'nice':
        if course != 'turning':
            side = random.randint(1, 2)
        if side == 2:
            turn_left()
        else:
            turn_right()
    else:
        targetHeading = math.radians(0)
        turning = 'turn'
        count_sides = 0


def turn_left():
    if targetHeading == math.radians(0) and state == 'done' and turning == 'turn':
        targetHeading = math.radians(90)
        state = 'progress'
        turning = 'turn2'
        course = 'turning'
        count_sides_turn += 1
    if targetHeading == math.radians(90) and state == 'done' and turning == 'turn2':
        course = 'turning'
        if count_sides == 3:
            turning = 'turn2_1'
            count_sides = 0
        count_sides += 1
    if targetHeading == math.radians(90) and state == 'done' and turning == 'turn2_1':
        course = 'turning'
        if count_sides == 3:
            targetHeading = math.radians(-1)
            state = 'progress'
            turning = 'turn3'
            count_sides = 0
        count_sides += 1
    if targetHeading == math.radians(0) and state == 'done' and turning == 'turn3':
        course = 'turning'
        if count_sides == 3:
            turning = 'turn3_1'
            count_sides = 0
        count_sides += 1
    if targetHeading == math.radians(0) and state == 'done' and turning == 'turn3_1':
        course = 'turning'
        if count_sides == 3:
            targetHeading = math.radians(-90)
            state = 'progress'
            turning = 'turn4'
            count_sides = 0
            count_sides_turn = 0
        count_sides += 1
    if targetHeading == math.radians(-90) and state == 'done' and turning == 'turn4':
        targetHeading = math.radians(4)
        state = 'progress'
        turning = 'done'
        course = 'nice'
        count_sides = 0


def turn_right():
    global targetHeading, state, turning, course, count_sides
    if targetHeading == math.radians(0) and state == 'done' and turning == 'turn':
        targetHeading = math.radians(-90)


# Actuation function
def sysCall_actuation():
    global leftVelocity, rightVelocity, leftTargetVelocity, rightTargetVelocity, acceleration, targetHeading
    dt = sim.getSimulationTimeStep()

    turnToHeading()
    go()

    if leftTargetVelocity > leftVelocity:
        leftVelocity += acceleration * dt
        if leftVelocity > leftTargetVelocity:
            leftVelocity = leftTargetVelocity
    else:
        leftVelocity -= acceleration * dt
        if leftVelocity < leftTargetVelocity:
            leftVelocity = leftTargetVelocity

    if rightTargetVelocity > rightVelocity:
        rightVelocity += acceleration * dt
        if rightVelocity > rightTargetVelocity:
            rightVelocity = rightTargetVelocity
    else:
        rightVelocity -= acceleration * dt
        if rightVelocity < rightTargetVelocity:
            rightVelocity = rightTargetVelocity

    sim.writeCustomTableData(cat1, '__ctrl__', {'vel': rightVelocity})
    sim.writeCustomTableData(cat2, '__ctrl__', {'vel': leftVelocity})

    if dynamicSimulation:
        for i in range(4):
            sim.setJointTargetVelocity(leftDynamicMotors[i], -leftVelocity / wheelRadius)
            sim.setJointTargetVelocity(rightDynamicMotors[i], -rightVelocity / wheelRadius)

        d = interTrackDistance
        linVar = sim.getSimulationTimeStep() * (leftVelocity + rightVelocity) / 2.0
        rotVar = sim.getSimulationTimeStep() * math.atan((rightVelocity - leftVelocity) / d)
        position = sim.getObjectPosition(vehicleHandle, sim.handle_parent)
        orientation = sim.getObjectOrientation(vehicleHandle, sim.handle_parent)
        xDir = [math.cos(orientation[2]), math.sin(orientation[2]), 0.0]
        position[0] += xDir[0] * linVar
        position[1] += xDir[1] * linVar
        orientation[2] += rotVar
        sim.setObjectPosition(vehicleHandle, sim.handle_parent, position)
        sim.setObjectOrientation(vehicleHandle, sim.handle_parent, orientation)


# Sensing function
def sysCall_sensing():
    global dist_fr, dist_fr2
    res_fr, dist_fr, pt_fr, x, y = sim.readProximitySensor(sound)
    res_fr2, dist_fr2, pt_fr2, x, y = sim.readProximitySensor(sound2)
    res_l, dist_l, pt_l, x, y = sim.handleProximitySensor(sensor_left)
    res_r, dist_r, pt_r, x, y = sim.handleProximitySensor(sensor_right)


try:
    sysCall_init()
    while True:
        sysCall_actuation()
        sysCall_sensing()
        sim.step()
except Exception as err:
    traceback.format_exc(traceback.format_exc(Exception, err))
    sim.stopSimulation()