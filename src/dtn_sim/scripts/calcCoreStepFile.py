#!/usr/bin/env python3

"""Generate Step file for use in my custom player"""

import argparse
from collections import defaultdict
from math import sqrt, atan2, sin, cos
from enum import Enum

SIM_TIME = 4330  # Bag runtime in s
SIM_STEP_SIZE = 0.2
DRONE_WAIT_TIME = 20
HEADER = f"""# original robot step size {SIM_STEP_SIZE}
"""


class DroneState(Enum):
    UNKNOWN = 0
    TO_BASE = 1
    TO_ROBOT = 2
    AT_BASE = 3
    AT_ROBOT = 4


class PatternType(Enum):
    WP = 0
    CIRCLE = 1
    OPTIMIZED = 2


oldPositions = {}

flightPatterns = {
    "parallel": {
        "type": PatternType.WP,
        "drone1": [
            (-50, 100),
            (150, 100),
            (150, 75),
            (-50, 75),
            (-50, 50),
            (150, 50),
            (150, 25),
            (-50, 25),
            (-50, 0),
            (150, 0),
            (150, -25),
            (-50, -25),
            (-50, -50),
            (150, -50),
            (150, -75),
            (-50, -75),
            (-50, -100),
            (150, -100),
        ],
    },
    "parallelLarge": {
        "type": PatternType.WP,
        "drone1": [
            (-50, 100),
            (150, 100),
            (150, 50),
            (-50, 50),
            (-50, 0),
            (150, 0),
            (150, -50),
            (-50, -50),
            (-50, -100),
            (150, -100),
        ],
    },
    "crosshatch": {
        "type": PatternType.WP,
        "drone1": [
            (-50, 100),
            (150, 100),
            (150, 75),
            (-50, 75),
            (-50, 50),
            (150, 50),
            (150, 25),
            (-50, 25),
            (-50, 0),
            (150, 0),
            (150, -25),
            (-50, -25),
            (-50, -50),
            (150, -50),
            (150, -75),
            (-50, -75),
            (-50, -100),
            (150, -100),
            # second half
            (150, 100),
            (125, 100),
            (125, -100),
            (100, -100),
            (100, 100),
            (75, 100),
            (75, -100),
            (50, -100),
            (50, 100),
            (25, 100),
            (25, -100),
            (0, -100),
            (0, 100),
            (-25, 100),
            (-25, -100),
            (-50, -100),
        ],
    },
    "crosshatchLarge": {
        "type": PatternType.WP,
        "drone1": [
            (-50, 100),
            (150, 100),
            (150, 50),
            (-50, 50),
            (-50, 0),
            (150, 0),
            (150, -50),
            (-50, -50),
            (-50, -100),
            (150, -100),
            # second half
            (150, 100),
            (100, 100),
            (100, -100),
            (50, -100),
            (50, 100),
            (0, 100),
            (0, -100),
            (-50, -100),
        ],
    },
    "circle": {"type": PatternType.CIRCLE, "drone1": ((50, 0), 100, (-50, 0))},
    "circleDouble": {
        "type": PatternType.CIRCLE,
        "drone1": ((50, 0), 100, (-50, 0)),
        "drone2": ((50, 0), 100, (150, 0)),
    },
    "circleSmall": {
        "type": PatternType.CIRCLE,
        "drone1": ((50, 0), 100, (-50, 0)),
        "drone2": ((50, 0), 50, (0, 0)),
    },
    "follow": {"type": PatternType.OPTIMIZED, "drone1": []},
    "cross": {
        "type": PatternType.WP,
        "drone1": [
            (-25, 75),
            (125, 75),
            (-25, -75),
            (125, -75),
            # second half
            (-25, 75),
            (-25, -75),
            (125, 75),
            (125, -75),
        ],
    },
}


def dist(p1, p2):
    x2 = (p1[0] - p2[0]) ** 2
    y2 = (p1[1] - p2[1]) ** 2
    return sqrt(x2 + y2)


def isInRange(p1, p2):
    return dist(p1, p2) < (args.range - 10)


def pointOnCircle(M, r, S, dist):
    # Calculate the distance between the start point and the center of the circle
    d = sqrt((S[0] - M[0]) ** 2 + (S[1] - M[1]) ** 2)
    # Check if the start point is on the circle
    if d != r:
        # Calculate the closest point on the circle to the start point
        sX = M[0] + r * (S[0] - M[0]) / d
        sY = M[1] + r * (S[1] - M[1]) / d
        S = (sX, sY)
    # Calculate the angle between the start point and the center of the circle
    angle = atan2(S[1] - M[1], S[0] - M[0])
    # Calculate the new angle based on the distance
    newAngle = angle + dist / r
    # Calculate the coordinates of the new point
    dx = M[0] + r * cos(newAngle)
    dy = M[1] + r * sin(newAngle)
    return (dx, dy)


def moveDrone(actualPos, desiredPos):
    distance = dist(actualPos, desiredPos)
    if 0 == distance:
        return actualPos
    delta = min(distance, args.speed * SIM_STEP_SIZE)
    x = actualPos[0] + (delta * (desiredPos[0] - actualPos[0])) / distance
    y = actualPos[1] + (delta * (desiredPos[1] - actualPos[1])) / distance
    return (x, y)


def getDroneOptimized(robot, drone):
    rx, ry = robot

    distance = dist(args.base, robot)
    if isInRange(robot, args.base):
        # No Airbridge needed
        return drone
    elif distance < (2 * args.range) - 10:
        # Drone in the middle should bridge the gap
        getDroneOptimized.state = DroneState.UNKNOWN
        x = (rx + args.base[0]) / 2
        y = (ry + args.base[1]) / 2
        return moveDrone(drone, (x, y))
    else:
        # Drone has to fly to bridge the gap
        if (
            DroneState.UNKNOWN == getDroneOptimized.state
            or DroneState.TO_BASE == getDroneOptimized.state
        ):
            if isInRange(drone, args.base):
                getDroneOptimized.state = DroneState.AT_BASE
                return drone
            else:
                getDroneOptimized.state = DroneState.TO_BASE
                return moveDrone(drone, args.base)
        elif DroneState.TO_ROBOT == getDroneOptimized.state:
            if isInRange(drone, robot):
                getDroneOptimized.state = DroneState.AT_ROBOT
                return drone
            else:
                return moveDrone(drone, robot)
        elif DroneState.AT_BASE == getDroneOptimized.state:
            if getDroneOptimized.counter < DRONE_WAIT_TIME / SIM_STEP_SIZE:
                getDroneOptimized.counter += 1
                return drone
            else:
                getDroneOptimized.counter = 0
                getDroneOptimized.state = DroneState.TO_ROBOT
                return moveDrone(drone, robot)
        elif DroneState.AT_ROBOT == getDroneOptimized.state:
            if getDroneOptimized.counter < DRONE_WAIT_TIME / SIM_STEP_SIZE:
                getDroneOptimized.counter += 1
                return moveDrone(drone, robot)
            else:
                getDroneOptimized.counter = 0
                getDroneOptimized.state = DroneState.TO_BASE
                return moveDrone(drone, args.base)


def getWp(pattern):
    global oldPositions

    # init
    result = ""
    nodes = [node for node in oldPositions.keys() if node != "robot"]

    for node in nodes:
        if getWp.cnt[node] >= len(pattern[node]):
            getWp.cnt[node] = 0

        currentWP = pattern[node][getWp.cnt[node]]
        newNode = moveDrone(oldPositions[node], currentWP)
        if dist(currentWP, newNode) < 0.01:
            getWp.cnt[node] += 1
        if dist(oldPositions[node], newNode) > 0.01:
            result += f"{node} {round(newNode[0], 2)} {round(newNode[1], 2)}\n"
        oldPositions[node] = newNode

    return result


def getCircle(pattern):
    global oldPositions

    # init
    result = ""
    nodes = [node for node in oldPositions.keys() if node != "robot"]

    for node in nodes:
        M, r, S = pattern[node]
        if getCircle.startReached[node]:
            # run on circle
            currentWP = pointOnCircle(
                M, r, oldPositions[node], args.speed * SIM_STEP_SIZE
            )
            newNode = moveDrone(oldPositions[node], currentWP)
        else:
            # go to start
            newNode = moveDrone(oldPositions[node], S)
            if dist(S, newNode) <= 0.01:
                newNode = S
                getCircle.startReached[node] = True

        if dist(oldPositions[node], newNode) > 0.01:
            result += f"{node} {round(newNode[0], 2)} {round(newNode[1], 2)}\n"
        oldPositions[node] = newNode

    return result


def getPatternPosition(pattern):
    global oldPositions
    if pattern["type"] == PatternType.OPTIMIZED:
        result = ""

        newDrone = getDroneOptimized(oldPositions["robot"], oldPositions["drone1"])
        if dist(oldPositions["drone1"], newDrone) > 0.01:
            result = f"drone1 {round(newDrone[0],2)} {round(newDrone[1],2)}\n"
            oldPositions["drone1"] = newDrone
        return result

    if pattern["type"] == PatternType.WP:
        return getWp(pattern)

    if pattern["type"] == PatternType.CIRCLE:
        return getCircle(pattern)


def reset():
    global oldPositions
    oldPositions.clear()
    oldPositions["robot"] = args.robot
    getCircle.startReached = defaultdict(lambda: False)
    getDroneOptimized.state = DroneState.UNKNOWN
    getDroneOptimized.counter = 0
    getWp.cnt = defaultdict(lambda: 0)


def initalPositions(pattern):
    global oldPositions
    for node in pattern.keys():
        if node != "type":
            oldPositions[node] = (args.base[0] + 3, args.base[1] + 3)


def buildStepFiles(robotSteps):
    global oldPositions
    for name, pattern in flightPatterns.items():
        reset()
        initalPositions(pattern)

        # Add 10 seconds to prevent loop before sim ends
        countdown = int((SIM_TIME + 10) / SIM_STEP_SIZE)

        with open(f"{args.output}/{name}.step", "w") as file:
            file.write(HEADER)

            # write inital position
            file.write(f"%0\n")
            file.write(f"robot {args.robot[0]} {args.robot[1]}\n")
            for node in pattern.keys():
                if node != "type":
                    file.write(
                        f"{node} {oldPositions[node][0]} {oldPositions[node][1]}\n"
                    )

            for i in range(1, countdown):
                # get pattern positions
                patternPos = getPatternPosition(pattern)

                # get robot position
                if i < len(robotSteps):
                    x, y = [float(value) for value in robotSteps[i].split()]
                    newRobot = (x + args.robot[0], y + args.robot[1])
                    if dist(oldPositions["robot"], newRobot) > 0.01:
                        oldPositions["robot"] = newRobot
                        robotPos = (
                            f"robot {round(newRobot[0], 2)} {round(newRobot[1], 2)}\n"
                        )
                    else:
                        robotPos = ""

                # write stuff to file
                if patternPos or robotPos:
                    # write time stamp
                    file.write(f"%{int(i * SIM_STEP_SIZE * 1000)}\n")
                if robotPos:
                    file.write(robotPos)
                if patternPos:
                    file.write(patternPos)


def main():
    global args
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument("input", help="input file path for robot movement")
    parser.add_argument("output", help="output file path for step files")
    parser.add_argument(
        "--base",
        help="position of the base station X Y",
        nargs=2,
        type=int,
        default=[0, 0],
    )
    parser.add_argument(
        "--robot",
        help="offset of the roboter X Y",
        nargs=2,
        type=int,
        default=[0, 0],
    )
    parser.add_argument(
        "--start",
        help="start at second t",
        type=int,
        default=0,
    )
    parser.add_argument(
        "--speed",
        help="speed of the drone in m/s",
        type=int,
        default=10,
    )
    parser.add_argument(
        "--range",
        help="range of the wifi in m",
        type=int,
        default=50,
    )

    args = parser.parse_args()

    with open(args.input, "r") as file:
        lines = file.readlines()[int(args.start / SIM_STEP_SIZE) : :]

    buildStepFiles(lines)


if __name__ == "__main__":
    main()
