#!/usr/bin/env python3

"""Generate Step file for use in core-automator"""

import argparse
from math import sqrt
from enum import Enum

SIM_TIME = 15 * 60
SIM_STEP_SIZE = 0.2
DRONE_WAIT_TIME = 20
HEADER = f"""%delay {SIM_STEP_SIZE}

# initial positions
"""


class DroneState(Enum):
    UNKNOWN = 0
    TO_BASE = 1
    TO_ROBOT = 2
    AT_BASE = 3
    AT_ROBOT = 4


state = DroneState.UNKNOWN


def dist(p1, p2):
    x2 = (p1[0] - p2[0]) ** 2
    y2 = (p1[1] - p2[1]) ** 2
    return sqrt(x2 + y2)


def isInRange(p1, p2):
    return dist(p1, p2) < (args.range - 10)


def moveDrone(actualPos, desiredPos):
    distance = dist(actualPos, desiredPos)
    if 0 == distance:
        return actualPos
    delta = min(distance, args.speed * SIM_STEP_SIZE)
    x = actualPos[0] + (delta * (desiredPos[0] - actualPos[0])) / distance
    y = actualPos[1] + (delta * (desiredPos[1] - actualPos[1])) / distance
    return (x, y)


def getDronePosition(robot, drone):
    global state

    if not hasattr(getDronePosition, "counter"):
        getDronePosition.counter = 0

    rx, ry = robot

    distance = dist(args.base, robot)
    if isInRange(robot, args.base):
        # No Airbridge needed
        return drone
    elif distance < (2 * args.range) - 10:
        # Drone in the middle should bridge the gap
        state = DroneState.UNKNOWN
        x = (rx + args.base[0]) / 2
        y = (ry + args.base[1]) / 2
        return moveDrone(drone, (x, y))
    else:
        # Drone has to fly to bridge the gap
        if DroneState.UNKNOWN == state or DroneState.TO_BASE == state:
            if isInRange(drone, args.base):
                state = DroneState.AT_BASE
                return drone
            else:
                state = DroneState.TO_BASE
                return moveDrone(drone, args.base)
        elif DroneState.TO_ROBOT == state:
            if isInRange(drone, robot):
                state = DroneState.AT_ROBOT
                return drone
            else:
                return moveDrone(drone, robot)
        elif DroneState.AT_BASE == state:
            if getDronePosition.counter < DRONE_WAIT_TIME / SIM_STEP_SIZE:
                getDronePosition.counter += 1
                return drone
            else:
                getDronePosition.counter = 0
                state = DroneState.TO_ROBOT
                return moveDrone(drone, robot)
        elif DroneState.AT_ROBOT == state:
            if getDronePosition.counter < DRONE_WAIT_TIME / SIM_STEP_SIZE:
                getDronePosition.counter += 1
                return moveDrone(drone, robot)
            else:
                getDronePosition.counter = 0
                state = DroneState.TO_BASE
                return moveDrone(drone, args.base)


def buildStepFile(input):
    lastDrone = (args.base[0] + 3, args.base[1] + 3)
    lastRobot = args.robot
    # Add 10 seconds to prevent loop before sim ends
    countdown = (SIM_TIME + 10) / SIM_STEP_SIZE
    with open(args.output, "w") as file:
        file.write(HEADER)

        # write init positions
        # file.write(f"robot {lastRobot[0]} {lastRobot[1]}\n")
        file.write(f"drone {lastDrone[0]} {lastDrone[1]}\n")

        for line in input:
            if countdown <= 0:
                return
            countdown -= 1
            x, y = [float(value) for value in line.split()]
            rx = x + args.robot[0]
            ry = y + args.robot[1]
            newDrone = getDronePosition((rx, ry), lastDrone)
            if dist(lastRobot, (rx, ry)) > 0.01:
                file.write(f"robot {round(rx, 2)} {round(ry, 2)}\n")
                lastRobot = (rx, ry)
            if dist(lastDrone, newDrone) > 0.01:
                file.write(f"drone {round(newDrone[0],2)} {round(newDrone[1],2)}\n")
                lastDrone = newDrone
            file.write("-- STEP\n")


def main():
    global args
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument("input", help="input file path for robot movement")
    parser.add_argument("output", help="output file path for step file")
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

    buildStepFile(lines)


if __name__ == "__main__":
    main()
