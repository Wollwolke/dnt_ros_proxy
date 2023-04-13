#!/usr/bin/env python3

"""Generate Step file for use in core-automator"""

import argparse
from math import sqrt
from enum import Enum

SIM_STEP_SIZE = 0.2
DRONE_WAIT_TIME = 15
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
    return dist(p1, p2) < (args.range - 5)


def moveDrone(actualPos, desiredPos):
    distance = dist(actualPos, desiredPos)
    delta = min(distance, args.speed * SIM_STEP_SIZE)
    x = actualPos[0] + (delta * (desiredPos[0] - actualPos[0])) / distance
    y = actualPos[1] + (delta * (desiredPos[1] - actualPos[1])) / distance
    return (x, y)


def getDronePosition(robot, drone):
    global state

    if not hasattr(getDronePosition, "counter"):
        getDronePosition.counter = 0

    rx, ry = robot
    dx, dy = drone

    distance = dist(args.base, robot)
    if distance < args.range:
        # No Airbridge needed
        return drone
    elif distance < 2 * args.range:
        # Drone in the middle should bridge the gap
        state = DroneState.UNKNOWN
        x = (rx + dx) / 2
        y = (ry + dy) / 2
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
    dx, dy = (args.base[0] + 3, args.base[1] + 3)
    rx, ry = args.robot
    with open(args.output, "w") as file:
        file.write(HEADER)

        # write init positions
        file.write(f"robot {rx} {ry}\n")
        file.write(f"drone {dx} {dy}\n")

        for line in input:
            file.write("-- STEP\n")
            x, y = line.split()
            rx = x + args.robot[0]
            ry = y + args.robot[1]
            newDrone = getDronePosition((rx, ry), (dx, dy))
            file.write(f"robot {rx} {ry}\n")
            if newDrone[0] != dx or newDrone[1] != dy:
                dx, dy = newDrone
                file.write(f"drone {dx} {dy}\n")


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
        lines = file.readlines()

    buildStepFile(lines)


if __name__ == "__main__":
    main()
