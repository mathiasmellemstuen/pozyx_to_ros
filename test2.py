from pypozyx import *
from pypozyx.core import PozyxCore

from PozyxTracking import PozyxTracking


if __name__ == '__main__':
    anchers = [
        DeviceCoordinates(0x0001, 1, Coordinates(0,0,2000)),
        DeviceCoordinates(0x0002, 1, Coordinates(3000,0,2000)),
        DeviceCoordinates(0x0003, 1, Coordinates(0,3000,2000)),
        DeviceCoordinates(0x0004, 1, Coordinates(3000,3000,2000))
    ]

    print(anchers[0].pos.data)

    p = PozyxTracking(anchers = anchers)

    p.recalibrateCoordinate(1500, 1500, 0)
