from typing import List
from pypozyx import *

class XYZ:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.z = 0
    
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

    def __str__(self):
       return f'X: {self.x} Y: {self.y} Z: {self.z}' 

    def getList(self):
        return [self.x, self.y, self.z]


class PozyxLocalizer:
    """
    Pozyx localizer class

    This class connects to a pozyx tag through a USB serial connection and returns the current position from the pozyx
    environment. 

    """
    def __init__(self, port=None, remoteID=0x00, remote=False, anchors=List, algorithm=POZYX_POS_ALG_UWB_ONLY, dimension=POZYX_2D, height=1000):

        self.deviceID = None # ID of the master device
        self.position = None # Positon of the tag
        self.pozyx = None
        self.remoteID = remoteID
        self.algorithm = algorithm
        self.dimension = dimension
        self.anchors = anchors
        self.height = height
        
        if not remote:
            self.remoteID = None

        if port is None:
            self.pozyx = self.createSerialConnectionToTag()
        else:
            self.pozyx = self.createSerialConnectionToTag(tagName=port)

        self.setAnchorsManually()


    def createSerialConnectionToTag(self, tagName=None):
        """
        Creating a serial connection either automatically or manually with providing the tagName. Returns a PozyxSerial object.
        """

        if tagName is None:
            serialPort = get_first_pozyx_serial_port()
            print(f'Auto assigning serial port: {serialPort}')
        else:
            serialPort = tagName
        
        connection = PozyxSerial(serialPort)

        if connection is None:
            print('No Pozyx connected. Check if one is connected')

        return connection

    def setAnchorsManually(self, saveToFlash=False):
        """
        Using anchors that is provided in self.anchors. 
        """

        status = self.pozyx.clearDevices(self.remoteID)

        for anchor in self.anchors:
            status &= self.pozyx.addDevice(anchor, self.remoteID)
        
        if len(self.anchors) > 4:
            status &= self.pozyx.setSelectionOfAnchors(POZYX_ANCHOR_SEL_AUTO)

        if saveToFlash:
            self.pozyx.saveAnchorIds(remote_id=self.remoteID)
            self.pozyx.saveRegisters([PozyxRegisters.POSITIONING_NUMBER_OF_ANCHORS], remote_id=self.remoteID)

        return status
        
    def loop(self):
        """
        Getting the position of the connected tag and returning it. 
        """
        self.position = Coordinates()
        status = self.pozyx.doPositioning(self.position, self.dimension, self.height, self.algorithm, remote_id=self.remoteID)

        if status == POZYX_SUCCESS: 
            print(self.positionToString())
        else: 
            print(f'Error: Do positioning failed due to {"failure" if status == POZYX_FAILURE else "timeout"}.')

    def recalibrateCoordinate(self, offsetX, offsetY, offsetZ):
        """Recalibrate coordinates for the pozyx system, to be removed"""

        newOrigo = XYZ()

        # Calculat the new origo for pozyx
        newOrigo.x -= (offsetX)
        newOrigo.y -= (offsetY)
        newOrigo.z -= (offsetZ)

        # Recalculate the coordinated for each anchor
        ## Get the base for calculations
        base = XYZ()
        base.x = self.anchors[0].pos.x
        base.y = self.anchors[0].pos.y
        base.z = self.anchors[0].pos.z

        ## Calculate the new pos for the remaining anchors
        for anchor in self.anchors:
            # Calculate for X
            if anchor.pos.x < newOrigo.x:
                anchor.pos.x = -(abs(anchor.pos.x) + abs(offsetX))
            else:
                anchor.pos.x = anchor.pos.x - offsetX

            # Calculate for Y
            if anchor.pos.y < newOrigo.y:
                anchor.pos.y = abs(anchor.pos.y) + abs(offsetY)
            else:
                anchor.pos.y = anchor.pos.y - offsetY

            # Calulate for Z
            if anchor.pos.z < newOrigo.z:
                anchor.pos.z = abs(anchor.pos.z) + abs(offsetZ)
            else:
                anchor.pos.z = anchor.pos.z - offsetZ
    
    def positionToString(self):
        """Returning a string with the x,y,z coordinates."""

        return f'Current position:\nX: {self.position[0] / 1000}\nY: {self.position[1] / 1000}\nZ: {self.position[2] / 1000}'


if __name__ == '__main__':
    anchors = [
        DeviceCoordinates(0x682c, 1, Coordinates(-1993, 1084, 0)),
        DeviceCoordinates(0x6869, 1, Coordinates(1954, -1739, 0)),
        DeviceCoordinates(0x680b, 1, Coordinates(-355, 1742, 0)),
        DeviceCoordinates(0x6851, 1, Coordinates(176, -3328, 0))
    ]

    localizer = PozyxLocalizer(anchors = anchors)

    while True:
        localizer.loop()
