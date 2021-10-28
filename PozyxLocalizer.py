from typing import List
from pypozyx import *

class XYZ:
    def __init__(self, *args):
        self.x = 0 if len(args) < 1 else args[0]
        self.y = 0 if len(args) < 2 else args[1]
        self.z = 0 if len(args) < 3 else args[2]

    def __str__(self):
       return f'X: {self.x} Y: {self.y} Z: {self.z}' 

    def __isub__(self, other):
        return XYZ(self.x - other.x, self.y - other.y, self.z - other.z)

    def __eq__(self, other):
        return XYZ(other.x, other.y, other.z)

    def __getitem__(self, key):
        t = [self.x, self.y, self.z]
        return t[key]

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

        offset = XYZ(offsetX, offsetY, offsetZ)
        
        origoDevice = self.getOrigoDevice()
        origo = origoDevice.pos.data

        # Calculat the new origo for pozyx
        newOrigo = XYZ()
        newOrigo -= offset

        # Recalculate the coordinated for each anchor
        for anchor in self.anchors:
            for i in range(2):
                anchorPos = anchor.pos[i]
                newOrigoPos = newOrigo[i]
                offsetPos = offset[i]

            if anchorPos < newOrigoPos:
                anchorPos -= (abs(anchorPos) + abs(offsetPos))
            else:
                anchorPos -= (anchorPos - offsetPos)

            # Calculate for X
            #if anchor.pos.x < newOrigo.x:
                #anchor.pos.x = -(abs(anchor.pos.x) + abs(offsetX))
            #else:
                #anchor.pos.x = -(anchor.pos.x - offsetX)

    def positionToString(self):
        """Returning a string with the x,y,z coordinates."""

        return f'Current position:\nX: {self.position[0] / 1000}m\nY: {self.position[1] / 1000}m\nZ: {self.position[2] / 1000}m'


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
