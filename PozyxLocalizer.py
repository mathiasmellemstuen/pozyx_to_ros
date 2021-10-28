from typing import List
from pypozyx import *
from pythonosc.udp_client import SimpleUDPClient

class XYZ:
    def __init__(self, *args):
        self.x = 0 if len(args) < 1 else args[0]
        self.y = 0 if len(args) < 2 else args[1]
        self.z = 0 if len(args) < 3 else args[2]

    def __str__(self):
       return f'X: {self.x} Y: {self.y} Z: {self.z}' 

    def getList(self):
        return [self.x, self.y, self.z]


class PozyxLocalizer:
    def __init__(self, port=None, remoteID=0x00, remote=False, useProcessing=True,
                 anchors=List, algorithm=POZYX_POS_ALG_UWB_ONLY, dimension=POZYX_2D, height=1000,
                 ip='127.0.01', networkPort=8888, oscUdpClient=None):
        self.deviceID = None        # ID of the master device
        self.networkID = None       # ID of the network
        self.remoteID = remoteID

        self.algorithm = algorithm
        self.dimension = dimension
        self.oscUdpClient = oscUdpClient

        self.anchors = anchors

        self.height = height
        self.position = None   # Positon of the tag

        # self.p = PozyxSerial(port)

        if not remote:
            self.remoteID = None

        if port is None:
            self.p = self.connectToTag()
        else:
            self.p = self.connectToTag(tagName=port)

        if useProcessing:
            oscUdpClient = SimpleUDPClient(ip, networkPort)

        self.setAnchorsManually()


    def connectToTag(self, tagName=None):
        if tagName is None:
            serialPort = get_serial_ports()[0].device
            print(serialPort)
        else:
            serialPort = tagName
        
        p = PozyxSerial(serialPort)

        if p is None:
            print('No Pozyx connected. Check if one is connected')

        return p

    def setAnchorsManually(self, saveToFlash=False):
        status = self.p.clearDevices(self.remoteID)

        for anchor in self.anchors:
            status &= self.p.addDevice(anchor, self.remoteID)
        
        if len(self.anchors) > 4:
            status &= self.p.setSelectionOfAnchors(POZYX_ANCHOR_SEL_AUTO)

        if saveToFlash:
            self.p.saveAnchorIds(remote_id=self.remoteID)
            self.p.saveRegisters([PozyxRegisters.POSITIONING_NUMBER_OF_ANCHORS], remote_id=self.remoteID)

        return status
        
    def loop(self):
        self.position = Coordinates()
        status = self.p.doPositioning(self.position, self.dimension, self.height, self.algorithm, remote_id=self.remoteID)

        if status == POZYX_SUCCESS: 
            print(self.positionToString())
        else: 
            print(f'Error: Do positioning failed due to {"failure" if status == POZYX_FAILURE else "timeout"}.')
        
    def getOrigoDevice(self):
        for anchor in self.anchors:
            if anchor.pos.data[0] == 0 and anchor.pos.data[1] == 0:
                return anchor

        return None

    def recalibrateCoordinate(self, offsetX, offsetY, offsetZ):
        """Recalibrate coordinates for the pozyx system, to be removed"""

        offset = XYZ()
        offset.x = offsetX
        offset.y = offsetY
        offset.z = offsetZ

        origoDevice = self.getOrigoDevice()
        origo = origoDevice.pos.data
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
            for i in range(2):
                anchorPos = anchor.pos[i]
                newOrigoPos = newOrigo[i]
                offsetPos = offset.getList()[i]

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
        return f'Current position:\nX: {self.position[0] / 1000}\nY: {self.position[1] / 1000}\nZ: {self.position[2] / 1000}'


if __name__ == '__main__':
    anchors = [
        DeviceCoordinates(0x682c, 1, Coordinates(-1993, 1084, 0)),
        DeviceCoordinates(0x6869, 1, Coordinates(1954, -1739, 0)),
        DeviceCoordinates(0x680b, 1, Coordinates(-355, 1742, 0)),
        DeviceCoordinates(0x6851, 1, Coordinates(176, -3328, 0))
    ]

    localizer = PozyxLocalizer(anchors = anchors, remoteID=0x6e66)

    while True:
        localizer.loop()
