from typing import List
from pypozyx import *
from pythonosc.udp_client import SimpleUDPClient

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
    def __init__(self, port=None, remoteID=0x00, remote=False, useProcessing=True,
                 anchors=List, algorithm=POZYX_POS_ALG_UWB_ONLY, dimention=POZYX_2D, height=1000,
                 ip='127.0.01', networkPort=8888, oscUdpClient=None):
        self.deviceID = None   # ID of the master device
        self.networkID = None  # IF of the network
        self.anchors = anchors
        self.position = None   # Positon of the tag
        self.height = height
        self.algorithm = algorithm
        self.oscUdpClient = oscUdpClient
        self.remoteID = remoteID
        self.dimension = dimention

        self.p = PozyxSerial(port)

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
        status = self.p.doPositioning(self.position, self.height, self.algorithm, remote_id=self.remoteID)

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
        return f'Current position:\nX: {self.position[0] / 1000}\nY: {self.position[1] / 1000}\nZ: {self.position[2] / 1000}'


if __name__ == '__main__':
    anchors = [
        DeviceCoordinates(0x682c, 1, Coordinates(0, 1664, 0)),
        DeviceCoordinates(0x6869, 1, Coordinates(1339, 0, 0)),
        DeviceCoordinates(0x680b, 1, Coordinates(3982, 4076, 0)),
        DeviceCoordinates(0x6851, 1, Coordinates(2056, 4994, 0))
    ]

    localizer = PozyxLocalizer(anchors = anchors, port="/dev/cu.usbmodem3551385E34381", remoteID=0x6e66)


    while True:
        localizer.loop()
