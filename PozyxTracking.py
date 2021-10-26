from typing import List
from pypozyx import *

class PozyxTracking:
    def __init__(self, port=None, remoteID=0x00, remote=False,
                 anchors=List, algorithm=POZYX_POS_ALG_UWB_ONLY, dimention=POZYX_3D, height=1000):
        self.deviceID = None   # ID of the master device
        self.networkID = None  # IF of the network
        self.anchors = anchors
        self.position = None   # Positon of the tag
        self.height = height
        self.algorithm = algorithm

        self.p = PozyxSerial(port)

        if not remote:
            self.remoteID = None

        if port is None:
            self.p = self.connectToTag()
        else:
            self.p = self.connectToTag(tagName=port)

        self.setAnchorsManually()


    def connectToTag(self, tagName=None):
        if tagName is None:
            serialPort = get_serial_ports()[0].device
            print(serialPort)
        else:
            serialPort = tagName
        
        return PozyxSerial(serialPort)

    def setAnchorsManually(self):
        status = self.p.clearDevices(self.remoteID)

        for anchor in self.anchors:
            status &= self.p.addDevice(anchor, self.remoteID)
        
        if len(self.anchors) > 4:
            status &= self.p.setSelectionOfAnchors(POZYX_ANCHOR_SEL_AUTO)

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
        newOrigo = [0,0,0]

        # Calculat the new origo for pozyx
        newOrigo[0] -= (offsetX)
        newOrigo[1] -= (offsetY)
        newOrigo[2] -= (offsetZ)

        # Recalculate the coordinated for each anchor
    
    def positionToString(self):
        return f'Current position:\nX: {self.position[0]}\nY: {self.position[1]}\n Z: {self.position[2]}'


if __name__ == '__main__':
    anchors = [
        DeviceCoordinates(0x0001, 1, Coordinates(0,0,2000)),
        DeviceCoordinates(0x0002, 1, Coordinates(3000,0,2000)),
        DeviceCoordinates(0x0003, 1, Coordinates(0,3000,2000)),
        DeviceCoordinates(0x0004, 1, Coordinates(3000,3000,2000))
    ]
    pozyxTracking = PozyxTracking(port = "/dev/cu.usbmodem3551385E34381", anchors = anchors)

    while True:
        pozyxTracking.loop()
