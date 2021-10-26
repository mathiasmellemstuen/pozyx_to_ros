from typing import List
from pypozyx import *

class PozyxTracking:
    def __init__(self, port=None, remoteID=0x00, remote=False,
                 anchors=List, algorithm=POZYX_POS_ALG_UWB_ONLY, dimention=POZYX_3D, height=1000):
        self.deviceID   # ID of the master device
        self.networkID  # IF of the network
        self.anchors = anchors
        self.position   # Positon of the tag

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

    def recalibrateCoordinate(self, offsetX, offsetY, offsetZ):
        """Recalibrate coordinates for the pozyx system"""
