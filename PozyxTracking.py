from typing import List
from pypozyx import *

class PozyxTracking:
    def __init__(self, port=None, remoteID=0x00, remote=False,
                 anchers=List, algorithm=POZYX_POS_ALG_UWB_ONLY, dimention=POZYX_3D, height=1000):
        self.deviceID   # ID of the master device
        self.networkID  # IF of the network
        self.anchers = anchers
        self.position   # Positon of the tag

        if not remote:
            self.remoteID = None

        if port is None:
            self.p = self.connectToTag()
        else:
            self.p = self.connectToTag(tagName=port)

        self.setAnchoersManualy()


    def connectToTag(self, tagName=None):
        if tagName is None:
            serialPort = get_serial_ports()[0].device
            print(serialPort)
        else:
            serialPort = tagName
        
        return PozyxSerial(serialPort)

    def setAnchoersManualy(self):
        status = self.p.clearDevices(self.remoteID)

        for anchoer in self.anchers:
            status &= self.p.addDevice(anchoer, self.remoteID)
        
        if len(self.anchers) > 4:
            status &= self.p.setSelectionOfAnchors(POZYX_ANCHOR_SEL_AUTO)

        return status
        
    def loop(self):
        self.position = Coordinates()
        status = self.p.doPositioning(self.position, self.height, self.algorithm, remote_id=self.remoteID)

    def recalibrateCoordinate(self, offsettX, offsettY, offsetZ):
        """Recalibrate cordinates for the pozyx system"""

