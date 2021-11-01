from sysconfig import parse_config_h
from pypozyx import *
import Vector3
import yaml

class PozyxLocalizer:
    """
    Pozyx localizer class

    This class connects to a pozyx tag through a USB serial connection and returns the current position from the pozyx
    environment. 

    """
    def __init__(self, anchors, port=None, remoteID=0x00, remote=False, algorithm=POZYX_POS_ALG_UWB_ONLY, dimension=POZYX_2D, height=1000):

        self.deviceID = None # ID of the master device
        self.position = None # Positon of the tag
        self.pozyx = None
        self.remoteID = remoteID
        self.algorithm = algorithm
        self.dimension = dimension
        self.anchors = anchors
        self.height = height

        if type(self.anchors) == str: 
            self.parseYamlConfig(self.anchors)

        if not remote:
            self.remoteID = None

        if port is None:
            self.pozyx = self.createSerialConnectionToTag()
        else:
            self.pozyx = self.createSerialConnectionToTag(tagName=port)

        self.setAnchorsManually()

    def parseYamlConfig(self, path):
        anchors = []

        with open(path, "r") as file: 
            configYaml = yaml.safe_load(file)
            for anchor in configYaml["anchors"]:
                coordinates = Coordinates(anchor["coordinates"]["x"], anchor["coordinates"]["y"], anchor["coordinates"]["z"])
                dc = DeviceCoordinates(anchor["id"], anchor["flag"], coordinates)
                anchors.append(dc)

        self.anchors = anchors

    def createSerialConnectionToTag(self, tagName=None):
        """
        Creating a serial connection either automatically or manually with providing the tagName. Returns a PozyxSerial object.
        """

        if tagName is None:
            serialPort = get_first_pozyx_serial_port()
            print('Auto assigning serial port: {serialPort}')
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
            print('Error: Do positioning failed due to {"failure" if status == POZYX_FAILURE else "timeout"}.')

    def recalibrateCoordinate(self, offsetX, offsetY, offsetZ):
        """Recalibrate coordinates for the pozyx system, to be removed"""

        offset = Vector3(offsetX, offsetY, offsetZ)
        
        origoDevice = self.getOrigoDevice()
        origo = origoDevice.pos.data

        # Calculat the new origo for pozyx
        newOrigo = Vector3()
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

    def positionToString(self):
        """Returning a string with the x,y,z coordinates."""

        return 'Current position:\nX: {self.position[0] / 1000}m\nY: {self.position[1] / 1000}m\nZ: {self.position[2] / 1000}m'

if __name__ == '__main__':
    anchors = [
        DeviceCoordinates(0x682c, 1, Coordinates(-1993, 1084, 0)),
        DeviceCoordinates(0x6869, 1, Coordinates(1954, -1739, 0)),
        DeviceCoordinates(0x680b, 1, Coordinates(-355, 1742, 0)),
        DeviceCoordinates(0x6851, 1, Coordinates(176, -3328, 0))
    ]

    localizer = PozyxLocalizer(anchors = "PozyxConfig.yaml")

    while True:
        localizer.loop()