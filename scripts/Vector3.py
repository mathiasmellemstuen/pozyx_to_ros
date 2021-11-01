class Vector3:
    def __init__(self, *args):
        self.x = 0 if len(args) < 1 else args[0]
        self.y = 0 if len(args) < 2 else args[1]
        self.z = 0 if len(args) < 3 else args[2]

    def __str__(self):
        return "X: " + str(self.x) + " Y: " + str(self.y) + " Z: " + str(self.z)

    def __isub__(self, other):
        return Vector3(self.x - other.x, self.y - other.y, self.z - other.z)

    def __eq__(self, other):
        return Vector3(other.x, other.y, other.z)

    def __getitem__(self, key):
        t = [self.x, self.y, self.z]
        return t[key]

    def getList(self):
        return [self.x, self.y, self.z]
