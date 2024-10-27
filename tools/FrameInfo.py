import numpy as np

class Detection:
    def __init__(self, xVal, yVal):
        self.X = xVal
        self.Y = yVal

    def update_pos(self, xVal, yVal):
        self.X = xVal
        self.Y = yVal

class Track:
    def __init__(self, state, xVal, yVal, P00, P11):
        self.X = xVal
        self.Y = yVal
        self.P00 = P00
        self.P11 = P11

        self.status = state

    def update_pos(self, xVal, yVal):
        self.X = xVal
        self.Y = yVal

    def update_status(self, state):
        self.status = state

class FrameInfo:

    def __init__(self):
        self.detections = []
        self.tracks = []
        self.frameValid = False
        self.detValid = False
        self.trackValid = False

    def append_detection(self, detection):
        if isinstance(detection,Detection):
            self.detections.append(detection)
        else:
            raise ValueError("Obj must be a Detection")
        
    def append_track(self, track):
        if isinstance(track, Track):
            self.tracks.append(track)
        else:
            raise ValueError("Obj must be a Track")
    
    def set_frame_valid(self, valid):
        if isinstance(valid, bool):
            self.frameValid = valid
        else:
            raise ValueError("Input must be a bool")

    def set_det_valid(self, valid):
        if isinstance(valid, bool):
            self.detValid = valid
        else:
            raise ValueError("Input must be a bool")

    def set_track_valid(self, valid):
        if isinstance(valid, bool):
            self.trackValid = valid
        else:
            raise ValueError("Input must be a bool")
        
def calculate_gaussian_2d(mean, P, bits, COLS, ROWS):

    x = np.linspace(0, COLS, bits)
    y = np.linspace(0, ROWS, bits)
    X, Y = np.meshgrid(x, y)

    pos = np.dstack((X, Y))
    norm_factor = 1.0

    if np.linalg.det(P) < 1e-10:
        P_inv = np.linalg.pinv(P)
    else:
        P_inv = np.linalg.inv(P)

    Z = norm_factor * np.exp(-0.5 * np.einsum('...k,kl,...l->...', pos - mean, P_inv, pos - mean))
    
    return Z