import cv2
import numpy as np


class PanZoomWindow(object):
    """ Controls an OpenCV window. Registers a mouse listener so that:
        1. right-dragging up/down zooms in/out
        2. right-clicking re-centers
        3. trackbars scroll vertically and horizontally
    You can open multiple windows at once if you specify different window names.
    You can pass in an onLeftClickFunction, and when the user left-clicks, this
    will call onLeftClickFunction(y,x), with y,x in original image coordinates."""

    def __init__(self, img, windowName='PanZoomWindow', onLeftClickFunction=None):
        self.WINDOW_NAME = windowName
        self.H_TRACKBAR_NAME = 'x'
        self.V_TRACKBAR_NAME = 'y'
        self.img = img
        self.onLeftClickFunction = onLeftClickFunction
        self.TRACKBAR_TICKS = 1000
        self.panAndZoomState = PanAndZoomState(img.shape, self)
        self.lButtonDownLoc = None
        self.mButtonDownLoc = None
        self.rButtonDownLoc = None
        cv2.namedWindow(self.WINDOW_NAME, cv2.WINDOW_NORMAL)
        self.redraw_image()
        cv2.createTrackbar(self.H_TRACKBAR_NAME, self.WINDOW_NAME, 0, self.TRACKBAR_TICKS, self.on_htrackbar_move)
        cv2.createTrackbar(self.V_TRACKBAR_NAME, self.WINDOW_NAME, 0, self.TRACKBAR_TICKS, self.on_vtrackbar_move)

    def on_vtrackbar_move(self, tickPosition):
        self.panAndZoomState.setYFractionOffset(float(tickPosition) / self.TRACKBAR_TICKS)

    def on_htrackbar_move(self, tickPosition):
        self.panAndZoomState.setXFractionOffset(float(tickPosition) / self.TRACKBAR_TICKS)

    def redraw_image(self):
        pzs = self.panAndZoomState
        cv2.imshow(self.WINDOW_NAME, self.img[pzs.ul[0]:pzs.ul[0] + pzs.shape[0], pzs.ul[1]:pzs.ul[1] + pzs.shape[1]])


class PanAndZoomState(object):
    """ Tracks the currently-shown rectangle of the image.
    Does the math to adjust this rectangle to pan and zoom."""
    MIN_SHAPE = np.array([50, 50])

    def __init__(self, imShape, parentWindow):
        self.ul = np.array([0, 0])  # upper left of the zoomed rectangle (expressed as y,x)
        self.imShape = np.array(imShape[0:2])
        self.shape = self.imShape  # current dimensions of rectangle
        self.parentWindow = parentWindow

    def zoom(self, relativeCy, relativeCx, zoomInFactor):
        self.shape = (self.shape.astype(np.float) / zoomInFactor).astype(np.int)
        # expands the view to a square shape if possible. (I don't know how to get the actual window aspect ratio)
        self.shape[:] = np.max(self.shape)
        self.shape = np.maximum(PanAndZoomState.MIN_SHAPE, self.shape)  # prevent zooming in too far
        c = self.ul + np.array([relativeCy, relativeCx])
        self.ul = (c - self.shape / 2).astype(np.int)
        self._fixBoundsAndDraw()

    def _fixBoundsAndDraw(self):
        """ Ensures we didn't scroll/zoom outside the image.
        Then draws the currently-shown rectangle of the image."""
        #        print("in self.ul: %s shape: %s"%(self.ul,self.shape))
        self.ul = np.maximum(0, np.minimum(self.ul, self.imShape - self.shape))
        self.shape = np.minimum(np.maximum(PanAndZoomState.MIN_SHAPE, self.shape), self.imShape - self.ul)
        #        print("out self.ul: %s shape: %s"%(self.ul,self.shape))
        yFraction = float(self.ul[0]) / max(1, self.imShape[0] - self.shape[0])
        xFraction = float(self.ul[1]) / max(1, self.imShape[1] - self.shape[1])
        cv2.setTrackbarPos(self.parentWindow.H_TRACKBAR_NAME, self.parentWindow.WINDOW_NAME,
                           int(xFraction * self.parentWindow.TRACKBAR_TICKS))
        cv2.setTrackbarPos(self.parentWindow.V_TRACKBAR_NAME, self.parentWindow.WINDOW_NAME,
                           int(yFraction * self.parentWindow.TRACKBAR_TICKS))
        self.parentWindow.redraw_image()

    def setYAbsoluteOffset(self, yPixel):
        self.ul[0] = min(max(0, yPixel), self.imShape[0] - self.shape[0])
        self._fixBoundsAndDraw()

    def setXAbsoluteOffset(self, xPixel):
        self.ul[1] = min(max(0, xPixel), self.imShape[1] - self.shape[1])
        self._fixBoundsAndDraw()

    def setYFractionOffset(self, fraction):
        """ pans so the upper-left zoomed rectange is "fraction" of the way down the image."""
        self.ul[0] = int(round((self.imShape[0] - self.shape[0]) * fraction))
        self._fixBoundsAndDraw()

    def setXFractionOffset(self, fraction):
        """ pans so the upper-left zoomed rectange is "fraction" of the way right on the image."""
        self.ul[1] = int(round((self.imShape[1] - self.shape[1]) * fraction))
        self._fixBoundsAndDraw()


if __name__ == "__main__":

    # seq_name = 'stKi'
    seq_name = 'monoKi'

    reduced = 0
    if reduced == 1:
        map_name = '{:s}_grid_map_red.pgm'.format(seq_name)
    else:
        map_name = '{:s}_grid_map_full.pgm'.format(seq_name)

    gmap = cv2.imread("./maps/{:s}".format(map_name), cv2.IMREAD_UNCHANGED)
    print
    gmap.shape
    window = PanZoomWindow(gmap, "test window")
    key = -1
    while key != ord('q') and key != 27:  # 27 = escape key
        key = cv2.waitKey(5)
    cv2.destroyAllWindows()
