import cv2

class CameraDisplay:
    def __init__(self, title=""):
        self.title = title

    def __del__(self):
        cv2.destroyAllWindows()

    def imshow(self, image):
        cv2.imshow(f"{self.title}", image)
        k = cv2.waitKey(1) & 0xFF
        if k == 27 or k == ord("q"):
           ret = True
        else:
           ret = False
        return ret
