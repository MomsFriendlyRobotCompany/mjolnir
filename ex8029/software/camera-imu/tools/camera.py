import cv2
import atexit
import numpy as np

# @atexit.register
def close():
    print("\nclose ...")
    cv2.destroyAllWindows()

class CameraDisplay:
    def __init__(self, title=""):
        self.title = title
        atexit.register(close)

    def __del__(self):
        print("\nbye ...")

    def imshow(self, image):
        cv2.imshow(f"{self.title}", image)
        k = cv2.waitKey(1) & 0xFF
        if k == 27 or k == ord("q"):
           ret = (True, k,)
        else:
           ret = (False, None,)
        return ret

class DisparityDisplay:
    def __init__(self, title="", stereo=None):
        atexit.register(close)

        self.title = title

        # maxdisp = 16*10
        # mindisp = 16*1
        # ndisp = maxdisp - mindisp
        # blksize = 11
        # window_size = 3*3
        # self.stereo = cv2.StereoSGBM_create(
        #     numDisparities=ndisp,
        #     blockSize=blksize,
        #     P1 = 8*1*window_size**2,  # parameters for disparity matching
        #     P2 = 32*1*window_size**2,
        #     # mode=True
        # )
        win_size = 5
        self.stereo = cv2.StereoBM_create(
            numDisparities=16*2,
            blockSize=11,
            # P1 = win_size**2,
            # P2 = 8*win_size**2
        )

    def imshow(self, image):
        if len(image.shape) > 2:
            raise Exception("Image must be grayscale")

        w = image.shape[1]//2 # get image width
        imgL = image[:,:w]
        imgR = image[:,w:]

        # scale --------------------------------------------------------------
        height, width = imgL.shape
        scale = 2
        imgL = cv2.pyrDown(imgL, dstsize= (width // scale, height // scale))
        imgR = cv2.pyrDown(imgR, dstsize= (width // scale, height // scale))

        # smooth -------------------------------------------------------------
        win = 11
        # imgL = cv2.GaussianBlur(imgL, (win,win), -1)
        # imgR = cv2.GaussianBlur(imgR, (win,win), -1)

        # Stereo disparity ---------------------------------------------------
        disparity = self.stereo.compute(imgL,imgR)
        d = disparity < 0
        disparity[d] = 0
        print(np.amin(disparity), np.amax(disparity))

        # visualize ----------------------------------------------------------
        disparity = cv2.normalize(
            disparity,
            None,
            alpha=1,
            beta=255,
            norm_type=cv2.NORM_MINMAX,
            dtype=cv2.CV_8U)

        cv2.imshow(f"{self.title}", disparity)

        k = cv2.waitKey(1) & 0xFF
        if k == 27 or k == ord("q"):
           ret = (True, k,)
        else:
           ret = (False, None,)
        return ret
