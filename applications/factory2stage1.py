from framework.core import *

import functional_factory2stage1
from framework.loger import *


class Application():
    def __init__(self, videoPath):
        self.videoPath = videoPath
        self.framesList = []
        self.packets = []
        self.currentFrame = None
        self.packetThreads = []

    def run(self):
        left1 = 600
        right1 = 1500
        up1 = 30 
        lower1 = 120

        left2 = left1
        right2 = right1
        up2 = up1 + 610
        lower2 = lower1 + 610

        first_box = Box(left1, lower1, right1, up1)
        second_box = Box(left2, lower2, right2, up2)

        first_box_motion_state = ControlState()
        second_box_motion_state = ControlState()


        upper_controller = MotionFeatureDetectorController(first_box_motion_state, "неопределенное движение сверху от конвейера, время:", first_box, -1, -1, handle_function = functional_factory2stage1.handleUpperEvent)

        lower_controller = MotionFeatureDetectorController(second_box_motion_state, "неопределенное движение снизу от конвейера, время:", second_box, -1, -1, handle_function = functional_factory2stage1.handleLowerEvent)

        cap = cv.VideoCapture(self.videoPath)
        
        if cap.isOpened():
            global prevFrame
            prevFrameRes, prevFrame = cap.read()
        self.currentFrame = prevFrame

        frameCounter = 1
        init = True
        while(cap.isOpened()):
            frameCounter += 1
            ret, frame = cap.read()
            if frame is None:
                break
            frameInit = frame.copy()
            self.currentFrame = frameInit.copy()

            if ret == True and frameCounter % frameRate == 0:
                if init != True:
                    millis = cap.get(0)

                    upper_controller.controlInBaseMode(prevFrame.copy(), frame.copy(), millis)
                    lower_controller.controlInBaseMode(prevFrame.copy(), frame.copy(), millis)

                    first_box.paintBox(frame)
                    second_box.paintBox(frame)
                    
                    self.currentFrame = frame
                    
                    cv.imshow("mat", self.currentFrame)
                    
                else:
                    init = False
                prevFrame = frameInit.copy()
                
                if cv.waitKey(1) & 0xFF == ord('q'):
                    break
            
            elif ret == True:
                continue
            else:
                cap = cv.VideoCapture(self.videoPath)
                #break
        
        cap.release()
        cv.destroyAllWindows()

start_t = time.time()
# УКАЗАТъ ПРАВИЛЬНЫЙ ПУТЬ ДО ВИДЕО
Application("../videos/factory2/30889-video.mp4").run()
print("time of application work --> ", time.time() - start_t)