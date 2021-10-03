from framework.core import *

import functional_chuli
from framework.loger import *


class Application():
    def __init__(self, videoPath):
        self.videoPath = videoPath
        self.framesList = []
        self.packets = []
        self.currentFrame = None
        self.packetThreads = []

    def run(self):
        left1 = 230
        right1 = 330
        up1 = 490 
        lower1 = 780

        left2 = 630
        right2 = 730
        up2 = 490 
        lower2 = 780

        left3 = 1300
        right3 = 1500
        up3 = 490
        lower3 = 780

        left4 = 1800
        right4 = 2100
        up4 = 880
        lower4 = 1000

        left5 = 350
        right5 = 650
        up5 = 380
        lower5 = 480

        first_box = Box(left1, lower1, right1, up1)
        second_box = Box(left2, lower2, right2, up2)
        third_box = Box(left3, lower3, right3, up3)
        four_box = Box(left4, lower4, right4, up4)
        fifth_box = Box(left5, lower5, right5, up5)

        first_box_motion_state = ControlState()
        second_box_motion_state = ControlState()

        third_box_motion_state = ControlState()
        four_box_motion_state = ControlState()
        
        fifth_box_motion_state = ControlState()


        start_point_controller = MotionFeatureDetectorController(first_box_motion_state, "палеты начали движение, время:", first_box, 7, -1, handle_function = functional_chuli.handleStartPointEvent, time_delta_between_event = 5000)

        box_intro_point_controller = MotionFeatureDetectorController(second_box_motion_state, "палеты заехали в бокс, время:", second_box, 7, -1, handle_function = functional_chuli.handleBoxIntroEvent)

        box_exit_point_controller = MotionContourDetectorController(third_box_motion_state,  "выезд из бокса, время:", third_box, 40000, handle_function = functional_chuli.handleBoxExitEvent)

        box_robot_point_controller = MotionContourDetectorController(four_box_motion_state,  "робот перенес кубы на упаковку, время:", four_box, 20000, handle_function = functional_chuli.handleRobotEvent)

        droping_start_point_controller = MotionFeatureDetectorController(fifth_box_motion_state, "незпланированное движение сверху от конвеера, время:", fifth_box, -1, 0, handle_function = functional_chuli.handleDropingEvent, time_delta_between_event = 5000)


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

                    start_point_controller.controlInBaseMode(prevFrame.copy(), frame.copy(), millis)
                    box_intro_point_controller.controlInBaseMode(prevFrame.copy(), frame.copy(), millis)
                    box_exit_point_controller.controlInBaseMode(frame.copy(), millis)
                    box_robot_point_controller.controlInBaseMode(frame.copy(), millis)
                    droping_start_point_controller.controlInBaseMode(prevFrame.copy(), frame.copy(), millis)
                
                    first_box.paintBox(frame)
                    second_box.paintBox(frame)
                    third_box.paintBox(frame)
                    four_box.paintBox(frame)
                    fifth_box.paintBox(frame)
                    
                    self.currentFrame = frame
                    
                    cv.imshow("mat", self.currentFrame[up4-500: lower4+100, left4-1700 : right4+200])
                    
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
Application("../videos/chuli/152835-video.mp4").run()
print("time of application work --> ", time.time() - start_t)