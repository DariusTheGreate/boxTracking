from framework.loger import *

staticLoger = Loger("log_factory2stage2.txt")
staticLoger.clear_stuff()

def handleRightEvent(time, millis):
    print("неопределнное движение справа от конвейера", str(time)) 
    staticLoger.log_stuff("неопределнное движение справа от конвейера ", str(time))
    print("aboba")
