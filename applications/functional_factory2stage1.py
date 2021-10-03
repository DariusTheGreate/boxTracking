from framework.loger import *

staticLoger = Loger("log_factory2stage1.txt")
staticLoger.clear_stuff()

def handleUpperEvent(time, millis):
    print("неопределнное движение сверху от конвейера", str(time)) 
    staticLoger.log_stuff("неопределнное движение сверху от конвейера ", str(time))
    print("aboba")

def handleLowerEvent(time, millis):
    print("неопределнное движение снизу от конвейера", str(time)) 
    staticLoger.log_stuff("неопределнное движение снизу от конвейера ", str(time))    
    print("bebra")
