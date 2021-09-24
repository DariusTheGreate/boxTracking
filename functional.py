import datetime

def handleStartPointEvent(time, millis):
    print("коробки начали движение", str(time), "время логирования -> ", str(datetime.timedelta(milliseconds = int(millis)))) 
    
    print("aboba")

def handleBoxIntroEvent(time, millis):
    print("коробки заехали в упаковочный бокс", str(time), "время логирования -> ", str(datetime.timedelta(milliseconds = int(millis)))) 
    
    print("bebra")

def handleBoxExitEvent(time, millis):
    print("коробки выехали из бокса", str(time), "время логирования -> ", str(datetime.timedelta(milliseconds = int(millis)))) 
    
    print("putin")

def handleRobotEvent(time, millis):
    print("робот перенес коробки", str(time), "время логирования -> ", str(datetime.timedelta(milliseconds = int(millis)))) 
    
    print("sereja")