from loger import staticLoger

def handleStartPointEvent(time, millis):
    print("коробки начали движение", str(time)) 
    staticLoger.log_stuff("коробки начали движение ", str(time))
    print("aboba")

def handleBoxIntroEvent(time, millis):
    print("коробки заехали в упаковочный бокс", str(time)) 
    staticLoger.log_stuff("коробки заехали в упаковочный бокс ", str(time))    
    print("bebra")

def handleBoxExitEvent(time, millis):
    print("коробки выехали из бокса", str(time)) 
    staticLoger.log_stuff("коробки выехали из бокса ", str(time))
    print("putin")

def handleRobotEvent(time, millis):
    print("робот перенес коробки", str(time)) 
    staticLoger.log_stuff("робот перенес коробки ", str(time))
    print("sereja")