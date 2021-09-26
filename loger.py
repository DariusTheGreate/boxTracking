import datetime
import time

class Loger():
    def __init__(self, log_file_path):
        self.log_file_path = log_file_path

    def log_stuff(self, *args):
        with open(self.log_file_path, 'a+') as file:
            file.seek(0)
            # If file is not empty then append '\n'
            data = file.read(100)
            if len(data) > 0:
                file.write("\n")
            
            file.write("время логирования -> ")
            file.write(str(datetime.timedelta(milliseconds = int(time.time()))))
            file.write(str(" "))
            
            for item in args:
                file.write(item)


staticLoger = Loger("log.txt")