from threading import Timer
def hello(): 
    pass
    Timer(1.0, hello) .start()
 
t = Timer(1.0, hello) 
t.start()