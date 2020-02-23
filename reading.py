import serial as s1
import time
from tkinter import *
#button(1,0),x,y,xtilt,ytilt,ztilt

b1 = "up"
xold, yold = None, None

def main(values,button):
    root = Tk()
    drawing_area = Canvas(root)
    drawing_area.pack()
    motion(values)
    b1down()
    b1up()
    root.mainloop()

def b1down(event):
    global b1
    b1 = "down"

def b1up(event):
    global b1, xold, yold
    b1 = "up"
    xold = None
    yold = None

def motion(values,event):
    if b1 == "down":
        global xold, yold
        if xold is not None and yold is not None:
            event.widget.create_line(xold,yold,xold,yold,smooth=TRUE)
                          # here's where you draw it. smooth. neat.
        xold = values[0]
        yold = values[1]

ser = s1.Serial('/dev/cu.usbmodem14201',115200)
time.sleep(2)

data = []
for i in range(50):
    b = ser.readline()
    string_n = b.decode()
    string = string_n.rstrip()
    print(string)
    data.append(string)
    time.sleep(0.1)

ser.close()

if __name__ == "__main__":
    var = "20,10"

    button = 1
    main(var,button)