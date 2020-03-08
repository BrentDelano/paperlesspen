import serial as s1
import time
import numpy as np
from tkinter import *
#button(1,0),x,y,xtilt,ytilt,ztilt

b1 = "up"
xold, yold = None, None
canvas_W = 750
canvas_H = 600
count = 0;

if __name__ == "__main__":
    # RANGE OF OUTPUT 0-7 for
    # weightet
    # button, x,y ()
    root = Tk()
    drawing_area = Canvas(root, width=canvas_W, height=canvas_H)
    drawing_area.pack()
    #ser = s1.Serial('/dev/cu.usbmodem14201', 115200)

    while True:
        '''
            b = ser.readline()
            string_n = b.decode()
            string = string_n.rstrip()
            cord = string.split(",")
            val = cord.astype(np.float)
            val * 10
            val = val.astype(np.int)
            buttonOutput = val[0]
            del val[0]
            main(val,ButtonOutput)
            '''
        root.mainloop()


def main(values,button):
    if button == 10:
        motion(values,drawing_area)
        b1down(button)
        b1up(button)

def b1down(button):
    global b1
    if button == 1:
        b1 = "down"

def b1up(button):
    global b1, xold, yold
    if button == 0:
        b1 = "up"
        xold = None
        yold = None

def motion(values,canvas):

    if b1 == "down":
        global xold, yold
        print(xold)
        print(yold)
        if xold is not None and yold is not None:
            canvas.create_line(xold,yold,xold+20,yold+20,smooth=TRUE)
            print("dhould be drawing")
        xold = values[0]
        yold = values[1]
        print(xold)
        print(yold)