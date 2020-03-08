import serial as s1
import numpy as np

from tkinter import *

class main:

    def __init__(self,root):
        self.root = root
        self.toggleVar = False
        self.setUpCanvas()
        self.old_x = None
        self.old_y = None
        self.c.bind('<space>', self.command)

    def updatingCanvas(self, values, button):
        if button == 1.0:
            if self.old_x and self.old_y:
                self.c.create_line(self.old_x,self.old_y,values[0],values[1],capstyle=ROUND, smooth=True)
            self.old_x = values[0]
            self.old_y = values[1]

    def drawFromValues(self):
        ser = s1.Serial('/dev/cu.usbmodem14201', 115200)
        while True:
            b = ser.readline()
            string_n = b.decode()
            string = string_n.rstrip()
            cord = string.split(",")
            floatArray = np.array(cord)
            print(cord)
            val = floatArray.astype(float)
            buttonOutput = val[0]
            val * 100
            newVals = np.delete(val, 0)
            #newVals.astype(int)
            self.updatingCanvas(newVals, buttonOutput)

    def command(self,e):
        self.toggleVar = not self.toggleVar
        if self.toggleVar:
            self.drawFromValues()

    def setUpCanvas(self):
        self.c = Canvas(self.root)
        self.c.pack()

        menu = Menu(self.root)
        self.root.config(menu=menu)
        optionsMenu = Menu(menu)
        menu.add_cascade(label='Options', menu=optionsMenu)
        optionsMenu.add_command(label='Exit',command=self.root.destroy)

if __name__ == "__main__":
    root = Tk()
    main(root)
    root.mainloop()

'''
b1 = "down"
xold, yold = None, None
canvas_W = 750
canvas_H = 600
count = 0;

def main(values,button,drawing_area):
    #b1down(button)
    #b1up(button)
    motion(values,drawing_area)

def b1down(button):
    global b1
    if button == 1.0:
        b1 = "down"

def b1up(button):
    global b1, xold, yold
    if button == 0.0:
        b1 = "up"
        xold = None
        yold = None

def motion(values,canvas):

    if b1 == "down":
        global xold, yold
        xold = values[0]
        yold = values[1]
        if xold is not None and yold is not None:
            canvas.create_line(xold,yold,xold+10,yold+10,smooth=TRUE)


if __name__ == "__main__":
    root = Tk()
    drawing_area = Canvas(root, width=canvas_W, height=canvas_H)
    drawing_area.pack()
    ser = s1.Serial('/dev/cu.usbmodem14201', 115200)

    while True:
        b = ser.readline()
        string_n = b.decode()
        string = string_n.rstrip()
        cord = string.split(",")
        floatArray = np.array(cord)
        print(cord)
        #converts the string array as a float
        val = floatArray.astype(float)
        #gets the button output. 0.0 as button up, 1.0 as button down
        buttonOutput = val[0]
        #mutiplies each element by 100 to fit in the canvas
        val * 100
        #deletes the first element, as that served as the button
        newVals = np.delete(val,0)
        #convert the float array as an int array
        newVals.astype(int)
        #calls the method and updates it as the whole thing runs on a loop.
        main(newVals,buttonOutput,drawing_area)
        #print("reached")
        #root.mainloop()
'''
