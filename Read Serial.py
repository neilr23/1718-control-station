#3/19/2018 BZ
#Displays all serial monitor outputs from Arduino onto GUI: Python

import serial
import time
from Tkinter import*

baudRate=9600
comport='com3' #add user input

data=serial.Serial(comport,baudRate)

root = Tk()
canvas = Canvas(root, width=640, height=480, bg="white")
canvas.pack()

id = canvas.create_text(320,240, text='CONNECTION ESTABLISHED: '+ comport + ', ' + str(baudRate))
canvas.update()
time.sleep(2)
x=1
while x<2:
   data_raw = data.readline()
   canvas.itemconfigure(id, text=data_raw)
   root.update()
root.mainloop()
