import serial
import matplotlib.pyplot as plt 
import numpy as np 
import struct
from matplotlib.animation import FuncAnimation

# Configure the COM port
com_port = 'COM4'  # Replace with your COM port (e.g., 'COM3', 'COM4', etc.)
baud_rate = 115200   # Set the baud rate (must match the device's configuration)

# Open the serial port

def send_byte(ser,i):
    btz = struct.pack('B',i)
    ser.write(btz)

def rec_arr(ser, cmd):
    send_byte(ser,cmd)
    bytez = ser.read(4)
    nrbytez = struct.unpack('I',bytez)[0]
    data = ser.read(nrbytez)

    frmt = 'H'*(nrbytez//2)
    arr = struct.unpack(frmt,data)
    return arr

def rec_float_arr(ser, cmd):
    send_byte(ser,cmd)
    bytez = ser.read(4)
    nrbytez = struct.unpack('I',bytez)[0]
    data = ser.read(nrbytez)

    frmt = 'f'*(nrbytez//4)
    arr = struct.unpack(frmt,data)
    return arr


fig, ax = plt.subplots()
line1, = ax.plot([], [], '.-', label='adcData')
line2, = ax.plot([], [], '.-', label='sine')
ax.legend()

def init():
    line1.set_data([], [])
    line2.set_data([], [])
    return line1, line2

def update(frame):
    adcData = rec_arr(ser, 2)
    sine = rec_arr(ser, 3)
    theta = rec_float_arr(ser, 4)  # If you want to use theta, you can print or plot it as needed
    meas = rec_float_arr(ser, 5)[0]
    measf = rec_float_arr(ser, 6)[0]
    #print(np.rad2deg(theta[1]))
    print(meas,'\t', measf)
    x = np.arange(len(adcData))
    line1.set_data(x, adcData)
    line2.set_data(x, sine)
    ax.relim()
    ax.autoscale_view()
    #ax.set_title(f"Measured value: {measf}")
    return line1, line2




with serial.Serial(com_port, baud_rate, timeout=3) as ser:
    ani = FuncAnimation(fig, update, frames=1000, init_func=init, blit=True, interval=100)
    plt.show()
