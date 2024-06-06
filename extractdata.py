#Bernice Lien 400382544

import serial
import math
import os
s = serial.Serial('COM5',115200,timeout=10) #Change COM ports here

print("Opening: " + s.name)

# reset the buffers of the UART port to delete the remaining data in the buffers
s.reset_output_buffer()
s.reset_input_buffer()

# wait for user's signal to start the program
input("Press Enter to start communication...")
# send the character 's' to MCU via UART
# Press enter start the transmission
i = 0
x = 0

#change variable x_i to preferred depth increment
x_i = 100
s.write('s'.encode())

#change directory to where you want your .xyz file to be
directory = r"C:\\Users\berni\Downloads\Desktop\\lab"
#change the file name to what you want the file to be called
file_path = os.path.join(directory, "tof_radar.xyz")

#open file and only append
f = open(file_path,"a")

while True:
    line = s.readline()
    line_de = line.decode()
    line_de = line_de[0:-3] #delete everything after space
    
    
    if (line_de.isdigit() == True):
        angle = (i/32)*2*math.pi
        val = int(line_de)
        z = round(math.sin(angle)*val)
        y = round(math.cos(angle)*val)
        f.write('{} {} {}\n'.format(x,y,z))
        i += 1
    if(line_de.isdigit() == False):
        f.close()
        f = open(file_path,"a")
        
    if i == 32:
        i = 0
        x += x_i

    



