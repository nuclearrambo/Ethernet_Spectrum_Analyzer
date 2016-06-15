 
import socket
import sys
import re
import matplotlib.pyplot as plotter
from matplotlib import animation
import binascii
import numpy as np

HOST = ''   # Symbolic name meaning all available interfaces
PORT = 2222 # Arbitrary non-privileged port
 
# Datagram (udp) socket
try :
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    print 'Socket created'
except socket.error, msg :
    print 'Failed to create socket. Error Code : ' + str(msg[0]) + ' Message ' + msg[1]
    sys.exit()
 
 
# Bind socket to local host and port
try:
    s.bind((HOST, PORT))
except socket.error , msg:
    print 'Bind failed. Error Code : ' + str(msg[0]) + ' Message ' + msg[1]
    sys.exit()
     
print 'Socket bind complete'

flag = 0;
figure = plotter.figure()
ax = plotter.axes()
plotter.ion()


float_string_array = "0";
spectrum_data = np.zeros((512, 1))
freq_axis = np.linspace(0, 263e3, 512)
line, = plotter.plot(freq_axis, spectrum_data)
ymin = 40
ymax = 90

plotter.ylim([ymin, ymax])
plotter.xlim([0, 263e3])
#now keep talking with the client
while 1:
    # receive data from client (data, addr)

    d = s.recvfrom(2222)
    i = 0;
    k = 0;
    split_data =  re.split('\n', d[0])

    if ('I0.00' not in split_data):
        if flag >= 1:
            float_string_array = float_string_array + split_data
            flag += 1

    else:
        float_string_array = ''
        float_string_array = split_data
        float_string_array[0] = '0.00'
        flag = 1

    for i in range(0, len(float_string_array)-1):
        try:
            spectrum_data[i][0] = 10*np.log10(float(float_string_array[i])/1e-3)
        except ValueError:
            spectrum_data[i][0] = 10*np.log10(0.00)
        except IndexError:
            break


    #print float_string_array

    if flag == 2:
        line.set_ydata(spectrum_data)
        plotter.draw()
        plotter.show()
        plotter.pause(0.001)

s.close()
