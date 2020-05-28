# -*- coding: utf-8 -*-
import serial, time
import numpy as np


feather = serial.Serial('COM39', 115200, timeout=0.1)





# time.sleep(0.1)
# feather.write("1o".encode())
# time.sleep(0.1)
# feather.write("1o".encode())
# time.sleep(0.1)
# feather.write("1o".encode())
# time.sleep(0.1)
# for i in range(7):
#     mes = str(i) + 'o'
#     feather.write(mes.encode())
#     time.sleep(0.1)
while True:
    try:
        mes = feather.readline()#.encode()
        if len(mes) >= 1:
            print(mes)
    except:
        pass