# chapter 28 in python

# sudo apt-get install python3-pip
# python3 -m pip install pyserial
# sudo apt-get install python3-matplotlib

import serial
ser = serial.Serial('COM4',230400)
print('Opening port: ')
print(ser.name)

from genref import genRef
import matplotlib.pyplot as plt 
from statistics import mean 
def read_plot_matrix():
    n_str = ser.read_until(b'\n');  # get the number of data points to receive
    n_int = int(n_str) # turn it into an int
    print('Data length = ' + str(n_int))
    ref = []
    data = []
    data_received = 0
    while data_received < n_int:
        dat_str = ser.read_until(b'\n');  # get the data as a string, ints seperated by spaces
        dat_f = list(map(float,dat_str.split())) # now the data is a list
        ref.append(dat_f[0])
        data.append(dat_f[1])
        data_received = data_received + 1
    meanzip = zip(ref,data)
    meanlist = []
    for i,j in meanzip:
        meanlist.append(abs(i-j))
    score = mean(meanlist)
    t = range(len(ref)) # index array
    plt.plot(t,ref,'r*-',t,data,'b*-')
    plt.title('Score = ' + str(score))
    plt.ylabel('value')
    plt.xlabel('index')
    plt.show()


has_quit = False
# menu loop
while not has_quit:
    print('PIC32 MOTOR DRIVER INTERFACE')
    # display the menu options; this list will grow
    print('\td: Dummy Command \n\tb: Current Sensor \n\tc: Encoder Position \n\tq: Quit \
           \n\tr: Mode \n\te: Reset Encoder \n\tf: Set Motor PWM \n\tp: Power Off Motor \
           \n\tg: Set Gains \n\th: Get Gains \n\tk: Test Current Gains \n\ti: Set Position Gains \
           \n\tj: Get Position Gains \n\tl: Go to Angle \n\to: Execute Trajectory \n\tm: Step Graph \
           \n\tn: Cubic Graph') # '\t' is a tab
    # read the user's choice
    selection = input('\nENTER COMMAND: ')
    selection_endline = selection+'\n'
     
    # send the command to the PIC32
    ser.write(selection_endline.encode()); # .encode() turns the string into a char array
    
    # take the appropriate action
    # there is no switch() in python, using if elif instead
    if (selection == 'd'):
        # example operation
        n_str = input('Enter number: ') # get the number to send
        n_int = int(n_str) # turn it into an int
        print('number = ' + str(n_int)) # print it to the screen to double check

        ser.write((str(n_int)+'\n').encode()); # send the number
        n_str = ser.read_until(b'\n');  # get the incremented number back
        n_int = int(n_str) # turn it into an int
        print('Got back: ' + str(n_int) + '\n') # print it to the screen
    elif (selection == 'c'):
        n_str = ser.read_until(b'\n')
        n_int = int(n_str)
        print('Encoder Position: ' + str(n_int) + '\n')
    elif (selection == 'q'):
        print('Exiting client')
        has_quit = True; # exit client
        # be sure to close the port
        ser.close()
    elif (selection == 'r'):
        n_str = ser.read_until(b'\n'); 
        n_int = int(n_str)
        if n_int == 0:
            mode = 'IDLE'
        elif n_int == 1:
            mode = 'PWM'
        elif n_int == 2:
            mode = 'ITEST'
        elif n_int == 3:
            mode = 'HOLD'
        elif n_int == 4:
            mode = 'TRACK'
        print('Mode: ' + mode + '\n')
    elif (selection == 'b'):
        n_str = ser.read_until(b'\n')
        n_float = float(n_str)
        print('Current: ' + str(n_float) + '\n')
    elif (selection == 'e'):
        print('Reset Encoder\n')
    elif (selection == 'p'):
        print('Power Off Motor\n')
    elif (selection == 'f'):
        n_str = input('Enter PWM: ') # get the number to send
        n_int = int(n_str) # turn it into an int
        print('PWM = ' + str(n_int)) # print it to the screen to double check
        ser.write((str(n_int)+'\n').encode()); # send the number
    elif (selection == 'g'):
        n_str = input('Set current gains Kp: ')
        n_float = float(n_str) 
        print('Kp = ' + str(n_float)+ '\r\n')
        n1_str = input('Set current gains Ki: ')
        n1_float = float(n1_str)
        print('Ki = ' + str(n1_float)) # print it to the screen to double check
        ser.write((str(n_float)+' '+str(n1_float)+'\n').encode()); # send the number
    elif (selection == 'h'):
        print('Current gains: ')
        n_str = ser.read_until(b'\n'); 
        n_float = float(n_str)  
        print('Kp = ' + str(n_float) + '\r\n')
        n_str = ser.read_until(b'\n');  # get the incremented number back
        n_float = float(n_str) 
        print('Ki = ' + str(n_float) + '\r\n')
    elif (selection == 'k'):
        read_plot_matrix()
    elif (selection == 'i'):
        n_str = input('Set pKp: ')
        n_float = float(n_str) 
        print('pKp = ' + str(n_float)+ '\r\n')
        n1_str = input('Set pKi: ')
        n1_float = float(n1_str)
        print('pKi = ' + str(n1_float))
        n2_str = input('Set pKd: ')
        n2_float = float(n2_str)
        print('pKd = ' + str(n2_float))
        ser.write((str(n_float)+' '+str(n1_float)+' '+str(n2_float)+'\n').encode()); # send the number
    elif (selection == 'j'):
        print('Position gains: ')
        n_str = ser.read_until(b'\n'); 
        n_float = float(n_str)  
        print('pKp = ' + str(n_float) + '\r\n')
        n_str = ser.read_until(b'\n');  # get the incremented number back
        n_float = float(n_str) 
        print('pKi = ' + str(n_float) + '\r\n')
        n_str = ser.read_until(b'\n');  # get the incremented number back
        n_float = float(n_str) 
        print('pKd = ' + str(n_float) + '\r\n')
    elif (selection == 'l'):
        ang_str = input('Go to angle (deg): ')
        ang = float(ang_str) 
        print('Desired angle: ' + str(ang))
        ser.write((str(ang) + '\n').encode()); 
        print('Desired angle set \n')
        read_plot_matrix()
    elif (selection == 'm'):
        ref = genRef('step')
        #print(len(ref))
        t = range(len(ref))
        plt.plot(t,ref,'r*-')
        plt.ylabel('ange in degrees')
        plt.xlabel('index')
        plt.show()
        # send 
        ser.write((str(len(ref))+'\n').encode())
        for i in ref:
            ser.write((str(i)+'\n').encode())
    elif (selection == 'n'):
        ref = genRef('cubic')
        #print(len(ref))
        t = range(len(ref))
        plt.plot(t,ref,'r*-')
        plt.ylabel('ange in degrees')
        plt.xlabel('index')
        plt.show()
        # send 
        ser.write((str(len(ref))+'\n').encode())
        for i in ref:
            ser.write((str(i)+'\n').encode())
    elif (selection == 'o'):
        read_plot_matrix()
    else:
        print('Invalid Selection ' + selection_endline)



