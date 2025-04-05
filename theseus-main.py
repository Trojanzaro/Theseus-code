import serial, time
import csv, io


port = serial.Serial("/dev/ttyS0")
 # sudo chmod 777 /dev/ttyS0 

t1 = time.time()
t = []
v1 = []
v2 = []
v3 = []
v4 = []
try:
    
    
    # if port.isOpen():
    #     port.write("0 0 0 0\n".encode())
    #     port.flush()
    if port.isOpen():
        port.write(b'6.28 -6.28 -6.28 6.28\n')
        port.flush()
    while time.time() - t1 <= 10:
        
        if port.in_waiting > 0:
            t.append(time.time() - t1)
            rcv = port.readline()
            av = rcv.decode('ascii').rstrip()
            angular_velocities = av.split(" ")
            
            print(time.time() - t1, angular_velocities)
            
            v1.append(angular_velocities[0])
            v2.append(angular_velocities[1])
            v3.append(angular_velocities[2])
            v4.append(angular_velocities[3])

    with open('eggs.csv', 'w', newline='') as csvfile:
        for i in range(0, len(t)):
            spamwriter = csv.DictWriter(csvfile, fieldnames=['t', 'v1', 'v2', 'v3', 'v4'])
            spamwriter.writerow({'t': t[i], 'v1': v1[i],'v2': v2[i], 'v3': v3[i], 'v4': v4[i]})
except Exception as ex:
    print("error, f u" + str(ex))
    with open('eggs.csv', 'w', newline='') as csvfile:
        for i in range(0, len(t)):
            spamwriter = csv.DictWriter(csvfile, fieldnames=['t', 'v1', 'v2', 'v3', 'v4'])
            spamwriter.writerow({'t': t[i], 'v1': v1[i],'v2': v2[i], 'v3': v3[i], 'v4': v4[i]})