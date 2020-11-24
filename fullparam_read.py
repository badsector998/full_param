import serial
import time
import csv
import os

string_conn = '/dev/ttyUSB0'
arrSize = 13
dataRecorder = [0]*arrSize
fields = ['Lat', 'Lon', 'Alt', 'Speed', 'Yaw', 'Pitch', 'Roll', 'ax', 'ay', 'az', 'gx', 'gy', 'gz']



try:
	arduino = serial.Serial(string_conn, 115200, timeout = 1)
except:
	print('Please check your port!')

time.sleep(2)

def settingUp(notif):
	msgs = notif.decode('utf-8')
	msgs.split()
	print(msgs)
	if "chars" in msgs:
		userInput = input("Press any key")
		arduino.write(userInput.encode())
		return settingUp(arduino.readline())
	if "interrupt" in msgs:
		return 0
	else:
		return settingUp(arduino.readline())

def getVal():
	data = arduino.readline().decode('utf-8')
	splitLine = data.split(' ')
	if "invalid!" in splitLine:
		return getVal()
	else:
		return splitLine

with open('dataSample_fullParam', 'w') as f:
	write = csv.writer(f)
	write.writerow(fields)

readyStatus = settingUp(arduino.readline())
time.sleep(5)

while 1:
	if readyStatus == 0:
		try:
			rawData = getVal()
			for i in range(0, arrSize):
				dataRecorder[i] = rawData[i]
				print(dataRecorder[i])
			print('\n')
			with open('dataSample_fullParam','a+', newline='') as f:
				write = csv.writer(f)
				write.writerow(dataRecorder)
		except:
			print("Keyboard interupt")
			break
	else:
		print("System not ready, terminating!")
		break
