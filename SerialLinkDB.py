import MySQLdb
import time
import string
import random
import serial
def splitString(str):
	result = []
	for s in str.split():
		if s.isdigit():
			result.append(int(s))
	return result


# configure the serial connections
ser = serial.Serial(
	port='/dev/cu.usbserial-FTWVXL40B',
	baudrate=115200,
	parity=serial.PARITY_NONE,
	stopbits=serial.STOPBITS_ONE,
	bytesize=serial.EIGHTBITS,
	xonxoff=False,
	rtscts=False,
	dsrdtr=False,
	timeout=2
)

#create database object and connect
db = MySQLdb.connect(host="eu-cdbr-azure-west-b.cloudapp.net", 
                     user="bd4f86da652db2",
                      passwd="95905cbf",
                      db="methtesA9fvknZn4")
 

cur = db.cursor() 
cur.execute("TRUNCATE TABLE lightsensor;")

count = 0;
startTimestampRead = 0
startTimestamp = 0

while (True):
	if (ser.inWaiting() > 0):
		print "reading data"
		serdata = ser.readline()
		numbers = splitString(serdata)
		hardwareId = numbers[0]
		data = numbers[1]
		timestamp = numbers[2]

		#if first reading, save current time and first timestamp read, can then calculate time of reading for all other readings
		if count==0:
			startTimestampRead = int(timestamp)
			startTimestamp = int(time.time())
			timestamp = startTimestamp
		#calculate the time the reading was made
		else:
			timestamp = startTimestamp + (timestamp-startTimestampRead)

		print "ID: "+str(hardwareId)+"|| reading: "+str(data)+" || timestamp: "+str(timestamp)
		
		count+=1

		#query="INSERT INTO lightsensor(timestamp,data)VALUES("+str(num)+",'"+str(serdata)+"');"
		#print query
		#cur.execute(query)
		#db.commit()
		#count = count+1

ser.close()
exit()


