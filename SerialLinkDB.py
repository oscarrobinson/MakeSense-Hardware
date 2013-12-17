import MySQLdb
import time
import string
import random
import serial

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
timestamps = dict()

while (True):
	if (ser.inWaiting() > 0):
		print "reading data"
		serdata = ser.readline()
		print serdata
		dataList = serdata.split()
		hardwareId = dataList[0]
		data = dataList[2]
		timestamp = dataList[4]

		#if first reading for a stamp, save timestamp stuff so can calculate its timestamp
		if hardwareId not in timestamps:
			startTimestampRead = int(timestamp)
			startTimestamp = int(time.time())
			timestampDat = [startTimestampRead, startTimestamp]
			timestamps[hardwareId] = timestampDat


		timestamp = int((timestamps[hardwareId])[1]) + (int(timestamp)-int((timestamps[hardwareId])[0]))

		print "ID: "+str(hardwareId)+"|| reading: "+str(data)+" || timestamp: "+str(timestamp)
		
		count+=1

		#query="INSERT INTO lightsensor(timestamp,data)VALUES("+str(num)+",'"+str(serdata)+"');"
		#print query
		#cur.execute(query)
		#db.commit()
		#count = count+1

ser.close()
exit()


