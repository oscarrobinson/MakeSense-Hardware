import MySQLdb
import time
import string
import random
import serial

cpuserial = "0000000000000000"

# configure the serial connections
ser = serial.Serial(
	port='/dev/cu.usbserial-1d11B',
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
db = MySQLdb.connect(host="eu-cdbr-azure-north-b.cloudapp.net", 
                     user="b56834bde0c85e",
                      passwd="87a230d7",
                      db="makesensemain")
 

cur = db.cursor() 
cur.execute("TRUNCATE TABLE sensor_data;")

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
		ontologyId = dataList[6]

		#if first reading for a stamp, save timestamp stuff so can calculate its timestamp
		if hardwareId not in timestamps:
			startTimestampRead = int(timestamp)
			startTimestamp = int(time.time())
			timestampDat = [startTimestampRead, startTimestamp]
			timestamps[hardwareId] = timestampDat


		timestamp = int((timestamps[hardwareId])[1]) + (int(timestamp)-int((timestamps[hardwareId])[0]))

		print "ID: "+str(hardwareId)+"|| reading: "+str(data)+" || timestamp: "+str(timestamp)+" || ontologyId: "+str(ontologyId)
		
		count+=1

		query="INSERT INTO sensor_data(id,data,timestamp,networkid,ontologyid) VALUES(\'"+str(hardwareId)+"\', \'"+str(data)+"\',\'"+str(timestamp)+"\',\'"+str(cpuserial)+"\',\'"+str(ontologyId)+"\');"
		print query
		cur.execute(query)
		db.commit()
		count = count+1

ser.close()
exit()


