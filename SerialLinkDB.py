import MySQLdb
import time
import string
import random
import serial

cpuserial = "213801249821408"

# configure the serial connections
ser = serial.Serial(
	port='/dev/cu.usbserial-1a12B',
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

count = 0;
startTimestampRead = 0
startTimestamp = 0
timestamps = dict()
sensor_ids = []

while (True):
	if (ser.inWaiting() > 0):
		print "reading data"
		serdata = ser.readline()
		print serdata
		dataList = serdata.split()
		sensorId = dataList[0]
		data = dataList[2]
		timestamp = dataList[4]
		sensorOnt = 1

		#if first reading for a stamp, save timestamp stuff so can calculate its timestamp
		if sensorId not in timestamps:
			startTimestampRead = int(timestamp)
			startTimestamp = int(time.time())
			timestampDat = [startTimestampRead, startTimestamp]
			timestamps[sensorId] = timestampDat


		timestamp = int((timestamps[sensorId])[1]) + (int(timestamp)-int((timestamps[sensorId])[0]))

		print "ID: "+str(sensorId)+"|| reading: "+str(data)+" || timestamp: "+str(timestamp)+" || ontologyId: "+str(sensorOnt)
		
		count+=1

		if sensorId not in sensor_ids:
			sensor_ids.append(sensorId)
			query = "SELECT COUNT(1) FROM sensors WHERE sensor_id=\'"+sensorId+"\'"
			cur.execute(query)
			db.commit()
			if cur.fetchone()[0]==0:
				print "Adding "+sensorId+" to sensors"
				query = "INSERT INTO sensors(sensor_id, network_id, ontology_id) VALUES(\'"+sensorId+"\',\'"+cpuserial+"\',\'"+str(sensorOnt)+"\')"
				cur.execute(query)
				db.commit()

		print "id: "+sensorId+" || timestamp: "+str(timestamp)+" || data: "+str(data)

		query = "INSERT INTO data(sensor_id, timestamp, reading) VALUES(\'"+sensorId+"\',\'"+str(timestamp)+"\',\'"+str(data)+"\')"
		cur.execute(query)
		db.commit()

ser.close()
exit()


