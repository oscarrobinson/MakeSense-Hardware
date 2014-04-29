import MySQLdb
import time
import string
import random
import serial

def getserial():
	cpuserial = "0000000000000000"
	try:
		f=open('/proc/cpuinfo','r')
		for line in f:
			if line[0:6]=='Serial':
				cpuserial=line[10:26]
		f.close()
	except:
		cpuserial="ERROR00000000000"
	return cpuserial
 
# configure the serial connections
ser = serial.Serial(
	port="/dev/ttyAMA0",
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


count = 1;

startTimestampRead = 0
startTimestamp = 0
timestamps = dict()

sensor_ids = []

serdata = ""

netId = getserial()

while (True):
        read = ser.read()
        if read!=None:
                serdata=serdata+read
        if read == ';':
                print serdata
		serdata = serdata[:-1] #remove the ; from the end of the string so timestamp properly parsed
                dataList = serdata.split()
		sensorId = dataList[0]
		data = dataList[2]
		timestamp = dataList[4]
		ontologyId = dataList[6]


		#if first reading for a stamp, save timestamp stuff so can calculate its timestamp
		if sensorId not in timestamps:
			startTimestampRead = int(timestamp)
			startTimestamp = int(time.time())
			timestampDat = [startTimestampRead, startTimestamp]
			timestamps[sensorId] = timestampDat


		timestamp = int((timestamps[sensorId])[1]) + (int(timestamp)-int((timestamps[sensorId])[0]))


		if sensorId not in sensor_ids:
			sensor_ids.append(sensorId)
			query = "SELECT COUNT(1) FROM sensors WHERE sensor_id=\'"+sensorId+"\'"
			cur.execute(query)
			db.commit()
			if cur.fetchone()[0]==0:
				print "Adding "+sensorId+" to sensors"
				query = "INSERT INTO sensors(sensor_id, network_id, ontology_id) VALUES(\'"+sensorId+"\',\'"+netId+"\',\'"+ontologyId+"\')"
				cur.execute(query)
				db.commit()


		query = "INSERT INTO data(sensor_id, timestamp, reading) VALUES(\'"+sensorId+"\',\'"+str(timestamp)+"\',\'"+data+"\')"
		cur.execute(query)
		db.commit()

		count = count+1
		serdata=""


ser.close()
exit()


