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
cur.execute("TRUNCATE TABLE sensor_data;")

count = 1;

startTimestampRead = 0
startTimestamp = 0
timestamps = dict()

serdata = ""

while (True):
        read = ser.read()
        if read!=None:
                serdata=serdata+read
        if read == ';':
                print serdata
		serdata = serdata[:-1] #remove the ; from the end of the string so timestamp properly parsed
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

		
		piId = getserial()
		count+=1
		print "ID: "+str(hardwareId)+"|| reading: "+str(data)+" || timestamp: "+str(timestamp)+" || piId: "+piId+" || ontologyId: "+str(ontologyId)
		query="INSERT INTO sensor_data(id,data,timestamp,networkid,ontologyid) VALUES(\'"+str(hardwareId)+"\', \'"+str(data)+"\',\'"+str(timestamp)+"\',\'"+str(piId)+"\',\'"+str(ontologyId)+"\');"
		print query
		cur.execute(query)
		db.commit()
		count = count+1
		serdata=""


ser.close()
exit()


