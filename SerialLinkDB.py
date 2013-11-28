import MySQLdb
import time
import string
import random
import serial
 
# configure the serial connections
ser = serial.Serial(
	port='/dev/ttyUSB1',
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

#count = 1;
while (True):
	if (ser.inWaiting() > 0):
		print "reading data"
		serdata = float(ser.readline())
		print serdata
		num = time.time()
		query="INSERT INTO lightsensor(timestamp,data)VALUES("+str(num)+",'"+str(serdata)+"');"
		print query
		cur.execute(query)
		db.commit()
		print "time " + str(num) 
		#count = count+1

ser.close()
exit()


