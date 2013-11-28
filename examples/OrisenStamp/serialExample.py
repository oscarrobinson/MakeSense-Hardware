import time
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

db = MySQLdb.connect(host="eu-cdbr-azure-west-b.cloudapp.net", 
                     user="bd4f86da652db2",
                      passwd="95905cbf",
                      db="methtesA9fvknZn4")

cur = db.cursor() 
cur.execute("TRUNCATE TABLE randomdata;")

ser.isOpen()

count = 1;
while (True):
    if (ser.inWaiting() > 0):
				serdata = ser.readline()
				print '>>' + str(count) + ' ' + serdata
				count = count+1
                 query="INSERT INTO randomdata(id,randomString)VALUES("+num+",'"+readserial+"');"
                cur.execute(query)
                db.commit()


ser.close()
exit()
