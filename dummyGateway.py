import MySQLdb
import time
import string
import random

piId = "00000000f01243e9"
sensor1ID = "af0278fe920101fe"
sensor2ID = "af0278fe019283fe"
sensor3ID = "af0278fe912131415"
sensor1ont = "1"
sensor2ont = "1"
sensor3ont = "2"
sensor1data = 20
sensor2data = 20
sensor3data = 23




#create database object and connect
db = MySQLdb.connect(host="eu-cdbr-azure-north-b.cloudapp.net", 
                     user="b56834bde0c85e",
                      passwd="87a230d7",
                      db="makesensemain")
 

cur = db.cursor() 

sensor_ids = []

counter = 0

netId = ""
sensorId = ""
sensorOnt = ""
sensorData = 0
sensorTime = 0

while (True):

		counter+=1

		if counter == 1:
			netId = piId
			sensorId = sensor1ID
			sensorOnt = sensor1ont
			sensorData = sensor1data
			sensor1data = random.randint(10,1000)

		elif counter == 2:
			netId = piId
			sensorId = sensor2ID
			sensorOnt = sensor2ont
			sensorData = sensor2data
			sensor2data = random.randint(10,1000)

		elif counter == 3:
			netId = piId
			sensorId = sensor3ID
			sensorOnt = sensor3ont
			sensorData = sensor3data
			sensor3data = random.randint(10,25)
			counter = 0

		sensorTime = time.time()


		if sensorId not in sensor_ids:
			sensor_ids.append(sensorId)
			query = "SELECT COUNT(1) FROM sensors WHERE sensor_id=\'"+sensorId+"\'"
			cur.execute(query)
			db.commit()
			if cur.fetchone()[0]==0:
				print "Adding "+sensorId+" to sensors"
				query = "INSERT INTO sensors(sensor_id, network_id, ontology_id) VALUES(\'"+sensorId+"\',\'"+netId+"\',\'"+sensorOnt+"\')"
				cur.execute(query)
				db.commit()

		print "id: "+sensorId+" || timestamp: "+str(sensorTime)+" || data: "+str(sensorData)

		query = "INSERT INTO data(sensor_id, timestamp, reading) VALUES(\'"+sensorId+"\',\'"+str(sensorTime)+"\',\'"+str(sensorData)+"\')"
		cur.execute(query)
		db.commit()



		time.sleep(1)

exit()


