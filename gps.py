
import serial

SERIAL_PORT = "/dev/serial0"
running = True

# In the NMEA message, the position gets transmitted as:
# DDMM.MMMMM, where DD denotes the degrees and MM.MMMMM denotes the minutes
# DD.MMMM. This method converts a transmitted string to the desired format
def formatDegreesMinutes(coordinates, digits):
	parts = coordinates.split(".")

	if (len(parts) != 2):
		return coordinates
	if (digits > 3 or digits < 2):
        	return coordinates
    
	left = parts[0]
	right = parts[1]
	degrees = str(left[:digits])
	minutes = str(right[:3])
	return degrees + "." + minutes

def parse_nmea(data):
    if not data.startswith(b'$'):
        return "Not a valid NMEA sentence"

    try:
        fields = data.decode('ascii').strip().split(',')
        sentence_type = fields[0][3:6]

        if sentence_type == "GGA":
            time = fields[1]
            latitude = fields[2] + " " + fields[3] if fields[2] else "No fix"
            longitude = fields[4] + " " + fields[5] if fields[4] else "No fix"
            fix_quality = fields[6]
            return {
                "type": "GGA",
                "time": time[:2] + ":" + time[2:4] + ":" + time[4:],
                "latitude": latitude,
                "longitude": longitude,
                "fix_quality": fix_quality
            }

        elif sentence_type == "RMC":
            time = fields[1]
            status = fields[2]
            date = fields[9]
            if status != 'A':
                return f"RMC sentence indicates no valid fix: Status {status}"
            return {
                "type": "RMC",
                "time": time[:2] + ":" + time[2:4] + ":" + time[4:],
                "status": status,
                "date": "20" + date[4:] + "-" + date[2:4] + "-" + date[:2]
            }

        else:
            return "Unsupported NMEA type"

    except (IndexError, ValueError) as e:
        return f"Error parsing NMEA data: {e}"

# Additional logic could be added to recheck for data if no valid fix is obtained
# This method reads the data from the serial port, the GPS dongle is attached to, and then parses the NMEA messages it 
# transmits. gps is the serial port, that's used to communicate with the GPS adapter
def getPositionData(gps):
	data = gps.readline()
	message = data[0:6]
	print(parse_nmea(data))
        # GPRMC = Recommended minimum specific GPS/Transit data
        # Reading the GPS fix data is an alternative approach that also works
	#longitude = formatDegreesMinutes(parts[5], 3)
	#latitude = formatDegreesMinutes(parts[3], 2)
	#print("Your position: lon = " + str(longitude) + ", lat = " + str(latitude))

        # Handle other NMEA messages and unsupp
print("Application started!")
gps = serial.Serial(SERIAL_PORT, baudrate = 9600, timeout = 1)

while running:
	try:
		getPositionData(gps)
	except KeyboardInterrupt:
		running = False
		gps.close()
		print("Application closed!")
