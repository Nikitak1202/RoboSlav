import pynmea2
import serial

SERIAL_PORT = "/dev/serial0"
running = True

def getPositionData(gps):
	data = gps.readline()
	message = data[0:6]
	return data

def parse_gps_data(data):
	lines = data.splitlines()
	for line in lines:
		line = line.decode('utf-8')
		if line.startswith('$GNRMC') or line.startswith('$GNGGA'):
			try:
				#print('inside try')
				msg = pynmea2.parse(line)
				if hasattr(msg, 'latitude') and hasattr(msg, 'longitude'):
					print(f'Latitude: {msg.latitude} {msg.lat_dir}, Longitude: {msg.longitude} {msg.lon_dir}')
				#print(line)
				return msg.latitude,msg.lat_dir, msg.longitude, msg.lon_dir 
			except pynmea2.ParseError:
				print('parse error')
				pass


# Пример данных
gps = serial.Serial(SERIAL_PORT, baudrate = 9600, timeout = 1)
while running:
# Вызов функции парсинга
	parse_gps_data(getPositionData(gps))

