#!/usr/bin/env python
import serial


# Функция для преобразования координат в формат DD.MMMM
def formatDegreesMinutes(coordinates, digits):
    # Разбиваем координаты на градусы и минуты
    parts = coordinates.split(".")

    # Проверяем корректность формата координат
    if (len(parts) != 2):
        return coordinates
    # Проверяем, что значение digits находится в допустимом диапазоне
    if (digits > 3 or digits < 2):
        return coordinates
    
    # Получаем градусы и минуты
    left = parts[0]
    right = parts[1]
    degrees = str(left[:digits])
    minutes = str(right[:3])

    # Возвращаем преобразованные координаты
    return degrees + "." + minutes


# Функция для чтения данных GPS и вывода координат
def getPositionData(gps):
    # Читаем данные с последовательного порта
    data = gps.readline()
    # Извлекаем тип сообщения
    message = data[0:6]

    if (message == "$GPRMC"):
        # GPRMC - рекомендуемые минимальные данные GPS/Транзита
        # Проверяем наличие фиксации GPS
        parts = data.split(",")

        if parts[2] == 'V':
            print("Предупреждение: GPS-приемник не зафиксировал спутники")
            
            return 0

        else:
            # Извлекаем координаты из сообщения GPRMC
            longitude = formatDegreesMinutes(parts[5], 3)
            latitude = formatDegreesMinutes(parts[3], 2)
            print("Ваше местоположение: долгота = " + str(longitude) + ", широта = " + str(latitude))

            return longitude, latitude

    else:
        # Обработка других сообщений NMEA и неподдерживаемых строк
        pass

    return 0


# ТЕСТИМ
# Порт, к которому подключено GPS-устройство
SERIAL_PORT = "/dev/serial0"
# Открываем последовательный порт для связи с GPS-устройством
gps = serial.Serial(SERIAL_PORT, baudrate = 9600, timeout = 0.5)

running = True # Флаг, определяющий работу программы
while running:
    try:
        # Получаем и выводим координаты
        longitude, latitude = getPositionData(gps)

    except KeyboardInterrupt:
        # Обработка прерывания пользователем (Ctrl+C)
        running = False
        # Закрываем соединение с последовательным портом
        gps.close()
        print("Приложение закрыто!")
        
    except:
        # Обработка других ошибок
        print("Ошибка в приложении!")