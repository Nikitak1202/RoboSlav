#!/usr/bin/env python
import RPi.GPIO as GPIO  
import time  


# Функция измерения расстояния
def ReadUS(TRIG, ECHO):  
    # Отправляем короткий импульс на пин TRIG для запуска измерения
    GPIO.output(TRIG, True)  
    time.sleep(0.00001)  
    GPIO.output(TRIG, False)  

    # Запускаем таймер для измерения времени прохождения сигнала до приемника
    while GPIO.input(ECHO) == 0:  
        pulse_start = time.time()  

    # Как только сигнал придет, останавливаем таймер
    while GPIO.input(ECHO) == 1:  
        pulse_end = time.time()  

    # Вычисляем длительность импульса
    pulse_duration = pulse_end - pulse_start  

    # Рассчитываем расстояние на основе времени прохождения звукового сигнала
    distance = pulse_duration * 17150  # Множим на скорость звука в см/с
    distance = round(distance, 2)  

    return distance  # Возвращаем расстояние в см

# Тест
GPIO.setmode(GPIO.BCM)  
GPIO.setwarnings(False)  

# Определение пинов для каждого датчика
TRIG0 = 23
ECHO0 = 12

TRIG1 = 24
ECHO1 = 16

#TRIG2 = 25
#ECHO2 = 20

#TRIG3 = 8
#ECHO3 = 21

# Настройка пинов GPIO как входов и выходов
GPIO.setup(TRIG0, GPIO.OUT)
GPIO.setup(ECHO0, GPIO.IN)

GPIO.setup(TRIG1, GPIO.OUT)
GPIO.setup(ECHO1, GPIO.IN)

#GPIO.setup(TRIG2, GPIO.OUT)
#GPIO.setup(ECHO2, GPIO.IN)

#GPIO.setup(TRIG3, GPIO.OUT)
#GPIO.setup(ECHO3, GPIO.IN)

# Установка всех пинов TRIG в низкий уровень
GPIO.output(TRIG0, False)
GPIO.output(TRIG1, False)
#GPIO.output(TRIG2, False)
#GPIO.output(TRIG3, False)

# Измерение расстояния для каждого датчика
distance_1 = ReadUS(TRIG0, ECHO0)
print('Distance from sensor 1:', distance_1, 'cm')

distance_2 = ReadUS(TRIG1, ECHO1)
print('Distance from sensor 2:', distance_2, 'cm')

#distance_3 = ReadUS(TRIG2, ECHO2)
#print('Distance from sensor 3:', distance_3, 'cm')

#distance_4 = ReadUS(TRIG3, ECHO3)
#print('Distance from sensor 4:', distance_4, 'cm')
