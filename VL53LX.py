#!/usr/bin/env python
import VL53L1X  

# Функция измерения расстояния
def ReadIR(IR):
    distance_cm = IR.get_distance() / 10
    print("Distance: {}сm".format(distance_cm))

    return distance_cm


# Тестим
# Создание экземпляра дальномера VL53L1X
IR = VL53L1X.VL53L1X(i2c_bus=1, i2c_address=0x29)
IR.open()

# Время измерения в микросекундах и время между измерениями в миллисекундах.
IR.start_ranging(0)
IR.set_timing(66000, 70)

# Начало измерений
IR.start_ranging(1)  # Начало измерений
                      # 0 = Неизменно
                      # 1 = Короткий диапазон
                      # 2 = Средний диапазон
                      # 3 = Длинный диапазон

dist = ReadIR(IR)
IR.stop_ranging()
