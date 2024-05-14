import time
import cv2
from picamera2 import Picamera2


def take_photo(camera, filename):
    # Задержка для стабилизации камеры
    time.sleep(2)
    
    # Захватываем кадр с камеры
    frame = camera.capture_image("main")
    
    # Сохраняем кадр в файл
    cv2.imwrite(filename, frame)


def capture_frame(camera):
    # Задержка для стабилизации камеры
    time.sleep(2)
    # Захватываем кадр с камеры
    frame = camera.capture_image("main")

    return frame


# ТЕСТИМ
# Создаем экземпляр камеры
picam2 = Picamera2()
picam2.start()

while True:
    # Захватываем кадр с камеры
    frame = capture_frame(picam2)

    # Отображаем кадр
    cv2.imshow("Camera Stream", frame)

    # Ожидание нажатия клавиши 'p' для съемки фотографии
    if cv2.waitKey(1) & 0xFF == ord('p'):
        take_photo(picam2, "photo.jpg")
        print("Фотография сохранена как 'photo.jpg'")
        
    # Ожидание нажатия клавиши 'q' для выхода из цикла
    elif cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Закрываем окно вывода
cv2.destroyAllWindows()

# Закрываем камеру
picam2.close()