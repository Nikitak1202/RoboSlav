import time
import cv2
import numpy as np
from picamera2 import Picamera2


def take_photo(camera, filename):
    # Задержка для стабилизации камеры
	time.sleep(2)
    
    # Захватываем кадр с камеры
	frame = camera.capture_image("main")
	frame = np.array(frame)
    # Сохраняем кадр в файл
	cv2.imwrite(filename, frame)


def capture_frame(camera):
    # Задержка для стабилизации камеры
	time.sleep(2)
    # Захватываем кадр с камеры
	frame = camera.capture_image("main")
	frame = cv2.cvtColor(np.array(frame), cv2.COLOR_RGB2BGR)

	return frame


# CALIBRATION
# Create an instance of the camera
picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"size": (1920, 1080)}))
picam2.start()

photo_counter = 1
while True:
    # Generate a unique filename for each photo
    filename = f"photo_{photo_counter}.jpg"
    
    cv2.imshow(filename, capture_frame(picam2))
    key = cv2.waitKey(0)
    cv2.destroyAllWindows()
    
    # 13 is the ASCII code for Enter, 32 for Space
    if key == 13:
        take_photo(picam2, filename)
        print(f"Photo {photo_counter} saved as '{filename}'")
        photo_counter += 1
	
    elif key == 32:
        continue
	
    else:
        break

    # Wait for 5 seconds before capturing the next photo
    time.sleep(5)

# Close the camera
picam2.close()
