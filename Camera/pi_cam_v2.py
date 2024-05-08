import time


def image(camera):
    camera.start()
    time.sleep(1)
    image = camera.capture_image("main")

    return image 


def save_imgs(camera):
    for _ in range(10):
        # Capture one image with the default configurations.
        camera.start_and_capture_file("test.jpg")