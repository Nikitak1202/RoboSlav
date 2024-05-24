import cv2  
import numpy as np  
import glob  # Используется для получения файлов с определенным шаблоном


# Размеры шахматной доски
number_of_squares_X = 10  # Количество квадратов шахматной доски по оси X
number_of_squares_Y = 7   # Количество квадратов шахматной доски по оси Y
nX = number_of_squares_X - 1  # Количество внутренних углов по оси X
nY = number_of_squares_Y - 1  # Количество внутренних углов по оси Y
square_size = 0.0017  # Размер стороны квадрата в метрах

# Установка критериев завершения. Процесс завершается при достижении заданной точности
# или при выполнении определенного количества итераций.
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Определение координат точек в трехмерной системе координат
object_points_3D = np.zeros((nX * nY, 3), np.float32)

# Координаты X и Y
object_points_3D[:, :2] = np.mgrid[0:nY, 0:nX].T.reshape(-1, 2)

object_points_3D = object_points_3D * square_size

# Векторы трехмерных точек для всех изображений шахматной доски (в мировой системе координат)
object_points = []

# Векторы двумерных точек для всех изображений шахматной доски (в системе координат камеры)
image_points = []


def main():
    # Получение пути к файлам изображений в текущем каталоге
    images = glob.glob('*.jpg')

    # Обработка каждого изображения с шахматной доской поочередно
    for image_file in images:

        # Загрузка изображения
        image = cv2.imread(image_file)

        # Преобразование изображения в оттенки серого
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Поиск углов на шахматной доске
        success, corners = cv2.findChessboardCorners(gray, (nY, nX), None)

        # Если углы найдены, рисуем их
        if success == True:

            # Добавляем трехмерные точки
            object_points.append(object_points_3D)

            # Находим более точные координаты углов
            corners_2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

            # Добавляем двумерные точки
            image_points.append(corners_2)

            # Рисуем углы на изображении
            cv2.drawChessboardCorners(image, (nY, nX), corners_2, success)

            # Отображаем изображение. Используется для тестирования.
            cv2.imshow("Image", image)

            # Отображаем изображение на некоторое время. Используется для тестирования.
            cv2.waitKey(1000)

    # Выполнение калибровки камеры для получения матрицы камеры, коэффициентов искажения и т. д.
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(object_points,
                                                        image_points,
                                                        gray.shape[::-1],
                                                        None,
                                                        None)

    # Сохранение параметров в файл
    cv_file = cv2.FileStorage('calibration_chessboard.yaml', cv2.FILE_STORAGE_WRITE)
    cv_file.write('K', mtx)
    cv_file.write('D', dist)
    cv_file.release()

    # Загрузка параметров из сохраненного файла
    cv_file = cv2.FileStorage('calibration_chessboard.yaml', cv2.FILE_STORAGE_READ)
    mtx = cv_file.getNode('K').mat()
    dst = cv_file.getNode('D').mat()
    cv_file.release()

    # Отображение ключевых параметров калибровки камеры
    print("Матрица камеры:")
    print(mtx)

    print("\nКоэффициенты искажения:")
    print(dist)

    # Закрытие всех окон
    cv2.destroyAllWindows()


if __name__ == '__main__':
    print(__doc__)
    main()
