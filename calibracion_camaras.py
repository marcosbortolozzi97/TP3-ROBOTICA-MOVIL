import cv2
import numpy as np
import glob

# Directorios con las imágenes de calibración
imagenes_izq = sorted(glob.glob('calibration_images/left/*.png'))
imagenes_der = sorted(glob.glob('calibration_images/right/*.png'))

# Definir el patrón del tablero (6x6 esquinas internas)
pattern_size = (6, 6)
tamaño_cuadro = 0.04  # metros
criterio = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Puntos 3D del patrón
objp = np.zeros((pattern_size[0]*pattern_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2)
objp *= tamaño_cuadro

# Listas para almacenar puntos
objpoints = []  # puntos 3D
imgpointsL, imgpointsR = [], []  # puntos 2D de izquierda y derecha

for imgL_path, imgR_path in zip(imagenes_izq, imagenes_der):
    imgL = cv2.imread(imgL_path)
    imgR = cv2.imread(imgR_path)
    grayL = cv2.cvtColor(imgL, cv2.COLOR_BGR2GRAY)
    grayR = cv2.cvtColor(imgR, cv2.COLOR_BGR2GRAY)

    retL, cornersL = cv2.findChessboardCorners(grayL, pattern_size, None)
    retR, cornersR = cv2.findChessboardCorners(grayR, pattern_size, None)

    if retL and retR:
        objpoints.append(objp)
        cornersL = cv2.cornerSubPix(grayL, cornersL, (11,11), (-1,-1), criterio)
        cornersR = cv2.cornerSubPix(grayR, cornersR, (11,11), (-1,-1), criterio)
        imgpointsL.append(cornersL)
        imgpointsR.append(cornersR)

# Calibrar cada cámara por separado
retL, K1, D1, _, _ = cv2.calibrateCamera(objpoints, imgpointsL, grayL.shape[::-1], None, None)
retR, K2, D2, _, _ = cv2.calibrateCamera(objpoints, imgpointsR, grayR.shape[::-1], None, None)

# Calibración estéreo
retval, K1, D1, K2, D2, R, T, E, F = cv2.stereoCalibrate(
    objpoints, imgpointsL, imgpointsR,
    K1, D1, K2, D2,
    grayL.shape[::-1],
    flags=cv2.CALIB_FIX_INTRINSIC
)

# Guardar resultados en archivo YAML
cv_file = cv2.FileStorage("stereo_calibration.yml", cv2.FILE_STORAGE_WRITE)
cv_file.write("K1", K1)
cv_file.write("D1", D1)
cv_file.write("K2", K2)
cv_file.write("D2", D2)
cv_file.write("R", R)
cv_file.write("T", T)
cv_file.release()

print("✅ Calibración estéreo completada. Resultados guardados en 'stereo_calibration.yml'")
