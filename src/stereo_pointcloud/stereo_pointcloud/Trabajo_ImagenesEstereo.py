#!/usr/bin/env python3
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import struct
import time
from rclpy.clock import Clock
import pandas as pd
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


# ===============================
#  CONFIGURACIÓN GENERAL
# ===============================

# Par de imágenes estéreo 
left_path  = "/ros2_ws/calibration_images/left/left0196.png"
right_path = "/ros2_ws/calibration_images/right/right0196.png"

# Parámetros obtenidos en la calibración
K1 = np.array([[459.9506992155325, 0., 357.41440556012645],
               [0., 459.1113148843603, 241.25736250438041],
               [0., 0., 1.]])
D1 = np.array([ -0.3092079114390503, 0.12457048941415867, 0.00068263441107457043, 0.00020682357351609729, -0.02597872987735567])
K2 = np.array([[459.21421257351994, 0., 372.43603176763759],
               [0., 458.27227075186613, 255.01747600476853],
               [0., 0., 1.]])
D2 = np.array([-0.29488588129416304,0.10237052484622675,-0.00017189541181734664,-8.0163248365025416e-05, -0.016788918064821794])
R = np.array([[0.99999600589540949, 0.000243000389779664, -0.0028158735835643305],
               [-0.00024314852905690045, 0.99999996907350663, -5.2266420122372733e-05],
               [0.0028158607957187729, 5.2950886884679242e-05, 0.9999960340542269]])
T = np.array([[ -0.079005444761042987], 
              [-0.0011780653874839397],
              [0.00049433644680386111]]) 

# ===============================
#  FUNCIONES
# ===============================

def rectificar_imagenes(imgL, imgR, K1, D1, K2, D2, R, T):
    """Rectifica las imágenes estéreo utilizando los parámetros de calibración."""
    image_size = imgL.shape[::-1]

    T = np.array(T).reshape(3, 1)

    R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(
        K1, D1, K2, D2, image_size, R, T, alpha=0
    )

    map1x, map1y = cv2.initUndistortRectifyMap(K1, D1, R1, P1, image_size, cv2.CV_32FC1)
    map2x, map2y = cv2.initUndistortRectifyMap(K2, D2, R2, P2, image_size, cv2.CV_32FC1)

    rectL = cv2.remap(imgL, map1x, map1y, cv2.INTER_LINEAR)
    rectR = cv2.remap(imgR, map2x, map2y, cv2.INTER_LINEAR)

    return rectL, rectR, Q


def calcular_mapa_disparidad(rectL, rectR, metodo="SGBM"):
    """Calcula y visualiza el mapa de disparidad entre dos imágenes rectificadas."""
    # Parámetros típicos de ventana
    min_disp = 0
    num_disp = 16 * 10  # múltiplo de 16
    block_size = 9

    if metodo == "BM":
        stereo = cv2.StereoBM_create(numDisparities=num_disp, blockSize=block_size)
    else:
        stereo = cv2.StereoSGBM_create(
            minDisparity=min_disp,
            numDisparities=num_disp,
            blockSize=block_size,
            P1=8 * 3 * block_size**2,
            P2=32 * 3 * block_size**2,
            disp12MaxDiff=1,
            uniquenessRatio=10,
            speckleWindowSize=100,
            speckleRange=32
        )

    print(" Calculando mapa de disparidad...")
    disparidad = stereo.compute(rectL, rectR).astype(np.float32) / 16.0

    # Normalizar para visualizar correctamente
    disp_vis = cv2.normalize(disparidad, None, 0, 255, cv2.NORM_MINMAX)
    disp_vis = np.uint8(disp_vis)

    #  Guardar ambas versiones (original y filtrada)
    cv2.imwrite("mapa_disparidad.png", disp_vis)  
    print(" Mapa de disparidad guardado como 'mapa_disparidad.png'")
    
    return disparidad



def reconstruccion_densa_desde_disparidad(disparidad, Q, img_color=None,
                                       min_disp=0.1, max_z=50.0, downsample=1):
    """
    Reproyecta mapa de disparidad a puntos 3D usando Q.
    - disparidad: ndarray float32 (disparidad en píxeles, p.ej. resultado/16.0)
    - Q: matriz 4x4 de stereoRectify
    - img_color: imagen BGR (opcionales) para tomar color de cada punto
    - min_disp: umbral mínimo de disparidad válido (>0)
    - max_z: filtrar puntos con Z>max_z (metros)
    - downsample: tomar 1 cada N puntos (para reducir densidad)
    Devuelve: puntos_3D Nx3 y colores Nx3 (uint8) o None si no hay colores.
    """
    # asegurar tipos
    disp = disparidad.astype(np.float32)
    Q = np.array(Q, dtype=np.float64)

    # reprojectar
    points_3d = cv2.reprojectImageTo3D(disp, Q)   # shape (H,W,3), float32/float64
    # máscara de disparidad válida
    mask_valid = (disp > min_disp) & np.isfinite(disp)

    # opcional: máscara por Z después de reproyección (evita puntos con Z negativa/inf)
    Z = points_3d[:, :, 2]
    mask_valid = mask_valid & (Z > 0) & (Z < max_z)

    # obtener coords y colores
    pts = points_3d[mask_valid]
    colors = None
    if img_color is not None:
        if len(img_color.shape) == 2:
            # gris -> convertir a BGR
            col_img = cv2.cvtColor(img_color, cv2.COLOR_GRAY2BGR)
        else:
            col_img = img_color
        colors = col_img[mask_valid]

    # downsample
    if downsample > 1 and pts.shape[0] > 0:
        pts = pts[::downsample]
        if colors is not None:
            colors = colors[::downsample]

    print(f"Reconstrucción densa: puntos válidos = {pts.shape[0]}")
    return pts.astype(np.float32), (colors.astype(np.uint8) if colors is not None else None)


def guardar_ply(filename, puntos, colores=None):
    """
    Guarda una nube Nx3 (y opcionalmente colores Nx3) en formato PLY ASCII.
    """
    n = puntos.shape[0]
    with open(filename, 'w') as f:
        f.write("ply\nformat ascii 1.0\n")
        f.write(f"element vertex {n}\n")
        f.write("property float x\nproperty float y\nproperty float z\n")
        if colores is not None:
            f.write("property uchar red\nproperty uchar green\nproperty uchar blue\n")
        f.write("end_header\n")
        if colores is None:
            for p in puntos:
                f.write(f"{p[0]} {p[1]} {p[2]}\n")
        else:
            for p, c in zip(puntos, colores):
                f.write(f"{p[0]} {p[1]} {p[2]} {int(c[2])} {int(c[1])} {int(c[0])}\n")
    print(f"Guardado PLY: {filename}")



def extraer_features(imgL, imgR):
    """Detecta y describe features usando ORB en ambas imágenes."""
    orb = cv2.ORB_create(
        nfeatures=1000,
        scaleFactor=1.2,
        nlevels=8,
        edgeThreshold=31,
        patchSize=31
    )

    kpL, desL = orb.detectAndCompute(imgL, None)
    kpR, desR = orb.detectAndCompute(imgR, None)

    print(f"Features izquierda: {len(kpL)}, derecha: {len(kpR)}")

    imgL_kp = cv2.drawKeypoints(imgL, kpL, None, color=(0, 255, 0), flags=0)
    imgR_kp = cv2.drawKeypoints(imgR, kpR, None, color=(0, 255, 0), flags=0)

    return imgL_kp, imgR_kp, kpL, kpR, desL, desR



def emparejar_features(des1, des2, kp1, kp2, rectL, rectR):
    """Empareja los descriptores entre ambas imágenes y muestra los resultados."""
    # Matcher de fuerza bruta con norma Hamming
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    matches = bf.match(des1, des2)

    # Ordenar por distancia (de menor a mayor)
    matches = sorted(matches, key=lambda x: x.distance)

    print(f"Total de matches encontrados: {len(matches)}")

    # Dibujar todos los matches
    img_matches_all = cv2.drawMatches(rectL, kp1, rectR, kp2, matches, None,
                                      flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)

    # Filtrar matches con distancia < 30
    good_matches = [m for m in matches if m.distance < 30]
    img_matches_good = cv2.drawMatches(rectL, kp1, rectR, kp2, good_matches, None,
                                       flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
    

    # Guardar para el informe
    cv2.imwrite("totalidad_matches.png", img_matches_all)
    cv2.imwrite("matches_cercanos.png", img_matches_good)
    print(" Matches guardados en 'totalidad_matches.png' y 'matches_cercanos.png'")
    
    return matches, good_matches


def filtrar_correspondencias_ransac(kp1, kp2, matches, rectL, rectR):
    """Filtra matches espurios usando RANSAC y calcula la homografía."""
    if len(matches) < 4:
        print("No hay suficientes matches para calcular homografía.")
        return None, matches

    # Convertir a puntos (Nx2)
    pts1 = np.float32([kp1[m.queryIdx].pt for m in matches])
    pts2 = np.float32([kp2[m.trainIdx].pt for m in matches])

    # Estimar homografía con RANSAC
    H, mask = cv2.findHomography(pts1, pts2, cv2.RANSAC, 5.0)
    if H is None:
        print("No se pudo calcular la homografía.")
        return None, matches

    inliers = mask.ravel().astype(bool)
    num_inliers = np.sum(inliers)
    print(f"RANSAC: {num_inliers}/{len(matches)} inliers ({100*num_inliers/len(matches):.1f}%)")

    # Filtrar matches válidos
    matches_filtrados = [m for i, m in enumerate(matches) if inliers[i]]

    # Guardar visualizaciones 
    img_matches_ransac = cv2.drawMatches(rectL, kp1, rectR, kp2, matches_filtrados, None,
                                         flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
    cv2.imwrite("matches_filtrados_ransac.png", img_matches_ransac)

    # Puntos transformados
    ptsL = np.float32([kp1[m.queryIdx].pt for m in matches_filtrados]).reshape(-1, 1, 2)
    ptsL_trans = cv2.perspectiveTransform(ptsL, H)

    img_trans = rectR.copy()
    for p in ptsL_trans:
        cv2.circle(img_trans, tuple(np.int32(p[0])), 3, (0, 0, 255), -1)

    cv2.imwrite("puntos_transformados_ransac.png", img_trans)

    print("Guardadas: matches_filtrados_ransac.png y puntos_transformados_ransac.png")

    return H, matches_filtrados



def triangular_puntos_3D(K1, D1, K2, D2, R, T, kp1, kp2, matches):
    """Triangula puntos 3D a partir de correspondencias estéreo. Devuelve Nx3 numpy array."""
    if len(matches) == 0:
        print("No hay matches para triangular.")
        return np.zeros((0,3), dtype=np.float32)

    # Asegurar shapes
    K1 = np.array(K1, dtype=np.float64).reshape(3,3)
    K2 = np.array(K2, dtype=np.float64).reshape(3,3)
    R  = np.array(R, dtype=np.float64).reshape(3,3)
    T  = np.array(T, dtype=np.float64).reshape(3,1)

    # Matrices de proyección 3x4
    P1 = K1 @ np.hstack((np.eye(3), np.zeros((3,1))))
    P2 = K2 @ np.hstack((R, T))                             # cámara derecha

    # Extraer los puntos correspondientes
    pts1 = np.float32([kp1[m.queryIdx].pt for m in matches]).T  # (2, N)
    pts2 = np.float32([kp2[m.trainIdx].pt for m in matches]).T  # (2, N)

    # Triangulación
    points_3D_hom = cv2.triangulatePoints(P1, P2, pts1, pts2)

    # Convertir de coordenadas homogéneas a cartesianas
    points_3D = cv2.convertPointsFromHomogeneous(points_3D_hom.T)
    points_3D = points_3D.reshape(-1, 3)  # Asegura forma Nx3

    print(f"Se generaron {len(points_3D)} puntos 3D")

    # guardado
    np.savetxt("nube_puntos_3D.txt", points_3D, fmt="%.6f")
    print("Guardado: nube_puntos_3D.txt")

    return points_3D



def puntos_a_pointcloud2(points, frame_id="world"):
    """Convierte Nx3 numpy array en sensor_msgs/PointCloud2."""
    msg = PointCloud2()
    msg.header.frame_id = frame_id
    msg.height = 1
    msg.width = points.shape[0]
    msg.is_dense = False
    msg.is_bigendian = False
    msg.point_step = 12  # 3 floats * 4 bytes
    msg.row_step = msg.point_step * points.shape[0]

    msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
    ]

    # Empaquetar datos
    buffer = []
    for p in points:
        buffer.append(struct.pack('fff', float(p[0]), float(p[1]), float(p[2])))
    msg.data = b"".join(buffer)

    msg.header.stamp = Clock().now().to_msg()
    return msg


class PointCloudPublisher(Node):
    def __init__(self, topic_name, points):
        super().__init__(f"{topic_name.replace('/', '_')}_pub")
        self.pub = self.create_publisher(PointCloud2, topic_name, 10)
        self.get_logger().info(f"Nodo '{self.get_name()}' publicando en '{topic_name}'")

    def publish_once(self, points):
        """Publica una nube de puntos una vez."""
        msg = puntos_a_pointcloud2(points, frame_id="world")
        self.pub.publish(msg)



def publicar_dos_nubes_ros2(puntos_original, puntos_filtrados, duracion_s=30.0):
    import time
    rclpy.init()

    node_orig = PointCloudPublisher("stereo/pointcloud_original", puntos_original)
    node_filt = PointCloudPublisher("stereo/pointcloud_filtrada", puntos_filtrados)

    start = time.time()
    try:
        while time.time() - start < duracion_s and rclpy.ok():
            # publicar ambas nubes una tras otra
            node_orig.publish_once(puntos_original)
            node_filt.publish_once(puntos_filtrados)
            rclpy.spin_once(node_orig, timeout_sec=0.05)
            rclpy.spin_once(node_filt, timeout_sec=0.05)
            time.sleep(0.1)
    except KeyboardInterrupt:
        print(" Publicación interrumpida por el usuario.")
    finally:
        node_orig.destroy_node()
        node_filt.destroy_node()
        rclpy.shutdown()


def publicar_nube_ros2(puntos_3D, timeout_s=10.0):
    """Publica una nube de puntos 3D en ROS2 por unos segundos."""
    import time
    rclpy.init()
    node = PointCloudPublisher("stereo/pointcloud_densa", puntos_3D)
    node.get_logger().info("Publicando nube densa en /stereo/pointcloud_densa...")

    start = time.time()
    try:
        while time.time() - start < timeout_s:
            node.publish_once(puntos_3D)
            rclpy.spin_once(node, timeout_sec=0.1)
            time.sleep(0.1)
    finally:
        node.get_logger().info("Publicación de nube densa finalizada.")
        node.destroy_node()
        rclpy.shutdown()



def publicar_trayectoria_ros2(archivo_csv, timeout_s=20.0):

    rclpy.init()
    node = Node("groundtruth_path_pub")
    pub = node.create_publisher(Path, "/groundtruth_path", 10)

    # --- Leer CSV saltando la línea de encabezado comentada ---
    columnas = [
        "timestamp", "p_RS_R_x [m]", "p_RS_R_y [m]", "p_RS_R_z [m]",
        "q_RS_w []", "q_RS_x []", "q_RS_y []", "q_RS_z []",
        "v_RS_R_x [m s^-1]", "v_RS_R_y [m s^-1]", "v_RS_R_z [m s^-1]",
        "b_w_RS_S_x [rad s^-1]", "b_w_RS_S_y [rad s^-1]", "b_w_RS_S_z [rad s^-1]",
        "b_a_RS_S_x [m s^-2]", "b_a_RS_S_y [m s^-2]", "b_a_RS_S_z [m s^-2]"
    ]

    try:
        df = pd.read_csv(archivo_csv, skiprows=1, names=columnas)
    except Exception as e:
        node.get_logger().error(f"No se pudo leer el CSV: {e}")
        rclpy.shutdown()
        return

    # --- Verificar columnas necesarias ---
    requeridas = ['p_RS_R_x [m]', 'p_RS_R_y [m]', 'p_RS_R_z [m]']
    if not all(col in df.columns for col in requeridas):
        node.get_logger().error(f"Columnas no encontradas en el CSV. Detectadas: {df.columns}")
        node.destroy_node()
        rclpy.shutdown()
        return

    df = df[requeridas].dropna()

    # --- Construir el mensaje Path ---
    path_msg = Path()
    path_msg.header.frame_id = "world"

    for _, row in df.iterrows():
        pose = PoseStamped()
        pose.header.frame_id = "world"
        pose.pose.position.x = float(row['p_RS_R_x [m]'])
        pose.pose.position.y = float(row['p_RS_R_y [m]'])
        pose.pose.position.z = float(row['p_RS_R_z [m]'])
        path_msg.poses.append(pose)

    node.get_logger().info(f"Publicando trayectoria con {len(path_msg.poses)} poses en /groundtruth_path...")

    start = time.time()
    while time.time() - start < timeout_s:
        pub.publish(path_msg)
        rclpy.spin_once(node, timeout_sec=0.1)
        time.sleep(0.1)

    node.get_logger().info(" Trayectoria publicada correctamente.")
    node.destroy_node()
    rclpy.shutdown()



def estimar_pose_monocular(K, kpL, kpR, matches, baseline=0.54):
    """Estima la rotación y traslación entre cámaras usando visión monocular."""
    if len(matches) < 8:
        print("No hay suficientes matches para estimar la pose.")
        return None, None

    # Obtener puntos emparejados
    pts1 = np.float32([kpL[m.queryIdx].pt for m in matches])
    pts2 = np.float32([kpR[m.trainIdx].pt for m in matches])

    # Calcular la matriz esencial
    E, mask = cv2.findEssentialMat(pts1, pts2, K, method=cv2.RANSAC, prob=0.999, threshold=1.0)
    print(f"Matriz Esencial:\n{E}")

    # Recuperar la pose relativa (R, t)
    _, R_est, t_est, mask_pose = cv2.recoverPose(E, pts1, pts2, K)
    print(f"Rotación estimada:\n{R_est}")
    print(f"Traslación unitaria estimada:\n{t_est.T}")

    # Escalar la traslación con el baseline
    t_est = t_est * baseline
    print(f"Traslación escalada (baseline={baseline} m):\n{t_est.T}")

    return R_est, t_est



def publicar_poses_ros2(R_est, t_est, timeout_s=20.0):
    """
    Publica dos poses en RViz:
    - camera_left en el origen (frame_id = 'world')
    - camera_right_est posicionada con la rotación y traslación estimada
    """
    import time
    from geometry_msgs.msg import PoseStamped, Quaternion
    from transforms3d.quaternions import mat2quat as quaternion_from_matrix

    rclpy.init()
    node = Node("poses_camaras_pub")
    pub_left = node.create_publisher(PoseStamped, "/camera_left_pose", 10)
    pub_right = node.create_publisher(PoseStamped, "/camera_right_est_pose", 10)

    # Pose de la cámara izquierda (origen)
    pose_left = PoseStamped()
    pose_left.header.frame_id = "world"
    pose_left.pose.position.x = 0.0
    pose_left.pose.position.y = 0.0
    pose_left.pose.position.z = 0.0
    pose_left.pose.orientation.w = 1.0

    # Pose de la cámara derecha estimada
    pose_right = PoseStamped()
    pose_right.header.frame_id = "world"
    pose_right.pose.position.x = float(t_est[0])
    pose_right.pose.position.y = float(t_est[1])
    pose_right.pose.position.z = float(t_est[2])

    # Convertir R_est a cuaternión
    rot4x4 = np.eye(4)
    rot4x4[:3, :3] = R_est
    q = quaternion_from_matrix(rot4x4)
    pose_right.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

    node.get_logger().info("Publicando poses de cámaras en RViz...")

    start = time.time()
    while time.time() - start < timeout_s:
        now = node.get_clock().now().to_msg()
        pose_left.header.stamp = now
        pose_right.header.stamp = now
        pub_left.publish(pose_left)
        pub_right.publish(pose_right)
        rclpy.spin_once(node, timeout_sec=0.1)
        time.sleep(0.1)

    node.get_logger().info("Publicación finalizada.")
    node.destroy_node()
    rclpy.shutdown()




# ===============================
# 🚀 PROGRAMA PRINCIPAL
# ===============================

def main():
    # Leer imágenes originales
    imgL = cv2.imread(left_path, cv2.IMREAD_GRAYSCALE)
    imgR = cv2.imread(right_path, cv2.IMREAD_GRAYSCALE)

    if imgL is None or imgR is None:
        print("Error: no se pudieron cargar las imágenes.")
        return

    # Rectificación
    rectL, rectR, Q = rectificar_imagenes(imgL, imgR, K1, D1, K2, D2, R, T)

    # --- Calcular y guardar el mapa de disparidad ---
    disparidad = calcular_mapa_disparidad(rectL, rectR, metodo="SGBM")

    # reconstrucción densa
    pts_densa, cols_densa = reconstruccion_densa_desde_disparidad(disparidad, Q, img_color=rectL, min_disp=0.5, max_z=40.0,   downsample=4)

    # guardar y publicar
    if pts_densa.shape[0] > 0:
        guardar_ply("nube_densa.ply", pts_densa, cols_densa)
        publicar_nube_ros2(pts_densa, timeout_s=10.0)   # publica en /stereo/pointcloud_densa por ejemplo

    # Extracción de features
    rectL_kp, rectR_kp, kpL, kpR, desL, desR = extraer_features(rectL, rectR)

    # Mostrar features extraidos
    vis = np.hstack((rectL_kp, rectR_kp))
    cv2.imwrite("Features_Extraidos_izquierda.png", rectL_kp)
    cv2.imwrite("Features_Extraidos_derecha.png", rectR_kp)
    print(" Imagenes guardadas: Features_Extraidos_izquierda.png y Features_Extraidos_derecha.png")


    # Emparejar features -> devuelve matches y good_matches
    matches, good_matches = emparejar_features(desL, desR, kpL, kpR, rectL_kp, rectR_kp)


    # --- Triangulamos con good_matches ---
    print(f"Triangulando con {len(good_matches)} matches...")
    puntos_3D_original = triangular_puntos_3D(K1, D1, K2, D2, R, T, kpL, kpR, good_matches)


    # --- Aplicar RANSAC ---
    H, matches_filtrados = filtrar_correspondencias_ransac(kpL, kpR, good_matches, rectL, rectR)

    # --- Triangulamos usando solo los matches filtrados ---
    print(f"Triangulando con {len(matches_filtrados)} matches filtrados por RANSAC...")
    puntos_3D_filtrados = triangular_puntos_3D(K1, D1, K2, D2, R, T, kpL, kpR, matches_filtrados)

    # --- Transformar la nube al marco 'world' ---
    puntos_3D_original_world  = (R @ puntos_3D_original.T  + T.reshape(3, 1)).T
    puntos_3D_filtrados_world = (R @ puntos_3D_filtrados.T + T.reshape(3, 1)).T
    
    # --- Publicar ambos en ROS2 simultáneamente ---
    if len(puntos_3D_original) > 0 and len(puntos_3D_filtrados) > 0:
        publicar_dos_nubes_ros2(puntos_3D_original_world, puntos_3D_filtrados_world)
    else:
        print("No hay puntos 3D para publicar.")


    # --- Publicar la trayectoria ground-truth ---
    csv_path = "/ros2_ws/MH_01_easy/mav0/state_groundtruth_estimate0/data.csv"
    publicar_trayectoria_ros2(csv_path, timeout_s=10.0)


    # --- Estimar Pose entre cámaras ---
    print("\n🔹 Estimando pose monocular entre cámaras...")
    R_est, t_est = estimar_pose_monocular(K1, kpL, kpR, good_matches, baseline=0.54)

    if R_est is not None and t_est is not None:
        print(" Pose estimada correctamente.")
        # Visual simple en consola
        print("R_est:\n", R_est)
        print("t_est (m):\n", t_est)

    # --- Publicar las poses en RViz ---
    if R_est is not None and t_est is not None:
        publicar_poses_ros2(R_est, t_est, timeout_s=20.0)



if __name__ == "__main__":
    main()

