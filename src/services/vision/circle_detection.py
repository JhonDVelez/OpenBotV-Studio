"""
Módulo de detección de esferas de color mediante segmentación HSV.

Función plana ``run_circle_detection`` que procesa un frame y detecta
las esferas más grandes de cada color configurado, devolviendo su
centro, radio y elipse ajustada. Se encola en un QThreadPool mediante
``QRunnable.create(run_circle_detection, ...)`` (ver camera_worker).

Conexiones:
    - Ejecutado por un QThreadPool.
    - Reporta resultados a traves de `detection_callback`.
    - Reporta errores a traves de `error_callback`.
"""

import numpy as np
import cv2

# Rangos HSV por defecto para cada color.
COLORES = {
    "amarillo": (20, 100, 100, 30, 255, 255),
    "verde":    (40, 70, 70, 80, 255, 255),
    "azul":     (100, 150, 50, 130, 255, 255),
    "naranja":  (5, 150, 150, 15, 255, 255),
    "morado":   (130, 94, 117, 180, 255, 255),
}


def run_circle_detection(frame_umat: cv2.UMat, frame_id: int,
                         roi: np.ndarray | None, hsv_colors: dict | None,
                         detection_callback, error_callback) -> None:
    """Detecta la esfera más grande de cada color en el frame.

    Aplica máscara de ROI si está definida, convierte a HSV,
    segmenta por rango de color, aplica morfología, encuentra
    contornos y calcula propiedades geométricas.

    Args:
        frame_umat (cv2.UMat): Frame como UMat para procesamiento OpenCL.
        frame_id (int): Identificador único del frame.
        roi (np.ndarray): Polígono de región de interés (máscara).
        hsv_colors (dict): Rangos HSV personalizados o None para usar
            los predeterminados (``COLORES``).
        detection_callback (callable): Función para reportar resultados.
        error_callback (callable): Función para reportar errores.

    Callback:
        dict con forma:
        {
            "amarillo": {"circle": ..., "center": (x, y), "radius": float},
            ...
        }
        Solo incluye colores encontrados.
    """
    try:
        if roi is not None:
            mask = cv2.cvtColor(cv2.multiply(
                frame_umat, 0), cv2.COLOR_BGR2GRAY)
            roi_umat = cv2.UMat(roi.astype('int32'))
            cv2.drawContours(mask, [roi_umat], 0,
                             (255, 255, 255, 1), thickness=-1)
            masked_frame = cv2.bitwise_and(
                frame_umat, frame_umat, mask=mask)
        else:
            masked_frame = frame_umat
        hsv = cv2.cvtColor(masked_frame, cv2.COLOR_BGR2HSV)

        resultados = {}

        for nombre_color, (hmin, smin, vmin, hmax, smax, vmax) in (
                hsv_colors or COLORES).items():
            lower = cv2.UMat(np.array([hmin, smin, vmin], dtype="uint8"))
            upper = cv2.UMat(np.array([hmax, smax, vmax], dtype="uint8"))
            mask = cv2.inRange(hsv, lower, upper, None)

            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )

            if contours:
                largest_contour = max(contours, key=cv2.contourArea)
                area = cv2.contourArea(largest_contour)

                if area > 500:
                    moments = cv2.moments(largest_contour)
                    if abs(moments["m00"]) < 1e-9:
                        continue

                    center = (
                        moments["m10"] / moments["m00"],
                        moments["m01"] / moments["m00"],
                    )
                    _, enclosing_radius = cv2.minEnclosingCircle(
                        largest_contour)

                    circle = None
                    if len(largest_contour.get()) >= 5:
                        circle = cv2.fitEllipse(largest_contour)

                    resultados[nombre_color] = {
                        "circle": circle,
                        "center": center,
                        "radius": float(enclosing_radius),
                    }
        detection_callback(
            frame_id, resultados if resultados else None)
    except (cv2.error, ValueError, AttributeError) as e:
        error_callback(
            f"Error al detectar esfera: {type(e).__name__}: {e} (run_circle_detection)")
