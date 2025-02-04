# VERSION UNA SOLA CAPTURA
import cv2
from ultralytics import YOLO
import sounddevice as sd
import numpy as np
import time
import pywhatkit as kit

# Load the YOLOv8 model
model = YOLO(r"C:/PYTHON/detector_vehiculos/policia_nacional/policia_nacional_50epocas.pt")  # load a custom model

def beep(frequency, duration):
    # Generar una señal sinusoidal con la frecuencia deseada
    samples = np.arange(int(duration * 44100))
    waveform = np.sin(2 * np.pi * frequency * samples / 44100)
    # Reproducir la señal utilizando sounddevice
    sd.play(waveform, samplerate=44100, blocking=True)


# Función para obtener la fecha y hora actual en el formato deseado
def obtener_fecha_hora():
    now = time.strftime("%d/%m/%Y - %H:%M:%S")
    return now


sonido_reproducido = False
captura_realizada = False
contador_captura = 0
numero_whatsapp = '+34677347408'
carpeta_fotogramas = 'C:/PYTHON/detector_vehiculos/policia_nacional/capturas'
cap = cv2.VideoCapture('C:/PYTHON/detector_vehiculos/policia_nacional/(1).mp4')
# cap = cv2.VideoCapture(2)

tiempo_inicio_deteccion = None
PERIODO_DETECCION = 2  # 2 segundos
MARGEN_CENTRO = 0.2  # Margen del centro en porcentaje de la anchura de la pantalla

while cap.isOpened():
    success, frame = cap.read()
    if success:
        results = model(frame, conf=0.9)
        annotated_frame = results[0].plot()
        cv2.imshow("DETECTOR DE VEHICULOS POLICIALES", annotated_frame)

        if len(results[0]) > 0:            
            # Obtener las dimensiones del frame y el centro
            frame_height, frame_width, _ = frame.shape
            frame_center_x = frame_width / 2
            frame_center_y = frame_height / 2

            # Obtener las coordenadas del bounding box del primer objeto detectado
            box = results[0].boxes.xywh[0].cpu().numpy()
            box_center_x, box_center_y, box_width, box_height = box

            # Verificar si el centro del bounding box está en el centro del frame
            if (frame_center_x - MARGEN_CENTRO * frame_width < box_center_x < frame_center_x + MARGEN_CENTRO * frame_width and
                frame_center_y - MARGEN_CENTRO * frame_height < box_center_y < frame_center_y + MARGEN_CENTRO * frame_height):

                if tiempo_inicio_deteccion is None:
                    tiempo_inicio_deteccion = time.time()
                tiempo_transcurrido = time.time() - tiempo_inicio_deteccion

                if tiempo_transcurrido > PERIODO_DETECCION and not captura_realizada:
                    # Agregar texto con la fecha y hora actual a la imagen
                    fecha_hora = obtener_fecha_hora()
                    # Agregar texto con borde negro
                    cv2.putText(annotated_frame, fecha_hora, (10, 570), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (150, 0, 0), 3, cv2.LINE_AA)

                    # Agregar texto blanco encima del texto negro
                    cv2.putText(annotated_frame, fecha_hora, (10, 570), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 1, cv2.LINE_AA)
                    beep(1000, 0.250)  # 1 kHz durante 500 milisegundos 
                    ruta_fotograma = f"{carpeta_fotogramas}/{contador_captura}.jpg"
                    cv2.imwrite(ruta_fotograma, annotated_frame)
                    captura_realizada = True                    
                    contador_captura = contador_captura + 1
                    # Enviar la imagen por WhatsApp
                    kit.sendwhats_image(numero_whatsapp, ruta_fotograma, "Vehiculo policial detectado (POLICIA NACIONAL)")
                    if not sonido_reproducido:                        
                        sonido_reproducido = True
            else:
                tiempo_inicio_deteccion = None
                captura_realizada = False
                sonido_reproducido = False
        else:
            tiempo_inicio_deteccion = None
            captura_realizada = False
            sonido_reproducido = False

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
    else:
        break

cap.release()
cv2.destroyAllWindows()


# -------------------------------------------------------------------------------------------------------------------
# # VERSION MULTIPLES CAPTURAS
# import cv2
# from ultralytics import YOLO
# import sounddevice as sd
# import numpy as np
# import pandas as pd
# import time

# # Load the YOLOv8 model
# model = YOLO(f"C:\PYTHON\detector_vehiculos\policia_nacional\policia_nacional_50epocas.pt")  # load a custom model

# def beep(frequency, duration):
#     # Generar una señal sinusoidal con la frecuencia deseada
#     samples = np.arange(int(duration * 44100))
#     waveform = np.sin(2 * np.pi * frequency * samples / 44100)

#     # Reproducir la señal utilizando sounddevice
#     sd.play(waveform, samplerate=44100, blocking=True)

# sonido_reproducido = False
# fotogramas_detectados = []
# contador_fotogramas = 0
# carpeta_fotogramas = f'C:\PYTHON\detector_vehiculos\policia_nacional\capturas'

# # Open the video file
# # cap = cv2.VideoCapture(f'C:\PYTHON\detector_vehiculos\policia_nacional\(1).mp4')
# cap = cv2.VideoCapture(0)

# # Loop through the video frames
# while cap.isOpened():
#     # Read a frame from the video
#     success, frame = cap.read()

#     if success:
#         # Run YOLOv8 inference on the frame
#         results = model(frame, conf=0.9)

#         # Visualize the results on the frame
#         annotated_frame = results[0].plot()

#         # Display the annotated frame
#         cv2.imshow("DETECTOR DE VEHICULOS POLICIALES", annotated_frame)

#         if len(results[0]) > 0:
#             ruta_fotograma = f"{carpeta_fotogramas}/{contador_fotogramas}.jpg"
#             cv2.imwrite(ruta_fotograma, annotated_frame)
#             contador_fotogramas = contador_fotogramas + 1
#             # Reproducir sonido si aún no se ha reproducido
#             if not sonido_reproducido:
#                 beep(1000, 0.25)  # 1 kHz durante 500 milisegundos
#                 sonido_reproducido = True

#         else:
#             sonido_reproducido = False

#         # Break the loop if 'q' is pressed
#         if cv2.waitKey(1) & 0xFF == ord("q"):
#             break
#     else:
#         # Break the loop if the end of the video is reached
#         break

# # Release the video capture object and close the display window
# cap.release()
# cv2.destroyAllWindows()
# ------------------------------------------------------------------------------------------------------------------
