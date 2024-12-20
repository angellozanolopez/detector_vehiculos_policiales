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










































# # VERSION MULTIPLES CAPTURAS SIN TEMPORIZAR
# import pathlib
# temp = pathlib.PosixPath
# pathlib.PosixPath = pathlib.WindowsPath

# import torch
# import cv2
# import numpy as np
# import sounddevice as sd
# import numpy as np
# import time

# def beep(frequency, duration):
#     # Generar una señal sinusoidal con la frecuencia deseada
#     samples = np.arange(int(duration * 44100))
#     waveform = np.sin(2 * np.pi * frequency * samples / 44100)

#     # Reproducir la señal utilizando sounddevice
#     sd.play(waveform, samplerate=44100, blocking=True)

# # Función para obtener la fecha y hora actual en el formato deseado
# def obtener_fecha_hora():
#     now = time.strftime("%d/%m/%Y - %H:%M:%S")
#     return now

# # Leemos el modelo
# model = torch.hub.load('ultralytics/yolov5', 'custom', path='C:/PYTHON/detector_personas/siana.pt')

# # Establecer el umbral de confianza
# model.conf = 0.9 

# # Realizamos videocaptura
# # cap = cv2.VideoCapture('C:/PYTHON/detector_personas/persona01.mp4')
# cap = cv2.VideoCapture(1)

# # Variables para controlar el estado de la detección y los contadores de fotogramas y detecciones
# sonido_reproducido = False
# contador_fotogramas = 0
# contador_detecciones = 0

# # Carpeta para almacenar fotogramas
# carpeta_fotogramas = 'C:/PYTHON/detector_personas/capturas'

# # Lista para almacenar los fotogramas cuando se detecta un autobús
# fotogramas_detectados = []

# # Variable para controlar si se ha capturado un fotograma para el autobús actual
# captura_realizada = False

# while True:
#     # Realizamos lectura de la videocaptura
#     ret, frame = cap.read()

#     # Redimensionamos la imagen a 640x480
#     frame_resized = cv2.resize(frame, (640, 480))

#     # Realizamos la detección con el umbral de confianza de 80%
#     results = model(frame_resized)

#     # Mostramos la detección en el frame
#     frame_render = np.squeeze(results.render())
#     cv2.imshow('Detector de personas', frame_render)

#     # Si se detecta un autobús
#     if len(results.xyxy[0]) > 0:
#         # Reproducir sonido si aún no se ha reproducido
#         # if not sonido_reproducido:
#         #     beep(1000, 0.25)  # 1 kHz durante 500 milisegundos
#         #     sonido_reproducido = True

#         # Agregar el fotograma actual a la lista de fotogramas detectados
#         fotogramas_detectados.append(frame_resized)

#         # Incrementar el contador de detecciones de autobuses
#         contador_detecciones += 1

#     else:
#         # Si se ha detectado un autobús y ya no se detecta, guardar los fotogramas detectados
#         if len(fotogramas_detectados) > 0:
#             # Almacenar los fotogramas detectados con el nombre apropiado
#             for idx, fotograma in enumerate(fotogramas_detectados):
#                 ruta_fotograma = f"{carpeta_fotogramas}/{contador_fotogramas}_{idx}.jpg"
#                 cv2.imwrite(ruta_fotograma, fotograma)
#                 print(f"Fotograma {contador_fotogramas}_{idx} de la detección {contador_detecciones} guardado en {ruta_fotograma}")

#             # Incrementar el contador de fotogramas
#             contador_fotogramas += len(fotogramas_detectados)

#             # Reiniciar la lista de fotogramas detectados
#             fotogramas_detectados = []

#         sonido_reproducido = False
#         contador_detecciones = 0
    
#     # Leer el teclado
#     t = cv2.waitKey(1)
#     if t == 27:
#         break

# cap.release()
# cv2.destroyAllWindows()



















# # VERSION UNA SOLA CAPTURA
# import pathlib
# temp = pathlib.PosixPath
# pathlib.PosixPath = pathlib.WindowsPath

# import torch
# import cv2
# import numpy as np
# import sounddevice as sd
# import numpy as np
# import time
# import pywhatkit as kit

# def beep(frequency, duration):
#     # Generar una señal sinusoidal con la frecuencia deseada
#     samples = np.arange(int(duration * 44100))
#     waveform = np.sin(2 * np.pi * frequency * samples / 44100)

#     # Reproducir la señal utilizando sounddevice
#     sd.play(waveform, samplerate=44100, blocking=True)

# # Función para obtener la fecha y hora actual en el formato deseado
# def obtener_fecha_hora():
#     now = time.strftime("%d/%m/%Y - %H:%M:%S")
#     return now

# # Leemos el modelo
# model = torch.hub.load('ultralytics/yolov5', 'custom', path='C:/PYTHON/detector_personas/siana.pt')

# # Establecer el umbral de confianza
# # model.conf = 0.4 


# # Realizamos videocaptura
# cap = cv2.VideoCapture('C:/PYTHON/detector_personas/persona01.mp4')
# # cap = cv2.VideoCapture(0)

# # Variables para controlar el estado de la detección y los contadores de fotogramas y detecciones
# sonido_reproducido = False
# contador_fotogramas = 0
# contador_detecciones = 0

# # Carpeta para almacenar fotogramas
# carpeta_fotogramas = 'C:/PYTHON/detector_personas/capturas'

# # Variable para controlar el tiempo de captura
# tiempo_captura = None

# # Variable para controlar si se ha capturado un fotograma para el autobús actual
# captura_realizada = False

# # Número de WhatsApp al que enviar la imagen
# numero_whatsapp = "+34677347408"

# while True:
#     # Realizamos lectura de la videocaptura
#     ret, frame = cap.read()

#     # Redimensionamos la imagen a 640x480
#     frame_resized = cv2.resize(frame, (640, 480))

#     # Realizamos la detección con el umbral de confianza de 80%
#     results = model(frame_resized)

#     # Mostramos la detección en el frame
#     frame_render = np.squeeze(results.render())
#     cv2.imshow('Detector de personas', frame_render)

#     # Si se detecta un autobús
#     if len(results.xyxy[0]) > 0:
#         # Reproducir sonido si aún no se ha reproducido
#         if not sonido_reproducido:
#             beep(1000, 0.25)  # 1 kHz durante 500 milisegundos
#             sonido_reproducido = True

#         # Si no hay un tiempo de captura activo, lo inicializamos
#         if tiempo_captura is None:
#             tiempo_captura = time.time()

#         # Si han pasado más de 2 segundos desde la detección, realizamos la captura
#         if time.time() - tiempo_captura >= 1 and not captura_realizada:
#             # Agregar texto con la fecha y hora actual a la imagen
#             fecha_hora = obtener_fecha_hora()
#             # Agregar texto con borde negro
#             cv2.putText(frame_render, fecha_hora, (10, 470), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 3, cv2.LINE_AA)

#             # Agregar texto blanco encima del texto negro
#             cv2.putText(frame_render, fecha_hora, (10, 470), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 1, cv2.LINE_AA)


#             # Almacenar el fotograma con el nombre apropiado
#             ruta_fotograma = f"{carpeta_fotogramas}/{contador_fotogramas}.jpg"
#             cv2.imwrite(ruta_fotograma, frame_render)
#             print(f"Fotograma {contador_fotogramas} de la detección {contador_detecciones} guardado en {ruta_fotograma}")

#             # Enviar la imagen por WhatsApp
#             kit.sendwhats_image(numero_whatsapp, ruta_fotograma, "Autobus detectado")

#             # Incrementar el contador de fotogramas
#             contador_fotogramas += 1

#             # Reiniciar el tiempo de captura
#             tiempo_captura = None

#             # Indicar que se ha realizado la captura para este autobús
#             captura_realizada = True

#         # Incrementar el contador de detecciones de autobuses
#         contador_detecciones += 1

#     # Si no se detecta un autobús, restablecer el estado del sonido y el tiempo de captura
#     else:
#         sonido_reproducido = False
#         tiempo_captura = None

#         # Restablecer la bandera de captura realizada
#         captura_realizada = False

#     # Leer el teclado
#     t = cv2.waitKey(1)
#     if t == 27:
#         break

# cap.release()
# cv2.destroyAllWindows()

























# # VERSION UNA SOLA CAPTURA
# import pathlib
# temp = pathlib.PosixPath
# pathlib.PosixPath = pathlib.WindowsPath

# import torch
# import cv2
# import numpy as np
# import sounddevice as sd
# import numpy as np
# import time
# import pywhatkit as kit

# def beep(frequency, duration):
#     # Generar una señal sinusoidal con la frecuencia deseada
#     samples = np.arange(int(duration * 44100))
#     waveform = np.sin(2 * np.pi * frequency * samples / 44100)

#     # Reproducir la señal utilizando sounddevice
#     sd.play(waveform, samplerate=44100, blocking=True)

# # Función para obtener la fecha y hora actual en el formato deseado
# def obtener_fecha_hora():
#     now = time.strftime("%d/%m/%Y - %H:%M:%S")
#     return now

# # Leemos el modelo
# model = torch.hub.load('ultralytics/yolov5', 'custom', path='C:/PYTHON/detector_vehiculos/autobuses/best.pt')

# # Establecer el umbral de confianza
# model.conf = 0.4 


# # Realizamos videocaptura
# # cap = cv2.VideoCapture('C:/PYTHON/detector_vehiculos/autobuses/autobus3.mp4')
# cap = cv2.VideoCapture(0)

# # Variables para controlar el estado de la detección y los contadores de fotogramas y detecciones
# sonido_reproducido = False
# contador_fotogramas = 0
# contador_detecciones = 0

# # Carpeta para almacenar fotogramas
# carpeta_fotogramas = 'C:/PYTHON/detector_vehiculos/autobuses/capturas'

# # Variable para controlar el tiempo de captura
# tiempo_captura = None

# # Variable para controlar si se ha capturado un fotograma para el autobús actual
# captura_realizada = False

# # Número de WhatsApp al que enviar la imagen
# numero_whatsapp = "+34677347408"

# while True:
#     # Realizamos lectura de la videocaptura
#     ret, frame = cap.read()

#     # Redimensionamos la imagen a 640x480
#     frame_resized = cv2.resize(frame, (640, 480))

#     # Realizamos la detección con el umbral de confianza de 80%
#     results = model(frame_resized)

#     # Mostramos la detección en el frame
#     frame_render = np.squeeze(results.render())
#     cv2.imshow('Detector de autobuses', frame_render)

#     # Si se detecta un autobús
#     if len(results.xyxy[0]) > 0:
#         # Reproducir sonido si aún no se ha reproducido
#         if not sonido_reproducido:
#             beep(1000, 0.25)  # 1 kHz durante 500 milisegundos
#             sonido_reproducido = True

#         # Si no hay un tiempo de captura activo, lo inicializamos
#         if tiempo_captura is None:
#             tiempo_captura = time.time()

#         # Si han pasado más de 2 segundos desde la detección, realizamos la captura
#         if time.time() - tiempo_captura >= 2 and not captura_realizada:
#             # Agregar texto con la fecha y hora actual a la imagen
#             fecha_hora = obtener_fecha_hora()
#             # Agregar texto con borde negro
#             cv2.putText(frame_render, fecha_hora, (10, 470), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 3, cv2.LINE_AA)

#             # Agregar texto blanco encima del texto negro
#             cv2.putText(frame_render, fecha_hora, (10, 470), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 1, cv2.LINE_AA)


#             # Almacenar el fotograma con el nombre apropiado
#             ruta_fotograma = f"{carpeta_fotogramas}/{contador_fotogramas}.jpg"
#             cv2.imwrite(ruta_fotograma, frame_render)
#             print(f"Fotograma {contador_fotogramas} de la detección {contador_detecciones} guardado en {ruta_fotograma}")

#             # Enviar la imagen por WhatsApp
#             kit.sendwhats_image(numero_whatsapp, ruta_fotograma, "Autobus detectado")

#             # Incrementar el contador de fotogramas
#             contador_fotogramas += 1

#             # Reiniciar el tiempo de captura
#             tiempo_captura = None

#             # Indicar que se ha realizado la captura para este autobús
#             captura_realizada = True

#         # Incrementar el contador de detecciones de autobuses
#         contador_detecciones += 1

#     # Si no se detecta un autobús, restablecer el estado del sonido y el tiempo de captura
#     else:
#         sonido_reproducido = False
#         tiempo_captura = None

#         # Restablecer la bandera de captura realizada
#         captura_realizada = False

#     # Leer el teclado
#     t = cv2.waitKey(1)
#     if t == 27:
#         break

# cap.release()
# cv2.destroyAllWindows()
















































# import pathlib
# temp = pathlib.PosixPath
# pathlib.PosixPath = pathlib.WindowsPath

# import torch
# import cv2
# import numpy as np
# import sounddevice as sd
# import numpy as np
# import time
# import pywhatkit as kit

# def beep(frequency, duration):
#     # Generar una señal sinusoidal con la frecuencia deseada
#     samples = np.arange(int(duration * 44100))
#     waveform = np.sin(2 * np.pi * frequency * samples / 44100)

#     # Reproducir la señal utilizando sounddevice
#     sd.play(waveform, samplerate=44100, blocking=True)

# # Leemos el modelo
# model = torch.hub.load('ultralytics/yolov5', 'custom', path='C:/PYTHON/detector_vehiculos/autobuses/best.pt')

# # Establecer el umbral de confianza
# model.conf = 0.5

# # Realizamos videocaptura
# # cap = cv2.VideoCapture('C:/PYTHON/detector_vehiculos/autobuses/autobus2.mp4')
# cap = cv2.VideoCapture(0)

# # Variables para controlar el estado de la detección y los contadores de fotogramas y detecciones
# sonido_reproducido = False
# contador_fotogramas = 0
# contador_detecciones = 0

# # Carpeta para almacenar fotogramas
# carpeta_fotogramas = 'C:/PYTHON/detector_vehiculos/autobuses/capturas'

# # Variable para controlar el tiempo de captura
# tiempo_captura = None

# # Variable para controlar si se ha capturado un fotograma para el autobús actual
# captura_realizada = False

# # Número de WhatsApp al que enviar la imagen
# numero_whatsapp = "+34677347408"

# while True:
#     # Realizamos lectura de la videocaptura
#     ret, frame = cap.read()

#     # Redimensionamos la imagen a 640x480
#     frame_resized = cv2.resize(frame, (640, 480))

#     # Realizamos la detección con el umbral de confianza de 80%
#     results = model(frame)

#     # Mostramos la detección en el frame
#     cv2.imshow('Detector de autobuses', np.squeeze(results.render()))    

#     # Si se detecta un autobús
#     if len(results.xyxy[0]) > 0:
#         # Reproducir sonido si aún no se ha reproducido
#         if not sonido_reproducido:
#             beep(1000, 0.35)  # 1 kHz durante 500 milisegundos
#             sonido_reproducido = True

#         # Si no hay un tiempo de captura activo, lo inicializamos
#         if tiempo_captura is None:
#             tiempo_captura = time.time()

#         # Si han pasado más de 2 segundos desde la detección, realizamos la captura
#         if time.time() - tiempo_captura >= 2 and not captura_realizada:
#             # Almacenar el fotograma con el nombre apropiado
#             ruta_fotograma = f"{carpeta_fotogramas}/{contador_fotogramas}.jpg"
#             cv2.imwrite(ruta_fotograma, frame)
#             print(f"Fotograma {contador_fotogramas} de la detección {contador_detecciones} guardado en {ruta_fotograma}")

#             # Enviar la imagen por WhatsApp
#             kit.sendwhats_image(numero_whatsapp, ruta_fotograma, "Autobus detectado")

#             # Incrementar el contador de fotogramas
#             contador_fotogramas += 1

#             # Reiniciar el tiempo de captura
#             tiempo_captura = None

#             # Indicar que se ha realizado la captura para este autobús
#             captura_realizada = True

#         # Incrementar el contador de detecciones de autobuses
#         contador_detecciones += 1

#     # Si no se detecta un autobús, restablecer el estado del sonido y el tiempo de captura
#     else:
#         sonido_reproducido = False
#         tiempo_captura = None

#         # Restablecer la bandera de captura realizada
#         captura_realizada = False

#     # Leer el teclado
#     t = cv2.waitKey(1)
#     if t == 27:
#         break

# cap.release()
# cv2.destroyAllWindows()


























































# import pathlib
# temp = pathlib.PosixPath
# pathlib.PosixPath = pathlib.WindowsPath

# import torch
# import cv2
# import numpy as np
# import sounddevice as sd
# import numpy as np
# import time

# def beep(frequency, duration):
#     # Generar una señal sinusoidal con la frecuencia deseada
#     samples = np.arange(int(duration * 44100))
#     waveform = np.sin(2 * np.pi * frequency * samples / 44100)

#     # Reproducir la señal utilizando sounddevice
#     sd.play(waveform, samplerate=44100, blocking=True)

# # Leemos el modelo
# model = torch.hub.load('ultralytics/yolov5', 'custom', path='C:/PYTHON/detector_vehiculos/autobuses/best.pt')

# # Establecer el umbral de confianza
# model.conf = 0.5

# # Realizamos videocaptura
# cap = cv2.VideoCapture('C:/PYTHON/detector_vehiculos/autobuses/autobus2.mp4')
# # cap = cv2.VideoCapture(0)

# # Variables para controlar el estado de la detección y los contadores de fotogramas y detecciones
# sonido_reproducido = False
# contador_fotogramas = 0
# contador_detecciones = 0

# # Carpeta para almacenar fotogramas
# carpeta_fotogramas = 'C:/PYTHON/detector_vehiculos/autobuses/capturas'

# # Variable para controlar el tiempo de captura
# tiempo_captura = None

# # Variable para controlar si se ha capturado un fotograma para el autobús actual
# captura_realizada = False

# while True:
#     # Realizamos lectura de la videocaptura
#     ret, frame = cap.read()

#     # Redimensionamos la imagen a 640x480
#     frame_resized = cv2.resize(frame, (640, 480))

#     # Realizamos la detección con el umbral de confianza de 80%
#     results = model(frame)

#     # Mostramos la detección en el frame
#     cv2.imshow('Detector de autobuses', np.squeeze(results.render()))    

#     # Si se detecta un autobús
#     if len(results.xyxy[0]) > 0:
#         # Reproducir sonido si aún no se ha reproducido
#         if not sonido_reproducido:
#             beep(1000, 0.35)  # 1 kHz durante 500 milisegundos
#             sonido_reproducido = True

#         # Si no hay un tiempo de captura activo, lo inicializamos
#         if tiempo_captura is None:
#             tiempo_captura = time.time()

#         # Si han pasado más de 2 segundos desde la detección, realizamos la captura
#         if time.time() - tiempo_captura >= 2 and not captura_realizada:
#             # Almacenar el fotograma con el nombre apropiado
#             ruta_fotograma = f"{carpeta_fotogramas}/{contador_fotogramas}.jpg"
#             cv2.imwrite(ruta_fotograma, frame)
#             print(f"Fotograma {contador_fotogramas} de la detección {contador_detecciones} guardado en {ruta_fotograma}")

#             # Incrementar el contador de fotogramas
#             contador_fotogramas += 1

#             # Reiniciar el tiempo de captura
#             tiempo_captura = None

#             # Indicar que se ha realizado la captura para este autobús
#             captura_realizada = True

#         # Incrementar el contador de detecciones de autobuses
#         contador_detecciones += 1

#     # Si no se detecta un autobús, restablecer el estado del sonido y el tiempo de captura
#     else:
#         sonido_reproducido = False
#         tiempo_captura = None

#         # Restablecer la bandera de captura realizada
#         captura_realizada = False

#     # Leer el teclado
#     t = cv2.waitKey(1)
#     if t == 27:
#         break

# cap.release()
# cv2.destroyAllWindows()














































# import pathlib
# temp = pathlib.PosixPath
# pathlib.PosixPath = pathlib.WindowsPath

# import torch
# import cv2
# import numpy as np
# import sounddevice as sd
# import numpy as np
# import time

# def beep(frequency, duration):
#     # Generar una señal sinusoidal con la frecuencia deseada
#     samples = np.arange(int(duration * 44100))
#     waveform = np.sin(2 * np.pi * frequency * samples / 44100)

#     # Reproducir la señal utilizando sounddevice
#     sd.play(waveform, samplerate=44100, blocking=True)

# # Leemos el modelo
# model = torch.hub.load('ultralytics/yolov5', 'custom', path='C:/PYTHON/detector_vehiculos/autobuses/best.pt')

# # Establecer el umbral de confianza
# model.conf = 0.5

# # Realizamos videocaptura
# # cap = cv2.VideoCapture('C:/PYTHON/detector_vehiculos/autobuses/autobus2.mp4')
# cap = cv2.VideoCapture(0)

# # Variables para controlar el estado de la detección y los contadores de fotogramas y detecciones
# sonido_reproducido = False
# contador_fotogramas = 0
# contador_detecciones = 0

# # Carpeta para almacenar fotogramas
# carpeta_fotogramas = 'C:/PYTHON/detector_vehiculos/autobuses/capturas'

# # Variable para controlar el tiempo de captura
# tiempo_captura = None

# # Variable para controlar si se ha capturado un fotograma para el autobús actual
# captura_realizada = False

# while True:
#     # Realizamos lectura de la videocaptura
#     ret, frame = cap.read()

#     # Redimensionamos la imagen a 640x480
#     frame_resized = cv2.resize(frame, (640, 480))

#     # Realizamos la detección con el umbral de confianza de 80%
#     results = model(frame)

#     # Mostramos la detección en el frame
#     cv2.imshow('Detector de autobuses', np.squeeze(results.render()))    

#     # Si se detecta un autobús
#     if len(results.xyxy[0]) > 0:
#         # Reproducir sonido si aún no se ha reproducido
#         if not sonido_reproducido:
#             beep(1000, 0.35)  # 1 kHz durante 500 milisegundos
#             sonido_reproducido = True

#         # Si no hay un tiempo de captura activo, lo inicializamos
#         if tiempo_captura is None:
#             tiempo_captura = time.time()

#         # Si han pasado más de 2 segundos desde la detección, realizamos la captura
#         if time.time() - tiempo_captura >= 2 and not captura_realizada:
#             # Almacenar el fotograma con el nombre apropiado
#             ruta_fotograma = f"{carpeta_fotogramas}/{contador_fotogramas}.jpg"
#             cv2.imwrite(ruta_fotograma, frame)
#             print(f"Fotograma {contador_fotogramas} de la detección {contador_detecciones} guardado en {ruta_fotograma}")

#             # Incrementar el contador de fotogramas
#             contador_fotogramas += 1

#             # Reiniciar el tiempo de captura
#             tiempo_captura = None

#             # Indicar que se ha realizado la captura para este autobús
#             captura_realizada = True

#         # Incrementar el contador de detecciones de autobuses
#         contador_detecciones += 1

#     # Si no se detecta un autobús, restablecer el estado del sonido y el tiempo de captura
#     else:
#         sonido_reproducido = False
#         tiempo_captura = None

#         # Restablecer la bandera de captura realizada
#         captura_realizada = False

#     # Leer el teclado
#     t = cv2.waitKey(1)
#     if t == 27:
#         break

# cap.release()
# cv2.destroyAllWindows()
















































# # # PARA VER LA WEBCAM EN DIRECTO
# import pathlib
# temp = pathlib.PosixPath
# pathlib.PosixPath = pathlib.WindowsPath

# import torch
# import cv2
# import numpy as np
# import sounddevice as sd
# import numpy as np
# # import time

# def beep(frequency, duration):
#     # Generar una señal sinusoidal con la frecuencia deseada
#     samples = np.arange(int(duration * 44100))
#     waveform = np.sin(2 * np.pi * frequency * samples / 44100)

#     # Reproducir la señal utilizando sounddevice
#     sd.play(waveform, samplerate=44100, blocking=True)

# # Leemos el modelo
# model = torch.hub.load('ultralytics/yolov5', 'custom', path='C:/PYTHON/detector_vehiculos/autobuses/best.pt')

# # Establecer el umbral de confianza
# model.conf = 0.5

# # Realizamos videocaptura
# # cap = cv2.VideoCapture('C:/PYTHON/detector_vehiculos/autobuses/autobus2.mp4')
# cap = cv2.VideoCapture(0)

# # Variables para controlar el estado de la detección y los contadores de fotogramas y detecciones
# sonido_reproducido = False
# contador_fotogramas = 0
# contador_detecciones = 0

# # Carpeta para almacenar fotogramas
# carpeta_fotogramas = 'C:/PYTHON/detector_vehiculos/autobuses/capturas'

# while True:
#     # Realizamos lectura de la videocaptura
#     ret, frame = cap.read()

#     # Redimensionamos la imagen a 640x480
#     frame_resized = cv2.resize(frame, (640, 480))

#     # Realizamos la detección con el umbral de confianza de 80%
#     results = model(frame)

#     # Mostramos la detección en el frame
#     cv2.imshow('Detector de autobuses', np.squeeze(results.render()))    

#     # Si se detecta un autobús
#     if len(results.xyxy[0]) > 0:
#         # Reproducir sonido si aún no se ha reproducido
#         if not sonido_reproducido:
#             beep(1000, 0.35)  # 1 kHz durante 500 milisegundos
#             sonido_reproducido = True

#         # Incrementar el contador de detecciones de autobuses
#         contador_detecciones += 1

#         # Almacenar el fotograma con el nombre apropiado
#         ruta_fotograma = f"{carpeta_fotogramas}/{contador_fotogramas}.jpg"
#         cv2.imwrite(ruta_fotograma, frame)
#         print(f"Fotograma {contador_fotogramas} de la detección {contador_detecciones} guardado en {ruta_fotograma}")

#         # Incrementar el contador de fotogramas
#         contador_fotogramas += 1

#     # Si no se detecta un autobús, restablecer el estado del sonido
#     else:
#         sonido_reproducido = False

#     # Leer el teclado
#     t = cv2.waitKey(1)
#     if t == 27:
#         break

# cap.release()
# cv2.destroyAllWindows()













































# import pathlib
# temp = pathlib.PosixPath
# pathlib.PosixPath = pathlib.WindowsPath


# # Importamos librerias
# import torch
# import cv2
# import numpy as np

# # Leemos el modelo
# model = torch.hub.load('ultralytics/yolov5', 'custom', path = 'C:/PYTHON/detector_vehiculos/autobuses/autobuses.pt')

# # Realizamos videocaptura
# cap = cv2.VideoCapture(0)

# while True:
#     # Realizamos lectura de la videocaptura
#     ret, frame = cap.read()

#     # Realizamos la deteccion
#     detect = model(frame)

#     # info = detect.pandas().xyxy[0]
#     # print(info)

    
#     # Mostramos FPS
#     cv2.imshow('Detector de autobuses', np.squeeze(detect.render()))

#     # Leer el teclado
#     t = cv2.waitKey(1)
#     if t == 27:
#         break

# cap.release()
# cv2.destroyAllWindows()

