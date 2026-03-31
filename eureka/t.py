import cv2
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision

model_path = "hand_landmarker.task"

BaseOptions = python.BaseOptions
HandLandmarker = vision.HandLandmarker
HandLandmarkerOptions = vision.HandLandmarkerOptions
VisionRunningMode = vision.RunningMode

options = HandLandmarkerOptions(
    base_options=BaseOptions(model_asset_path=model_path),
    running_mode=VisionRunningMode.VIDEO,
    num_hands=1
)

landmarker = HandLandmarker.create_from_options(options)

cap = cv2.VideoCapture(0)

def dedos_levantados(landmarks):

    dedos = []

    # polegar (usa eixo X)
    if landmarks[4].x > landmarks[3].x:
        dedos.append(1)
    else:
        dedos.append(0)

    # indicador
    dedos.append(1 if landmarks[8].y < landmarks[6].y else 0)

    # medio
    dedos.append(1 if landmarks[12].y < landmarks[10].y else 0)

    # anelar
    dedos.append(1 if landmarks[16].y < landmarks[14].y else 0)

    # mindinho
    dedos.append(1 if landmarks[20].y < landmarks[18].y else 0)

    return dedos


timestamp = 0

while True:

    ret, frame = cap.read()
    if not ret:
        break

    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)

    result = landmarker.detect_for_video(mp_image, timestamp)

    if result.hand_landmarks:

        for hand in result.hand_landmarks:

            dedos = dedos_levantados(hand)

            texto = f"Dedos: {dedos}"

            cv2.putText(frame, texto, (30,50),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1, (0,255,0), 2)

    cv2.imshow("Hand", frame)

    timestamp += 1

    if cv2.waitKey(1) == 27:
        break

cap.release()
cv2.destroyAllWindows()