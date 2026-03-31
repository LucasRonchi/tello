import cv2
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
from djitellopy import Tello
import time


class Detection:

    def __init__(self):

        model_path = "model/hand_landmarker.task"

        BaseOptions = python.BaseOptions
        HandLandmarker = vision.HandLandmarker
        HandLandmarkerOptions = vision.HandLandmarkerOptions
        VisionRunningMode = vision.RunningMode

        options = HandLandmarkerOptions(
            base_options=BaseOptions(model_asset_path=model_path),
            running_mode=VisionRunningMode.VIDEO,
            num_hands=2
        )

        self.landmarker = HandLandmarker.create_from_options(options)

        # Conectar ao Tello
        self.tello = Tello()
        self.tello.connect()

        print("Bateria:", self.tello.get_battery())

        # Iniciar stream
        self.tello.streamon()

        self.frame_read = self.tello.get_frame_read()

        self.frame_timestamp = 0

        self.GESTURES = {
            (1,1,1,1,1,1,1,1): "NOTHING",
            (0,0,0,0,0,0,0,0): "TAKEOFF",
            (1,0,0,0,1,0,0,0): "LAND",
            (1,0,0,1,1,0,0,1): "FLIP",
        }

        self.COMMANDS = {
            "TAKEOFF": self.tello.takeoff,
            "FLIP": lambda: self.tello.flip('b'),
            "LAND": self.tello.land,
        }

        self.last_gesture = None
        self.gesture_count = 0
        self.gesture_threshold = 5

        self.last_command_time = 0
        self.command_delay = 3

    def detect(self):

        frame = self.frame_read.frame

        if frame is None:
            return

        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        mp_image = mp.Image(
            image_format=mp.ImageFormat.SRGB,
            data=frame_rgb
        )

        result = self.landmarker.detect_for_video(
            mp_image,
            self.frame_timestamp
        )

        h, w, _ = frame.shape

        fingers = [0]*8

        if result.hand_landmarks:

            for i, hand_landmarks in enumerate(result.hand_landmarks):

                for lm in hand_landmarks:

                    x = int(lm.x * w)
                    y = int(lm.y * h)

                    cv2.circle(frame, (x, y), 5, (0,255,0), -1)

                f = self.raised_fingers(hand_landmarks)

                hand = result.handedness[i][0].category_name

                if hand == "Left":
                    fingers[0:4] = f
                else:
                    fingers[4:8] = f

        gesture = self.detect_gesture(fingers)

        print(fingers, gesture)

        cv2.putText(
            frame,
            gesture,
            (30,50),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (0,255,0),
            2
        )

        cv2.imshow("Tello Hand Control", frame)

        self.process_gesture(gesture)

        if command:
            command()

        self.frame_timestamp += 1


    def process_gesture(self, gesture):

        if gesture == self.last_gesture:
            self.gesture_count += 1
        else:
            self.gesture_count = 1
            self.last_gesture = gesture

        if self.gesture_count >= self.gesture_threshold:

            command = self.COMMANDS.get(gesture)

            now = time.time()

            if command and now - self.last_command_time > self.command_delay:

                print("EXECUTANDO:", gesture)

                command()

                self.last_command_time = now
                self.gesture_count = 0


    def raised_fingers(self, landmarks):

        dedos = []

        dedos.append(1 if landmarks[8].y < landmarks[6].y else 0)
        dedos.append(1 if landmarks[12].y < landmarks[10].y else 0)
        dedos.append(1 if landmarks[16].y < landmarks[14].y else 0)
        dedos.append(1 if landmarks[20].y < landmarks[18].y else 0)

        return dedos

    def detect_gesture(self, fingers):

        return self.GESTURES.get(tuple(fingers), "UNKNOWN")

    def send_command(self, gesture):

        return self.COMANDS.get(tuple(gesture), lambda _: print('Not Send Command'))


    def close(self):

        self.tello.streamoff()
        self.tello.end()

        cv2.destroyAllWindows()

    def run(self):

        while True:

            self.detect()

            if cv2.waitKey(1) & 0xFF == 27:
                break

        self.close()


d = Detection()
d.run()