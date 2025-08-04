import cv2
import mediapipe as mp
import numpy as np
from collections import deque
import math

# Gesture thresholds
SWIPE_THRESHOLD = 100  # pixels
CIRCLE_BUFFER_SIZE = 30
CIRCLE_RADIUS_THRESHOLD = 20  # pixels
CIRCLE_COMPLETION_RATIO = 0.6  # % of circle arc required

# Initialize MediaPipe
mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils

hands = mp_hands.Hands(
    static_image_mode=False,
    max_num_hands=1,
    min_detection_confidence=0.7,
    min_tracking_confidence=0.7,
)

# Trajectory buffer
points = deque(maxlen=CIRCLE_BUFFER_SIZE)
gesture_cooldown = 0


def detect_swipe(start, end):
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    if abs(dx) > abs(dy):  # horizontal
        if dx > SWIPE_THRESHOLD:
            return "MOVE -Y (Swipe Right)"
        elif dx < -SWIPE_THRESHOLD:
            return "MOVE +Y (Swipe Left)"
    else:  # vertical
        if dy > SWIPE_THRESHOLD:
            return "MOVE -X (Swipe Down)"
        elif dy < -SWIPE_THRESHOLD:
            return "MOVE +X (Swipe Up)"
    return None


def detect_circle(pts):
    if len(pts) < 10:
        return None
    cx = np.mean([p[0] for p in pts])
    cy = np.mean([p[1] for p in pts])
    radius = np.mean([math.hypot(p[0] - cx, p[1] - cy) for p in pts])
    if radius < CIRCLE_RADIUS_THRESHOLD:
        return None

    # Calculate angles
    angles = [math.atan2(p[1] - cy, p[0] - cx) for p in pts]
    total_angle = 0
    for i in range(1, len(angles)):
        d = angles[i] - angles[i - 1]
        d = (d + math.pi) % (2 * math.pi) - math.pi  # Normalize
        total_angle += d
    if abs(total_angle) > 2 * math.pi * CIRCLE_COMPLETION_RATIO:
        return "ROTATE Z+" if total_angle < 0 else "ROTATE Z-"
    return None


# Open webcam
cap = cv2.VideoCapture(0)
start_point = None

while cap.isOpened():
    success, image = cap.read()
    if not success:
        break

    # Flip for natural interaction
    image = cv2.flip(image, 1)
    image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    results = hands.process(image_rgb)

    h, w, _ = image.shape
    index_finger_pos = None

    if results.multi_hand_landmarks:
        for hand_landmarks in results.multi_hand_landmarks:
            # Index fingertip (landmark 8)
            x = int(hand_landmarks.landmark[8].x * w)
            y = int(hand_landmarks.landmark[8].y * h)
            index_finger_pos = (x, y)

            mp_drawing.draw_landmarks(image, hand_landmarks, mp_hands.HAND_CONNECTIONS)

    # Gesture detection logic
    if index_finger_pos:
        points.append(index_finger_pos)

        if gesture_cooldown == 0 and len(points) >= 10:
            gesture = detect_swipe(points[0], points[-1])
            if gesture is None:
                gesture = detect_circle(list(points))

            if gesture:
                print(f"✅ Gesture detected: {gesture}")
                gesture_cooldown = 20  # cooldown frames

    if gesture_cooldown > 0:
        gesture_cooldown -= 1

    # Visualize
    for p in points:
        cv2.circle(image, p, 3, (0, 255, 0), -1)

    cv2.imshow("Hand Gesture Control", image)
    if cv2.waitKey(1) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()
