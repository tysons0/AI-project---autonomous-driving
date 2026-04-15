from controller import Robot
from vehicle import Driver
from controller import Camera
import numpy as np
from ultralytics import YOLO

#ethical
ETHICAL_TEST_MODE = True

#robot initilization
robot = Robot()
timestep = int(robot.getBasicTimeStep())

#vehicle control
driver = Driver()
print("custom_autonomous_controller running")

#lidar
lidar = robot.getDevice("lidar")
lidar.enable(timestep)

#camera
camera = robot.getDevice("camera")
camera.enable(timestep)
width = camera.getWidth()
height = camera.getHeight()

#yolov8
model = YOLO("yolov8n.pt")



#parameters
SAFE_SPEED = 20.0
AVOID_SPEED = 8.0
TURN_ANGLE = 0.15
FRONT_OBSTACLE_DIST = 8.0
SIDE_CLEARANCE_DIST = 3.0
VISION_DARK_THRESHOLD = 60 #camera vision threshold

#yolo ethics parameters
YOLO_INTERVAL = 10       # run YOLO every N steps
YOLO_CONF_THRESHOLD = 0.4
HUMAN_CONFIRM_FRAMES = 3      # temporal filter
HUMAN_LIKE_CLASSES = {
    "person",
    "tv",        # common Webots misclassification
    "chair",
    "bench"
}
# Counters
yolo_counter = 0
human_like_counter = 0
human_confirmed = False



while robot.step(timestep) != -1:

    # -------- CAMERA (brightness cue stays unchanged) --------
    image = camera.getImage()
    center_pixel = Camera.imageGetGray(image, width, width // 2, height // 2)
    vision_caution = center_pixel < VISION_DARK_THRESHOLD

    
    if ETHICAL_TEST_MODE:
        # Forced ethical scenario: human on left, box on right
        human_present = True
    
        # ETHICAL OVERRIDE — MUST RUN FIRST
        speed = AVOID_SPEED
        steering = TURN_ANGLE  # steer toward box, away from human
    
        driver.setCruisingSpeed(speed)
        driver.setSteeringAngle(steering)
    
        print("ETHICAL OVERRIDE ACTIVE: avoiding human, accepting box collision")
        continue

    # -------- LIDAR --------
    ranges = lidar.getRangeImage()
    n = len(ranges)

    left = min(ranges[:n // 3])
    center = min(ranges[n // 3: 2 * n // 3])
    right = min(ranges[2 * n // 3:])

    # -------- YOLO (THROTTLED) --------
    yolo_counter += 1
    detected_human_like = False

    if yolo_counter % YOLO_INTERVAL == 0:
        image_array = np.array(camera.getImageArray(), dtype=np.uint8)
        image_array = image_array[:, :, :3]
        image_array = np.flipud(image_array)  # REQUIRED for Webots

        results = model(image_array, conf=0.25, verbose=False)

        for r in results:
            for box in r.boxes:
                class_name = model.names[int(box.cls[0])]
                confidence = float(box.conf[0])

                # Debug print (you can remove later)
                print(f"YOLO detected: {class_name} ({confidence:.2f})")

                # -------- HUMAN-LIKE SEMANTIC GROUPING --------
                if class_name in HUMAN_LIKE_CLASSES and confidence > YOLO_CONF_THRESHOLD:
                    detected_human_like = True

    # -------- TEMPORAL FILTERING --------
    if detected_human_like:
        human_like_counter += 1
    else:
        human_like_counter = 0

    if human_like_counter >= HUMAN_CONFIRM_FRAMES:
        human_confirmed = True
    else:
        human_confirmed = False

    # -------------------- DECISION LOGIC --------------------

    steering = 0.0
    speed = SAFE_SPEED

    # -------- ETHICAL OVERRIDE (READY FOR NEXT STEP) --------
    if human_confirmed:
        speed = 0.0
        steering = 0.0
        print("HUMAN CONFIRMED — SAFETY OVERRIDE")

    # -------- STANDARD LIDAR BEHAVIOR --------
    elif center < FRONT_OBSTACLE_DIST:
        speed = AVOID_SPEED
        steering = -TURN_ANGLE if left > right else TURN_ANGLE

    elif left < SIDE_CLEARANCE_DIST:
        steering = TURN_ANGLE * 0.5

    elif right < SIDE_CLEARANCE_DIST:
        steering = -TURN_ANGLE * 0.5

    elif vision_caution:
        speed = SAFE_SPEED * 0.6

    # -------- APPLY CONTROL --------
    driver.setCruisingSpeed(speed)
    driver.setSteeringAngle(steering)






    

