#include <webots/camera.h>
#include <webots/device.h>
#include <webots/display.h>
#include <webots/gps.h>
#include <webots/lidar.h>
#include <webots/robot.h>
#include <webots/vehicle/driver.h>
#include <webots/camera_recognition_object.h>

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

// ================== CONSTANTS ==================
#define TIME_STEP 50
#define UNKNOWN 99999.99

// PID Configuration
#define KP 0.25
#define KI 0.006
#define KD 2
#define FILTER_SIZE 3

// Ethics & Avoidance Configuration
#define NORMAL_SPEED        25.0     
#define ETHICAL_SPEED       5.0     
#define OBSTACLE_THRESHOLD  15.0     
#define LNAV_GAIN           0.007    

// ================== GLOBAL DEVICES & STATE ==================
WbDeviceTag camera, sick, gps;
bool has_camera = false, has_gps = false, enable_collision_avoidance = false;
int camera_width = -1, camera_height = -1, sick_width = -1;
double camera_fov = -1.0, sick_fov = -1.0;
bool autodrive = true, PID_need_reset = false;

// ================== UTILITY FUNCTIONS ==================
void safe_print(const char *msg) {
  printf("%s\n", msg);
  fflush(stdout);
}

void set_speed(double kmh) {
  if (kmh > 250.0) kmh = 250.0;
  wbu_driver_set_cruising_speed(kmh);
}

void set_steering_angle(double wheel_angle) {
  if (wheel_angle > 0.5) wheel_angle = 0.5;
  if (wheel_angle < -0.5) wheel_angle = -0.5;
  wbu_driver_set_steering_angle(wheel_angle);
}

// ================== PHASE 3: SEMANTIC AVOIDANCE ==================
double get_lidar_avoidance_steering() {
  if (!enable_collision_avoidance) return UNKNOWN;

  const float *ranges = wb_lidar_get_range_image(sick);
  int mid = sick_width / 2;
  int scan_range = sick_width / 4; // 90 Degree Vision

  double left_danger = 0.0;
  double right_danger = 0.0;
  bool obstacle_detected = false;

  // Get AI Data
  int n = wb_camera_recognition_get_number_of_objects(camera);
  const WbCameraRecognitionObject *objs = wb_camera_recognition_get_objects(camera);

  for (int i = mid - scan_range; i < mid + scan_range; i++) {
    double dist = ranges[i];
    if (dist > OBSTACLE_THRESHOLD) continue;

    obstacle_detected = true;
    double ray_angle = ((double)i / sick_width - 0.5) * sick_fov;
    
    // Weight: Closer objects = much higher danger
    double weight = (OBSTACLE_THRESHOLD - dist);
    double moral_multiplier = 1.0;

    // AI SENSOR FUSION
    for (int j = 0; j < n; j++) {
      // Use the object's center position in the camera view
      double obj_angle = objs[j].position[0]; 

      if (fabs(obj_angle - ray_angle) < 0.2) {
        // Identify by Color (from your previous setup)
        double r = objs[j].colors[0];
        double b = objs[j].colors[2];

        if (r > 0.8) { // HUMAN (RED)
          moral_multiplier = 50.0;
        } else if (b > 0.8) { // BOX/BARREL (BLUE)
          moral_multiplier = 0.1; // Treat as almost invisible compared to humans
        }
      }
    }

    if (i < mid) left_danger += (weight * moral_multiplier);
    else right_danger += (weight * moral_multiplier);
  }

  if (!obstacle_detected) return UNKNOWN;

  // FIX: Danger on LEFT must result in a RIGHT turn (Negative value in Webots)
  // FIX: Danger on RIGHT must result in a LEFT turn (Positive value in Webots)
  double steer = (right_danger - left_danger) * LNAV_GAIN;

  // Log the decision for your AI Class presentation
  if (left_danger > right_danger) {
      printf("AI LOG: Human on Right. Calculating Repulsive Vector to Right.\n");
  } else {
      printf("AI LOG: Human on Left. Calculating Repulsive Vector to Left.\n");
  }

  // Saturation (Safety Cap)
  if (steer > 0.4) steer = 0.4;
  if (steer < -0.4) steer = -0.4;

  return steer;
}




// ================== PHASE 2: LINE FOLLOWING ==================
double process_camera_image(const unsigned char *image) {
  const unsigned char REF[3] = {95, 187, 203}; 
  int sumx = 0, count = 0;
  for (int i = 0; i < camera_width * camera_height; i++, image += 4) {
    if (abs(image[0]-REF[0]) + abs(image[1]-REF[1]) + abs(image[2]-REF[2]) < 30) {
      sumx += i % camera_width;
      count++;
    }
  }
  return (count == 0) ? UNKNOWN : ((double)sumx / count / camera_width - 0.5) * camera_fov;
}

double applyPID(double angle) {
  static double last = 0.0, integral = 0.0;
  if (PID_need_reset) { last = angle; integral = 0.0; PID_need_reset = false; }
  double diff = angle - last;
  integral += angle;
  last = angle;
  return KP * angle + KI * integral + KD * diff;
}

// ================== MAIN ==================
int main(int argc, char **argv) {
  wbu_driver_init();

  for (int i = 0; i < wb_robot_get_number_of_devices(); i++) {
    WbDeviceTag dev = wb_robot_get_device_by_index(i);
    const char *name = wb_device_get_name(dev);
    if (!strcmp(name, "camera")) has_camera = true;
    if (!strcmp(name, "Sick LMS 291")) enable_collision_avoidance = true;
  }

  if (has_camera) {
    camera = wb_robot_get_device("camera");
    wb_camera_enable(camera, TIME_STEP);
    wb_camera_recognition_enable(camera, TIME_STEP); // Enable AI Recognition
    camera_width = wb_camera_get_width(camera);
    camera_height = wb_camera_get_height(camera);
    camera_fov = wb_camera_get_fov(camera);
  }

  if (enable_collision_avoidance) {
    sick = wb_robot_get_device("Sick LMS 291");
    wb_lidar_enable(sick, TIME_STEP);
    sick_width = wb_lidar_get_horizontal_resolution(sick);
    sick_fov = wb_lidar_get_fov(sick);
  }

  set_speed(NORMAL_SPEED);
  safe_print("PHASE 3: SEMANTIC ETHICS ONLINE");

  while (wbu_driver_step() != -1) {
    double avoidance_steer = get_lidar_avoidance_steering();

    if (avoidance_steer != UNKNOWN) {
      set_speed(ETHICAL_SPEED);
      wbu_driver_set_brake_intensity(0.0);
      set_steering_angle(avoidance_steer);
      
      const float *ranges = wb_lidar_get_range_image(sick);
      if (ranges[sick_width/2] < 5.0) {
          wbu_driver_set_brake_intensity(0.8);
          safe_print("CRITICAL COLLISION RISK!");
      }
      PID_need_reset = true; 
    }  
    else if (autodrive && has_camera) {
      const unsigned char *image = wb_camera_get_image(camera);  
      double line_angle = process_camera_image(image);

      if (line_angle != UNKNOWN) {
        set_speed(NORMAL_SPEED);
        wbu_driver_set_brake_intensity(0.0);
        set_steering_angle(applyPID(line_angle));
      } else {
        wbu_driver_set_brake_intensity(0.4);
        PID_need_reset = true;
      }
    }
  }

  wbu_driver_cleanup();
  return 0;
}