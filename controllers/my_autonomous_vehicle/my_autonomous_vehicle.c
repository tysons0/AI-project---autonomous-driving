#include <webots/camera.h>
#include <webots/device.h>
#include <webots/display.h>
#include <webots/gps.h>
#include <webots/keyboard.h>
#include <webots/lidar.h>
#include <webots/robot.h>
#include <webots/vehicle/driver.h>

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

// ================== CONSTANTS ==================
enum { X, Y, Z };
#define TIME_STEP 50
#define UNKNOWN 99999.99

// PID for Line Following
#define KP 0.25
#define KI 0.006
#define KD 2
#define FILTER_SIZE 3

// ================== ETHICS & AVOIDANCE CONFIG ==================
#define NORMAL_SPEED        25.0     
#define ETHICAL_SPEED       10.0     // Slightly raised for better momentum
#define OBSTACLE_THRESHOLD  15.0     
#define LNAV_GAIN           0.007    

// ================== DEVICES ==================
WbDeviceTag camera, sick, gps;
bool has_camera = false, has_gps = false, enable_collision_avoidance = false;
int camera_width = -1, camera_height = -1, sick_width = -1;
double camera_fov = -1.0, sick_fov = -1.0, steering_angle = 0.0;
bool autodrive = true, PID_need_reset = false;

// ================== UTILITY ==================
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
  steering_angle = wheel_angle;
  wbu_driver_set_steering_angle(wheel_angle);
}

// ================== LIDAR AVOIDANCE LOGIC ==================
double get_lidar_avoidance_steering() {
  if (!enable_collision_avoidance) return UNKNOWN;

  const float *range_array = wb_lidar_get_range_image(sick);
  int mid = sick_width / 2;
  
  // WIDER VISION: Scan middle 90 degrees (1/4 of the indices for 180 FOV)
  int scan_range = sick_width / 4; 
  int start = mid - scan_range;
  int end = mid + scan_range;

  double left_danger = 0.0, right_danger = 0.0;
  bool obstacle_detected = false;

  for (int i = start; i < end; i++) {
    float dist = range_array[i];
    if (dist < OBSTACLE_THRESHOLD) { 
      obstacle_detected = true;
      double weight = OBSTACLE_THRESHOLD - dist;
      if (i < mid) left_danger += weight;
      else right_danger += weight;
    }
  }

  return obstacle_detected ? (left_danger - right_danger) * LNAV_GAIN : UNKNOWN;
}

// ================== CAMERA LINE FOLLOW ==================
int color_diff(const unsigned char a[3], const unsigned char b[3]) {
  int diff = 0;
  for (int i = 0; i < 3; i++) diff += abs(a[i] - b[i]);
  return diff;
}

double process_camera_image(const unsigned char *image) {
  const unsigned char REF[3] = {95, 187, 203}; 
  int sumx = 0, count = 0;
  for (int i = 0; i < camera_width * camera_height; i++, image += 4) {
    if (color_diff(image, REF) < 30) {
      sumx += i % camera_width;
      count++;
    }
  }
  return (count == 0) ? UNKNOWN : ((double)sumx / count / camera_width - 0.5) * camera_fov;
}

double filter_angle(double new_value) {
  static double values[FILTER_SIZE];
  static bool first = true;
  if (first || new_value == UNKNOWN) {
    memset(values, 0, sizeof(values));
    first = false;
  } else {
    memmove(values, values + 1, (FILTER_SIZE - 1) * sizeof(double));
  }
  if (new_value == UNKNOWN) return UNKNOWN;
  values[FILTER_SIZE - 1] = new_value;
  double sum = 0.0;
  for (int i = 0; i < FILTER_SIZE; i++) sum += values[i];
  return sum / FILTER_SIZE;
}

double applyPID(double angle) {
  static double last = 0.0, integral = 0.0;
  if (PID_need_reset) {
    last = angle; integral = 0.0; PID_need_reset = false;
  }
  if (signbit(angle) != signbit(last)) integral = 0.0;
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
    if (!strcmp(name, "gps")) has_gps = true;
    if (!strcmp(name, "Sick LMS 291")) enable_collision_avoidance = true;
  }

  if (has_camera) {
    camera = wb_robot_get_device("camera");
    wb_camera_enable(camera, TIME_STEP);
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

  if (has_gps) {
    gps = wb_robot_get_device("gps");
    wb_gps_enable(gps, TIME_STEP);
  }

  set_speed(NORMAL_SPEED);
  safe_print("90 DEGREE VISION - CAUTION MODE");

  while (wbu_driver_step() != -1) {
    double avoidance_steer = get_lidar_avoidance_steering();

    if (avoidance_steer != UNKNOWN) {
      set_speed(ETHICAL_SPEED);
      wbu_driver_set_brake_intensity(0.0);
      set_steering_angle(avoidance_steer);
      
      const float *ranges = wb_lidar_get_range_image(sick);
      if (ranges[sick_width/2] < 5.0) {
          wbu_driver_set_brake_intensity(0.8);
          safe_print("EMERGENCY BRAKE!");
      } else {
          safe_print("ETHICAL SWERVE ACTIVE");
      }
      PID_need_reset = true; 
    }  
    else if (autodrive && has_camera) {
      const unsigned char *image = wb_camera_get_image(camera);
      double line_angle = filter_angle(process_camera_image(image));

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