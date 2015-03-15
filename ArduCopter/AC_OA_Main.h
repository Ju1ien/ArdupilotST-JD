/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#if OA_ENABLED == ENABLED

#ifndef AC_OA_MAIN_H
#define AC_OA_MAIN_H
/**
#include <AP_Param.h>
#include <AP_Math.h>
#include <AP_InertialNav.h>     // Inertial Navigation library
#include <AC_AttitudeControl.h> // Attitude control library
**/
// OA defines
//General
#define HALF_SQRT_2             0.70710678118654757f
#define HALF_PI                 1.57079632679489662f
#define QUARTER_PI              0.78539816339744831f
#define OA_VEL_0                5.0f   // in cm/s, velocity under which we consider it's 0, used to check for object in map, or not.
                                    // to-do: adjust this value from logs (eg: desired vel when loitering, attention au vent!?)

// copter properties
#define COPTER_DIAMETER                60      // in cm
#define COPTER_HEIGHT                  30      // in cm
#define COPTER_SIZE_SAFETY_FACTOR      1.2f    // default value, will be checked on OA init

// Gimbal - 2 axis Pan+Tilt
#define MAX_SCAN_PAN_ANGLE      2.09f          // Rad <=> 120°. Angle max toléré sur l'axe du Pan pour le scan. Soit +/-60° par rapport au vecteur vitesse
#define SERVO_TURN_RATE         8.73f          // Rad/s. Turn rate of servo, see datasheet. We guess both Y an Z servos have the same turn rate => 0.12s / 60°
#define DEFAULT_SCAN_PITCH      0.122f         // Rad <=> 7°
#define GIMBAL_YAW_OFFSET       0.0f           // Rad. angle between the copter heading and the gimbal aiming at mid_course of pan servo. Normally=0. If gimbal aiming at the right of the heading, offset is positive.

// mapping
#define OA_MAP_SIZE_X       32      // nombre de cubes de mapping sur l'axe des X (lat)
#define OA_MAP_SIZE_Y       32      // nombre de cubes de mapping sur l'axe des Y (lon)
#define OA_MAP_SIZE_Z       16       // nombre de cubes de mapping sur l'axe des Z (alt)
#define OA_MAP_RES          25      // in cm. côté du cube de mapping
#define OA_MIN_DIST         100     // in cm. distance minimale d'approche (cela permet de fixer la limite de scan réalisable par la nacelle)
#define OA_CHECK_FACTOR     xxx   
#define OA_CHECK_SAFE_RATIO 0.05f    // [0;1] for each check step of the map, the ratio of safe_cells/total_cells should be >= OA_CHECK_FACTOR to consider this step is safe
                                    // it works while there is no object in any cell but only safe cell or unchecked cells
#define OA_SCAN_RES  (0.4f*OA_MAP_RES)    // pitch distance relating to 

// private functions
static bool oa_lrf_read();
static void oa_update_map_from_lrf_read();
static void oa_update_map_from_copter_pos_and_vel();
//static int oa_select_scan_algo(float &Angle_y, const Vector3f& velocity);
static int16_t oa_select_scan_algo(float &Angle_y); // got compîlation errors when defining it as static scan_algo oa_select_scan_algo(...
static void oa_move_map(int8_t scheduler_step);
static void oa_check_object_in_map(int8_t scheduler_step);
static float oa_max_vector_abs_value(const Vector3f& v);
static void oa_run_scan_algo(int16_t &scan_algo, float &Ay, float &Az);
static void oa_gimbal_control(int16_t &scan_algo, float &Az);

// public functions
static void oa_set_vel_xyz(Vector2f anticipated_pilot_xy_des_vel, float target_climb_rate);   // called by loiter flight mode
static int16_t oa_get_max_braking_dist();  // called by wpnav or poscontrol lib
static bool oa_is_object_detected();  // called by wpnav or poscontrol lib
static bool oa_enabled();

#define sgn(x) ((x)<0 ? -1 : 1)

#endif	// AC_OA_MAIN_H
#endif  // OA_ENABLED == ENABLED