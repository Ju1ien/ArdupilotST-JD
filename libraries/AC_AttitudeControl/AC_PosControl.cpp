/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#include <AP_HAL.h>
#include <AC_PosControl.h>

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AC_PosControl::var_info[] PROGMEM = {
    // @Param: THR_HOVER
    // @DisplayName: Throttle Hover
    // @Description: The autopilot's estimate of the throttle required to maintain a level hover.  Calculated automatically from the pilot's throttle input while in stabilize mode
    // @Range: 0 1000
    // @Units: Percent*10
    // @User: Advanced
    AP_GROUPINFO("THR_HOVER",       0, AC_PosControl, _throttle_hover, POSCONTROL_THROTTLE_HOVER),

    AP_GROUPEND
};

// Default constructor.
// Note that the Vector/Matrix constructors already implicitly zero
// their values.
//
AC_PosControl::AC_PosControl(const AP_AHRS& ahrs, const AP_InertialNav& inav,
                             const AP_Motors& motors, AC_AttitudeControl& attitude_control,
                             AC_P& p_alt_pos, AC_P& p_alt_rate, AC_PID& pid_alt_accel,
                             AC_P& p_pos_xy, AC_PID& pid_rate_lat, AC_PID& pid_rate_lon) :
    _ahrs(ahrs),
    _inav(inav),
    _motors(motors),
    _attitude_control(attitude_control),
    _p_alt_pos(p_alt_pos),
    _p_alt_rate(p_alt_rate),
    _pid_alt_accel(pid_alt_accel),
    _p_pos_xy(p_pos_xy),
    _pid_rate_lat(pid_rate_lat),
    _pid_rate_lon(pid_rate_lon),
    _dt(POSCONTROL_DT_10HZ),
    _last_update_xy_ms(0),
    _last_update_z_ms(0),
    _last_update_vel_xyz_ms(0),
    _speed_down_cms(POSCONTROL_SPEED_DOWN),
    _speed_up_cms(POSCONTROL_SPEED_UP),
    _speed_cms(POSCONTROL_SPEED),
    _accel_z_cms(POSCONTROL_ACCEL_Z),
    _accel_cms(POSCONTROL_ACCEL_XY),
    _leash(POSCONTROL_LEASH_LENGTH_MIN),
    _leash_down_z(POSCONTROL_LEASH_LENGTH_MIN),
    _leash_up_z(POSCONTROL_LEASH_LENGTH_MIN),
    _roll_target(0.0f),
    _pitch_target(0.0f),
    _alt_max(0.0f),
    _distance_to_target(0.0f),
    _xy_step(0),
    _dt_xy(0.0f),
    _vel_xyz_step(0)
{
    AP_Param::setup_object_defaults(this, var_info);

    // initialise flags
    _flags.force_recalc_xy = false;
#if HAL_CPU_CLASS >= HAL_CPU_CLASS_150
    _flags.slow_cpu = false;
#else
    _flags.slow_cpu = true;
#endif
    _flags.recalc_leash_xy = true;
    _flags.recalc_leash_z = true;
    _flags.keep_xy_I_terms = false;
    _flags.reset_desired_vel_to_pos = true;
    _flags.reset_rate_to_accel_xy = true;
    _flags.reset_rate_to_accel_z = true;
    _flags.reset_accel_to_throttle = true;
}

///
/// z-axis position controller
///


/// set_dt - sets time delta in seconds for all controllers (i.e. 100hz = 0.01, 400hz = 0.0025)
void AC_PosControl::set_dt(float delta_sec)
{
    _dt = delta_sec;

    // update rate controller's d filter
    _pid_alt_accel.set_d_lpf_alpha(POSCONTROL_ACCEL_Z_DTERM_FILTER, _dt);
}

/// set_speed_z - sets maximum climb and descent rates
/// To-Do: call this in the main code as part of flight mode initialisation
///     calc_leash_length_z should be called afterwards
///     speed_down should be a negative number
void AC_PosControl::set_speed_z(float speed_down, float speed_up)
{
    // ensure speed_down is always negative
    speed_down = (float)-fabs(speed_down);

    if (((float)fabs(_speed_down_cms-speed_down) > 1.0f) || ((float)fabs(_speed_up_cms-speed_up) > 1.0f)) {
        _speed_down_cms = speed_down;
        _speed_up_cms = speed_up;
        _flags.recalc_leash_z = true;
    }
}

/// set_accel_z - set vertical acceleration in cm/s/s
void AC_PosControl::set_accel_z(float accel_cmss)
{
    if ((float)fabs(_accel_z_cms-accel_cmss) > 1.0f) {
        _accel_z_cms = accel_cmss;
        _flags.recalc_leash_z = true;
    }
}

/// set_alt_target_with_slew - adjusts target towards a final altitude target
///     should be called continuously (with dt set to be the expected time between calls)
///     actual position target will be moved no faster than the speed_down and speed_up
///     target will also be stopped if the motors hit their limits or leash length is exceeded
void AC_PosControl::set_alt_target_with_slew(float alt_cm, float dt)
{
    float alt_change = alt_cm-_pos_target.z;

    // adjust desired alt if motors have not hit their limits
    if ((alt_change<0 && !_motors.limit.throttle_lower) || (alt_change>0 && !_motors.limit.throttle_upper)) {
        _pos_target.z += constrain_float(alt_change, _speed_down_cms*dt, _speed_up_cms*dt);
    }

    // do not let target get too far from current altitude
    float curr_alt = _inav.get_altitude();
    _pos_target.z = constrain_float(_pos_target.z,curr_alt-_leash_down_z,curr_alt+_leash_up_z);
}

/// set_alt_target_from_climb_rate - adjusts target up or down using a climb rate in cm/s
///     should be called continuously (with dt set to be the expected time between calls)
///     actual position target will be moved no faster than the speed_down and speed_up
///     target will also be stopped if the motors hit their limits or leash length is exceeded
void AC_PosControl::set_alt_target_from_climb_rate(float climb_rate_cms, float dt)
{
    // adjust desired alt if motors have not hit their limits
    // To-Do: add check of _limit.pos_up and _limit.pos_down?
    if ((climb_rate_cms<0 && !_motors.limit.throttle_lower) || (climb_rate_cms>0 && !_motors.limit.throttle_upper)) {
        _pos_target.z += climb_rate_cms * dt;
    }
}

// get_alt_error - returns altitude error in cm
float AC_PosControl::get_alt_error() const
{
    return (_pos_target.z - _inav.get_altitude());
}

/// set_target_to_stopping_point_z - returns reasonable stopping altitude in cm above home
void AC_PosControl::set_target_to_stopping_point_z()
{
    // check if z leash needs to be recalculated
    calc_leash_length_z();

    get_stopping_point_z(_pos_target);
}

/// get_stopping_point_z - sets stopping_point.z to a reasonable stopping altitude in cm above home
void AC_PosControl::get_stopping_point_z(Vector3f& stopping_point) const
{
    const float curr_pos_z = _inav.get_altitude();
    float curr_vel_z = _inav.get_velocity_z();

    float linear_distance;  // half the distance we swap between linear and sqrt and the distance we offset sqrt
    float linear_velocity;  // the velocity we swap between linear and sqrt

    // if position controller is active add current velocity error to avoid sudden jump in acceleration
    if (is_active_z()) {
        curr_vel_z += _vel_error.z;
    }

    // calculate the velocity at which we switch from calculating the stopping point using a linear function to a sqrt function
    linear_velocity = _accel_z_cms/_p_alt_pos.kP();

    if ((float)fabs(curr_vel_z) < linear_velocity) {
        // if our current velocity is below the cross-over point we use a linear function
        stopping_point.z = curr_pos_z + curr_vel_z/_p_alt_pos.kP();
    } else {
        linear_distance = _accel_z_cms/(2.0f*_p_alt_pos.kP()*_p_alt_pos.kP());
        if (curr_vel_z > 0){
            stopping_point.z = curr_pos_z + (linear_distance + curr_vel_z*curr_vel_z/(2.0f*_accel_z_cms));
        } else {
            stopping_point.z = curr_pos_z - (linear_distance + curr_vel_z*curr_vel_z/(2.0f*_accel_z_cms));
        }
    }
    stopping_point.z = constrain_float(stopping_point.z, curr_pos_z - POSCONTROL_STOPPING_DIST_Z_MAX, curr_pos_z + POSCONTROL_STOPPING_DIST_Z_MAX);
}

/// init_takeoff - initialises target altitude if we are taking off
void AC_PosControl::init_takeoff()
{
    const Vector3f& curr_pos = _inav.get_position();

    _pos_target.z = curr_pos.z + POSCONTROL_TAKEOFF_JUMP_CM;

    // freeze feedforward to avoid jump
    freeze_ff_z();

    // shift difference between last motor out and hover throttle into accelerometer I
    _pid_alt_accel.set_integrator(_motors.get_throttle_out()-_throttle_hover);
}

// is_active_z - returns true if the z-axis position controller has been run very recently
bool AC_PosControl::is_active_z() const
{
    return ((hal.scheduler->millis() - _last_update_z_ms) <= POSCONTROL_ACTIVE_TIMEOUT_MS);
}

/// update_z_controller - fly to altitude in cm above home
void AC_PosControl::update_z_controller()
{
    // check time since last cast
    uint32_t now = hal.scheduler->millis();
    if (now - _last_update_z_ms > POSCONTROL_ACTIVE_TIMEOUT_MS) {
        _flags.reset_rate_to_accel_z = true;
        _flags.reset_accel_to_throttle = true;
    }
    _last_update_z_ms = now;

    // check if leash lengths need to be recalculated
    calc_leash_length_z();

    // call position controller
    pos_to_rate_z();
}

/// calc_leash_length - calculates the vertical leash lengths from maximum speed, acceleration
///     called by pos_to_rate_z if z-axis speed or accelerations are changed
void AC_PosControl::calc_leash_length_z()
{
    if (_flags.recalc_leash_z) {
        _leash_up_z = calc_leash_length(_speed_up_cms, _accel_z_cms, _p_alt_pos.kP());
        _leash_down_z = calc_leash_length(-_speed_down_cms, _accel_z_cms, _p_alt_pos.kP());
        _flags.recalc_leash_z = false;
    }
}

// pos_to_rate_z - position to rate controller for Z axis
// calculates desired rate in earth-frame z axis and passes to rate controller
// vel_up_max, vel_down_max should have already been set before calling this method
void AC_PosControl::pos_to_rate_z()
{
    float curr_alt = _inav.get_altitude();
    float linear_distance;  // half the distance we swap between linear and sqrt and the distance we offset sqrt.

    // clear position limit flags
    _limit.pos_up = false;
    _limit.pos_down = false;

    // do not let target alt get above limit
    if (_alt_max > 0 && _pos_target.z > _alt_max) {
        _pos_target.z = _alt_max;
        _limit.pos_up = true;
    }

    // calculate altitude error
    _pos_error.z = _pos_target.z - curr_alt;

    // do not let target altitude get too far from current altitude
    if (_pos_error.z > _leash_up_z) {
        _pos_target.z = curr_alt + _leash_up_z;
        _pos_error.z = _leash_up_z;
        _limit.pos_up = true;
    }
    if (_pos_error.z < -_leash_down_z) {
        _pos_target.z = curr_alt - _leash_down_z;
        _pos_error.z = -_leash_down_z;
        _limit.pos_down = true;
    }

    // check kP to avoid division by zero
    if (_p_alt_pos.kP() != 0.0f) {
        linear_distance = _accel_z_cms/(2.0f*_p_alt_pos.kP()*_p_alt_pos.kP());
        if (_pos_error.z > 2*linear_distance ) {
            _vel_target.z = safe_sqrt(2.0f*_accel_z_cms*(_pos_error.z-linear_distance));
        }else if (_pos_error.z < -2.0f*linear_distance) {
            _vel_target.z = -safe_sqrt(2.0f*_accel_z_cms*(-_pos_error.z-linear_distance));
        }else{
            _vel_target.z = _p_alt_pos.get_p(_pos_error.z);
        }
    }else{
        _vel_target.z = 0;
    }

    // call rate based throttle controller which will update accel based throttle controller targets
    rate_to_accel_z();
}

// rate_to_accel_z - calculates desired accel required to achieve the velocity target
// calculates desired acceleration and calls accel throttle controller
void AC_PosControl::rate_to_accel_z()
{
    const Vector3f& curr_vel = _inav.get_velocity();
    float p;                                // used to capture pid values for logging
    float desired_accel;                    // the target acceleration if the accel based throttle is enabled, otherwise the output to be sent to the motors

    // check speed limits
    // To-Do: check these speed limits here or in the pos->rate controller
    _limit.vel_up = false;
    _limit.vel_down = false;
    if (_vel_target.z < _speed_down_cms) {
        _vel_target.z = _speed_down_cms;
        _limit.vel_down = true;
    }
    if (_vel_target.z > _speed_up_cms) {
        _vel_target.z = _speed_up_cms;
        _limit.vel_up = true;
    }

    // reset last velocity target to current target
    if (_flags.reset_rate_to_accel_z) {
        _vel_last.z = _vel_target.z;
        _flags.reset_rate_to_accel_z = false;
    }

    // feed forward desired acceleration calculation
    if (_dt > 0.0f) {
    	if (!_flags.freeze_ff_z) {
    		_accel_feedforward.z = (_vel_target.z - _vel_last.z)/_dt;
        } else {
    		// stop the feed forward being calculated during a known discontinuity
    		_flags.freeze_ff_z = false;
    	}
    } else {
    	_accel_feedforward.z = 0.0f;
    }

    // store this iteration's velocities for the next iteration
    _vel_last.z = _vel_target.z;

    // reset velocity error and filter if this controller has just been engaged
    if (_flags.reset_rate_to_accel_z) {
        // Reset Filter
        _vel_error.z = 0;
        desired_accel = 0;
        _flags.reset_rate_to_accel_z = false;
    } else {
        _vel_error.z = (_vel_target.z - curr_vel.z);
    }

    // calculate p
    p = _p_alt_rate.kP() * _vel_error.z;

    // consolidate and constrain target acceleration
    desired_accel = _accel_feedforward.z + p;
    desired_accel = constrain_int32(desired_accel, -32000, 32000);

    // set target for accel based throttle controller
    accel_to_throttle(desired_accel);
}

// accel_to_throttle - alt hold's acceleration controller
// calculates a desired throttle which is sent directly to the motors
void AC_PosControl::accel_to_throttle(float accel_target_z)
{
    float z_accel_meas;         // actual acceleration
    int32_t p,i,d;              // used to capture pid values for logging

    // Calculate Earth Frame Z acceleration
    z_accel_meas = -(_ahrs.get_accel_ef().z + GRAVITY_MSS) * 100.0f;

    // reset target altitude if this controller has just been engaged
    if (_flags.reset_accel_to_throttle) {
        // Reset Filter
        _accel_error.z = 0;
        _flags.reset_accel_to_throttle = false;
    } else {
        // calculate accel error and Filter with fc = 2 Hz
        // To-Do: replace constant below with one that is adjusted for update rate
        _accel_error.z = _accel_error.z + 0.11164f * (constrain_float(accel_target_z - z_accel_meas, -32000, 32000) - _accel_error.z);
    }

    // separately calculate p, i, d values for logging
    p = _pid_alt_accel.get_p(_accel_error.z);

    // get i term
    i = _pid_alt_accel.get_integrator();

    // update i term as long as we haven't breached the limits or the I term will certainly reduce
    // To-Do: should this be replaced with limits check from attitude_controller?
    if ((!_motors.limit.throttle_lower && !_motors.limit.throttle_upper) || (i>0&&_accel_error.z<0) || (i<0&&_accel_error.z>0)) {
        i = _pid_alt_accel.get_i(_accel_error.z, _dt);
    }

    // get d term
    d = _pid_alt_accel.get_d(_accel_error.z, _dt);

    // To-Do: pull min/max throttle from motors
    // To-Do: we had a contraint here but it's now removed, is this ok?  with the motors library handle it ok?
    _attitude_control.set_throttle_out((int16_t)p+i+d+_throttle_hover, true);
    
    // to-do add back in PID logging?
}

///
/// position controller
///

/// set_accel_xy - set horizontal acceleration in cm/s/s
///     calc_leash_length_xy should be called afterwards
void AC_PosControl::set_accel_xy(float accel_cmss)
{
    if ((float)fabs(_accel_cms-accel_cmss) > 1.0f) {
        _accel_cms = accel_cmss;
        _oa_brake_xy_acc = _accel_cms*OA_ACCEL_XY_BRAKE_RATIO;
        _flags.recalc_leash_xy = true;
    }
}

/// set_speed_xy - set horizontal speed maximum in cm/s
///     calc_leash_length_xy should be called afterwards
void AC_PosControl::set_speed_xy(float speed_cms)
{
    if ((float)fabs(_speed_cms-speed_cms) > 1.0f) {
        _speed_cms = speed_cms;
        _flags.recalc_leash_xy = true;
    }
}

/// set_pos_target in cm from home
void AC_PosControl::set_pos_target(const Vector3f& position)
{
    _pos_target = position;

    // initialise roll and pitch to current roll and pitch.  This avoids a twitch between when the target is set and the pos controller is first run
    // To-Do: this initialisation of roll and pitch targets needs to go somewhere between when pos-control is initialised and when it completes it's first cycle
    //_roll_target = constrain_int32(_ahrs.roll_sensor,-_attitude_control.lean_angle_max(),_attitude_control.lean_angle_max());
    //_pitch_target = constrain_int32(_ahrs.pitch_sensor,-_attitude_control.lean_angle_max(),_attitude_control.lean_angle_max());
}

/// set_xy_target in cm from home
void AC_PosControl::set_xy_target(float x, float y)
{
    _pos_target.x = x;
    _pos_target.y = y;
}

/// set_target_to_stopping_point_xy - sets horizontal target to reasonable stopping position in cm from home
void AC_PosControl::set_target_to_stopping_point_xy()
{
    // check if xy leash needs to be recalculated
    calc_leash_length_xy();

    get_stopping_point_xy(_pos_target);
}

/// get_stopping_point_xy - calculates stopping point based on current position, velocity, vehicle acceleration
///     distance_max allows limiting distance to stopping point
///     results placed in stopping_position vector
///     set_accel_xy() should be called before this method to set vehicle acceleration
///     set_leash_length() should have been called before this method
void AC_PosControl::get_stopping_point_xy(Vector3f &stopping_point) const
{
	const Vector3f curr_pos = _inav.get_position();
	Vector3f curr_vel = _inav.get_velocity();
    float linear_distance;      // the distance at which we swap from a linear to sqrt response
    float linear_velocity;      // the velocity above which we swap from a linear to sqrt response
    float stopping_dist;		// the distance within the vehicle can stop
    float kP = _p_pos_xy.kP();

    // add velocity error to current velocity
    if (is_active_xy()) {
        curr_vel.x += _vel_error.x;
        curr_vel.y += _vel_error.y;
    }

    // calculate current velocity
    float vel_total = pythagorous2(curr_vel.x, curr_vel.y);

    // avoid divide by zero by using current position if the velocity is below 10cm/s, kP is very low or acceleration is zero
    if (kP <= 0.0f || _accel_cms <= 0.0f) {
        stopping_point.x = curr_pos.x;
        stopping_point.y = curr_pos.y;
        return;
    }

    // calculate point at which velocity switches from linear to sqrt
    linear_velocity = _accel_cms/kP;

    // calculate distance within which we can stop
    if (vel_total < linear_velocity) {
    	stopping_dist = vel_total/kP;
    } else {
        linear_distance = _accel_cms/(2.0f*kP*kP);
        stopping_dist = linear_distance + (vel_total*vel_total)/(2.0f*_accel_cms);
    }

    // constrain stopping distance
    stopping_dist = constrain_float(stopping_dist, 0, _leash);

    // convert the stopping distance into a stopping point using velocity vector
    stopping_point.x = curr_pos.x + (stopping_dist * curr_vel.x / vel_total);
    stopping_point.y = curr_pos.y + (stopping_dist * curr_vel.y / vel_total);
}

/// get_distance_to_target - get horizontal distance to loiter target in cm
float AC_PosControl::get_distance_to_target() const
{
    return _distance_to_target;
}

// is_active_xy - returns true if the xy position controller has been run very recently
bool AC_PosControl::is_active_xy() const
{
    return ((hal.scheduler->millis() - _last_update_xy_ms) <= POSCONTROL_ACTIVE_TIMEOUT_MS);
}

/// init_xy_controller - initialise the xy controller
///     sets target roll angle, pitch angle and I terms based on vehicle current lean angles
///     should be called once whenever significant changes to the position target are made
///     this does not update the xy target
void AC_PosControl::init_xy_controller()
{
    // reset xy controller to first step
    _xy_step = 0;

    // set roll, pitch lean angle targets to current attitude
    _roll_target = _ahrs.roll_sensor;
    _pitch_target = _ahrs.pitch_sensor;

    // initialise I terms from lean angles
    if (!_flags.keep_xy_I_terms) {
        // reset last velocity if this controller has just been engaged or dt is zero
        lean_angles_to_accel(_accel_target.x, _accel_target.y);
        _pid_rate_lat.set_integrator(_accel_target.x);
        _pid_rate_lon.set_integrator(_accel_target.y);
    } else {
        // reset keep_xy_I_term flag in case it has been set
        _flags.keep_xy_I_terms = false;
    }

    // flag reset required in rate to accel step
    _flags.reset_desired_vel_to_pos = true;
    _flags.reset_rate_to_accel_xy = true;

    // update update time
    _last_update_xy_ms = hal.scheduler->millis();
}

/// update_xy_controller - run the horizontal position controller - should be called at 100hz or higher
void AC_PosControl::update_xy_controller(bool use_desired_velocity)
{
    // catch if we've just been started
    uint32_t now = hal.scheduler->millis();
    if ((now - _last_update_xy_ms) >= POSCONTROL_ACTIVE_TIMEOUT_MS) {
        init_xy_controller();
    }

    // check if xy leash needs to be recalculated
    calc_leash_length_xy();

    // reset step back to 0 if loiter or waypoint parents have triggered an update and we completed the last full cycle
    if (_flags.force_recalc_xy && _xy_step > 3) {
        _flags.force_recalc_xy = false;
        _xy_step = 0;
    }

    // run loiter steps
    switch (_xy_step) {
        case 0:
            // capture time since last iteration
            _dt_xy = (now - _last_update_xy_ms) / 1000.0f;
            _last_update_xy_ms = now;

            // translate any adjustments from pilot to loiter target
            desired_vel_to_pos(_dt_xy);
            _xy_step++;
            break;
        case 1:
            // run position controller's position error to desired velocity step
            pos_to_rate_xy(use_desired_velocity,_dt_xy);
            _xy_step++;
            break;
        case 2:
            // run position controller's velocity to acceleration step
            rate_to_accel_xy(_dt_xy);
            _xy_step++;
            break;
        case 3:
            // run position controller's acceleration to lean angle step
            accel_to_lean_angles();
            _xy_step++;
            break;
    }
}

/// init_vel_controller_xyz - initialise the velocity controller - should be called once before the caller attempts to use the controller
void AC_PosControl::init_vel_controller_xyz()
{
    // force the xy velocity controller to run immediately
    _vel_xyz_step = 3;

    // set roll, pitch lean angle targets to current attitude
    _roll_target = _ahrs.roll_sensor;
    _pitch_target = _ahrs.pitch_sensor;

    // reset last velocity if this controller has just been engaged or dt is zero
    lean_angles_to_accel(_accel_target.x, _accel_target.y);
    _pid_rate_lat.set_integrator(_accel_target.x);
    _pid_rate_lon.set_integrator(_accel_target.y);

    // flag reset required in rate to accel step
    _flags.reset_desired_vel_to_pos = true;
    _flags.reset_rate_to_accel_xy = true;

    // set target position in xy axis
    const Vector3f& curr_pos = _inav.get_position();
    set_xy_target(curr_pos.x, curr_pos.y);

    // move current vehicle velocity into feed forward velocity
    const Vector3f& curr_vel = _inav.get_velocity();
    set_desired_velocity_xy(curr_vel.x, curr_vel.y);

    // record update time
    _last_update_vel_xyz_ms = hal.scheduler->millis();
}

/// update_velocity_controller_xyz - run the velocity controller - should be called at 100hz or higher
///     velocity targets should we set using set_desired_velocity_xyz() method
///     callers should use get_roll() and get_pitch() methods and sent to the attitude controller
///     throttle targets will be sent directly to the motors
void AC_PosControl::update_vel_controller_xyz()
{
    // capture time since last iteration
    uint32_t now = hal.scheduler->millis();
    float dt_xy = (now - _last_update_vel_xyz_ms) / 1000.0f;

    // check if xy leash needs to be recalculated
    calc_leash_length_xy();

    // we will run the horizontal component every 4th iteration (i.e. 50hz on Pixhawk, 20hz on APM)
    if (dt_xy >= POSCONTROL_VEL_UPDATE_TIME) {

        // record update time
        _last_update_vel_xyz_ms = now;

        // sanity check dt
        if (dt_xy >= POSCONTROL_ACTIVE_TIMEOUT_MS) {
            dt_xy = 0.0f;
        }

        // apply desired velocity request to position target
        desired_vel_to_pos(dt_xy);

        // run position controller's position error to desired velocity step
        pos_to_rate_xy(true, dt_xy);

        // run velocity to acceleration step
        rate_to_accel_xy(dt_xy);

        // run acceleration to lean angle step
        accel_to_lean_angles();
    }

    // update altitude target
    set_alt_target_from_climb_rate(_vel_desired.z, _dt);

    // run z-axis position controller
    update_z_controller();
}

///
/// private methods
///

/// calc_leash_length - calculates the horizontal leash length given a maximum speed, acceleration
///     should be called whenever the speed, acceleration or position kP is modified
void AC_PosControl::calc_leash_length_xy()
{
    if (_flags.recalc_leash_xy) {
        _leash = calc_leash_length(_speed_cms, _accel_cms, _p_pos_xy.kP());
        _flags.recalc_leash_xy = false;
    }
}

/// desired_vel_to_pos - move position target using desired velocities
void AC_PosControl::desired_vel_to_pos(float nav_dt)
{
    // range check nav_dt
    if( nav_dt < 0 ) {
        return;
    }

    // update target position
    if (_flags.reset_desired_vel_to_pos) {
        _flags.reset_desired_vel_to_pos = false;
    } else {
        _pos_target.x += _vel_desired.x * nav_dt;
        _pos_target.y += _vel_desired.y * nav_dt;
    }
}

/// pos_to_rate_xy - horizontal position error to velocity controller
///     converts position (_pos_target) to target velocity (_vel_target)
///     when use_desired_rate is set to true:
///         desired velocity (_vel_desired) is combined into final target velocity and
///         velocity due to position error is reduce to a maximum of 1m/s
void AC_PosControl::pos_to_rate_xy(bool use_desired_rate, float dt)
{
    Vector3f curr_pos = _inav.get_position();
    float linear_distance;      // the distance we swap between linear and sqrt velocity response
    float kP = _p_pos_xy.kP();

    // avoid divide by zero
    if (kP <= 0.0f) {
        _vel_target.x = 0.0f;
        _vel_target.y = 0.0f;
    }else{
        // calculate distance error
        _pos_error.x = _pos_target.x - curr_pos.x;
        _pos_error.y = _pos_target.y - curr_pos.y;

        // constrain target position to within reasonable distance of current location
        _distance_to_target = pythagorous2(_pos_error.x, _pos_error.y);
        if (_distance_to_target > _leash && _distance_to_target > 0.0f) {
            _pos_target.x = curr_pos.x + _leash * _pos_error.x/_distance_to_target;
            _pos_target.y = curr_pos.y + _leash * _pos_error.y/_distance_to_target;
            // re-calculate distance error
            _pos_error.x = _pos_target.x - curr_pos.x;
            _pos_error.y = _pos_target.y - curr_pos.y;
            _distance_to_target = _leash;
        }

        // calculate the distance at which we swap between linear and sqrt velocity response
        linear_distance = _accel_cms/(2.0f*kP*kP);

        if (_distance_to_target > 2.0f*linear_distance) {
            // velocity response grows with the square root of the distance
            float vel_sqrt = safe_sqrt(2.0f*_accel_cms*(_distance_to_target-linear_distance));
            _vel_target.x = vel_sqrt * _pos_error.x/_distance_to_target;
            _vel_target.y = vel_sqrt * _pos_error.y/_distance_to_target;
        }else{
            // velocity response grows linearly with the distance
            _vel_target.x = _p_pos_xy.kP() * _pos_error.x;
            _vel_target.y = _p_pos_xy.kP() * _pos_error.y;
        }

        // decide velocity limit due to position error
        float vel_max_from_pos_error;
        if (use_desired_rate) {
            // if desired velocity (i.e. velocity feed forward) is being used we limit the maximum velocity correction due to position error to 2m/s
            vel_max_from_pos_error = POSCONTROL_VEL_XY_MAX_FROM_POS_ERR;
        }else{
            // if desired velocity is not used, we allow position error to increase speed up to maximum speed
            vel_max_from_pos_error = _speed_cms;
        }

        // scale velocity to stays within limits
        float vel_total = pythagorous2(_vel_target.x, _vel_target.y);
        if (vel_total > vel_max_from_pos_error) {
            _vel_target.x = vel_max_from_pos_error * _vel_target.x/vel_total;
            _vel_target.y = vel_max_from_pos_error * _vel_target.y/vel_total;
        }

        // add desired velocity (i.e. feed forward).
        if (use_desired_rate) {
            _vel_target.x += _vel_desired.x;
            _vel_target.y += _vel_desired.y;
        }
    }
}

/// rate_to_accel_xy - horizontal desired rate to desired acceleration
///    converts desired velocities in lat/lon directions to accelerations in lat/lon frame
void AC_PosControl::rate_to_accel_xy(float dt)
{
    const Vector3f &vel_curr = _inav.get_velocity();  // current velocity in cm/s
    float accel_total;                          // total acceleration in cm/s/s
    float lat_i, lon_i;

    // reset last velocity target to current target
    if (_flags.reset_rate_to_accel_xy) {
        _vel_last.x = _vel_target.x;
        _vel_last.y = _vel_target.y;
        _flags.reset_rate_to_accel_xy = false;
    }

    // feed forward desired acceleration calculation
    if (dt > 0.0f) {
    	if (!_flags.freeze_ff_xy) {
    		_accel_feedforward.x = (_vel_target.x - _vel_last.x)/dt;
    		_accel_feedforward.y = (_vel_target.y - _vel_last.y)/dt;
        } else {
    		// stop the feed forward being calculated during a known discontinuity
    		_flags.freeze_ff_xy = false;
    	}
    } else {
    	_accel_feedforward.x = 0.0f;
    	_accel_feedforward.y = 0.0f;
    }

    // store this iteration's velocities for the next iteration
    _vel_last.x = _vel_target.x;
    _vel_last.y = _vel_target.y;

    // calculate velocity error
    _vel_error.x = _vel_target.x - vel_curr.x;
    _vel_error.y = _vel_target.y - vel_curr.y;

    // get current i term
    lat_i = _pid_rate_lat.get_integrator();
    lon_i = _pid_rate_lon.get_integrator();

    // update i term if we have not hit the accel or throttle limits OR the i term will reduce
    if ((!_limit.accel_xy && !_motors.limit.throttle_upper) || ((lat_i>0&&_vel_error.x<0)||(lat_i<0&&_vel_error.x>0))) {
        lat_i = _pid_rate_lat.get_i(_vel_error.x, dt);
    }
    if ((!_limit.accel_xy && !_motors.limit.throttle_upper) || ((lon_i>0&&_vel_error.y<0)||(lon_i<0&&_vel_error.y>0))) {
        lon_i = _pid_rate_lon.get_i(_vel_error.y, dt);
    }

    // combine feed forward accel with PID output from velocity error
    _accel_target.x = _accel_feedforward.x + _pid_rate_lat.get_p(_vel_error.x) + lat_i + _pid_rate_lat.get_d(_vel_error.x, dt);
    _accel_target.y = _accel_feedforward.y + _pid_rate_lon.get_p(_vel_error.y) + lon_i + _pid_rate_lon.get_d(_vel_error.y, dt);

    // scale desired acceleration if it's beyond acceptable limit
    // To-Do: move this check down to the accel_to_lean_angle method?
    accel_total = pythagorous2(_accel_target.x, _accel_target.y);
    if (accel_total > POSCONTROL_ACCEL_XY_MAX && accel_total > 0.0f) {
        _accel_target.x = POSCONTROL_ACCEL_XY_MAX * _accel_target.x/accel_total;
        _accel_target.y = POSCONTROL_ACCEL_XY_MAX * _accel_target.y/accel_total;
        _limit.accel_xy = true;     // unused
    } else {
        // reset accel limit flag
        _limit.accel_xy = false;
    }
}

/// accel_to_lean_angles - horizontal desired acceleration to lean angles
///    converts desired accelerations provided in lat/lon frame to roll/pitch angles
void AC_PosControl::accel_to_lean_angles()
{
    float accel_right, accel_forward;
    float lean_angle_max = _attitude_control.lean_angle_max();

    // To-Do: add 1hz filter to accel_lat, accel_lon

    // rotate accelerations into body forward-right frame
    accel_forward = _accel_target.x*_ahrs.cos_yaw() + _accel_target.y*_ahrs.sin_yaw();
    accel_right = -_accel_target.x*_ahrs.sin_yaw() + _accel_target.y*_ahrs.cos_yaw();

    // update angle targets that will be passed to stabilize controller
    _roll_target = constrain_float(fast_atan(accel_right*_ahrs.cos_pitch()/(GRAVITY_MSS * 100))*(18000/M_PI), -lean_angle_max, lean_angle_max);
    _pitch_target = constrain_float(fast_atan(-accel_forward/(GRAVITY_MSS * 100))*(18000/M_PI),-lean_angle_max, lean_angle_max);
}

// get_lean_angles_to_accel - convert roll, pitch lean angles to lat/lon frame accelerations in cm/s/s
void AC_PosControl::lean_angles_to_accel(float& accel_x_cmss, float& accel_y_cmss) const
{
    // rotate our roll, pitch angles into lat/lon frame
    accel_x_cmss = (GRAVITY_MSS * 100) * (-(_ahrs.cos_yaw() * _ahrs.sin_pitch() / max(_ahrs.cos_pitch(),0.5f)) - _ahrs.sin_yaw() * _ahrs.sin_roll() / max(_ahrs.cos_roll(),0.5f));
    accel_y_cmss = (GRAVITY_MSS * 100) * (-(_ahrs.sin_yaw() * _ahrs.sin_pitch() / max(_ahrs.cos_pitch(),0.5f)) + _ahrs.cos_yaw() * _ahrs.sin_roll() / max(_ahrs.cos_roll(),0.5f));
}

/// calc_leash_length - calculates the horizontal leash length given a maximum speed, acceleration and position kP gain
float AC_PosControl::calc_leash_length(float speed_cms, float accel_cms, float kP) const
{
    float leash_length;

    // sanity check acceleration and avoid divide by zero
    if (accel_cms <= 0.0f) {
        accel_cms = POSCONTROL_ACCELERATION_MIN;
    }

    // avoid divide by zero
    if (kP <= 0.0f) {
        return POSCONTROL_LEASH_LENGTH_MIN;
    }

    // calculate leash length
    if(speed_cms <= accel_cms / kP) {
        // linear leash length based on speed close in
        leash_length = speed_cms / kP;
    }else{
        // leash length grows at sqrt of speed further out
        leash_length = (accel_cms / (2.0f*kP*kP)) + (speed_cms*speed_cms / (2.0f*accel_cms));
    }

    // ensure leash is at least 1m long
    if( leash_length < POSCONTROL_LEASH_LENGTH_MIN ) {
        leash_length = POSCONTROL_LEASH_LENGTH_MIN;
    }

    return leash_length;
}

// oa_check_and_correct_desired_vel - apply OA limits & commands to desired_vel - called by the calc_loiter_desired_velocity() function
void AC_PosControl::oa_check_and_correct_desired_xy_vel(float dt)
{
    float kP = _p_pos_xy.kP();
    float des_vel_total = pythagorous2(_vel_desired.x, _vel_desired.y);     // based on user inputs and/or oa_vel limitations to pilot inputs (desired velocity)
    float des_acc_cmss;                                                     // used to limit pilot inputs (des_vel)    
    float stopping_dist;                                                    // based on user inputs and/or oa_vel limitations to pilot inputs (desired velocity)
    float max_stopping_dist = (float)_oa_max_braking_dist;                  // based on obj_detect measurement. max_stopping_dist = object_dist - safety_dist
    float max_safe_vel;                                                     // based on obj_detect measurement
    float linear_distance;                                                  // based on _oa_brake_xy_acc
    float linear_velocity;                                                  // based on _oa_brake_xy_acc      
    const Vector3f& curr_vel = _inav.get_velocity();                        // based on true (current) velocity
    float curr_vel_total = pythagorous2(curr_vel.x, curr_vel.y);            // based on true (current) velocity
    
    // avoid divide by zero if kP is very low or acceleration is zero
    if (kP <= 0.0f || _accel_cms <= 0.0f) {
        //stopping_dist = 0.0f;
        return;
    }
    
    _oa_brake_xy_acc = _accel_cms*OA_ACCEL_XY_BRAKE_RATIO;    
    // calculate point at which velocity switches from linear to sqrt
    linear_velocity = _oa_brake_xy_acc/kP;    
    // compute max safe velocity
    max_safe_vel = max_stopping_dist * kP;
    if (max_safe_vel > linear_velocity) {
        max_safe_vel = safe_sqrt(_oa_brake_xy_acc*(2*kP*kP*max_stopping_dist-_oa_brake_xy_acc))/kP;
    }
    
    // Here the different cases
    // case 1 : no object but limited safe distance (current cleared range), so limit max desired velocity
    // case 2 : object detected in the current path, check velocity and, when needed, engage a braking procedure to stop in time
    // case 3 : copter in the red zone (within the safety margin), has to engage a bounce or avoidance procedure (side or back step)
    
    // case 1: no object but limited range + pilot stick input not null => limit max desired velocity - human pilot mode only
    if((!_oa_is_object_detected)&&(des_vel_total>5.0f)){    //5 = no_vel but avoids /0
        // Pilot is trying to go too fast, limit the desired_velocity 
        if (des_vel_total > max_safe_vel) {
            // limit desired velocity
            if(curr_vel_total<max_safe_vel){    // if current velocity is still under the max_safe_vel, just limit the des_vel to max_safe_vel
                // do nothing, max_safe_vel value unchanged
                _flag_init_oa_control_des_vel_total = true;
            }else{  // current vel is too high, apply a speed reduction starting from curr_vel and compute an intermediary max_safe_vel (temporarly too high but we need smooth vel transitions (low pass filter)
                // we don't use the limited _oa_brake_xy_acc but the whole _accel_cms here as it is the speed limitation, we don't evaluate a threshold with headroom but do use the whole acc if needed
                if(_flag_init_oa_control_des_vel_total){
                    _oa_control_des_vel_total = curr_vel_total; // set initial velocity of the oa speed reduction curve
                    _flag_init_oa_control_des_vel_total = false;
                }
                des_acc_cmss = _accel_cms*(max_safe_vel-_oa_control_des_vel_total)/400.0f; // 400cms is set arbitrary. that means we will accelerate at the max _accel_cms when the difference between max_safe_vel and des_vel_total is >= 400cms
                _oa_control_des_vel_total += constrain_float(des_acc_cmss,-_accel_cms,_accel_cms)*dt;
                max_safe_vel = _oa_control_des_vel_total;
            }
        }else{  // des_vel acceptable, do not limit pilot inputs
            _flag_init_oa_control_des_vel_total = true;
        }
    }else{
        _flag_init_oa_control_des_vel_total = true;
    }
    
    // case 2 : object detected in the moving direction + copter moving, engage a braking procedure to stop in time - could be used for every pos_controlled flight modes
    if((_oa_is_object_detected)&&(curr_vel_total>5.0f)){    //the "5 cms" value represents the OA_VEL_0 of OA code.
        if(max_stopping_dist<=0.0f){           // max_stopping_dist could be negative if the copter is already inside the safety margin
            max_stopping_dist = 1.0f;          // so, set it to a minimal and not null value (avoid /0) to try and brake anyways (even if too late, do our best)
        }
        // if current velocity is over max safe velocity, engage braking. otherwise, do nothing
        if (curr_vel_total > max_safe_vel) {
            //_oa_brake_xy_acc est utilisé pour calculer stopping_dist, laissant ainsi un peu de marge pour le freinage. Ici on va donc calculer et appliquer la bonne accélération pour s'arrêter exactement à la limite
            // si la valeur d'accélération est au-delà de _accel_cms, alors on est cuit, envisager éventuellement une manoeuvre d'éviction, idem si max-stopping_dist est déjà négatif. cela revient au cas 3
            // compute the right acceleration to get vel = 0 at the safety margin limit position.
            if(max_stopping_dist<des_vel_total){ //linear
                _oa_brake_xy_acc = des_vel_total*des_vel_total/max_stopping_dist;
            }else{  //sqrt
                _oa_brake_xy_acc = kP*kP*max_stopping_dist - kP*safe_sqrt((kP*max_stopping_dist-des_vel_total)*(kP*max_stopping_dist+des_vel_total));
            }
            if(_flag_init_oa_brake_vel_total){
                _oa_brake_vel_total = curr_vel_total; // set initial velocity of the oa brake curve
                _flag_init_oa_brake_vel_total = false;
            }
            // limit _oa_brake_xy_acc to a decent value
            _oa_brake_vel_total -= constrain_float(_oa_brake_xy_acc,-_accel_cms,_accel_cms)*dt;
            max_safe_vel = _oa_brake_vel_total;
        }else{
            _flag_init_oa_brake_vel_total = true;
        }
    }else{
        _flag_init_oa_brake_vel_total = true;
    }
    
    // Apply _vel_desired.x/y corrections for both cases 1 or 2
    max_safe_vel = max(max_safe_vel, 0.0f);
    _vel_desired.x = max_safe_vel*_vel_desired.x/des_vel_total;
    _vel_desired.y = max_safe_vel*_vel_desired.y/des_vel_total;
    // to-do: compute the reduction factor = max_safe_vel/des_vel_total to apply that same ratio for z controller (if possible). and maybe also in the other way (from z to xy)?
    
    // case 3 : copter in the red zone (within the safety margin), has to engage an avoidance procedure (side or back step)
    //if(oa_object_in_safety_zone()?), limited to cases where pilot gives no input, or only with limited vel, ... that case is the most variable
    // TBD
    /*
    Créer fonction permettant de vérifier ds le mapping les cases environnantes et en déduire le target point
    Voir pour procédure d'éviction avancée en carré style toreador au lieu d'une marche arrière basique et potentiellement sans fin.
    */
    // use set_xy_target() ?    
    // no velocity, set des_vel to 0 ?
    //_vel_desired.x = 0.0f; ?
    //_vel_desired.y = 0.0f; ?
}