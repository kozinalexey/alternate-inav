/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#include <AP_HAL.h>
#include <AP_InertialNav.h>

extern const AP_HAL::HAL& hal;

// table of user settable parameters
const AP_Param::GroupInfo AP_InertialNav::var_info[] PROGMEM = {
    // koef for correction inav xy position from gps  
    // @Param: GPS_XY_POS
    // @DisplayName: Horizontal position gain for pool position inav to gps
    // @Description: big values pool position faster to gps
    // @Range: 0 1000
    // @Increment: 0.1
    AP_GROUPINFO("GPS_XY_POS",   1, AP_InertialNav, _gps_k_pos, 100),

    // @Param: TC_Z
    // @DisplayName: Vertical Time Constant
    // @Description: Time constant for baro and accel mixing. Higher TC decreases barometers impact on altitude estimate
    // @Range: 0 10
    // @Increment: 0.1
    AP_GROUPINFO("TC_Z",    2, AP_InertialNav, _time_constant_z, AP_INTERTIALNAV_TC_Z),

    // koef for correction inav xy speed from gps range 0-100 
    // @Param: GPS_XY_SPD
    // @DisplayName: inav speed correction
    // @Description: 0=no use gps, 100 = use gps speed only
    // @Range: 0 100
    // @Increment: 1
    AP_GROUPINFO("GPS_XY_SPD",   3, AP_InertialNav, _gps_k_spd, 100),

    AP_GROUPEND
};

// init - initialise library
void AP_InertialNav::init()
{
    // recalculate the gains
    update_gains();
}

// update - updates velocities and positions using latest info from ahrs and barometer if new data is available;
void AP_InertialNav::update(float dt)
{
    // discard samples where dt is too large
    if( dt > 0.1f ) {
        return;
    }

    // decrement ignore error count if required
    if (_flags.ignore_error > 0) {
        _flags.ignore_error--;
    }

    // check if new baro readings have arrived and use them to correct vertical accelerometer offsets.
    check_baro();

    // check if new gps readings have arrived and use them to correct position estimates
    check_gps();

    Vector3f accel_ef = _ahrs.get_accel_ef();
	Vector3f accel_ef_prev ;

    // remove influence of gravity
    accel_ef.z += GRAVITY_MSS;
    accel_ef *= 100.0f; //meters to cm per seconds
	
    // remove xy if not enabled
    if( !_xy_enabled ) {
        accel_ef.x = 0.0f;
        accel_ef.y = 0.0f;
    }

    //Convert North-East-Down to North-East-Up
    accel_ef.z = -accel_ef.z;

    // convert ef position error to horizontal body frame
    Vector2f position_error_hbf;
   // position_error_hbf.x = _position_error.x * _ahrs.cos_yaw() + _position_error.y * _ahrs.sin_yaw();
   // position_error_hbf.y = -_position_error.x * _ahrs.sin_yaw() + _position_error.y * _ahrs.cos_yaw();

   // float tmp = _k3_xy * dt;
   // accel_correction_hbf.x += position_error_hbf.x * tmp;
   // accel_correction_hbf.y += position_error_hbf.y * tmp;
    accel_correction_hbf.z += _position_error.z * _k3_z  * dt;

   // tmp = _k2_xy * dt;
	//if (_gps_k==0) { //without gps velocity corection
    //_velocity.x += _position_error.x * tmp;
    //_velocity.y += _position_error.y * tmp;
	//} 
    _velocity.z += _position_error.z * _k2_z  * dt;

   // tmp = _k1_xy * dt;
   // _position_correction.x += _position_error.x * tmp;
   // _position_correction.y += _position_error.y * tmp;
    _position_correction.z += _position_error.z * _k1_z  * dt;

    // convert horizontal body frame accel correction to earth frame
    //Vector2f accel_correction_ef;
    //accel_correction_ef.x = accel_correction_hbf.x * _ahrs.cos_yaw() - accel_correction_hbf.y * _ahrs.sin_yaw();
    //accel_correction_ef.y = accel_correction_hbf.x * _ahrs.sin_yaw() + accel_correction_hbf.y * _ahrs.cos_yaw();


    // calculate velocity increase adding new acceleration from accelerometers
    Vector3f velocity_increase;
   // velocity_increase.x = (accel_ef.x + accel_correction_ef.x) * dt;
   // velocity_increase.y = (accel_ef.y + accel_correction_ef.y) * dt;
    
	velocity_increase.x = (accel_ef.x + accel_ef_prev.x) / 2.f  * dt ; // average acceleration between two samples
	velocity_increase.y = (accel_ef.y + accel_ef_prev.y) / 2.f  * dt ;
	
	accel_ef_prev = accel_ef;

	velocity_increase.z = (accel_ef.z + accel_correction_hbf.z) * dt;

    // calculate new estimate of position
    //_position_base += (_velocity + velocity_increase*0.5) * dt;
	_position_base.z += (_velocity.z + velocity_increase.z * 0.5f) * dt;

    // update the corrected position estimate
    //_position = _position_base + _position_correction;
	_position.z = _position_base.z + _position_correction.z;


	    // calculate new velocity
	_velocity += velocity_increase;

	_position_correction.x = _velocity.x  * dt  ; // S=at2 + V0  // s =V0*t + V1*t
	_position_correction.y = _velocity.y  * dt  ;

	_position = _position + _position_correction;

	_position.x = _position.x * _gps_k_inav_pos +  _gps_position.x * _gps_k_gps_pos; //pool inav position to gps position
	_position.y = _position.y * _gps_k_inav_pos +  _gps_position.y * _gps_k_gps_pos;


	_velocity.x = _velocity.x * _gps_k_inav_spd + _gps_velocity_x * _gps_k_gps_spd ; //pool inav velocity to gps velocity
	_velocity.y = _velocity.y * _gps_k_inav_spd + _gps_velocity_y * _gps_k_gps_spd ;
	

    // store 3rd order estimate (i.e. estimated vertical position) for future use
    _hist_position_estimate_z.push_back(_position_base.z);

    // store 3rd order estimate (i.e. horizontal position) for future use at 10hz
    _historic_xy_counter++;
    if( _historic_xy_counter >= AP_INTERTIALNAV_SAVE_POS_AFTER_ITERATIONS ) {
        _historic_xy_counter = 0;
        _hist_position_estimate_x.push_back(_position_base.x);
        _hist_position_estimate_y.push_back(_position_base.y);
    }
}

//
// XY Axis specific methods
//

// position_ok - return true if position has been initialised and have received gps data within 3 seconds
bool AP_InertialNav::position_ok() const
{
    return _xy_init;
}

// check_gps - check if new gps readings have arrived and use them to correct position estimates
void AP_InertialNav::check_gps()
{
    const uint32_t now = hal.scheduler->millis();

    // compare gps time to previous reading
    const AP_GPS &gps = _ahrs.get_gps();
    if(gps.last_fix_time_ms() != _gps_last_time ) {
    
    Vector3f gpsvelocity = gps.velocity(); 
		_gps_velocity_x = gpsvelocity.x * 100.0f ; //convert from m to cm 
		_gps_velocity_y = gpsvelocity.y * 100.0f ; //convert from m to cm 

        // call position correction method
        correct_with_gps(now, gps.location().lng, gps.location().lat);

        // record gps time and system time of this update
        _gps_last_time = gps.last_fix_time_ms();
    }else{
        // if GPS updates stop arriving degrade position error to 10% over 2 seconds (assumes 100hz update rate)
        if (now - _gps_last_update > AP_INTERTIALNAV_GPS_TIMEOUT_MS) {
           // _position_error.x *= 0.9886f;
           // _position_error.y *= 0.9886f;

			//_position.x = _position.x * 0.998f + _position_base.x * 0.002f;  //pool to base
	        //_position.y = _position.y * 0.998f + _position_base.y * 0.002f;

			_gps_velocity_x =0; //todo remove
			_gps_velocity_y =0; //todo remove
            // increment error count
            if (_flags.ignore_error == 0 && _error_count < 255 && _xy_init) {
                _error_count++;
            }
        }
    }
}

// correct_with_gps - modifies accelerometer offsets using gps
void AP_InertialNav::correct_with_gps(uint32_t now, int32_t lon, int32_t lat)
{
    float dt,x,y;
    float hist_position_base_x, hist_position_base_y;

    // calculate time since last gps reading
    dt = (float)(now - _gps_last_update) * 0.001f;

    // update last gps update time
    _gps_last_update = now;

    // discard samples where dt is too large
    if( dt > 1.0f || dt == 0.0f || !_xy_init) {
        return;
    }



    // calculate distance from base location
    x = (float)(lat - _ahrs.get_home().lat) * LATLON_TO_CM;
    y = (float)(lon - _ahrs.get_home().lng) * _lon_to_cm_scaling;

    // sanity check the gps position.  Relies on the main code calling GPS_Glitch::check_position() immediatley after a GPS update
    if (_glitch_detector.glitching()) {
        // failed sanity check so degrate position_error to 10% over 2 seconds (assumes 5hz update rate)
        _position_error.x *= 0.7943f;
        _position_error.y *= 0.7943f;
    }else{
        // if our internal glitching flag (from previous iteration) is true we have just recovered from a glitch
        // reset the inertial nav position and velocity to gps values
  
		if (_flags.gps_glitching || _gps_k_spd ==0) {
            set_position_xy(x,y);
           // _position_error.x = 0.0f;
           // _position_error.y = 0.0f;
        }
		else
		   {_gps_position.x = x; 	_gps_position.y = y;}

	
/*		else{
            // ublox gps positions are delayed by 400ms
            // we store historical position at 10hz so 4 iterations ago
            if( _hist_position_estimate_x.is_full()) {
                hist_position_base_x = _hist_position_estimate_x.front();
                hist_position_base_y = _hist_position_estimate_y.front();
            }else{
                hist_position_base_x = _position_base.x;
                hist_position_base_y = _position_base.y;
            }

            // calculate error in position from gps with our historical estimate
            _position_error.x = x - (hist_position_base_x + _position_correction.x);
            _position_error.y = y - (hist_position_base_y + _position_correction.y);
        }
*/
	
	}

    // update our internal record of glitching flag so that we can notice a change
    _flags.gps_glitching = _glitch_detector.glitching();
}

// get accel based latitude
int32_t AP_InertialNav::get_latitude() const
{
    // make sure we've been initialised
    if( !_xy_init ) {
        return 0;
    }

    return _ahrs.get_home().lat + (int32_t)(_position.x/LATLON_TO_CM);
}

// get accel based longitude
int32_t AP_InertialNav::get_longitude() const
{
    // make sure we've been initialised
    if( !_xy_init ) {
        return 0;
    }

    return _ahrs.get_home().lng + (int32_t)(_position.y / _lon_to_cm_scaling);
}

// setup_home_position - reset state for home position change
void AP_InertialNav::setup_home_position(void)
{
    // set longitude to meters scaling to offset the shrinking longitude as we go towards the poles
    _lon_to_cm_scaling = longitude_scale(_ahrs.get_home()) * LATLON_TO_CM;

    // reset corrections to base position to zero
    _position_base.x = 0.0f;
    _position_base.y = 0.0f;
    _position_correction.x = 0.0f;
    _position_correction.y = 0.0f;
    _position.x = 0.0f;
    _position.y = 0.0f;

    // clear historic estimates
    _hist_position_estimate_x.clear();
    _hist_position_estimate_y.clear();

    // set xy as enabled
if ( _gps_k_spd !=0)
{
    _xy_enabled = true;
}
   _xy_init =true;
}

// get accel based latitude
float AP_InertialNav::get_latitude_err_cm() const
{
    // make sure we've been initialised
    if( !_xy_init ) {
        return 0;
    }

    return (_position.x - _gps_position.x);
}

// get accel based longitude
float AP_InertialNav::get_longitude_err_cm() const
{
    // make sure we've been initialised
    if( !_xy_init ) {
        return 0.0f;
    }

    return (_position.y - _gps_position.y);
}

// set_velocity_xy - set velocity in latitude & longitude directions (in cm/s)
void AP_InertialNav::set_velocity_xy(float x, float y)
{
    _velocity.x = x;
    _velocity.y = y;
}

// set_velocity_xy - set velocity in latitude & longitude directions (in cm/s)
float AP_InertialNav::get_velocity_xy() const
{
	return pythagorous2(_velocity.x, _velocity.y);
}

//
// Z Axis methods
//

// check_baro - check if new baro readings have arrived and use them to correct vertical accelerometer offsets
void AP_InertialNav::check_baro()
{
    uint32_t baro_update_time;

    // calculate time since last baro reading (in ms)
    baro_update_time = _baro.get_last_update();
    if( baro_update_time != _baro_last_update ) {
        const float dt = (float)(baro_update_time - _baro_last_update) * 0.001f; // in seconds
        // call correction method
        correct_with_baro(_baro.get_altitude()*100.0f, dt);
        _baro_last_update = baro_update_time;
    }
}


// correct_with_baro - modifies accelerometer offsets using barometer.  dt is time since last baro reading
void AP_InertialNav::correct_with_baro(float baro_alt, float dt)
{
    static uint8_t first_reads = 0;

    // discard samples where dt is too large
    if( dt > 0.5f ) {
        return;
    }

    // discard first 10 reads but perform some initialisation
    if( first_reads <= 10 ) {
        set_altitude(baro_alt);
        first_reads++;
    }

    // sanity check the baro position.  Relies on the main code calling Baro_Glitch::check_alt() immediatley after a baro update
    if (_baro_glitch.glitching()) {
        // failed sanity check so degrate position_error to 10% over 2 seconds (assumes 10hz update rate)
        _position_error.z *= 0.89715f;
    }else{
        // if our internal baro glitching flag (from previous iteration) is true we have just recovered from a glitch
        // reset the inertial nav alt to baro alt
        if (_flags.baro_glitching) {
            set_altitude(baro_alt);
            _position_error.z = 0.0f;
        }else{
            // 3rd order samples (i.e. position from baro) are delayed by 150ms (15 iterations at 100hz)
            // so we should calculate error using historical estimates
            float hist_position_base_z;
            if (_hist_position_estimate_z.is_full()) {
                hist_position_base_z = _hist_position_estimate_z.front();
            } else {
                hist_position_base_z = _position_base.z;
            }

            // calculate error in position from baro with our estimate
            _position_error.z = baro_alt - (hist_position_base_z + _position_correction.z);
        }
    }

    // update our internal record of glitching flag so that we can notice a change
    _flags.baro_glitching = _baro_glitch.glitching();
}

// set_altitude - set base altitude estimate in cm
void AP_InertialNav::set_altitude( float new_altitude)
{
    _position_base.z = new_altitude;
    _position_correction.z = 0;
    _position.z = new_altitude; // _position = _position_base + _position_correction
    _hist_position_estimate_z.clear(); // reset z history to avoid fake z velocity at next baro calibration (next rearm)
}

//
// Private methods
//

// update_gains - update gains from time constant (given in seconds)
void AP_InertialNav::update_gains()
{
 
   
	_gps_k_gps_spd = (float)_gps_k_spd/10000.0f;
	_gps_k_inav_spd = 1.0f - _gps_k_gps_spd ;

	_gps_k_gps_pos = (float)_gps_k_pos/10000.0f;
	_gps_k_inav_pos = 1.0f - _gps_k_gps_pos ;



    // Z axis time constant
    if (_time_constant_z == 0.0f) {
        _k1_z = _k2_z = _k3_z = 0.0f;
    }else{
        _k1_z = 3.0f / _time_constant_z;
        _k2_z = 3.0f / (_time_constant_z*_time_constant_z);
        _k3_z = 1.0f / (_time_constant_z*_time_constant_z*_time_constant_z);
    }
}

// set_velocity_z - get latest climb rate (in cm/s)
void AP_InertialNav::set_velocity_z(float z )
{
    _velocity.z = z;
}

// set_position_xy - sets inertial navigation position to given xy coordinates from home
void AP_InertialNav::set_position_xy(float x, float y)
{
    // reset position from home
    _position_base.x = x;
    _position_base.y = y;
    _position_correction.x = 0.0f;
    _position_correction.y = 0.0f;
	_gps_position.x = x; 
	_gps_position.y = y;
    // clear historic estimates
    _hist_position_estimate_x.clear();
    _hist_position_estimate_y.clear();

    // add new position for future use
    _historic_xy_counter = 0;
    _hist_position_estimate_x.push_back(_position_base.x);
    _hist_position_estimate_y.push_back(_position_base.y);
}
