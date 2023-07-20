#include "Copter.h"
#include "float.h"

/*
 * Init and run calls for Myfirst flight mode
 */

float _targetVel;
float _turnTime;
float _pitchingRate;
float _tagetPichingStep;
float _tagetPichDef;
int32_t _bfPitch;
uint16_t _fallingCount;
float _throttle_out;
//uint16_t _limmitAltcm;
bool ModeMyfirst::init(bool ignore_checks)
{
    // only allow flip from some flight modes, for example ACRO, Stabilize, AltHold or FlowHold flight modes
    if (!copter.flightmode->allows_flip()) {
        return false;
    }

    // if in acro or stabilize ensure throttle is above zero
    if (copter.ap.throttle_zero && (copter.flightmode->mode_number() == Mode::Number::ACRO || copter.flightmode->mode_number() == Mode::Number::STABILIZE)) {
        return false;
    }

    // ensure roll input is less than 40deg
    if (abs(channel_roll->get_control_in()) >= 4000) {
        return false;
    }

    // only allow flip when flying
    if (!motors->armed() || copter.ap.land_complete) {
        return false;
    }

    // capture original flight mode so that we can return to it after completion
    orig_control_mode = copter.flightmode->mode_number();
    //_limmitAltcm = safeAlt + radius_m;
    //uint16_t dropDist = (copter.rangefinder_state.alt_cm / 100) - _limmitAltcm;
    //uint16_t dropDist = 50;
    float ft = sqrt(fallDistance * 2 / 9.8);
    _throttle_out = 0;
    _fallingCount = ft * 400;
    _targetVel =  ft * 9.8;
    _turnTime = radius_m * 2 * 3.141592 / _targetVel;
    _pitchingRate = 360 * 100 / _turnTime;
    _tagetPichingStep = _pitchingRate / 400;
    if(_tagetPichingStep <= FLT_EPSILON)_tagetPichingStep = FLT_EPSILON;
    _tagetPichDef = _tagetPichingStep;
    _stage = Stage::Dive;
    return true;    
}

// Myfirst_run - runs the main Myfirst controller
// should be called at 100hz or more
void ModeMyfirst::run()
{
    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    switch (_stage)
    {
    case Stage::Start:
        /* code */
        break;

    case Stage::Dive:
        if(_fallingCount > 0){
            if(ahrs.pitch_sensor != diveAngle)
            {
                //float y = ahrs.yaw;
                attitude_control->set_throttle_out(0.1f,false,g.throttle_filt);
                attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(0.0f,diveAngle,0.0f);
            }
            else
            {
                attitude_control->set_throttle_out(0.0f,false,g.throttle_filt);
            }
            _fallingCount--;
        }
        else
        {
            _bfPitch = ahrs.pitch_sensor;
            _stage = Stage::Noseup;
            //attitude_control->set_throttle_out(0.1f,false,g.throttle_filt);
            //_stage = Stage::Fall;
        }
        break;

    case Stage::Noseup:
    if(ahrs.pitch_sensor > 0)
    {
        _stage = Stage::Coasting;
    }
    else
    {
        if(ahrs.pitch_sensor - _bfPitch >= _tagetPichDef)
        //if(ahrs.pitch_sensor < -60)
        {
            _bfPitch = ahrs.pitch_sensor;
            _tagetPichDef = _tagetPichingStep;
            _throttle_out = 1.0f;
            attitude_control->set_throttle_out(_throttle_out,false,g.throttle_filt);
            attitude_control->input_rate_bf_roll_pitch_yaw(0.0f,_pitchingRate,0.0f);
            //gcs().send_text(MAV_SEVERITY_INFO,"pullup");
        }
        else
        {
            _bfPitch = ahrs.pitch_sensor;
            _tagetPichDef += _tagetPichingStep;
            _throttle_out -= 0.01f;
            attitude_control->set_throttle_out(_throttle_out,false,g.throttle_filt);
            attitude_control->input_rate_bf_roll_pitch_yaw(0.0f,_pitchingRate,0.0f);
            //_stage = Stage::Turn;
            //attitude_control->set_throttle_out(1.0f,false,g.throttle_filt);
        }
    }
        break;

    case Stage::Coasting:
    //attitude_control->set_throttle_out(0.33f,false,g.throttle_filt);
    //attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(0.0f,0.0f,0.0f);
                attitude_control->set_throttle_out(0.5f,false,g.throttle_filt);
            copter.set_mode(Mode::Number::ALT_HOLD, ModeReason::UNKNOWN);
        break;

    case Stage::Abandon:
        // restore original flight mode
        if (!copter.set_mode(orig_control_mode, ModeReason::FLIP_COMPLETE)) {
            // this should never happen but just in case
            copter.set_mode(Mode::Number::STABILIZE, ModeReason::UNKNOWN);
        }
        // log abandoning flip
        //AP::logger().Write_Error(LogErrorSubsystem::FLIP, LogErrorCode::FLIP_ABANDONED);
        break;
    }
}
