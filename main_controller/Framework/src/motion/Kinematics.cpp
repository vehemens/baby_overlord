/*
 *   Kinematics.cpp
 *
 *   Author: ROBOTIS
 *
 */

#include <math.h>

#include "Config.h"

#include "Kinematics.h"
#include "minIni.h"

#define INVALID_VALUE -1

using namespace Robot;

Kinematics* Kinematics::m_UniqueInstance = new Kinematics();

Kinematics::Kinematics()
{
    CAMERA_DISTANCE = 33.2; //mm
    EYE_TILT_OFFSET_ANGLE = 40.0; //degree
    LEG_SIDE_OFFSET = 37.0; //mm
    THIGH_LENGTH = 93.0; //mm
    CALF_LENGTH = 93.0; //mm
    ANKLE_LENGTH = 33.5; //mm
    LEG_LENGTH = 219.5; //mm (THIGH_LENGTH + CALF_LENGTH + ANKLE_LENGTH)
}

Kinematics::~Kinematics()
{
}

void Kinematics::LoadINISettings(minIni* ini)
{
    const char *section = KINEMATICS_SECTION;
    double value = INVALID_VALUE;

    if((value = ini->getd(section, "camera_distance",       INVALID_VALUE)) != INVALID_VALUE)
      CAMERA_DISTANCE = value;
    if((value = ini->getd(section, "eye_tilt_offset_angle", INVALID_VALUE)) != INVALID_VALUE)
      EYE_TILT_OFFSET_ANGLE = value;
    if((value = ini->getd(section, "leg_side_offset",       INVALID_VALUE)) != INVALID_VALUE)
      LEG_SIDE_OFFSET = value;
    if((value = ini->getd(section, "thigh_length",          INVALID_VALUE)) != INVALID_VALUE)
      THIGH_LENGTH = value;
    if((value = ini->getd(section, "calf_length",           INVALID_VALUE)) != INVALID_VALUE)
      CALF_LENGTH = value;
    if((value = ini->getd(section, "ankle_length",          INVALID_VALUE)) != INVALID_VALUE)
      ANKLE_LENGTH = value;
    if((value = ini->getd(section, "leg_length",            INVALID_VALUE)) != INVALID_VALUE)
      LEG_LENGTH = value;
}

void Kinematics::SaveINISettings(minIni* ini)
{
    const char *section = KINEMATICS_SECTION;

    ini->put(section, "camera_distance",       CAMERA_DISTANCE);
    ini->put(section, "eye_tilt_offset_angle", EYE_TILT_OFFSET_ANGLE);
    ini->put(section, "leg_side_offset",       LEG_SIDE_OFFSET);
    ini->put(section, "thigh_length",          THIGH_LENGTH);
    ini->put(section, "calf_length",           CALF_LENGTH);
    ini->put(section, "ankle_length",          ANKLE_LENGTH);
    ini->put(section, "leg_length",            LEG_LENGTH);
}

#define PI (3.14159265)

bool Kinematics::computeIK(double *out, double x, double y, double z, double a, double b, double c)
{
	Matrix3D Tad, Tda, Tcd, Tdc, Tac;
	Vector3D vec;
    double _Rac, _Acos, _Atan, _k, _l, _m, _n, _s, _c, _theta;

	Tad.SetTransform(Point3D(x, y, z - LEG_LENGTH), Vector3D(a * 180.0 / PI, b * 180.0 / PI, c * 180.0 / PI));

	vec.X = x + Tad.m[2] * ANKLE_LENGTH;
    vec.Y = y + Tad.m[6] * ANKLE_LENGTH;
    vec.Z = (z - LEG_LENGTH) + Tad.m[10] * ANKLE_LENGTH;

    // Get Knee
	_Rac = vec.Length();
    _Acos = acos((_Rac * _Rac - THIGH_LENGTH * THIGH_LENGTH - CALF_LENGTH * CALF_LENGTH) / (2 * THIGH_LENGTH * CALF_LENGTH));
    if(isnan(_Acos) == 1)
		return false;
    *(out + 3) = _Acos;

    // Get Ankle Roll
    Tda = Tad;
	if(Tda.Inverse() == false)
        return false;
    _k = sqrt(Tda.m[7] * Tda.m[7] + Tda.m[11] * Tda.m[11]);
    _l = sqrt(Tda.m[7] * Tda.m[7] + (Tda.m[11] - ANKLE_LENGTH) * (Tda.m[11] - ANKLE_LENGTH));
    _m = (_k * _k - _l * _l - ANKLE_LENGTH * ANKLE_LENGTH) / (2 * _l * ANKLE_LENGTH);
    if(_m > 1.0)
        _m = 1.0;
    else if(_m < -1.0)
        _m = -1.0;
    _Acos = acos(_m);
    if(isnan(_Acos) == 1)
        return false;
    if(Tda.m[7] < 0.0)
        *(out + 5) = -_Acos;
    else
        *(out + 5) = _Acos;

    // Get Hip Yaw
	Tcd.SetTransform(Point3D(0, 0, -ANKLE_LENGTH), Vector3D(*(out + 5) * 180.0 / PI, 0, 0));
	Tdc = Tcd;
	if(Tdc.Inverse() == false)
        return false;
	Tac = Tad * Tdc;
    _Atan = atan2(-Tac.m[1] , Tac.m[5]);
    if(isinf(_Atan) == 1)
        return false;
    *(out) = _Atan;

    // Get Hip Roll
    _Atan = atan2(Tac.m[9], -Tac.m[1] * sin(*(out)) + Tac.m[5] * cos(*(out)));
    if(isinf(_Atan) == 1)
        return false;
    *(out + 1) = _Atan;

    // Get Hip Pitch and Ankle Pitch
    _Atan = atan2(Tac.m[2] * cos(*(out)) + Tac.m[6] * sin(*(out)), Tac.m[0] * cos(*(out)) + Tac.m[4] * sin(*(out)));
    if(isinf(_Atan) == 1)
        return false;
    _theta = _Atan;
    _k = sin(*(out + 3)) * CALF_LENGTH;
    _l = -THIGH_LENGTH - cos(*(out + 3)) * CALF_LENGTH;
	_m = cos(*(out)) * vec.X + sin(*(out)) * vec.Y;
	_n = cos(*(out + 1)) * vec.Z + sin(*(out)) * sin(*(out + 1)) * vec.X - cos(*(out)) * sin(*(out + 1)) * vec.Y;
    _s = (_k * _n + _l * _m) / (_k * _k + _l * _l);
    _c = (_n - _k * _s) / _l;
    _Atan = atan2(_s, _c);
    if(isinf(_Atan) == 1)
        return false;
    *(out + 2) = _Atan;
    *(out + 4) = _theta - *(out + 3) - *(out + 2);

    return true;
}
