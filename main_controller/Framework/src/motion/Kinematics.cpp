/*
 *   Kinematics.cpp
 *
 *   Author: ROBOTIS
 *
 */

#include <math.h>
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
