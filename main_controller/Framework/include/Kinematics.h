/*
 *   Kinematics.h
 *
 *   Author: ROBOTIS
 *
 */

#ifndef _KINEMATICS_H_
#define _KINEMATICS_H_

#include "Matrix.h"
#include "JointData.h"
#include "minIni.h"

#define KINEMATICS_SECTION "Kinematics"

namespace Robot
{
	class Kinematics
	{
	private:
		static Kinematics* m_UniqueInstance;
        Kinematics();

	protected:

	public:
		double CAMERA_DISTANCE; //mm
		double EYE_TILT_OFFSET_ANGLE; //degree
		double LEG_SIDE_OFFSET; //mm
		double THIGH_LENGTH; //mm
		double CALF_LENGTH; //mm
		double ANKLE_LENGTH; //mm
		double LEG_LENGTH; //mm (THIGH_LENGTH + CALF_LENGTH + ANKLE_LENGTH)

		~Kinematics();

		static Kinematics* GetInstance()			{ return m_UniqueInstance; }

		void LoadINISettings(minIni* ini);
		void SaveINISettings(minIni* ini);
	};
}

#endif
