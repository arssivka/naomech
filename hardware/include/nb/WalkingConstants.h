// This file is part of Man, a robotic perception, locomotion, and
// team strategy application created by the Northern Bites RoboCup
// team of Bowdoin College in Brunswick, Maine, for the Aldebaran
// Nao robot.
//
// Man is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// Man is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser Public License for more details.
//
// You should have received a copy of the GNU General Public License
// and the GNU Lesser Public License along with Man.  If not, see
// <http://www.gnu.org/licenses/>.

#ifndef _WalkingConstants_h_DEFINED
#define _WalkingConstants_h_DEFINED

#include <vector>
#include <boost/shared_ptr.hpp>
#include "Kinematics.h"


#include "MotionConstants.h"


enum SupportMode {
    SUPPORTING = 0,
    SWINGING,
    DOUBLE_SUPPORT,
    PERSISTENT_DOUBLE_SUPPORT
};

static const double LARM_WALK_ANGLES[Kinematics::ARM_JOINTS] =
        {M_PI_double / 2.0, .26, 0.0, 0.0};
static const double RARM_WALK_ANGLES[Kinematics::ARM_JOINTS] =
        {M_PI_double / 2.0, -.26, 0.0, 0.0};


//Sensitivity to new walk vectors -- currently 0, giving maximum sensitivity
//when a new vector differs by more than these constants, the internal
//walk vector held in the StepGenerator changes
const double NEW_VECTOR_THRESH_MMS = 0.0; //difference in speed in mm/second
const double NEW_VECTOR_THRESH_RADS = 0.0; //difference in speed in radians/second






#endif
