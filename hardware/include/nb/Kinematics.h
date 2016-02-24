#ifndef Kinematics_h_DEFINED
#define Kinematics_h_DEFINED

/**
 * This file is strictly specific to the Nao robot.
 * It contains infomation relevant to the physical configuration of the robot
 * including:
 * length of links
 * number of chains, etc.
 * It includes method definitions related to the calculation of forward and
 * inverse kinematics.
 * It also supports the creation of rotation and translation matrices.
 * NOTE: All of the lengths are in millimeters.
 */

#include <string.h>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/triangular.hpp>
#include <boost/numeric/ublas/vector_proxy.hpp>
#include <boost/numeric/ublas/lu.hpp>              // for lu_factorize
#include <boost/numeric/ublas/io.hpp>              // for cout

#include <math.h>
#include "NBMath.h"
#include "NBMatrixMath.h"
#include "CoordFrame.h"

namespace Kinematics {


    enum ChainID {
        HEAD_CHAIN = 0,
        LARM_CHAIN,
        LLEG_CHAIN,
        RLEG_CHAIN,
        RARM_CHAIN,
        LANKLE_CHAIN, // only goes to the ankle
        RANKLE_CHAIN  // (same)
    };

	// Interpolation types
	enum InterpolationType {
		INTERPOLATION_SMOOTH = 0,
		INTERPOLATION_LINEAR
	};

    /// Joint Name constants ///
    enum JointNames {
        HEAD_YAW = 0,
        HEAD_PITCH,
        // LARM,
        L_SHOULDER_PITCH,
        L_SHOULDER_ROLL,
        L_ELBOW_YAW,
        L_ELBOW_ROLL,
        // LLEG,
        L_HIP_YAW_PITCH,
        L_HIP_ROLL,
        L_HIP_PITCH,
        L_KNEE_PITCH,
        L_ANKLE_PITCH,
        L_ANKLE_ROLL,
        // RLEG,
        R_HIP_YAW_PITCH,
        R_HIP_ROLL,
        R_HIP_PITCH,
        R_KNEE_PITCH,
        R_ANKLE_PITCH,
        R_ANKLE_ROLL,
        // RARM,
        R_SHOULDER_PITCH,
        R_SHOULDER_ROLL,
        R_ELBOW_YAW,
        R_ELBOW_ROLL
    };
    static const unsigned int FIRST_HEAD_JOINT = HEAD_YAW;

    /**
     * (Deprecated)
    enum Motion_IntFlag {
        UNINT_INTR_CMD, // Un-interruptable interrupter command
        INT_INTR_CMD,   // Interruptable interupter command
        UNINT_CMD,      // Un-interruptable command
        INT_CMD         // Interruptable command
    };

    enum SupportLeg{
        BOTH_LEGS = 0,
        RIGHT_LEG,
        LEFT_LEG
    };
    **/

    static const unsigned int HEAD_JOINTS = 2;
    static const unsigned int ARM_JOINTS = 4;
    static const unsigned int LEG_JOINTS = 6;
    static const unsigned int NUM_CHAINS = 5;
	static const unsigned int NUM_BODY_CHAINS = 4;

    static const unsigned int NUM_JOINTS = HEAD_JOINTS + ARM_JOINTS*2 +
        LEG_JOINTS*2;
    static const unsigned int NUM_BODY_JOINTS = ARM_JOINTS*2 + LEG_JOINTS*2;
    static const unsigned int chain_lengths[NUM_CHAINS] = {2, 4, 6, 6, 4};
	static const unsigned int chain_first_joint[NUM_CHAINS] = {0,2,6,12,18};
	static const unsigned int chain_last_joint[NUM_CHAINS] = {1,5,11,17,21};

    static const std::string CHAIN_STRINGS[NUM_CHAINS] =
    { "Head",
      "LArm",
      "LLeg",
      "RLeg",
      "RArm" };

    //Note: this joint list constains RHipYawPitch, which is not compatible
    //with the DCM convention on naming joints (this joint is technially
    //nonexistant)
    static const std::string JOINT_STRINGS[NUM_JOINTS] =
    { "HeadYaw",
      "HeadPitch",
      "LShoulderPitch",
      "LShoulderRoll",
      "LElbowYaw",
      "LElbowRoll",
      "LHipYawPitch",
      "LHipRoll",
      "LHipPitch",
      "LKneePitch",
      "LAnklePitch",
      "LAnkleRoll",
      "RHipYawPitch",
      "RHipRoll",
      "RHipPitch",
      "RKneePitch",
      "RAnklePitch",
      "RAnkleRoll",
      "RShoulderPitch",
      "RShoulderRoll",
      "RElbowYaw",
      "RElbowRoll"};

    /**********    Bodily dimensions     ***********/

    static const double SHOULDER_OFFSET_Y = 98.0;
    static const double UPPER_ARM_LENGTH = 105.0;
    static const double LOWER_ARM_LENGTH = 55.95;
    static const double SHOULDER_OFFSET_Z = 100;
    static const double THIGH_LENGTH = 100.0;
    static const double TIBIA_LENGTH = 102.9;
    static const double NECK_OFFSET_Z = 126.5;
    static const double HIP_OFFSET_Y = 50.0;
    static const double HIP_OFFSET_Z = 85.0;
    static const double FOOT_HEIGHT = 45.19;

    // Camera
    static const double CAMERA_OFF_X = 48.80; // in millimeters
    static const double CAMERA_OFF_Z = 23.81;  // in millimeters
    static const double CAMERA_PITCH_ANGLE = 40.0 * TO_RAD; // 40 degrees

    /**********       Joint Bounds       ***********/
    static const double HEAD_BOUNDS[2][2] = {{-2.09,2.09},{-.785,.785}};

    // Order of arm joints: ShoulderPitch, SRoll, ElbowYaw, ERoll
    static const double LEFT_ARM_BOUNDS[][2] = {{-2.09,2.09},
                                               {0.0,1.65},
                                               {-2.09,2.09},
                                               {-1.57,0.0}};
    static const double RIGHT_ARM_BOUNDS[][2] = {{-2.09,2.09},
                                                {-1.65,0.0},
                                                {-2.09,2.09},
                                                {0.0,1.57}};

    // Order of leg joints: HYPitch HipRoll HipPitch  KneePitch APitch ARoll
    static const double LEFT_LEG_BOUNDS[][2] = {{-1.57,0.0},
                                               {-.349,.785},
                                               {-1.57,.436},
                                               {0.0,2.269},
                                               {-1.309,.524},
                                               {-.785,.349}};
    static const double RIGHT_LEG_BOUNDS[][2] = {{-1.57,0.0},
                                                {-.785,.349},
                                                {-1.57,.436},
                                                {0.0,2.269},
                                                {-1.309,.524},
                                                {-.349,.785}};

    /**********     joint velocity limits **********/
    //Set hardware values- nominal speed in rad/20ms
    //from $AL_DIR/doc/reddoc
    //M=motor r = reduction ratio

    static const double M1R1_NOMINAL = 0.0658;
    static const double M1R2_NOMINAL = 0.1012;
    static const double M2R1_NOMINAL = 0.1227;
    static const double M2R2_NOMINAL = 0.1065;

    static const double M1R1_NO_LOAD = 0.08308;
    static const double M1R2_NO_LOAD = 0.1279;
    static const double M2R1_NO_LOAD = 0.16528;
    static const double M2R2_NO_LOAD = 0.1438;

    static const double M1R1_AVG = (M1R1_NOMINAL + M1R1_NO_LOAD )*0.5;
    static const double M1R2_AVG = (M1R2_NOMINAL + M1R2_NO_LOAD )*0.5;
    static const double M2R1_AVG = (M2R1_NOMINAL + M2R1_NO_LOAD )*0.5;
    static const double M2R2_AVG = (M2R2_NOMINAL + M2R2_NO_LOAD )*0.5;

    static const double jointsMaxVelNominal[Kinematics::NUM_JOINTS] = {
        //head
        M2R2_NOMINAL, M2R1_NOMINAL,
        //left arm
        M2R1_NOMINAL, M2R2_NOMINAL, M2R1_NOMINAL, M2R2_NOMINAL,
        //left leg
        M1R1_NOMINAL, M1R1_NOMINAL, M1R2_NOMINAL,
        M1R2_NOMINAL, M1R2_NOMINAL, M1R1_NOMINAL,
        //right leg
        M1R1_NOMINAL, M1R1_NOMINAL, M1R2_NOMINAL,
        M1R2_NOMINAL, M1R2_NOMINAL, M1R1_NOMINAL,
        //right arm
        M2R2_NOMINAL, M2R2_NOMINAL, M2R1_NOMINAL, M2R2_NOMINAL
    };

    static const double jointsMaxVelNoLoad[Kinematics::NUM_JOINTS] = {
        //head
        M2R2_NO_LOAD, M2R1_NO_LOAD,
        //left arm
        M2R1_NO_LOAD, M2R2_NO_LOAD, M2R1_NO_LOAD, M2R2_NO_LOAD,
        //left leg
        M1R1_NO_LOAD, M1R1_NO_LOAD, M1R2_NO_LOAD,
        M1R2_NO_LOAD, M1R2_NO_LOAD, M1R1_NO_LOAD,
        //right leg
        M1R1_NO_LOAD, M1R1_NO_LOAD, M1R2_NO_LOAD,
        M1R2_NO_LOAD, M1R2_NO_LOAD, M1R1_NO_LOAD,
        //right arm
        M2R2_NO_LOAD, M2R2_NO_LOAD, M2R1_NO_LOAD, M2R2_NO_LOAD
    };

    static const double jointsMaxVelAvg[Kinematics::NUM_JOINTS] = {
        //head
        M2R2_AVG, M2R1_AVG,
        //left arm
        M2R1_AVG, M2R2_AVG, M2R1_AVG, M2R2_AVG,
        //left leg
        M1R1_AVG, M1R1_AVG, M1R2_AVG,
        M1R2_AVG, M1R2_AVG, M1R1_AVG,
        //right leg
        M1R1_AVG, M1R1_AVG, M1R2_AVG,
        M1R2_AVG, M1R2_AVG, M1R1_AVG,
        //right arm
        M2R2_AVG, M2R2_AVG, M2R1_AVG, M2R2_AVG
    };


    /**********      mDH parameters      ***********/

    enum mDHNames {
        ALPHA = 0,
        L,
        THETA,
        D
    };

    //                                  (alpha,  a ,  theta ,   d  )
    const double HEAD_MDH_PARAMS[2][4] = {{0.0 , 0.0,  0.0 , 0.0},
                                         {-M_PI_double/2, 0.0, -M_PI_double/2 , 0.0}};

    const double LEFT_ARM_MDH_PARAMS[4][4] = {{-M_PI_double/2,0.0,0.0,0.0},
                                             { M_PI_double/2,0.0,M_PI_double/2,0.0},
                                             { M_PI_double/2,0.0,0.0,UPPER_ARM_LENGTH},
                                             {-M_PI_double/2,0.0,0.0,0.0}};

    const double LEFT_LEG_MDH_PARAMS[6][4] = {{ -3*M_PI_double/4, 0.0,  -M_PI_double/2, 0.0},
                                             { -M_PI_double/2,   0.0,   M_PI_double/4, 0.0},
                                             { M_PI_double/2,    0.0,     0.0, 0.0},
                                             {   0.0,-THIGH_LENGTH,0.0, 0.0},
                                             {   0.0,-TIBIA_LENGTH,0.0, 0.0},
                                             {-M_PI_double/2,    0.0,     0.0, 0.0}};

    const double RIGHT_LEG_MDH_PARAMS[6][4]= {{ -M_PI_double/4,  0.0,   -M_PI_double/2, 0.0},
                                             { -M_PI_double/2,   0.0,  -M_PI_double/4, 0.0},
                                             {  M_PI_double/2,    0.0,    0.0, 0.0},
                                             { 0.0,-THIGH_LENGTH,0.0, 0.0},
                                             {0.0,-TIBIA_LENGTH,0.0,0.0},
                                             {-M_PI_double/2,0.0,0.0,0.0}};

    const double RIGHT_ARM_MDH_PARAMS[4][4] = {{-M_PI_double/2, 0.0,0.0,0.0},
                                              { M_PI_double/2, 0.0,M_PI_double/2,0.0},
                                              { M_PI_double/2, 0.0,0.0,UPPER_ARM_LENGTH},
                                              {-M_PI_double/2, 0.0,0.0,0.0}};

    static const double* MDH_PARAMS[NUM_CHAINS] = {&HEAD_MDH_PARAMS[0][0],
                                                  &LEFT_ARM_MDH_PARAMS[0][0],
                                                  &LEFT_LEG_MDH_PARAMS[0][0],
                                                  &RIGHT_LEG_MDH_PARAMS[0][0],
                                                  &RIGHT_ARM_MDH_PARAMS[0][0]};

    //Base transforms to get from body center to beg. of chain
    static const boost::numeric::ublas::matrix <double> HEAD_BASE_TRANSFORMS[1]
    = { CoordFrame4D::translation4D( 0.0,
                       0.0,
                       NECK_OFFSET_Z ) };

    static const boost::numeric::ublas::matrix <double> LEFT_ARM_BASE_TRANSFORMS[1]
    = { CoordFrame4D::translation4D( 0.0,
                       SHOULDER_OFFSET_Y,
                       SHOULDER_OFFSET_Z ) };

    static const boost::numeric::ublas::matrix <double> LEFT_LEG_BASE_TRANSFORMS[1]
    ={ CoordFrame4D::translation4D( 0.0,
                      HIP_OFFSET_Y,
                      -HIP_OFFSET_Z ) };

    static const boost::numeric::ublas::matrix <double> RIGHT_LEG_BASE_TRANSFORMS[1]
    ={ CoordFrame4D::translation4D( 0.0,
                      -HIP_OFFSET_Y,
                      -HIP_OFFSET_Z ) };

    static const boost::numeric::ublas::matrix <double> RIGHT_ARM_BASE_TRANSFORMS[1]
    ={ CoordFrame4D::translation4D( 0.0,
                      -SHOULDER_OFFSET_Y,
                      SHOULDER_OFFSET_Z ) };

    static const boost::numeric::ublas::matrix <double> * BASE_TRANSFORMS[NUM_CHAINS] =
    { &HEAD_BASE_TRANSFORMS[0],
      &LEFT_ARM_BASE_TRANSFORMS[0],
      &LEFT_LEG_BASE_TRANSFORMS[0],
      &RIGHT_LEG_BASE_TRANSFORMS[0],
      &RIGHT_ARM_BASE_TRANSFORMS[0] };

    //Base transforms to get from body center to beg. of chain
    static const boost::numeric::ublas::matrix <double> HEAD_END_TRANSFORMS[4]
    = { CoordFrame4D::rotation4D(CoordFrame4D::X_AXIS, M_PI_double/2),
        CoordFrame4D::rotation4D(CoordFrame4D::Y_AXIS,M_PI_double/2),
        CoordFrame4D::translation4D(CAMERA_OFF_X, 0, CAMERA_OFF_Z),
        CoordFrame4D::rotation4D(CoordFrame4D::Y_AXIS, CAMERA_PITCH_ANGLE) };


    static const boost::numeric::ublas::matrix <double> LEFT_ARM_END_TRANSFORMS[2]
    = { CoordFrame4D::rotation4D(CoordFrame4D::Z_AXIS, -M_PI_double/2),
        CoordFrame4D::translation4D(LOWER_ARM_LENGTH,0.0,0.0) };

    static const boost::numeric::ublas::matrix <double> LEFT_LEG_END_TRANSFORMS[3]
    = { CoordFrame4D::rotation4D(CoordFrame4D::Z_AXIS, M_PI_double),
        CoordFrame4D::rotation4D(CoordFrame4D::Y_AXIS, -M_PI_double/2),
        CoordFrame4D::translation4D(0.0,
                      0.0,
                      -FOOT_HEIGHT) };

    static const boost::numeric::ublas::matrix <double> RIGHT_LEG_END_TRANSFORMS[3] =
        { CoordFrame4D::rotation4D(CoordFrame4D::Z_AXIS, M_PI_double),
        CoordFrame4D::rotation4D(CoordFrame4D::Y_AXIS, -M_PI_double/2),
        CoordFrame4D::translation4D(0.0,
                      0.0,
                      -FOOT_HEIGHT) };

    static const boost::numeric::ublas::matrix <double> RIGHT_ARM_END_TRANSFORMS[2] =
        { CoordFrame4D::rotation4D(CoordFrame4D::Z_AXIS, -M_PI_double/2),
        CoordFrame4D::translation4D(LOWER_ARM_LENGTH,0.0,0.0) };


    static const boost::numeric::ublas::matrix <double> * END_TRANSFORMS[NUM_CHAINS] =
    { &HEAD_END_TRANSFORMS[0],
      &LEFT_ARM_END_TRANSFORMS[0],
      &LEFT_LEG_END_TRANSFORMS[0],
      &RIGHT_LEG_END_TRANSFORMS[0],
      &RIGHT_ARM_END_TRANSFORMS[0] };
    static const int NUM_BASE_TRANSFORMS[NUM_CHAINS] = {1,1,1,1,1};
    static const int NUM_END_TRANSFORMS[NUM_CHAINS] = {4,2,3,3,2};
    static const int NUM_JOINTS_CHAIN[NUM_CHAINS] = {2,4,6,6,4};

    //locally expressed constants (with respect to an individual joint
    //and the GLOBAL coordinate frame)

    static const double CHEST_MASS_Z = 43.42;
    static const double CHEST_MASS_X = -4.13;

    static const double HEAD_MASS_Z = 179.08 - NECK_OFFSET_Z;
    static const double UPPER_ARM_MASS_X = 49.1;
    static const double LOWER_ARM_MASS_X = 156.12 - UPPER_ARM_LENGTH;
    static const double THIGH_MASS_Z = 123.0 - HIP_OFFSET_Z;
    static const double TIBIA_MASS_Z = 251.1 - HIP_OFFSET_Z - THIGH_LENGTH;
    static const double FOOT_MASS_Z = 315.0 - HIP_OFFSET_Z -
        THIGH_LENGTH - TIBIA_LENGTH;
    static const double FOOT_MASS_X = 25.42;

    //Weight constants
    static const double CHEST_MASS_g = 1199.68;
    static const double HEAD_MASS_g = 540.91;
    static const double UPPER_ARM_MASS_g = 287.0;
    static const double LOWER_ARM_MASS_g = 142.44;
    static const double THIGH_MASS_g  = 600.02;
    static const double TIBIA_MASS_g  = 435.58;
    static const double FOOT_MASS_g  = 171.84;
    static const double TOTAL_MASS  =
        CHEST_MASS_g + HEAD_MASS_g +
        2.0*(UPPER_ARM_MASS_g + LOWER_ARM_MASS_g + THIGH_MASS_g +
              TIBIA_MASS_g + FOOT_MASS_g);

    //The locations of the massses are translated from their
    //global coordinate frame into the local frame in tuples like
    // {X,Y,Z,WEIGHT}

    static const double HEAD_INERTIAL_POS[2][4] = {
        {0.0, 0.0, 0.0, 0.0},
        {HEAD_MASS_Z, 0.0, 0.0, HEAD_MASS_g}};

    static const double LEFT_ARM_INERTIAL_POS[4][4] = {
        {0.0, 0.0, 0.0, 0.0},
        { 0.0, -UPPER_ARM_MASS_X, 0.0, UPPER_ARM_MASS_g},
        {0.0, 0.0, 0.0, 0.0},
        { 0.0,-UPPER_ARM_MASS_X, 0.0, LOWER_ARM_MASS_g}};

    //Z,X,Y is the correct order for most of the leg
    static const double LEFT_LEG_INERTIAL_POS[6][4] = {
        {0.0, 0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0, 0.0},
        {-THIGH_MASS_Z, 0.0, 0.0, THIGH_MASS_g},
        {-TIBIA_MASS_Z, 0.0, 0.0, TIBIA_MASS_g},
        {0.0, 0.0, 0.0, 0.0},
        {-FOOT_MASS_Z,  0.0, FOOT_MASS_X, FOOT_MASS_g}};
    static const double RIGHT_LEG_INERTIAL_POS[6][4] = {
        {0.0, 0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0, 0.0},
        {-THIGH_MASS_Z, 0.0, 0.0, THIGH_MASS_g},
        {-TIBIA_MASS_Z, 0.0, 0.0, TIBIA_MASS_g},
        {0.0, 0.0, 0.0, 0.0},
        {-FOOT_MASS_Z, 0.0, FOOT_MASS_X, FOOT_MASS_g}};
    static const double RIGHT_ARM_INERTIAL_POS[4][4] = {
        {0.0, 0.0, 0.0, 0.0},
        { 0.0, -UPPER_ARM_MASS_X, 0.0, UPPER_ARM_MASS_g},
        {0.0, 0.0, 0.0, 0.0},
        { 0.0,-UPPER_ARM_MASS_X, 0.0, LOWER_ARM_MASS_g}};

    static const double* INERTIAL_POS[NUM_CHAINS] = {&HEAD_INERTIAL_POS[0][0],
                                                    &LEFT_ARM_INERTIAL_POS[0][0],
                                                    &LEFT_LEG_INERTIAL_POS[0][0],
                                                    &RIGHT_LEG_INERTIAL_POS[0][0],
                                                    &RIGHT_ARM_INERTIAL_POS[0][0]};

    static const unsigned int MASS_INDEX = 3;

};

#endif
