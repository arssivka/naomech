#ifndef InverseKinematics_h
#define InverseKinematics_h

#include "Kinematics.h"
namespace Kinematics{
    /*
     * Declarations for constants and methods concerning forward and inverse
     * kinematics.
     */
    //Accuracy constants for dls
    //in mm, how close dls will get to the target
    static const double UNBELIEVABLY_LOW_ERROR = 0.01; //mm
    static const double REALLY_LOW_ERROR = 0.1; //mm
    static const double ACCEPTABLE_ERROR = 0.5; //mm
    static const double COARSE_ERROR     = 1.0; //mm

    static const double dampFactor = 0.4;
    static const double maxDeltaTheta = 0.5;
    static const int maxAnkleIterations = 60;
    static const int maxHeelIterations = 20;


    static const double HYP_NOT_SET = -1000.0;

    enum IKOutcome {
        STUCK = 0,
        SUCCESS = 1
    };

    struct IKLegResult {
        IKOutcome outcome;
        double angles[6];
    };

    const IKLegResult simpleLegIK(const ChainID chainID,
                                  const NBMath::ufvector3 & legGoal,
                                  double startAngles []);

    const IKLegResult legIK(const ChainID chainID,
                            const NBMath::ufvector3 &footGoal,
                            const NBMath::ufvector3 &footOrientation,
                            const NBMath::ufvector3 &bodyGoal,
                            const NBMath::ufvector3 &bodyOrientation,
                            const double HYPAngle = HYP_NOT_SET);
    /**
     * Wrapper method for IK which finds leg angles given a location
     * for the leg, and angles for the body relative to the world
     * and the HYP angle
     */
    const IKLegResult angleXYIK(const ChainID chainID,
                                const NBMath::ufvector3 & legGoal,
                                const double bodyAngleX,
                                const double bodyAngleY,
                                const double HYPAngle);

    const void clipChainAngles(const ChainID id,
                               double angles[]);
    const double getMinValue(const ChainID id, const int jointNumber);
    const double getMaxValue(const ChainID id, const int jointNumber);
    const NBMath::ufvector3 forwardKinematics(const ChainID id,
                                              const double angles[]);
    const NBMath::ufmatrix3 buildJacobians(const ChainID chainID,
                                              const double angles[]);


    // Both adjustment methods return whether the search was successful.
    // The correct angles required to fulfill the goal are returned through
    // startAngles by reference.
    const bool adjustAnkle(const ChainID chainID,
                           const NBMath::ufvector3 &goal,
                           double startAngles[],
                           const double maxError);
    const bool adjustHeel(const ChainID chainID,
                          const NBMath::ufvector3 &goal,
                          double startAngles[],
                          const double maxError);
    const IKLegResult dls(const ChainID chainID,
                          const NBMath::ufvector3 &goal,
                          const double startAngles[],
                          const double maxError = ACCEPTABLE_ERROR,
                          const double maxHeelError = UNBELIEVABLY_LOW_ERROR);


    const IKLegResult analyticLegIK(const ChainID chainID,
                                    const NBMath::ufvector3 &footGoal,
                                    const NBMath::ufvector3 &footOrientation,
                                    const NBMath::ufvector3 &bodyGoal,
                                    const NBMath::ufvector3 &bodyOrientation,
                                    const double givenHYPAngle = HYP_NOT_SET);


    NBMath::ufmatrix4 rotationHYPRightInv(const double HYP);
    NBMath::ufmatrix4 rotationHYPLeftInv(const double HYP);

};
#endif
