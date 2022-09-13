#ifndef _mtsVFFollowPositionTrajectory_h
#define _mtsVFFollowPositionTrajectory_h

#include <cisstVector/vctDynamicVectorTypes.h>
#include <cisstVector/vctDynamicMatrixTypes.h>
#include <sawConstraintController/mtsVFDataBase.h>
#include <sawConstraintController/prmKinematicsState.h>
#include <bigss_constraint_controller/mtsVFFollowSettlingGains.h>

class mtsVFFollowPositionTrajectory : public mtsVFBase
{
public:
    mtsVFFollowPositionTrajectory() : mtsVFBase() {}

    mtsVFFollowPositionTrajectory(const std::string &name, mtsVFDataBase *data)
        : mtsVFBase(name, data) {}

    void FillInTableauRefs(const mtsVFBase::CONTROLLERMODE mode, const double TickTime);

    void ConvertRefs(const mtsVFBase::CONTROLLERMODE mode, const double TickTime) {};
};

struct mtsVFDataFollowPositionTrajectory : mtsVFDataBase
{
    prmKinematicsState *current_kinematics;
    prmKinematicsState *goal_kinematics;
    double goal_linear_velocity = 0.0; // m/s

    mtsVFDataFollowPositionTrajectory() : mtsVFDataBase() {}

    mtsVFDataFollowPositionTrajectory(const mtsVFDataFollowPositionTrajectory &other) : mtsVFDataBase(other) {}

    ~mtsVFDataFollowPositionTrajectory() {}

    std::string HumanReadable(void) const
    {
        return std::string();
    }
};

#endif
