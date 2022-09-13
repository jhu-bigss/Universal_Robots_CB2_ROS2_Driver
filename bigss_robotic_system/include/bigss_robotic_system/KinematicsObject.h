/*
  Authors: Henry Phalen
*/

#ifndef KINEMATICSOBJECT_H
#define KINEMATICSOBJECT_H

#include <cisstVector.h>
#include <sawConstraintController/prmKinematicsState.h>
#include <cisstParameterTypes/prmStateJoint.h>

/**
 * @brief Base class that defines a notion of kinematic states (joints, position, velocity, etc.)
 * 
 */
class KinematicsObject{
    
    public:
    /**
     * @brief Construct a new Kinematics Object, base class
     * 
     * @param name 
     * @param num_joints 
     */
    KinematicsObject(const std::string& name="KinematicsObject", const int num_joints=0);

    ~KinematicsObject();

    int GetNumberOfJoints() const;
    
    /**
     * @brief Forces all derived classes to override of this function, allows for states to change
     * 
     */
    virtual bool UpdateState()=0;
    
    /**
     * @brief Sets the size of the kinematic state variables and initializes values to zero
     * 
     */
    void InitializeKinematicsStateSize();

    prmKinematicsState kinematics_state;
    int num_joints;
    std::string name;

    private:
    prmStateJoint joint_state;
};


#endif // KINEMATICSOBJECT_H
