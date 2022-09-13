/*
  Authors: Henry Phalen
*/

#include <bigss_robotic_system/KinematicsObject.h>
#include <cisstVector.h>
#include <sawConstraintController/prmKinematicsState.h>
#include <cisstParameterTypes/prmStateJoint.h>

KinematicsObject::KinematicsObject(const std::string& name, const int num_joints):
    name(name),
    num_joints(num_joints)
{
    InitializeKinematicsStateSize();
}

KinematicsObject::~KinematicsObject(){};

int KinematicsObject::GetNumberOfJoints() const
    {return num_joints;}

void KinematicsObject::InitializeKinematicsStateSize(){
    joint_state.SetSize(this->num_joints);
    joint_state.Position().SetAll(0.0);
    joint_state.Velocity().SetAll(0.0);
    joint_state.Effort().SetAll(0.0);
    kinematics_state.JointState = &joint_state;
    kinematics_state.Name = name;
    kinematics_state.Jacobian.SetSize(6,num_joints);
    kinematics_state.Jacobian.SetAll(0.0);
}
