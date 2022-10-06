#ifndef _BIGSSURInterface_h
#define _BIGSSURInterface_h


#include <bigss_robotic_system/RobotInterfaceObject.h>
#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <cisstParameterTypes/prmVelocityJointSet.h>
#include <cisstMultiTask/mtsGenericObject.h>
#include <cisstMultiTask/mtsFunctionWrite.h>
#include <cisstMultiTask/mtsFunctionRead.h>
#include <cisstParameterTypes/prmPositionCartesianGet.h>
#include <cisstParameterTypes/prmPositionJointGet.h>
#include <cisstParameterTypes/prmStateJoint.h>
#include <cisstCommon/cmnUnits.h>

class URInterface : public RobotInterfaceObject
{
    public:
        mtsFunctionWrite servo_jv;
        mtsFunctionRead measured_js;
        mtsFunctionRead measured_cp;
        mtsFunctionRead jacobian;

        URInterface(const vctFrm3& frame_offset_to_base=vctFrm3(), const std::string interface_name="URInterface", int num_joints=6, const std::string& base_frame_name="base", bool ctrk_naming=false):
            RobotInterfaceObject(frame_offset_to_base, interface_name, num_joints, base_frame_name),
            m_crtk_naming(ctrk_naming)
        {
            // Connect the functions that we need for bigss_robotic_system to their corresponding functions in the BIGSS UniversalRobot component
            mtsInterfaceRequired* ur_interface = this->AddInterfaceRequired(interface_name+"_required");
            command_interfaces.push_back(ur_interface); // TODO?: currently unused, but might allow for some degree of "auto-" or "self-connection"
            if(m_crtk_naming){
                ur_interface->AddFunction("servo_jv", this->servo_jv);
                ur_interface->AddFunction("measured_js", this->measured_js);
                ur_interface->AddFunction("measured_cp", this->measured_cp);
                ur_interface->AddFunction("jacobian", this->jacobian);
            }

            else{ //To work with un CTRK-compliant BIGSS util code
                ur_interface->AddFunction("SetVelocityJoint", this->servo_jv);
                ur_interface->AddFunction("GetPositionJoint", this->measured_js);
                ur_interface->AddFunction("GetPositionCartesian", this->measured_cp);
                ur_interface->AddFunction("GetGeoJacobian", this->jacobian);
            }            
           
            vctDynamicVector<std::string> joint_names(6); //TODO:cleanup init
            joint_names.Element(0) = "shoulder_pan_joint";
            joint_names.Element(1) = "shoulder_lift_joint";
            joint_names.Element(2) = "elbow_joint";
            joint_names.Element(3) = "wrist_1_joint";
            joint_names.Element(4) = "wrist_2_joint";
            joint_names.Element(5) = "wrist_3_joint";
            kinematics_state.JointState->SetName(joint_names); // Needed for robot_state_publisher in ROS
            m_joint_state.SetSize(num_joints);
            
        }

        ~URInterface(){}

        bool UpdateState(){
            // cisst-ros bridge does not publish unless Valid flag is true. Here we default to false so we don't publish old/bad data
            m_measured_cp.SetValid(false);
            kinematics_state.JointState->SetValid(false);
            mtsExecutionResult res;
            // Get UR joint state
            if(m_crtk_naming){
                prmStateJoint js;
                res = this->measured_js(js);
                m_joint_state.Position() = js.Position();
            }
            else{
                res = this->measured_js(m_joint_state);
            }
            if(!res)
                {CMN_LOG_INIT_ERROR << "Could not read robot joint state" << std::endl; return false;}
            kinematics_state.JointState->Position() = m_joint_state.Position();
            kinematics_state.JointState->SetValid(true);

            // Get UR forward kinematics
            res = this->measured_cp(m_measured_cp);
            if(!res)
                {CMN_LOG_INIT_ERROR << "Could not read robot cartesian position" << std::endl; return false;}
            m_measured_cp.SetValid(true); //allow data publish
            //TODO: Setting of reference frames should be moved into the base class of RobotInterface
            m_measured_cp.SetReferenceFrame(base_frame_name);
            m_measured_cp.SetMovingFrame(interface_name);
            this->kinematics_state.Frame = m_measured_cp.Position();

            // Get UR Jacobian
            res = this->jacobian(this->kinematics_state.Jacobian);
            if(!res){CMN_LOG_INIT_ERROR << "Could not read robot Jacobian" << std::endl; return false;}
            return true;
        }
        
        // TODO?: consider renaming to allow for better differentiation between move commands
        // The other option would be to have void MoveRobot(prmPositionJointSet& command), etc.
        void MoveRobot(prmVelocityJointSet& command){
            command.SetValid(true);
            if(m_crtk_naming){
                this->servo_jv(command);
            }
            else{
                this->servo_jv(command.Goal());
            }
        }

    private:
        prmPositionJointGet m_joint_state;
        bool m_crtk_naming;
};

#endif
