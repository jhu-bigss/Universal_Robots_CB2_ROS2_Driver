#include <ur_cb2_bigss_constraint_controller/mtsBIGSSVFRCM.h>
#include <ur_cb2_bigss_constraint_controller/helper.h>

CMN_IMPLEMENT_SERVICES(mtsBIGSSVFRCM);

void mtsBIGSSVFRCM::FillInTableauRefs(const CONTROLLERMODE CMN_UNUSED(mode), const double CMN_UNUSED(tickTime))
{
    mtsBIGSSVFDataRCM * RCM_Data = reinterpret_cast<mtsBIGSSVFDataRCM*>(Data);
    vctFrm3 frame_of_interest = RCM_Data->kinematics_for_last_joint_that_moves_closest_point->Frame * RCM_Data->offset_from_kinematics_to_frame_of_interest;
    // Constraint-Based RCM
    vct3 Axis = -frame_of_interest.Rotation() * vct3(1,0,0);
    ClosestPoint = GetClosestPointToGivenPointOnLine(frame_of_interest.Translation(), RCM_Data->RCM_point, Axis);
    // std::cout << "current_frame" << RCM_Data->kinematics_for_last_joint_that_moves_closest_point->Frame << std::endl;
    // std::cout << "frame_of_interest: " << frame_of_interest << std::endl;
    // std::cout << "Axis: " << Axis << std::endl;
    // std::cout << "Clostest Point: " << ClosestPoint << std::endl;
    // std::cout << "RCM_point: " << RCM_Data->RCM_point << std::endl;

    FillMoveConstraints3D(H,h,Axis,ClosestPoint-RCM_Data->RCM_point,0.001,RCM_Data->IneqConstraintRows);
  
    vctDoubleMat jac(RCM_Data->kinematics_for_last_joint_that_moves_closest_point->Jacobian);
    vctFrm3 transform(vctRot3::Identity(), ClosestPoint - RCM_Data->kinematics_for_last_joint_that_moves_closest_point->Frame.Translation());
    vctFrm3 self_fk = RCM_Data->kinematics_for_last_joint_that_moves_closest_point->Frame;
    vctDoubleMat jac_closest = HTPChangeBIGSSJacobianToOffsetFromEEF(jac, transform, self_fk);    
    vctDoubleMat RCM_Mat = H*jac_closest;

    vctDoubleMat full(6,8); //test fix sizes
    full.SetAll(0.0);
    PlaceMatIntoMatAtIndex(RCM_Mat,full,0);

  //   //populate matrix and vector
    IneqConstraintMatrixRef.Assign(full);
  //    std::cout << "IN RCM8" << std::endl;
   IneqConstraintVectorRef.Assign(h);
  //   std::cout << "IN RCM9" << std::endl;

    // ConvertRefs(mode,TickTime);

}