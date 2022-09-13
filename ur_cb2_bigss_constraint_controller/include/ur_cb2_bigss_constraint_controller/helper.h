/*
  Authors: Henry Phalen
*/

#ifndef BIGSSCONSTRAINTCONTROLLER_HELPER_H
#define BIGSSCONSTRAINTCONTROLLER_HELPER_H

#include <cisstParameterTypes/prmVelocityJointSet.h>
#include <cisstVector.h>

// TODO: Take out of global scope
// TODO? Should these functions exist either as static members of SeriallyAttachedRoboticSystem, or somewhere else? 
//   They may also be duplicates of existing CISST (or other) functionality

void SlicePrmVelocityJointSet(const prmVelocityJointSet& full_command, prmVelocityJointSet& sliced_command, const int begin, const int sliced_size);
void SliceVctVec(const vctDoubleVec& full_vec, vctDoubleVec& sliced_vec, const int begin,  const int sliced_size);
void SliceVctVec(const vctBoolVec& full_vec, vctBoolVec& sliced_vec, const int begin,  const int sliced_size);
void ConcatenateTwoVctVec(const vctVec& v1, const vctVec& v2, vctVec& v_out);
vctDoubleMat AdjointJacobianByFrm3(const vctDoubleMat &jacobian, const vctFrm3 &transform);
vctDoubleMat ChangeFrameOfJacobian(const vctDoubleMat &jacobian, const vctFrm3 &transform);
vctDoubleMat HTPChangeBIGSSJacobianToOffsetFromEEF(const vctDoubleMat &jacobian, const vctFrm3 &transform, const vctFrm3 &self_fk);
void Skew3VecTo3x3Mat(const vct3& vector, vctDouble3x3& mat);
void Skew3VecTo3x3Mat(const vct3& vector, vctDoubleMat& mat);
void InvSkew3x3MatTo3Vec(const vctDouble3x3& mat, vct3& vector);
void ConvertFrm3ToR6(const vctFrm3& frame, vctDoubleVec& r6_pos_axang);
void ConvertR6ToFrm3(const vctDoubleVec& r6_pos_axang, vctFrm3& frame);
vctFrm3 GetTransformFromTxt(const std::string &filename);
void CalculateCartesianErrorFromR6(const vctDoubleVec &target_cp_R6, const vctDoubleVec &measured_cp_R6, vct3 &pos_err, vct3 &rot_ax_ang_err);
void PlaceMatIntoMatAtIndex(vctDoubleMat& input_matrix, vctDoubleMat& full_matrix, size_t index);
void ExtractAngularJacobianToArbitrarySize(vctDoubleMat& full_jacobian, vctDoubleMat& out_jacobian, int out_cols);
void ExtractPositionalJacobianToArbitrarySize(vctDoubleMat& full_jacobian, vctDoubleMat& out_jacobian, int out_cols);
vct3 GetClosestPointToGivenPointOnLine(vct3 givenPt, vct3 linePoint, vct3 lineD);

#endif // BIGSSCONSTRAINTCONTROLLER_HELPER_H
