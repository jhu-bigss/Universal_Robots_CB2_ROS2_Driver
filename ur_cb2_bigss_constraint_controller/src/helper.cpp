/*
  Authors: Henry Phalen
*/

#include <ur_cb2_bigss_constraint_controller/helper.h>

// vct3 GetClosestPointToGivenPointOnLine(vct3 givenPt, vct3 linePoint, vct3 lineD);

/**
 * @brief Cuts out a contiguous piece of a prmVelocityJointSet object (e.g. to take system commands and disperse them to individual robots)
 *
 * @param full_command input command of full lenght
 * @param sliced_command container for output slice
 * @param begin index to start slice
 * @param sliced_size length of the slice
 */
void SlicePrmVelocityJointSet(const prmVelocityJointSet &full_command, prmVelocityJointSet &sliced_command, const int begin, const int sliced_size)
{

    if (sliced_command.Goal().size() != (sliced_size))
    {
        sliced_command.SetSize(sliced_size);
    }

    SliceVctVec(full_command.Mask(), sliced_command.Mask(), begin, sliced_size);
    SliceVctVec(full_command.Guard(), sliced_command.Guard(), begin, sliced_size);
    SliceVctVec(full_command.Goal(), sliced_command.Goal(), begin, sliced_size);
    SliceVctVec(full_command.Acceleration(), sliced_command.Acceleration(), begin, sliced_size);
    SliceVctVec(full_command.Deceleration(), sliced_command.Deceleration(), begin, sliced_size);
}

/**
 * @brief Cuts out contiguous piece of vctDoubleVec
 *
 * @param full_command input command of full lenght
 * @param sliced_command container for output slice
 * @param begin index to start slice
 * @param sliced_size length of the slice
 */
void SliceVctVec(const vctDoubleVec &full_vec, vctDoubleVec &sliced_vec, const int begin, const int sliced_size)
{
    // TODO? either this already exists and can be replace or could be better templated
    std::copy(full_vec.begin() + begin, full_vec.begin() + begin + sliced_size, sliced_vec.begin());
}

/**
 * @brief Cuts out contiguous piece of vctBoolVec
 *
 * @param full_command input command of full lenght
 * @param sliced_command container for output slice
 * @param begin index to start slice
 * @param sliced_size length of the slice
 */
void SliceVctVec(const vctBoolVec &full_vec, vctBoolVec &sliced_vec, const int begin, const int sliced_size)
{
    std::copy(full_vec.begin() + begin, full_vec.begin() + begin + sliced_size, sliced_vec.begin());
}

void ConcatenateTwoVctVec(const vctVec &v1, const vctVec &v2, vctVec &v_out)
{
    v_out.SetSize(v1.size() + v2.size());
    std::copy(v1.begin(), v1.end(), v_out.begin());
    std::copy(v2.begin(), v2.end(), v_out.begin() + v1.size());
}

/**
 * @brief Change the base reference frame of a Jacobian using a transformation from its current to new reference frames
 *
 * @param jacobian Matrix mapping joint velocities to cartesian velocities
 * @param transform Transform mapping from current to new base reference frames
 * @return vctDoubleMat Jacobian in new reference frame
 * @details See Mathematical Introduction to Robotic Manipulation (Murray, Li, Sastry 1994) pp 55-56
 */
vctDoubleMat AdjointJacobianByFrm3(const vctDoubleMat &jacobian, const vctFrm3 &transform)
{
    vctDoubleMat jac_out = jacobian;

    vctDouble3x3 p_hat;
    Skew3VecTo3x3Mat(transform.Translation(), p_hat);
    vctRot3 R = transform.Rotation();

    for (size_t i = 0; i < jac_out.cols(); i++)
    {
        vctDouble3 v_start;
        vctDoubleVec column = jac_out.Column(i);
        std::copy(column.begin(), column.begin() + 3, v_start.begin());
        vctDouble3 w_start;
        std::copy(column.begin() + 3, column.end(), w_start.begin());

        vctDouble3 v_target = R * v_start + p_hat * R * w_start;
        vctDouble3 w_target = R * w_start;
        std::copy(v_target.begin(), v_target.end(), jac_out.Column(i).begin());
        std::copy(w_target.begin(), w_target.end(), jac_out.Column(i).begin() + 3);
    }
    return jac_out;
}

// Khatib Intro to robotics 4.7.2 pg 106
vctDoubleMat ChangeFrameOfJacobian(const vctDoubleMat &jacobian, const vctFrm3 &transform)
{
    vctDoubleMat jac_out = jacobian;

    vctRot3 R = transform.Rotation();

    for (size_t i = 0; i < jac_out.cols(); i++)
    {
        vctDouble3 v_start;
        vctDoubleVec column = jac_out.Column(i);
        std::copy(column.begin(), column.begin() + 3, v_start.begin());
        vctDouble3 w_start;
        std::copy(column.begin() + 3, column.end(), w_start.begin());

        vctDouble3 v_target = R * v_start;
        vctDouble3 w_target = R * w_start;
        std::copy(v_target.begin(), v_target.end(), jac_out.Column(i).begin());
        std::copy(w_target.begin(), w_target.end(), jac_out.Column(i).begin() + 3);
    }
    return jac_out;
}

vctDoubleMat HTPChangeBIGSSJacobianToOffsetFromEEF(const vctDoubleMat &jacobian, const vctFrm3 &transform, const vctFrm3 &self_fk)
{
    vctDoubleMat jac_out = jacobian;
    vctRot3 self_FK_R = self_fk.Rotation();

    vctRot3 R = transform.Rotation();

    for (size_t i = 0; i < jac_out.cols(); i++)
    {
        vctDouble3 v_start;
        vctDoubleVec column = jac_out.Column(i);
        std::copy(column.begin(), column.begin() + 3, v_start.begin());
        vctDouble3 w_start;
        std::copy(column.begin() + 3, column.end(), w_start.begin());

        vctDouble3x3 w_start_hat;
        Skew3VecTo3x3Mat(w_start, w_start_hat);
        vctDouble3 v_target = v_start + w_start_hat * (self_FK_R * transform.Translation()); // put offset translation into origin/base coords
        vctDouble3 w_target = w_start;
        std::copy(v_target.begin(), v_target.end(), jac_out.Column(i).begin());
        std::copy(w_target.begin(), w_target.end(), jac_out.Column(i).begin() + 3);
    }
    return jac_out;
}

/**
 * @brief Mathematical skew or 'hat' operator taking vector to matrix (e.g. rotation vector axis*angle to element of so(3))
 *
 * @param vector Vector in R3
 * @param mat Resulting skew / hat matrix
 */
void Skew3VecTo3x3Mat(const vct3 &vector, vctDouble3x3 &mat)
{
    mat.SetAll(0.0);
    mat(2, 1) = vector(0); // x
    mat(0, 2) = vector(1); // y
    mat(1, 0) = vector(2); // z

    mat(1, 2) = -mat(2, 1); //-x
    mat(2, 0) = -mat(0, 2); //-y
    mat(0, 1) = -mat(1, 0); //-z
}

void Skew3VecTo3x3Mat(const vct3 &vector, vctDoubleMat &mat)
{
    mat.resize(3, 3);
    mat.SetAll(0.0);
    mat(2, 1) = vector(0); // x
    mat(0, 2) = vector(1); // y
    mat(1, 0) = vector(2); // z

    mat(1, 2) = -mat(2, 1); //-x
    mat(2, 0) = -mat(0, 2); //-y
    mat(0, 1) = -mat(1, 0); //-z
}

void InvSkew3x3MatTo3Vec(const vctDouble3x3 &mat, vct3 &vector)
{
    // could check to be sure diags are zero here
    vector(0) = mat(2, 1); // x
    vector(1) = mat(0, 2); // y
    vector(2) = mat(1, 0); // z
}

/**
 * @brief Represent a CISST vctFrm3 transformation as a CISST vctDoubleVec of length 6 of form [translation; axis*angle]
 *
 * @param frame Transformation matrix
 * @param r6_pos_axang Output vector with first 3 numbers as translation and second 3 numbers as unit axis vector multiplied by angle
 */
void ConvertFrm3ToR6(const vctFrm3 &frame, vctDoubleVec &r6_pos_axang)
{
    std::copy(frame.Translation().begin(), frame.Translation().end(), r6_pos_axang.begin());
    vctRodRot3 frame_rodrot(frame.Rotation(), false);
    std::copy(frame_rodrot.begin(), frame_rodrot.end(), r6_pos_axang.begin() + 3);
}

/**
 * @brief Represent a CISST vctDoubleVec of length 6 of form [translation; axis*angle] as a CISST vctFrm3 transformation
 *
 * @param r6_pos_axang Vector with first 3 numbers as translation and second 3 numbers as unit axis vector multiplied by angle
 * @param frame Output corresponding transformation matrix
 */
void ConvertR6ToFrm3(const vctDoubleVec &r6_pos_axang, vctFrm3 &frame)
{
    std::copy(r6_pos_axang.begin(), r6_pos_axang.begin() + 3, frame.Translation().begin());
    vctRodRot3 frame_rodrot;
    std::copy(r6_pos_axang.begin() + 3, r6_pos_axang.end(), frame_rodrot.begin());
    frame.Rotation() = vctMatRot3(frame_rodrot, true);
}

/**
 * @brief Read in a CISST vctFrm3 transformation from a text file with space separated transformation in homogeous (4x4) form [R,p]
 *
 * @param filename
 * @return vctFrm3 output transform
 */
vctFrm3 GetTransformFromTxt(const std::string &filename)
{
    vctFrm3 transform;
    std::ifstream file(filename);
    if (!file)
    {
        std::cout << "Error reading: " << filename << std::endl;
    }
    for (int i = 0; i < 3; i++)
    {
        file >>
            transform.Rotation()(i, 0) >>
            transform.Rotation()(i, 1) >>
            transform.Rotation()(i, 2) >>
            transform.Translation()(i);
    }
    file.close();
    transform.Rotation().NormalizedSelf();
    return transform;
}

/**
 * @brief Calculate cartesian error between two transforms represented in [translation; axis*angle] form (NOTE: you can't just subtract!)
 *
 * @param target_cp_R6 goal transform [translation; axis*angle]
 * @param measured_cp_R6 measured/ current transform [translation; axis*angle]
 * @param pos_err output position error
 * @param rot_ax_ang_err output rotational error
 * @bug Don't think math is right! Look at VFFollowSettlingGains --HTP
 */
void CalculateCartesianErrorFromR6(const vctDoubleVec &target_cp_R6, const vctDoubleVec &measured_cp_R6, vct3 &pos_err, vct3 &rot_ax_ang_err)
{
    // TODO? Could make a class member function that calls this one and sets robotState.CartesianPositionError?
    // TODO? Could return vctDoubleVec [pos_error, rot_error]
    vctFrm3 measured_cp_Frm3, target_cp_Frm3;
    ConvertR6ToFrm3(measured_cp_R6, measured_cp_Frm3);
    ConvertR6ToFrm3(target_cp_R6, target_cp_Frm3);
    vctFrm3 error_cp_Frm3;
    error_cp_Frm3.Rotation() = measured_cp_Frm3.Rotation().Inverse() * target_cp_Frm3.Rotation();
    error_cp_Frm3.Translation() = target_cp_Frm3.Translation() - measured_cp_Frm3.Translation();

    vctDoubleVec error_cp_R6(6);
    ConvertFrm3ToR6(error_cp_Frm3, error_cp_R6);
    // rotate back
    std::copy(error_cp_R6.begin(), error_cp_R6.begin() + 3, pos_err.begin());
    std::copy(error_cp_R6.begin() + 3, error_cp_R6.end(), rot_ax_ang_err.begin());
}

void PlaceMatIntoMatAtIndex(vctDoubleMat &input_matrix, vctDoubleMat &full_matrix, size_t index)
{
    // TODO: size checks
    for (size_t i = 0; i < input_matrix.cols(); i++)
        full_matrix.Column(index + i) = input_matrix.Column(i);
}

void ExtractAngularJacobianToArbitrarySize(vctDoubleMat &full_jacobian, vctDoubleMat &out_jacobian, int out_cols)
{
    out_jacobian.resize(3, out_cols);
    out_jacobian.SetAll(0.0);
    out_jacobian.Ref(3, full_jacobian.cols()) = full_jacobian.Ref(3, full_jacobian.cols(), 3, 0);
}

void ExtractPositionalJacobianToArbitrarySize(vctDoubleMat &full_jacobian, vctDoubleMat &out_jacobian, int out_cols)
{
    out_jacobian.resize(3, out_cols);
    out_jacobian.SetAll(0.0);
    out_jacobian.Ref(3, full_jacobian.cols()) = full_jacobian.Ref(3, full_jacobian.cols());
}

vct3 GetClosestPointToGivenPointOnLine(vct3 givenPt, vct3 linePoint, vct3 lineD)
{
    vct3 xcl;
    double normD = lineD(0) * lineD(0) + lineD(1) * lineD(1) + lineD(2) * lineD(2);
    double sqrtNorm = sqrt(normD);
    lineD(0) /= sqrtNorm;
    lineD(1) /= sqrtNorm;
    lineD(2) /= sqrtNorm;
    double bx = givenPt(0) - linePoint(0);
    double by = givenPt(1) - linePoint(1);
    double bz = givenPt(2) - linePoint(2);
    double d = bx * lineD(0) + by * lineD(1) + bz * lineD(2);
    xcl(0) = linePoint(0) + lineD(0) * d;
    xcl(1) = linePoint(1) + lineD(1) * d;
    xcl(2) = linePoint(2) + lineD(2) * d;
    return xcl;
}