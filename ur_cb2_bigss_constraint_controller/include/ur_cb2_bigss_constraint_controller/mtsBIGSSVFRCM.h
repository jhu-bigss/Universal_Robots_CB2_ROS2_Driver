#ifndef MTSBIGSSVFRCM_H
#define MTSBIGSSVFRCM_H

#include <cisstCommon/cmnGenericObject.h>
#include <cisstVector/vctDynamicVectorTypes.h>
#include <cisstVector/vctFixedSizeVectorTypes.h>
#include <cisstVector/vctDynamicMatrixTypes.h>
#include <cisstVector/vctFixedSizeVectorTypes.h>
#include <cisstNumerical/nmrLSqLin.h>
#include <sawConstraintController/mtsVFJointPos.h>
#include <sawConstraintController/mtsVFBase.h>
#include <cisstVector/vctTransformationTypes.h>
#include <sawConstraintController/mtsVFDataBase.h>

struct mtsBIGSSVFDataRCM : mtsVFDataBase
{
    prmKinematicsState *kinematics_for_last_joint_that_moves_closest_point;
    vctFrm3 offset_from_kinematics_to_frame_of_interest;
    vct3 RCM_point;

    mtsBIGSSVFDataRCM(void){};
    mtsBIGSSVFDataRCM(const mtsBIGSSVFDataRCM &other){};
    ~mtsBIGSSVFDataRCM(){};
    std::string HumanReadable(void) const;
};

class mtsBIGSSVFRCM : public mtsVFJointPosition
{
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION, CMN_LOG_LOD_RUN_VERBOSE)

public:
    mtsBIGSSVFRCM() : mtsVFJointPosition() {}
    mtsBIGSSVFRCM(const std::string &name, mtsBIGSSVFDataRCM *data) : mtsVFJointPosition(name, data)
    {
        H.SetSize(8, 6);
        h.SetSize(8);
        H.Zeros();
        h.Zeros();
    }
    void FillInTableauRefs(const mtsVFBase::CONTROLLERMODE mode, const double tickTime);

private:
    vct3 ClosestPoint;
    vctDoubleMat H;
    vctDoubleVec h;

    void FillMoveConstraints3D(vctDoubleMat &H, vctDoubleVec &h,
                               vct3 direction,
                               vct3 error,
                               double tolerance,
                               int tesselationNumber = 8)
    {
        H.SetSize(tesselationNumber, 6);
        h.SetSize(tesselationNumber);
        H.Zeros();
        h.Zeros();
        vct3 unitV_wd;
        vct3 unitV_pl;
        vctFixedSizeMatrix<double, 3, 3> R;
        ZAxisToRotation(direction, R);
        for (int i = 1; i <= tesselationNumber; i++)
        {
            unitV_pl.Assign(cos(i * 2 * 3.14159 / tesselationNumber), sin(i * 2 * 3.14159 / tesselationNumber), 0);
            unitV_wd.ProductOf(R, unitV_pl);
            H(i - 1, 0) = -unitV_wd.X();
            H(i - 1, 1) = -unitV_wd.Y();
            H(i - 1, 2) = -unitV_wd.Z();
            h(i - 1) = -tolerance - vctDotProduct(unitV_wd, -error);
        }
    }

    void ZAxisToRotation(vct3 Zaxis, vctFixedSizeMatrix<double, 3, 3> &R)
    {
        vct3 Dx, Dy;
        vct3 YVec3(0., 1., 0.);
        vct3 ZVec3(0., 0., 1.);
        Zaxis.Divide(Zaxis.Norm());
        if (fabs(vctDotProduct(Zaxis, ZVec3)) > 0.99)
        {
            Dx = (YVec3 % Zaxis);
        }
        else
        {
            Dx = (ZVec3 % Zaxis);
        }
        Dx.Divide(Dx.Norm());
        Dy = (Zaxis % Dx);
        Dy.Divide(Dy.Norm());
        R(0, 0) = Dx(0);
        R(1, 0) = Dx(1);
        R(2, 0) = Dx(2);
        R(0, 1) = Dy(0);
        R(1, 1) = Dy(1);
        R(2, 1) = Dy(2);
        R(0, 2) = Zaxis(0);
        R(1, 2) = Zaxis(1);
        R(2, 2) = Zaxis(2);
        bool sanityCheckOkay = true;
        for (int cc = 0; cc < 3; cc++)
        {
            for (int rr = 0; rr < 3; rr++)
            {
                if (!CMN_ISFINITE(R(rr, cc)))
                    sanityCheckOkay &= false;
            }
        }
        if (sanityCheckOkay == false)
        {
            CMN_LOG_CLASS_RUN_ERROR << "#### Dx ####" << std::endl
                                    << Dx << "########" << std::endl;
            CMN_LOG_CLASS_RUN_ERROR << "#### Dy ####" << std::endl
                                    << Dy << "########" << std::endl;
            CMN_LOG_CLASS_RUN_ERROR << "#### Zaxis ####" << std::endl
                                    << Zaxis << "########" << std::endl;
        }
    }

    // TODO: Don't have the next to functions copy / pasted here
    vctDoubleMat HTPChangeBIGSSJacobianToOffsetFromEEF(const vctDoubleMat &jacobian, const vctFrm3 &transform, const vctFrm3 &self_fk)
    {
        vctDoubleMat jac_out(jacobian);
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
            vctDouble3 v_target = v_start + w_start_hat * (self_FK_R.Inverse() * transform.Translation()); // put offset translation into origin/base coords
            vctDouble3 w_target = w_start;
            std::copy(v_target.begin(), v_target.end(), jac_out.Column(i).begin());
            std::copy(w_target.begin(), w_target.end(), jac_out.Column(i).begin() + 3);
        }
        return jac_out;
    }

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
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsBIGSSVFRCM);

#endif // MTSBIGSSVFRCM_H
