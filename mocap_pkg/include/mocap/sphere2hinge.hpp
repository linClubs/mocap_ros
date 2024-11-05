#pragma once

#include <iostream>
#include <cmath>
#include <vector>
#include <eigen3/Eigen/Dense>

// using namespace std;
// using namespace Eigen;
const double PI = M_PI;

Eigen::MatrixXd JacobianLeft(const Eigen::VectorXd &curJnts)
{
    using namespace Eigen;
    double alpha = curJnts(0), beta = curJnts(1), gamma = curJnts(2), theta = curJnts(3);
    MatrixXd A0 = MatrixXd::Zero(6, 3);

    double t2 = cos(alpha);
    double t3 = cos(beta);
    double t4 = cos(gamma);
    double t5 = sin(alpha);
    double t6 = sin(beta);
    double t7 = cos(theta);
    double t8 = sin(gamma);
    double t9 = sin(theta);
    double t10 = t2 * 1.786061951567319E-1;
    double t11 = t2 * 3.830222215594858E-1;
    double t12 = t2 * 8.213938048432681E-1;
    double t16 = t3 * t5 * -3.830222215594858E-1;
    double t17 = t3 * t5 * 3.830222215594858E-1;
    double t18 = t2 * t3 * 9.063077870366527E-1;
    double t19 = t3 * t5 * 8.213938048432681E-1;
    double t20 = t5 * t6 * 1.786061951567319E-1;
    double t21 = t2 * t6 * 4.226182617407019E-1;
    double t22 = t3 * t5 * -4.226182617407019E-1;
    double t23 = t3 * t5 * 4.226182617407019E-1;
    double t24 = t5 * t6 * 3.830222215594858E-1;
    double t25 = t5 * t6 * 9.063077870366527E-1;
    double t13 = t11 - 3.830222215594858E-1;
    double t14 = t10 + 8.213938048432681E-1;
    double t15 = t12 + 1.786061951567319E-1;
    double t32 = t19 + t24;
    double t33 = t22 + t25;
    double t26 = t3 * t13;
    double t27 = t3 * t14;
    double t28 = t6 * t13 * -1.0;
    double t30 = t6 * t15 * -1.0;
    double t34 = t26 + t30;
    double t35 = t27 + t28;
    A0(0, 0) = -t18 - t21;
    A0(0, 1) = t33;
    A0(1, 0) = t16 - t20;
    A0(1, 1) = t35;
    A0(2, 0) = t32;
    A0(2, 1) = t26 * -1.0 + t6 * t15;
    A0(3, 0) = t7 * (t18 + t21) * -1.0 + t9 * (t4 * t5 * 1.0 + t8 * (t2 * t3 * 4.226182617407019E-1 - t2 * t6 * 9.063077870366527E-1));
    A0(3, 1) = -t7 * (t23 - t25) - t8 * t9 * (t3 * t5 * 9.063077870366527E-1 + t5 * t6 * 4.226182617407019E-1) * 1.0;
    A0(3, 2) = t9 * (t4 * (t23 - t25) + t8 * (t2 * 1.0 + 4.384827237351328E-17) * 1.0) * 1.0;
    A0(4, 0) = t7 * (t17 + t20) * -1.0 - t9 * (t2 * t4 * 4.226182617407019E-1 + t8 * (t24 - t3 * t5 * 1.786061951567319E-1)) * 1.0;
    A0(4, 1) = t7 * t35 + t8 * t9 * (t26 + t6 * t14);
    A0(4, 2) = t9 * (t5 * t8 * 4.226182617407019E-1 - t4 * t35 * 1.0);
    A0(5, 0) = t9 * (t2 * t4 * 9.063077870366527E-1 - t8 * (t17 - t5 * t6 * 8.213938048432681E-1)) + t7 * t32;
    A0(5, 1) = t7 * t34 * -1.0 - t8 * t9 * (t3 * t15 + t6 * t13) * 1.0;
    A0(5, 2) = t9 * (t5 * t8 * 9.063077870366527E-1 - t4 * t34 * 1.0) * -1.0;
    return A0;
}

Eigen::VectorXd ResidualLeft(const Eigen::VectorXd &curJnts, const Eigen::Vector3d &upVec, const Eigen::Vector3d &lowVec)
{
    using namespace Eigen;
    double alpha = curJnts(0), beta = curJnts(1), gamma = curJnts(2), theta = curJnts(3);
    double upx = upVec(0), upy = upVec(1), upz = upVec(2);
    double lowx = lowVec(0), lowy = lowVec(1), lowz = lowVec(2);
    VectorXd A0(6);

    double t2 = cos(alpha);
    double t3 = cos(beta);
    double t4 = cos(gamma);
    double t5 = sin(alpha);
    double t6 = sin(beta);
    double t7 = cos(theta);
    double t8 = sin(gamma);
    double t9 = sin(theta);
    double t10 = t3 * t5 * 9.063077870366499E-1;
    double t11 = t5 * t6 * 4.226182617406994E-1;
    double t12 = t2 * 1.786061951567303E-1;
    double t13 = t2 * 3.83022221559489E-1;
    double t14 = t2 * 8.213938048432696E-1;
    double t15 = t14 + 1.786061951567304E-1;
    double t16 = t13 - 3.83022221559489E-1;
    double t17 = t12 + 8.213938048432697E-1;
    double t18 = t3 * t15;
    double t19 = t3 * t16;
    double t20 = t6 * t16;
    double t21 = t6 * t17;
    double t22 = -t20;
    A0[0] = -t10 - t11 - upx;
    A0[1] = t19 + t21 - upy;
    A0[2] = -t18 + t22 - upz;
    A0[3] = -lowx - t7 * (t10 + t11) - t9 * (t4 * (t2 * 1.0 + 4.384827237351354E-17) - t8 * (t3 * t5 * 4.226182617406994E-1 - t5 * t6 * 9.063077870366499E-1));
    A0[4] = -lowy + t7 * (t19 + t21) - t9 * (t4 * t5 * 4.226182617406994E-1 - t8 * (t20 - t3 * t17));
    A0[5] = -lowz - t7 * (t18 + t20) + t9 * (t4 * t5 * 9.063077870366499E-1 + t8 * (t19 - t6 * t15));

    return A0;
}

Eigen::MatrixXd JacobianRight(const Eigen::VectorXd &curJnts)
{
    using namespace Eigen;
    double alpha = curJnts(0), beta = curJnts(1), gamma = curJnts(2), theta = curJnts(3);
    MatrixXd A0 = MatrixXd::Zero(6, 3);

    double t2 = cos(alpha);
    double t3 = cos(beta);
    double t4 = cos(gamma);
    double t5 = sin(alpha);
    double t6 = sin(beta);
    double t7 = cos(theta);
    double t8 = sin(gamma);
    double t9 = sin(theta);
    double t10 = t2 * 1.786061951567319E-1;
    double t11 = t2 * 3.830222215594858E-1;
    double t12 = t2 * 8.213938048432681E-1;
    double t16 = t3 * t5 * 3.830222215594858E-1;
    double t17 = t2 * t3 * 9.063077870366527E-1;
    double t18 = t3 * t5 * 8.213938048432681E-1;
    double t19 = t5 * t6 * 1.786061951567319E-1;
    double t20 = t2 * t6 * 4.226182617407019E-1;
    double t21 = t3 * t5 * 4.226182617407019E-1;
    double t22 = t5 * t6 * 3.830222215594858E-1;
    double t23 = t5 * t6 * 9.063077870366527E-1;
    double t13 = t11 - 3.830222215594858E-1;
    double t14 = t10 + 8.213938048432681E-1;
    double t15 = t12 + 1.786061951567319E-1;
    double t28 = t21 + t23;
    double t24 = t3 * t13;
    double t25 = t3 * t14;
    double t26 = t6 * t13;
    double t27 = t6 * t15;
    double t29 = t24 + t27;
    double t30 = t25 + t26;

    A0(0, 0) = -t17 + t20;
    A0(0, 1) = t28;
    A0(1, 0) = t16 - t19;
    A0(1, 1) = t30;
    A0(2, 0) = t18 - t22;
    A0(2, 1) = t29;
    A0(3, 0) = t9 * (t4 * t5 * 1.0 - t8 * (t2 * t3 * 4.226182617407019E-1 + t2 * t6 * 9.063077870366527E-1)) * 1.0 - t7 * (t17 - t20) * 1.0;
    A0(3, 1) = t7 * t28 - t8 * t9 * (t3 * t5 * 9.063077870366527E-1 - t5 * t6 * 4.226182617407019E-1);
    A0(3, 2) = t9 * (t4 * t28 - t8 * (t2 * 1.0 + 4.384827237351328E-17) * 1.0) * -1.0;
    A0(4, 0) = t9 * (t2 * t4 * 4.226182617407019E-1 + t8 * (t22 + t3 * t5 * 1.786061951567319E-1)) + t7 * (t16 - t19) * 1.0;
    A0(4, 1) = t7 * t30 - t8 * t9 * (t24 - t6 * t14 * 1.0) * 1.0;
    A0(4, 2) = t9 * (t5 * t8 * 4.226182617407019E-1 + t4 * t30) * -1.0;
    A0(5, 0) = t9 * (t2 * t4 * 9.063077870366527E-1 + t8 * (t16 + t5 * t6 * 8.213938048432681E-1)) + t7 * (t18 - t22) * 1.0;
    A0(5, 1) = t7 * t29 + t8 * t9 * (t26 * 1.0 - t3 * t15) * 1.0;
    A0(5, 2) = t9 * (t5 * t8 * 9.063077870366527E-1 + t4 * t29) * -1.0;

    return A0;
}

Eigen::VectorXd ResidualRight(const Eigen::VectorXd &curJnts, const Eigen::Vector3d &upVec, const Eigen::Vector3d &lowVec)
{
    using namespace Eigen;
    double alpha = curJnts(0), beta = curJnts(1), gamma = curJnts(2), theta = curJnts(3);
    double upx = upVec(0), upy = upVec(1), upz = upVec(2);
    double lowx = lowVec(0), lowy = lowVec(1), lowz = lowVec(2);
    VectorXd A0(6);

    double t2 = cos(alpha);
    double t3 = cos(beta);
    double t4 = cos(gamma);
    double t5 = sin(alpha);
    double t6 = sin(beta);
    double t7 = cos(theta);
    double t8 = sin(gamma);
    double t9 = sin(theta);
    double t10 = t3 * t5 * 9.063077870366499E-1;
    double t11 = t5 * t6 * 4.226182617406994E-1;
    double t13 = t2 * 1.786061951567303E-1;
    double t14 = t2 * 3.83022221559489E-1;
    double t15 = t2 * 8.213938048432696E-1;
    double t12 = -t10;
    double t16 = t15 + 1.786061951567304E-1;
    double t17 = t14 - 3.83022221559489E-1;
    double t18 = t13 + 8.213938048432697E-1;
    double t19 = t3 * t16;
    double t20 = t3 * t17;
    double t21 = t6 * t17;
    double t22 = t6 * t18;
    A0[0] = t11 + t12 - upx;
    A0[1] = -t20 + t22 - upy;
    A0[2] = -t19 + t21 - upz;
    A0[3] = -lowx - t7 * (t10 - t11) - t9 * (t4 * (t2 * 1.0 + 4.384827237351354E-17) + t8 * (t3 * t5 * 4.226182617406994E-1 + t5 * t6 * 9.063077870366499E-1));
    A0[4] = -lowy - t7 * (t20 - t22) + t9 * (t4 * t5 * 4.226182617406994E-1 - t8 * (t21 + t3 * t18));
    A0[5] = -lowz - t7 * (t19 - t21) + t9 * (t4 * t5 * 9.063077870366499E-1 - t8 * (t20 + t6 * t16));

    return A0;
}

std::pair<bool, Eigen::Vector4d> sphere2hingeRight(const Eigen::Vector3d &upVec, const Eigen::Vector3d &lowVec, const Eigen::Vector4d &initialGuess = Eigen::Vector4d(0.01, 0.01, 0.01, 0.01))
{
    using namespace Eigen;

    Vector4d curJnts = initialGuess;
    curJnts[3] = -acos(upVec.dot(lowVec));
    for (int i = 0; i < 1000; i++)
    {
        auto resVal = ResidualRight(curJnts, upVec, lowVec);
        if (resVal.norm() < 1e-4)
        {
            for (auto &jnt : curJnts)
            {
                while (jnt < -PI)
                {
                    jnt += 2 * PI;
                }
                while (jnt > PI)
                {
                    jnt -= 2 * PI;
                }
            }
            return {true, curJnts};
        }
        Eigen::Matrix<double, 6, 3> Jval = JacobianRight(curJnts);
        curJnts.block<3, 1>(0, 0) -= Jval.completeOrthogonalDecomposition().pseudoInverse() * resVal;
    }
    return {false, Vector4d(0, 0, 0, 0)};
}

std::pair<bool, Eigen::Vector4d> sphere2hingeLeft(const Eigen::Vector3d &upVec, const Eigen::Vector3d &lowVec, const Eigen::Vector4d &initialGuess = Eigen::Vector4d(0.01, 0.01, 0.01, 0.01))
{
    using namespace Eigen;

    Vector4d curJnts = initialGuess;
    curJnts[3] = -acos(upVec.dot(lowVec));
    for (int i = 0; i < 1000; i++)
    {
        auto resVal = ResidualLeft(curJnts, upVec, lowVec);
        if (resVal.norm() < 1e-4)
        {
            for (auto &jnt : curJnts)
            {
                while (jnt < -PI)
                {
                    jnt += 2 * PI;
                }
                while (jnt > PI)
                {
                    jnt -= 2 * PI;
                }
            }
            return {true, curJnts};
        }
        Eigen::Matrix<double, 6, 3> Jval = JacobianLeft(curJnts);
        curJnts.block<3, 1>(0, 0) -= Jval.completeOrthogonalDecomposition().pseudoInverse() * resVal;
    }
    return {false, Vector4d(0, 0, 0, 0)};
}

std::pair<bool, Eigen::Vector4d> globalHandRot2UpLeft(const Eigen::Vector3d &upVec, const Eigen::Vector3d &lowVec, const Eigen::Vector4d &initialGuess = Eigen::Vector4d(0.01, 0.01, 0.01, 0.01))
{
    using namespace Eigen;

    Vector4d curJnts = initialGuess;
    curJnts[3] = -acos(upVec.dot(lowVec));
    for (int i = 0; i < 1000; i++)
    {
        auto resVal = ResidualLeft(curJnts, upVec, lowVec);
        if (resVal.norm() < 1e-4)
        {
            for (auto &jnt : curJnts)
            {
                while (jnt < -PI)
                {
                    jnt += 2 * PI;
                }
                while (jnt > PI)
                {
                    jnt -= 2 * PI;
                }
            }
            return {true, curJnts};
        }
        Eigen::Matrix<double, 6, 3> Jval = JacobianLeft(curJnts);
        curJnts.block<3, 1>(0, 0) -= Jval.completeOrthogonalDecomposition().pseudoInverse() * resVal;
    }
    return {false, Vector4d(0, 0, 0, 0)};
}
