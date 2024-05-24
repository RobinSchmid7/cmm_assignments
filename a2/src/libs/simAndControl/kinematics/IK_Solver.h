#pragma once

#include <robot/GeneralizedCoordinatesRobotRepresentation.h>
#include <robot/Robot.h>

namespace crl {

struct IK_EndEffectorTargets {
    RB *rb = nullptr;
    P3D p;       // local coordinates of end effector in rb's frame
    P3D target;  // target position in world frame
};

class IK_Solver {
public:
    IK_Solver(Robot *robot) : robot(robot) {}

    ~IK_Solver(void) {}

    /**
     * add IK end effector target to solver. Specify the end effector point p, which 
     * is specified in the local coordinates of rb and its target expressed in world frame.
     */
    void addEndEffectorTarget(RB *rb, P3D p, P3D target) {
        endEffectorTargets.push_back(IK_EndEffectorTargets());
        endEffectorTargets.back().rb = rb;
        endEffectorTargets.back().p = p;
        endEffectorTargets.back().target = target;
    }

    void solve(int nSteps = 10) {
        GeneralizedCoordinatesRobotRepresentation gcrr(robot);
        
        for (uint i = 0; i < nSteps; i++) {
            dVector q;
            gcrr.getQ(q);

            // get current generalized coordinates of the robots

            // TODO: Ex.2-2 Inverse Kinematics
            //
            // update generalized coordinates of the robot by solving IK.

            // remember, we don't update base pose since we assume it's already at
            // the target position and orientation
            dVector deltaq(q.size() - 6);
            deltaq.setZero();

            // TODO: here, compute deltaq using GD, Newton, or Gauss-Newton.
            // end effector targets are stored in endEffectorTargets vector.
            //
            // Hint:
            // - use gcrr.estimate_linear_jacobian(p, rb, dpdq) function for Jacobian matrix.
            // - if you already implemented analytic Jacobian, you can use gcrr.compute_dpdq(const P3D &p, RB *rb, Matrix &dpdq)
            // - don't forget we use only last q.size() - 6 columns (use block(0,6,3,q.size() - 6) function)
            // - when you compute inverse of the matrix, use ldlt().solve() instead of inverse() function. this is numerically more stable.
            //   see https://eigen.tuxfamily.org/dox-devel/group__LeastSquares.html

            // TODO: your implementation should be here.
            // Compute dq for each joint separately and sum up
            // Compare Gradient descent, Gauss Newton for linear and analytic jacobian

            // Use small regularizers to improve stability
            double lambdaLinear = 0.0001;
            double lambdaAnalytic = 0.0001;

            for (int j = 0; j < endEffectorTargets.size(); j++) {

                // Linear jacobian
                Matrix linearJacobianFull;
                gcrr.estimate_linear_jacobian(endEffectorTargets[j].p, endEffectorTargets[j].rb, linearJacobianFull);
                Matrix linearJacobianEE = linearJacobianFull.block(0,6,3,q.size()-6);
                Matrix linearJacobianEETranspose = linearJacobianEE.transpose();
                double szL = linearJacobianEE.cols();
                Matrix linearJacobianPseudoInverse = (linearJacobianEETranspose*linearJacobianEE + lambdaLinear * Matrix::Identity(szL,szL)).ldlt().solve(linearJacobianEETranspose);

                // Analytic jacobian
                Matrix analyticJacobianFull;
                gcrr.compute_dpdq(endEffectorTargets[j].p, endEffectorTargets[j].rb, analyticJacobianFull);
                Matrix analyticJacobianEE = analyticJacobianFull.block(0,6,3,q.size()-6);
                Matrix analyticJacobianEETranspose = analyticJacobianEE.transpose();
                double szA = analyticJacobianEE.cols();
                Matrix analyticJacobianPseudoInverse = (analyticJacobianEETranspose*analyticJacobianEE + lambdaAnalytic * Matrix::Identity(szA,szA)).ldlt().solve(analyticJacobianEETranspose);

                P3D pCurr = gcrr.getWorldCoordinates(endEffectorTargets[j].p, endEffectorTargets[j].rb);
                P3D pDiff = endEffectorTargets[j].target - pCurr;
                Vector3d vDiff(pDiff.x, pDiff.y, pDiff.z);

                // Different methods to compute update step

                // Gradient descent linear jacobian
//                deltaq += linearJacobianEETranspose*vDiff;
                // Gauss Newton linear jacobian
//                deltaq += linearJacobianPseudoInverse*vDiff;
                // Gradient descent analytic jacobian
//                deltaq += analyticJacobianEETranspose*vDiff;
                // Gauss Newton analytic jacobian

                // Go with Gauss Newton, regularized and analytic jacobian
                deltaq += analyticJacobianPseudoInverse*vDiff;

                // Comparing both jacobians numerically for one limb
//                if (j == 0) std::cout << "Linear Jacobian:\n" << linearJacobianEE << "\n\n\n";
//                if (j == 0) std::cout << "Analytic Jacobian:\n" << analyticJacobianEE << "\n\n\n";

                // Check eigenvalues of matrices to analyse stability
//                Eigen::EigenSolver<Eigen::MatrixXd> esL(linearJacobianEETranspose*linearJacobianEE + lambdaLinear * Matrix::Identity(szL,szL));
//                if (j == 0) std::cout << "Linear Jacobian Eigenvalues:\n" << esL.eigenvalues() << "\n\n\n";
//
//                Eigen::EigenSolver<Eigen::MatrixXd> esA(analyticJacobianEETranspose*analyticJacobianEE + lambdaAnalytic * Matrix::Identity(szA,szA));
//                if (j == 0) std::cout << "Analytic Jacobian Eigenvalues:\n" << esA.eigenvalues() << "\n\n\n";
                // Use small regularization to make it stable

            }

            q.tail(q.size() - 6) += deltaq;

            // now update gcrr with q
            gcrr.setQ(q);
        }

        gcrr.syncRobotStateWithGeneralizedCoordinates();

        // clear end effector targets
        // we will add targets in the next step again.
        endEffectorTargets.clear();
    }

private:
    Robot *robot;
    std::vector<IK_EndEffectorTargets> endEffectorTargets;
};

}  // namespace crl