#pragma once

#include <rbdl/rbdl.h>
#include <Eigen/Dense>

namespace rbdl_dls_solver
{

class DLSSolver
{
public:
    bool init();

    Eigen::VectorXd calculate(const Eigen::VectorXd& q, const Eigen::VectorXd& desired_v, double dt);
    Eigen::Isometry3d eePose(const Eigen::VectorXd& q);

private:
    RigidBodyDynamics::Model model_;
    Eigen::MatrixXd J_;
    unsigned int ee_id_ = 0;
    double lambda_ = 1e-3;
};

}  // namespace rbdl_dls_solver
