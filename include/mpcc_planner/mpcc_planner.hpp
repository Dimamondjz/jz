#ifndef MPCC_PLANNER_HPP
#define MPCC_PLANNER_HPP
#include <osqp_interface/osqp_interface.h>

#include <Eigen/Core>
#include <Eigen/Sparse>

#include "mpcc_planner/arc_spline.hpp"
#include "mpcc_planner/kinematic_model.hpp"
#include "yaml.h"
namespace mpcc {
    class MpccPlanner {
    private:
        double ts_;          //规划器时间间隔，采样时间
        int N_;              //预测步数
        double trackWidth_;  //轨道宽度
        double qvs_;
        double v_b_, a_b_, delta_b_, ddelta_b_, vs_b_,
                dt_;  //速度边界，加速度边界，速度转向边界，里程速度
        double delta_last_ = 0, tip_, a_brake_b_, ll_, r_min_;  //上一步的转向角
        int node_index_ = 0;
        double k_brake_ = 0, k_rvs_ = 0, vs_init_b_;
        std::vector<double> s_node_ = {}, r_node_ = {};
        arc_spline::ArcSpline s_;                        //样条函数
        kinematic_model::KinematicModel::Ptr modelPtr_;  //声明运动学模型智能指针
        osqp::OSQPInterface osqp_;                       //声明osqp接口

        // mpc matrices
        Eigen::MatrixXd state_list_;
        Eigen::MatrixXd input_list_;
        Eigen::SparseMatrix<double> P_, q_, A_, l_, u_;
        Eigen::SparseMatrix<double> P0_, q0_;
        Eigen::SparseMatrix<double> Cx_, lx_, ux_;  //状态约束
        Eigen::SparseMatrix<double> Cu_, lu_, uu_;  //输入控制约束

    public:
        MpccPlanner() {};

        ~MpccPlanner() {};

        void init();  //初始化
        static Eigen::MatrixXd circular_arc(double start_rad, double drad, int n,
                                            double x0, double y0, double R0);

        static Eigen::MatrixXd straight_line(double length, double slope_rad, int n,
                                             double x0, double y0);

        // core module
        int solveQP(kinematic_model::VectorX x0);  // Vector：状态空间
    };
}
#endif
