//
// Created by clint on 2020/11/25.
//

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
  MpccPlanner(){};
  ~MpccPlanner(){};
  void init();  //初始化
  static Eigen::MatrixXd circular_arc(double start_rad, double drad, int n,
                                      double x0, double y0, double R0);
  static Eigen::MatrixXd straight_line(double length, double slope_rad, int n,
                                       double x0, double y0);
  // core module
  std::vector<double> solveQP(kinematic_model::VectorX x0);  // Vector：状态空间
};

void MpccPlanner::init() {
  modelPtr_.reset(new kinematic_model::KinematicModel());  //智能指针重定向
  std::vector<double> track_points_x, track_points_y,
      track_s = {}, track_r = {}, track_ddeg = {};  //声明轨道x，y坐标向量
  YAML::Node config = YAML::LoadFile("../config/parameters.yaml");
  track_s = config["track_s"].as<std::vector<double>>();
  track_r = config["track_r"].as<std::vector<double>>();
  track_ddeg = config["track_ddeg"].as<std::vector<double>>();
  ts_ = config["ts"].as<double>();  //规划时间间隔
  dt_ = config["dt"].as<double>();
  N_ = config["N"].as<int>();                       //读取预测步数
  trackWidth_ = config["trackWidth"].as<double>();  //读取轨道宽度
  qvs_ = config["qvs"].as<double>();                //~0.01
  v_b_ = config["v_b"].as<double>();                //读取速度边界
  a_b_ = config["a_b"].as<double>();                //读取加速度边界
  a_brake_b_ = config["a_brake_b"].as<double>();    //读取减速度边界
  delta_b_ = config["delta_b"].as<double>();        //读取转向边界
  ddelta_b_ = config["ddelta_b"].as<double>();      //读取转向速度边界
  vs_b_ = config["vs_b"].as<double>();              //
  tip_ = config["tip"].as<double>();                //原地转弯角度
  ll_ = config["ll"].as<double>();                  //车轴距
  delta_b_ *= M_PI / 180;                           // deg转rad
  ddelta_b_ *= M_PI * ts_ / 180;                    // deg转rad
  vs_init_b_ = v_b_;
  k_rvs_ = (vs_init_b_ - 0.2) / (4 - 0.3);
  if (tip_) {
    r_min_ = ll_ / std::tan(delta_b_ - 20. / 180 * M_PI);
    N_ = 10;
    v_b_ = 0.2;
    vs_init_b_ = v_b_;
    track_s = {0.0625};
    track_r = {r_min_};
    track_ddeg = {tip_};
  }
  if (track_s.size()) {
    track_points_x = {0};
    track_points_y = {0};
    double x0 = 0.0, y0 = 0.0, length, slope_rad = 0.0, R0, drad, start_rad,
           s_sum = 0;
    int n = 720, anticlockwise;
    Eigen::MatrixXd straight_line_result, circle_arc_result;
    for (int i = 0; i < track_s.size(); ++i) {
      // std::cout<<i+1<<" "<<i+1<<" "<<i+1<<" "<<i+1<<" "<<i+1<<" "<<i+1<<"
      // "<<i+1<<" "<<std::endl;
      length = track_s[i];
      // std::cout<<"直线参数：\n"<<track_s[i]<<" "<<slope_rad*180./M_PI<<"
      // "<<n<<" "<<x0<<" "<<y0<<" "<<std::endl;
      if (length == 0) {
        straight_line_result.resize(2, 1);
        straight_line_result << x0, y0;
      } else {
        straight_line_result = straight_line(length, slope_rad, n, x0, y0);
      }
      // std::cout<<"直线离散点：\n"<<straight_line_result<<std::endl;
      anticlockwise = track_ddeg[i] > 0 ? 0 : 1;
      start_rad = slope_rad - M_PI / 2 + anticlockwise * M_PI;
      R0 = track_r[i];
      drad = track_ddeg[i] / 180. * M_PI;
      x0 = straight_line_result(0, straight_line_result.cols() - 1) -
           R0 * std::cos(start_rad);
      y0 = straight_line_result(1, straight_line_result.cols() - 1) -
           R0 * std::sin(start_rad);
      // std::cout<<"圆弧参数：\n"<< start_rad*180/M_PI<<" " << track_ddeg[i]<<"
      // "  <<x0<<" "<<y0<<" "<<R0<<std::endl;
      circle_arc_result = circular_arc(start_rad, drad, n, x0, y0, R0);
      // std::cout<<"圆弧离散点：\n"<<circle_arc_result<<std::endl;
      // std::cout<<"-----------------------------------------"<<std::endl;
      slope_rad = circle_arc_result(0, circle_arc_result.cols() - 1) +
                  M_PI / 2 + anticlockwise * M_PI;
      x0 = circle_arc_result(1, circle_arc_result.cols() - 1);
      y0 = circle_arc_result(2, circle_arc_result.cols() - 1);
      for (int j = 1; j < straight_line_result.cols(); ++j) {
        track_points_x.push_back(straight_line_result(0, j));
        track_points_y.push_back(straight_line_result(1, j));
      }
      for (int j = 1; j < circle_arc_result.cols(); ++j) {
        track_points_x.push_back(circle_arc_result(1, j));
        track_points_y.push_back(circle_arc_result(2, j));
      }
      if (length) {
        s_sum += length;
        s_node_.push_back(s_sum);
        r_node_.push_back(0);
      }
      if (drad) {
        s_sum += R0 * abs(drad);
        s_node_.push_back(s_sum);
        r_node_.push_back(R0);
      }
    }
  }
  std::cout << "里程节点：" << std::endl;
  for (double i : s_node_) {
    std::cout << i << std::endl;
  }
  std::cout << "转弯半径：" << std::endl;
  for (double i : r_node_) {
    std::cout << i << std::endl;
  }
  // if (tip_) {
  //   R_min_=ll_/std::tan(delta_b_-15./180*M_PI)+2;
  //   N_=10;
  //   v_b_=0.2;
  //   vs_b_=v_b_;
  //   track_points_x={};
  //   track_points_y={};
  //   int sgn = tip_>= 0 ? 1 : -1;
  //   double start_rad=tip_>=0?270.0/180*M_PI:90.0/180*M_PI;
  //   Eigen::MatrixXd result = circular_arc(start_rad,tip_,360,0.0,sgn*R_min_,
  //   R_min_); for (int i = 0; i < result.cols(); ++i){
  //     track_points_x.push_back(result(1,i));
  //     track_points_y.push_back(result(2,i));
  //   }
  // }
  // arc cubic spline s_
  s_.setWayPoints(track_points_x, track_points_y);  //生成样条曲线
  // std::cout << s_.arcL_<<std::endl;
  // if (tip_) {
  //   s_.set_arcL((abs(tip_)-2.7/180*M_PI)*R_min_);
  // }

  // mpc matrices
  using namespace kinematic_model;
  state_list_.setZero(n, N_);  //状态空间         状态空间数x预测数
  input_list_.setZero(m, N_);  //控制输入空间      控制输入空间数x预测数
  P_.resize(m * N_, m * N_);  //定义稀疏矩阵大小   控制输入空间数x预测数
  q_.resize(m * N_, 1);  //定义稀疏矩阵大小   控制输入空间数x1
  int constrains_n = 6;  // p, v, a, delta ,ddelta , vs //定义约束数量 p位置
                         // v速度  a加速器  delta转角  ddelta转角速度 vs里程速度
  A_.resize(constrains_n * N_ - 1, m * N_);  //定义稀疏矩阵大小   左乘约束系数A_
                                             //约束数*预测数-1x控制输入数*预测数
  l_.resize(constrains_n * N_ - 1, 1);  //定义约束上界大小   约束数*预测数-1x1
  u_.resize(constrains_n * N_ - 1, 1);  //定义约束上界大小   约束数*预测数-1x1
  // minium delta_u
  P0_.resize(m * N_,
             m * N_);  //首个输入P0_ 控制输入空间*预测数x控制输入空间*预测数
  q0_.resize(m * N_, 1);  //首个输入q0_       控制输入空间*预测数x1
  P0_.setIdentity();      //令P0_为单位矩阵
  for (int i = 1; i < N_; ++i) {
    P0_.coeffRef((i - 1) * m, i * m) = -1;
    P0_.coeffRef(i * m, (i - 1) * m) = -1;
    P0_.coeffRef(1 + (i - 1) * m, i * m + 1) = -1;
    P0_.coeffRef(1 + i * m, (i - 1) * m + 1) = -1;
    P0_.coeffRef(2 + (i - 1) * m, i * m + 2) = -1;
    P0_.coeffRef(2 + i * m, (i - 1) * m + 2) = -1;
    if (i < N_ - 1) {
      P0_.coeffRef(i * m, i * m) = 2;
      P0_.coeffRef(1 + i * m, 1 + i * m) = 2;
      P0_.coeffRef(2 + i * m, 2 + i * m) = 2;
    } else {
      P0_.coeffRef(i * m, i * m) = 1;
      P0_.coeffRef(1 + i * m, 1 + i * m) = 1;
      P0_.coeffRef(2 + i * m, 2 + i * m) = 1;
    }
  }

  P0_ *= 1e-4;  //将稀疏矩阵P0_乘一个系数
  for (int i = 0; i < N_; ++i) {
    q0_.coeffRef(i * m + 2, 0) = -qvs_;  //将稀疏矩阵q0_第i*3+2个赋值为-qvs
  }

  // p, v constrains //声明状态约束矩阵
  Cx_.resize(2 * N_, n * N_);  //状态约束数(2)*预测数x状态数*预测数
  lx_.resize(2 * N_, 1);       //状态约束数(2)*预测数x1
  ux_.resize(2 * N_, 1);       //状态约束数(2)*预测数x1
  // a beta vs constrains //声明控制输入矩阵
  Cu_.resize(m * N_,
             m * N_);  //定义Cu_稀疏矩阵大小 控制输入数*预测数x控制输入数*预测数
  Cu_.setIdentity();   //设为单位矩阵
  lu_.resize(m * N_, 1);  //控制输入数*1
  uu_.resize(m * N_, 1);  //控制输入数*1
  for (int i = 0; i < N_; ++i) {
    lu_.coeffRef(i * m + 0, 0) =
        a_brake_b_;  // lu_  (预测数|)[加速度下边界,首次转角下边界,-0.01]T
    uu_.coeffRef(i * m + 0, 0) =
        a_b_;  // uu_  (预测数|)[加速度上边界,首次转角上边界,里程速度上边界]T
    lu_.coeffRef(i * m + 1, 0) = delta_last_ - ddelta_b_;
    uu_.coeffRef(i * m + 1, 0) = delta_last_ + ddelta_b_;
    lu_.coeffRef(i * m + 2, 0) = 0;
    uu_.coeffRef(i * m + 2, 0) = vs_b_;
    Cx_.coeffRef(i * 2 + 1, i * n + 3) =
        1;  // Cx_  (预测数\)[[0,0,0,0,0],[0,0,0,1,0]]
    lx_.coeffRef(i * 2 + 1, 0) = 0;     // lx   (预测数|)[0,-0.1]T
    ux_.coeffRef(i * 2 + 1, 0) = v_b_;  // ux_  (预测数|)[0,速度上边界]T
  }

  // init state_list
  for (int i = 0; i < N_; ++i) {  //初始化state_list_  (预测数-)[x,y]T
    double s_k = 0;               //第k个s_
    state_list_.coeffRef(4, i) = s_k;         //令第5行全为s_k
    state_list_.block(0, i, 2, 1) = s_(s_k);  //令前两行为样条轨迹初始点x,y
  }
}

Eigen::MatrixXd MpccPlanner::circular_arc(double start_rad, double drad, int n,
                                          double x0, double y0, double R0) {
  double rad_spacing = 2 * M_PI / n;
  n = abs(drad) / rad_spacing + 1;
  Eigen::MatrixXd result(3, n);
  int sgn_anticlockwise = drad > 0 ? 1 : -1;
  int semi;
  for (int i = 0; i < n; i++) {
    double temp_rad = start_rad + sgn_anticlockwise * i * rad_spacing;
    temp_rad = temp_rad >= 2 * M_PI ? temp_rad - 2 * M_PI : temp_rad;
    temp_rad = temp_rad < 0 ? temp_rad + 2 * M_PI : temp_rad;
    result(0, i) = temp_rad;
    result(1, i) = R0 * std::cos(result(0, i)) + x0;
    semi = (result(0, i) >= 0 && result(0, i) <= M_PI) ? 1 : -1;
    result(2, i) =
        semi * std::sqrt(R0 * R0 - (result(1, i) - x0) * (result(1, i) - x0)) +
        y0;
  }
  return result;
}
Eigen::MatrixXd MpccPlanner::straight_line(double length, double slope_rad,
                                           int n, double x0, double y0) {
  double x_spacing = length / n * std::cos(slope_rad);
  double y_spacing = length / n * std::sin(slope_rad);
  Eigen::MatrixXd result(2, n + 1);
  for (int i = 0; i < n + 1; ++i) {
    result(0, i) = x0 + i * x_spacing;
    result(1, i) = y0 + i * y_spacing;
  }
  return result;
}
// void MpccPlanner::plan_callback(
//    const ros::TimerEvent& event) {                  //控制器回调函数
//  kinematic_model::VectorX x0 = state_list_.col(0);  //获取小车当前状态
//  solveQP(x0);  //将小车初始状态输入QP求解器求解
//  visPtr_->publish_traj(state_list_, s_);  //发布初始状态和里程
//}
std::vector<double> MpccPlanner::solveQP(kinematic_model::VectorX x0) {
  std::cout << "小车当前世界坐标:"
            << "(" << x0(0, 0) << "," << x0(1, 0) << ")" << std::endl;
  std::cout << "小车当前车速:" << x0(3, 0) << std::endl;
  std::cout << "小车当前加速度:" << input_list_(0, 0) << std::endl;
  std::cout << "小车当前航向角:" << x0(2, 0) * 180 / M_PI << std::endl;
  std::cout << "小车当前舵轮转角:" << input_list_(1, 0) * 180 / M_PI
            << std::endl;
  std::cout << "小车当前里程:" << x0(4, 0) << std::endl;
  std::cout << "小车总里程:" << s_.arcL() << std::endl;
  // if (x0(4,0)>s_node_[node_index_]) ++node_index_;
  node_index_ = x0(4, 0) > s_node_[node_index_] ? ++node_index_ : node_index_;
  node_index_ =
      node_index_ > s_node_.size() - 1 ? s_node_.size() - 1 : node_index_;
  double current_vs, next_vs;
  current_vs = r_node_[node_index_] == 0
                   ? vs_init_b_
                   : k_rvs_ * (r_node_[node_index_] - 0.3) + 0.2;
  if (node_index_ == s_node_.size() - 1) {
    next_vs = 0;
  } else {
    next_vs = r_node_[node_index_ + 1] == 0
                  ? vs_init_b_
                  : k_rvs_ * (r_node_[node_index_ + 1] - 0.3) + 0.2;
  }
  current_vs = current_vs > vs_init_b_ ? vs_init_b_ : current_vs;
  next_vs = next_vs > vs_init_b_ ? vs_init_b_ : next_vs;
  vs_b_ = current_vs;

  // calculate big state-space matrices
  /* *                BB                AA
   * x1    /       B    0  ... 0 \    /   A \
   * x2    |      AB    B  ... 0 |    |  A2 |
   * x3  = |    A^2B   AB  ... 0 |u + | ... |x0 + gg
   * ...   |     ...  ...  ... 0 |    | ... |
   * xN    \A^(n-1)B  ...  ... B /    \ A^N /
   *
   *     X = BB * U + AA * x0 + gg
   * */
  Eigen::MatrixXd BB, AA, gg;  //声明AA,BB,gg矩阵
  using namespace kinematic_model;
  BB.setZero(
      n * N_,
      m * N_);  //定义BB零矩阵大小  状态空间数*预测数x控制输入空间数*预测数
  AA.setZero(n * N_, n);  //定义AA零矩阵大小  状态空间数*预测数x状态空间数
  gg.setZero(n * N_, 1);  //定义gg零矩阵大小  状态空间数*预测数x1
  MatrixAd Ad = modelPtr_->Ad(
      state_list_.col(0),
      input_list_.col(
          0));  //获取Ad Bd gd矩阵，将当前小车的状态和输入状态传到MPC模型中
  MatrixBd Bd = modelPtr_->Bd(state_list_.col(0), input_list_.col(0));
  Matrixgd gd = modelPtr_->gd(state_list_.col(0), input_list_.col(0));
  BB.block(0, 0, n, m) = Bd;  //将Ad Bd gd首项块矩阵赋值
  AA.block(0, 0, n, n) = Ad;
  gg.block(0, 0, n, 1) = gd;
  for (int i = 1; i < N_; ++i) {  //预测剩下的预测数-1步，并填充BB AA gg矩阵
    Ad = modelPtr_->Ad(state_list_.col(i), input_list_.col(i));
    Bd = modelPtr_->Bd(state_list_.col(i), input_list_.col(i));
    gd = modelPtr_->gd(state_list_.col(i), input_list_.col(i));
    BB.block(n * i, 0, n, m * N_) = Ad * BB.block(n * (i - 1), 0, n, m * N_);
    BB.block(n * i, m * i, n, m) = Bd;
    AA.block(n * i, 0, n, n) = Ad * AA.block(n * (i - 1), 0, n, n);
    gg.block(n * i, 0, n, 1) = Ad * gg.block(n * (i - 1), 0, n, 1) + gd;
  }
  Eigen::SparseMatrix<double> BB_sparse =
      BB.sparseView();  //去除BB AA gg矩阵中的0元素
  Eigen::SparseMatrix<double> AA_sparse = AA.sparseView();
  Eigen::SparseMatrix<double> gg_sparse = gg.sparseView();
  // calculate object function
  Eigen::SparseMatrix<double> Q, q;  //声明稀疏矩阵Q,q
  Q.resize(n * N_, n * N_);  //定义Q矩阵大小 状态空间数*预测数x状态空间数*预测数
  q.resize(n * N_, 1);       //定义q矩阵大小  状态空间数*预测数x1
  double l_dynamic_delta_b;  //声明动态转角下边界
  double u_dynamic_delta_b;  //声明动态转角上边界
  int node;
  for (int i = 0; i < N_; ++i) {
    double s0 = state_list_.coeff(4, i);  // s0获取小车当前状态的里程
    // double s00 = std::fmod(s0+s_.arcL(), s_.arcL());
    // //s00小车跑超过一圈的里程数  s_.arcL()为一圈里程
    double s00 = s0 < s_.arcL() ? s0 : s_.arcL();
    Eigen::Vector2d p = s_(s00);      //小车当前位置[x,y]T
    Eigen::Vector2d dp = s_(s00, 1);  //小车当前速度[vx,vy]T

    double c0 = p(0) - dp(0) * s0;
    double b0 = dp(0);
    double c1 = p(1) - dp(1) * s0;
    double b1 = dp(1);
    // (x~ - x)^2 + (y~ - y)^2
    // Q.coeffRef(i*n + 0, i*n + 0) = 1;
    // Q.coeffRef(i*n + 1, i*n + 1) = 1;
    // Q.coeffRef(i*n + 4, i*n + 4) = 1;//b0*b0+b1*b1;
    // Q.coeffRef(i*n + 0, i*n + 4) = -b0;
    // Q.coeffRef(i*n + 4, i*n + 0) = -b0;
    // Q.coeffRef(i*n + 1, i*n + 4) = -b1;
    // Q.coeffRef(i*n + 4, i*n + 1) = -b1;
    // q.coeffRef(i*n + 0, 0) = -2*c0;
    // q.coeffRef(i*n + 1, 0) = -2*c1;
    // q.coeffRef(i*n + 4, 0) = 2*b0*c0+2*b1*c1;

    // {(x~ - x)x' + (y~ - y)y'} ^2
    double d0 = c0 * b0 + c1 * b1;  // (d0 -b0*x - b1*y + s) ^2
    double ql = 1;
    Q.coeffRef(i * n + 0, i * n + 0) = b0 * b0;
    Q.coeffRef(i * n + 1, i * n + 1) = b1 * b1;
    Q.coeffRef(i * n + 4, i * n + 4) = 1;
    Q.coeffRef(i * n + 0, i * n + 4) = -b0;
    Q.coeffRef(i * n + 4, i * n + 0) = -b0;
    Q.coeffRef(i * n + 1, i * n + 4) = -b1;
    Q.coeffRef(i * n + 4, i * n + 1) = -b1;
    q.coeffRef(i * n + 0, 0) = -2 * b0 * d0;
    q.coeffRef(i * n + 1, 0) = -2 * b1 * d0;
    q.coeffRef(i * n + 4, 0) = 2 * d0;

    //
    double d1 = c0 * b1 - c1 * b0;  // (d1 -b1*x + b0*y) ^2
    double qc = 1;
    Q.coeffRef(i * n + 0, i * n + 0) += b1 * b1 * qc;
    Q.coeffRef(i * n + 1, i * n + 1) += b0 * b0 * qc;
    q.coeffRef(i * n + 0, 0) += -2 * b1 * d1 * qc;
    q.coeffRef(i * n + 1, 0) += 2 * b0 * d1 * qc;
    // Cx constrains
    Cx_.coeffRef(i * 2 + 0, i * n + 0) = dp(1);
    Cx_.coeffRef(i * 2 + 0, i * n + 1) = -dp(0);
    lx_.coeffRef(i * 2 + 0, 0) = -trackWidth_ / 2 + dp(1) * p(0) - dp(0) * p(1);
    ux_.coeffRef(i * 2 + 0, 0) = trackWidth_ / 2 + dp(1) * p(0) - dp(0) * p(1);
    // lx_.coeffRef(i*2 + 0, 0) = -1e6;
    // ux_.coeffRef(i*2 + 0, 0) =  1e6;
    if (s0 >= s_.arcL()) {  //停车约束
      vs_b_ = 0.0;
      ddelta_b_ = 0.0;
    } else if (k_brake_) {  //添加减速约束
      // vs_b_=k_brake_*(s0-s_.arcL())*(N_-1-i)/(N_-1);
      // vs_b_=k_brake_*(x0(4,0)-s_node_[node_index_])+next_vs;
      vs_b_ = k_brake_ * (s0 - s_node_[node_index_]) * (N_ - 1 - i) / (N_ - 1) +
              next_vs;
      vs_b_ = vs_b_ < next_vs ? next_vs : vs_b_;
      vs_b_ = vs_b_ > current_vs ? current_vs : vs_b_;
    } else if (next_vs < current_vs &&
               s_node_[node_index_] - s0 <
                   (x0(3, 0) * x0(3, 0) - next_vs * next_vs) /
                       (2 * abs(a_brake_b_))) {  //减速标志位
      k_brake_ = (next_vs - x0(3, 0)) / (s_node_[node_index_] - s0);
    } else {
      k_brake_ = 0;
    }
    if (vs_b_ > x0(3, 0) + a_b_ * dt_) {  //限制加速不能过快
      std::cout << "当前小车路段的vs_b_实际约束：" << x0(3, 0) + a_b_ * dt_
                << std::endl;
    } else {
      std::cout << "当前小车路段的vs_b_实际约束：" << vs_b_ << std::endl;
    }
    uu_.coeffRef(i * m + 2, 0) =
        vs_b_ > x0(3, 0) + a_b_ * dt_ ? x0(3, 0) + a_b_ * dt_ : vs_b_;
    l_dynamic_delta_b = delta_last_ - ddelta_b_;  //转向角动态约束
    u_dynamic_delta_b = delta_last_ + ddelta_b_;
    if (l_dynamic_delta_b < -delta_b_) {
      l_dynamic_delta_b = -delta_b_;
    } else if (u_dynamic_delta_b > delta_b_) {
      u_dynamic_delta_b = delta_b_;
    }
    lu_.coeffRef(i * m + 1, 0) = l_dynamic_delta_b;
    uu_.coeffRef(i * m + 1, 0) = u_dynamic_delta_b;
  }
  std::cout << "当前小车位置节点：" << node_index_ << std::endl;
  std::cout << "当前小车路段的vs_b_约束：" << current_vs << std::endl;
  std::cout << "下一路段小车vs_b_约束：" << next_vs << std::endl;
  if (next_vs < current_vs &&
      s_node_[node_index_] - x0(4, 0) <=
          (current_vs * current_vs - next_vs * next_vs) / (2 * abs(a_brake_b_)))
    std::cout << "减速状态<<<<<" << std::endl;
  // // calculate constrains
  Eigen::SparseMatrix<double> Cx = Cx_ * BB_sparse;  //上下拼接Cx_和Cu_矩阵
  Eigen::SparseMatrix<double> x0_sp = x0.sparseView();
  Eigen::SparseMatrix<double> gg_sp = gg.sparseView();
  Eigen::SparseMatrix<double> lx = lx_ - Cx_ * AA_sparse * x0_sp - Cx_ * gg_sp;
  Eigen::SparseMatrix<double> ux = ux_ - Cx_ * AA_sparse * x0_sp - Cx_ * gg_sp;
  Eigen::SparseMatrix<double> A_T = A_.transpose();
  A_T.middleCols(0, Cx.rows()) = Cx.transpose();
  A_T.middleCols(Cx.rows(), Cu_.rows()) = Cu_.transpose();
  A_ = A_T.transpose();
  for (int i = 0; i < lx.rows(); ++i) {  //上下拼接lx_和lu_矩阵
    l_.coeffRef(i, 0) = lx.coeff(i, 0);
    u_.coeffRef(i, 0) = ux.coeff(i, 0);
  }
  for (int i = 0; i < lu_.rows(); ++i) {  //上下拼接ux_和uu_矩阵
    l_.coeffRef(i + lx.rows(), 0) = lu_.coeff(i, 0);
    u_.coeffRef(i + lx.rows(), 0) = uu_.coeff(i, 0);
  }
  int start_row =
      Cx_.rows() + Cu_.rows();  //在总约束稀疏矩阵A_下添加ddelta约束矩阵系数矩阵
  for (int i = 0; i < N_ - 1; ++i) {  // ddelta约束
    A_.coeffRef(start_row + i, m * i + 1) = 1;
    A_.coeffRef(start_row + i, m * i + 4) = -1;
    l_.coeffRef(start_row + i, 0) = -ddelta_b_;
    u_.coeffRef(start_row + i, 0) = ddelta_b_;
  }
  // std::cout<< "↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓"<<std::endl;
  // std::cout<< A_<<std::endl;
  // std::cout<< "---------------------------------------------"<<std::endl;
  // std::cout<<l_<<std::endl;
  // std::cout<< "---------------------------------------------"<<std::endl;
  // std::cout<< u_<<std::endl;
  // // std::cout<< "---------------------------------------------"<<std::endl;
  // // std::cout<< b1<<std::endl;
  // std::cout<< "↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑"<<std::endl;
  // A_ = Cu_;
  // l_ = lu_;
  // u_ = uu_;
  // calculate big QP matrices
  Eigen::SparseMatrix<double> BBT_sparse = BB_sparse.transpose();
  P_ = P0_ + BBT_sparse * Q * BB_sparse;
  q_ = q0_ + 2 * BBT_sparse * Q.transpose() * (AA_sparse * x0_sp + gg_sparse) +
       BBT_sparse * q;
  // std::cout << "BB=[\n" << BB << "];" << std::endl;
  // std::cout << "AA=[\n" << AA << "];" << std::endl;
  // std::cout << "gg=[\n" << gg << "];" << std::endl;
  // std::cout << "Q=[\n" << Q.toDense() << "];" << std::endl;
  // std::cout << "Cx_=[\n" << Cx_.toDense() << "];" << std::endl;
  // std::cout << "lx_=[\n" << lx_.toDense() << "];" << std::endl;
  // std::cout << "ux_=[\n" << ux_.toDense() << "];" << std::endl;
  // std::cout << "q'=[\n" << q.toDense() << "];" << std::endl;
  // std::cout << "P=[\n" << P_.toDense() << "];" << std::endl;
  // std::cout << "q=[\n" << q_.toDense() << "];" << std::endl;
  // std::cout << "A=[\n" << A_.toDense() << "];" << std::endl;
  // std::cout << "l=[\n" << l_.toDense() << "];" << std::endl;
  // std::cout << "u=[\n" << u_.toDense() << "];" << std::endl;
  P_ = P_ * 2;
  osqp_.updateMatrices(P_, q_, A_, l_, u_);
  osqp_.solveQP();
  if (osqp_.solveStatus() != 1) {
    assert(false);
  }
  auto solPtr = osqp_.solPtr();
  kinematic_model::VectorX x_last = x0;
  for (int i = 0; i < N_; ++i) {  //预测结果
    input_list_.coeffRef(0, i) = solPtr->x[3 * i + 0];
    input_list_.coeffRef(1, i) = solPtr->x[3 * i + 1];
    input_list_.coeffRef(2, i) = solPtr->x[3 * i + 2];
    state_list_.col(i) = modelPtr_->nextState(x_last, input_list_.col(i));
    x_last = state_list_.col(i);
  }

  delta_last_ = input_list_(1, 0);  //记录最后一次的转角
  // std::cout << "input:\n" << input_list_.transpose() << std::endl;
  // std::cout << "state:\n" << state_list_.transpose() << std::endl;
  // assert(false);
  // update state_list and input_list

  // std::cout << "time: " << (t2-t1).toSec() << std::endl;
  // std::cout << "*************************" << std::endl;
  std::vector<double> output = {};
  output.push_back(state_list_(3, 0));
  output.push_back(input_list_(0, 0));
  output.push_back(input_list_(1, 0));
  output.push_back(state_list_(0, 0));
  output.push_back(state_list_(1, 0));
  output.push_back(state_list_(2, 0));
  output.push_back(state_list_(3, 0));
  output.push_back(state_list_(4, 0));
  return output;
}

}  // namespace mpcc
