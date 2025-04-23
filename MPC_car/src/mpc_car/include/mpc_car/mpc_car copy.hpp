#pragma once
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>

#include <arc_spline/arc_spline.hpp>
#include <deque>
#include <iosqp/iosqp.hpp>

namespace mpc_car {

static constexpr int n = 6;  // state [x y phi v]，状态的维度
static constexpr int m = 2;  // input [a delta]，输入的维度
typedef Eigen::Matrix<double, n, n> MatrixA;//线性化时的A
typedef Eigen::Matrix<double, n, m> MatrixB;//线性化时的B
typedef Eigen::Vector6d VectorG;//线性化时的G
typedef Eigen::Vector6d VectorX;//状态向量
typedef Eigen::Vector2d VectorU;//输入向量

class MpcCar {
 private:
  ros::NodeHandle nh_;
  ros::Publisher ref_pub_, traj_pub_, traj_delay_pub_;

  double ll_; // 车长
  double dt_; // 预测周期的时长
  double rho_; // 过程的权重？
  int N_; // 预测的步长
  double rhoN_; // 最终状态的权重

  double v_max_, a_max_, da_max_, delta_max_, ddelta_max_; // bound约束
  double delay_;//延迟

  bool path_direction_ = 1;//1表示前进

  arc_spline::ArcSpline s_;//参考轨迹
  double desired_v_;//期待的速度？预估的一个速度？？

  osqp::IOSQP qpSolver_;

  std::vector<VectorX> predictState_;//预测出来的state
  std::vector<VectorU> predictInput_;//预测出来的input
  std::deque<VectorU> historyInput_;//先前的input，用于有delay的场景
  int history_length_;//由于delay造成的延迟的预测周期数
  VectorX x0_observe_;//预测时的状态，由car_simulator直接龙格库塔积分得到
  std::vector<VectorX> compensateDelayX0_;
  bool useCompensate1 = 0, useCompensate2 = 0;
  // 线性化时用到的几个矩阵
  // x_{k+1} = Ad * x_{k} + Bd * u_k + gd
  MatrixA Ad_;//4*4
  MatrixB Bd_;//4*2
  VectorG gd_;//4*1

  /**整合为这样的形式，就可以直接放到求解器里面计算了？
   * OSQP 求解器接口:
   * minimize J     0.5 x^T P_ x + q_^T x ( P_ 和 q_ 是 cost 的权重矩阵)
   * subject to   l_ <= A_ * x <= u_ (A_，l_，u_ 是整合之后的约束矩阵)
   **/
  // 上面公式里面的x是指要优化的量，是输入u吧？？
  Eigen::SparseMatrix<double> P_, q_, A_, l_, u_;

  /* *
   *               /  x1  \
   *               |  x2  |
   *  lx_ <=  Cx_  |  x3  |  <= ux_
   *               | ...  |
   *               \  xN  /
   * */
  //和state相关的约束
  Eigen::SparseMatrix<double> Cx_, lx_, ux_;  // p, v constrains
  /* *
   *               /  u0  \
   *               |  u1  |
   *  lu_ <=  Cu_  |  u2  |  <= uu_
   *               | ...  |
   *               \ uN-1 /
   * */
  //和input相关的约束
  Eigen::SparseMatrix<double> Cu_, lu_, uu_;  // [a delta] constrains
  Eigen::SparseMatrix<double> Qx_; // 这个变量是 Q 变量 状态跟踪权重 (6x6)
  Eigen::SparseMatrix<double> S_;   // 输入增量权重 (2x2)

  // 参考轨迹容器（每个时刻的扩展状态 [x_ref, y_ref, v_ref, theta_ref, 0, 0]）
  std::vector<Eigen::VectorXd> X_ref_; 

  // /* 在某点处线性化 */
  // void linearization(const double& phi,
  //                    const double& v,
  //                    const double& delta) 
  // {
  //   // x_{k+1} = Ad * x_{k} + Bd * u_k + gd
  //   /* TODO: 对 Ad_(A_k), Bd_(B_k), gd_(g_k) 矩阵进行赋值 */
  //   // 矩阵初始化
  //   Ad_.setIdentity();
  //   Bd_.setZero();
  //   gd_.setZero();

  //   // 对 Ad_(A_k) 矩阵进行赋值，参考 A_c 状态矩阵的公式
  //   MatrixA Ac_; // 中间矩阵，最后赋值回给 Ad_ 矩阵
  //   Ac_.setZero();
  //   Ac_.coeffRef(0, 2) = -1 * v * sin(phi);
  //   Ac_.coeffRef(0, 3) = cos(phi);
  //   Ac_.coeffRef(1, 2) = v * cos(phi);
  //   Ac_.coeffRef(1, 3) = sin(phi);
  //   Ac_.coeffRef(2, 3) = tan(delta)/ll_;
  //   Ad_ += Ac_ * dt_; // 参考 A_k = I + T_s * A_c

  //   // 对 Bd_(B_k) 矩阵进行赋值，参考 B_c 矩阵的公式
  //   Bd_.coeffRef(3, 0) = 1;
  //   Bd_.coeffRef(2, 1) = v/(ll_ * cos(delta) * cos(delta));
  //   Bd_ *= dt_; // 参考 B_k = T_s * B_c

  //   // 对 gd_(g_k) 进行赋值，参考 g_c 矩阵的公式
  //   gd_(0) = v*phi*sin(phi);
  //   gd_(1) = -1*v*phi*cos(phi);
  //   gd_(2) = -1*v*delta/(ll_ * cos(delta) * cos(delta));
  //   gd_(3) = 0;
  //   gd_ *= dt_; // g_k = T_s * g_c

  //   return;
  // }

  /* 在某点处线性化（适配输入增量控制） */
  void linearization(const double& phi,
                    const double& v,
                    const double& delta) 
  {
    // 扩展后的状态维度：4（原状态） + 2（历史输入 a_{k-1}, δ_{k-1}） = 6
    const int extended_dim = 6; 
    const int control_dim = 2;

    // 重置矩阵维度
    Ad_.resize(extended_dim, extended_dim);
    Bd_.resize(extended_dim, control_dim);
    gd_.resize(extended_dim);

    Ad_.setZero();
    Bd_.setZero();
    gd_.setZero(); // 增量控制中 g_k 被省略

    /*-----------------------------------------------
    1. 计算原系统离散化后的 A_k 和 B_k
    ------------------------------------------------*/
    // 连续时间雅可比矩阵 A_c
    Eigen::Matrix4d A_continuous;
    A_continuous.setZero();
    A_continuous(0, 2) = -v * sin(phi);    // ∂ẋ/∂θ = -v*sin(phi)
    A_continuous(0, 3) = cos(phi);         // ∂ẋ/∂v = cos(phi)
    A_continuous(1, 2) = v * cos(phi);     // ∂ẏ/∂θ = v*cos(phi)
    A_continuous(1, 3) = sin(phi);         // ∂ẏ/∂v = sin(phi)
    A_continuous(2, 3) = tan(delta)/ll_;   // ∂θ̇/∂v = tan(δ)/L

    // 离散化后的 A_k = I + A_c * Δt
    Eigen::Matrix4d A_discrete = Eigen::Matrix4d::Identity() + A_continuous * dt_;

    // 连续时间输入矩阵 B_c
    Eigen::Matrix<double, 4, 2> B_continuous;
    B_continuous.setZero();
    B_continuous(3, 0) = 1.0;  // ∂v̇/∂a = 1（加速度输入）
    B_continuous(2, 1) = v / (ll_ * pow(cos(delta), 2));  // ∂θ̇/∂δ = v/(L*cos²δ)

    // 离散化后的 B_k = B_c * Δt
    Eigen::Matrix<double, 4, 2> B_discrete = B_continuous * dt_;

    /*-----------------------------------------------
    2. 构建扩展后的增量控制模型矩阵
      Ad_ = [[A_k, B_k], 
            [ 0,   I]]
      Bd_ = [[B_k],
            [  I]]
    ------------------------------------------------*/
    // 填充 Ad_
    Ad_.block(0, 0, 4, 4) = A_discrete;        // 左上块：原状态动态
    Ad_.block(0, 4, 4, 2) = B_discrete;        // 右上块：历史输入的影响
    Ad_.block(4, 4, 2, 2) = Eigen::Matrix2d::Identity(); // 右下块：历史输入的保持

    // 填充 Bd_
    Bd_.block(0, 0, 4, 2) = B_discrete;        // 上部：原输入通道
    Bd_.block(4, 0, 2, 2) = Eigen::Matrix2d::Identity(); // 下部：增量输入的积分

    return;
  }

  /* 在轨迹 s_ 上，求某一点 (x, y) 处的状态和输入 (phi, v, delta) */
  void calLinPoint(const double& s0, double& phi, double& v, double& delta) 
  {
    // std::cout << "所期望的x, y:" << s_(s0, 0) << std::endl;
    Eigen::Vector2d dxy = s_(s0, 1);
    Eigen::Vector2d ddxy = s_(s0, 2);
    double dx = dxy.x();
    double dy = dxy.y();
    double ddx = ddxy.x();
    double ddy = ddxy.y();
    double dphi = (ddy * dx - dy * ddx) / (dx * dx + dy * dy);//简化的曲率公式
    if (path_direction_ == 1) 
      phi = atan2(dy, dx);
    else
      phi = atan2(dy, dx) - M_PI; /* 在这里实现倒车 */
    
    if (s0 >= s_.arcL()) // 如果进度已经完成，就将车停下
    {
      v = 0.0;
      delta = 0.0;
    }
    else
    {
      // std::cout << "s_.arcL() - s0 = " << s_.arcL() - s0 << std::endl;
      // std::cout << "s_.arcL() - s0 = " << s_.arcL() - s0 << std::endl;
      // std::cout << "s_.arcL() - s0 = " << s_.arcL() - s0 << std::endl;
      // std::cout << "s_.arcL() - s0 = " << s_.arcL() - s0 << std::endl;
      // std::cout << "s_.arcL() - s0 = " << s_.arcL() - s0 << std::endl;
      // if (s_.arcL() - s0 > 1.0)
      // {
        v = desired_v_;
        delta = atan2(ll_ * dphi / (v), 1.0);// ERROR?这里不是应该再除以一个v吗？
        // if(path_direction_ == 0) delta *= -1; //倒车
        // delta = atan2(ll_ * dphi , 1.0);// ERROR?这里不是应该再除以一个v吗？
      // }
      // else
      // {
      //   v = (s_.arcL() - s0) / 1.0 * desired_v_;
      //   delta = atan2(ll_ * dphi / (v) , 1.0);
      // }
      
    }
  }

  inline VectorX diff(const VectorX& state,
                      const VectorU& input) const {
    VectorX ds;
    double phi = state(2);
    double v = state(3);
    double a = input(0);
    double delta = input(1);
    ds(0) = v * cos(phi);
    ds(1) = v * sin(phi);
    ds(2) = v / ll_ * tan(delta);
    ds(3) = a;
    return ds;
  }

  inline void step(VectorX& state, const VectorU& input, const double dt) const {
    // Runge–Kutta
    VectorX k1 = diff(state, input);
    VectorX k2 = diff(state + k1 * dt / 2, input);
    VectorX k3 = diff(state + k2 * dt / 2, input);
    VectorX k4 = diff(state + k3 * dt, input);
    state = state + (k1 + k2 * 2 + k3 * 2 + k4) * dt / 6;
  }
  VectorX compensateDelay0(const VectorX& x0) {
    VectorX x0_delay = x0;
    return x0_delay;
  }

  VectorX compensateDelay1(const VectorX& x0) {
    useCompensate1 = 1;
    VectorX x0_delay = x0;
    double dt = 0.001;
    for (double t = delay_; t > 0; t -= dt) {
      int i = std::ceil(t / dt_);
      VectorU input = historyInput_[history_length_ - i];
      step(x0_delay, input, dt);
    }
    return x0_delay;
  }

  VectorX compensateDelay2(const VectorX &x0){
    useCompensate2 = 1;
    compensateDelayX0_.clear();
    compensateDelayX0_.push_back(x0);
    VectorX x0_delay = x0;
    double s0 = s_.findS(x0_delay.head(2));
    double phi, v, delta;
    double last_phi = x0(2);
    for (int i = 0; i < history_length_; ++i) {
      calLinPoint(s0, phi, v, delta);//
      if (phi - last_phi > M_PI) {
        phi -= 2 * M_PI;
      } else if (phi - last_phi < -M_PI) {
        phi += 2 * M_PI;
      }
      last_phi = phi;
      linearization(phi, v, delta);
      x0_delay = Ad_ * x0_delay + Bd_ * historyInput_[i] + gd_;//这里要构造A
      compensateDelayX0_.push_back(x0_delay);
      s0 += abs(x0_delay[3]) * dt_;
      s0 = s0 < s_.arcL() ? s0 : s_.arcL();
    }
    return x0_delay;
  }

 public:
  MpcCar(ros::NodeHandle& nh) : nh_(nh) {
    // load map
    std::vector<double> track_points_x, track_points_y;
    nh.getParam("track_points_x", track_points_x);
    nh.getParam("track_points_y", track_points_y);
    nh.getParam("desired_v", desired_v_);
    s_.setWayPoints(track_points_x, track_points_y);
    // load parameters
    nh.getParam("ll", ll_);
    nh.getParam("dt", dt_);
    nh.getParam("rho", rho_);
    nh.getParam("N", N_);
    nh.getParam("rhoN", rhoN_);
    nh.getParam("v_max", v_max_);
    nh.getParam("a_max", a_max_);
    nh.getParam("delta_max", delta_max_);
    nh.getParam("ddelta_max", ddelta_max_);
    nh.getParam("delay", delay_);
    history_length_ = std::ceil(delay_ / dt_);//向上取整

    ref_pub_ = nh.advertise<nav_msgs::Path>("path_ref", 1);
    traj_pub_ = nh.advertise<nav_msgs::Path>("traj_mpc", 1);
    traj_delay_pub_ = nh.advertise<nav_msgs::Path>("traj_mpc_delay", 1);

    /* TODO: set initial value of Ad, Bd, gd */
    Ad_.setIdentity();
    Bd_.setZero();
    gd_.setZero();

    
    // set size of sparse matrices
    P_.resize(m * N_, m * N_);
    q_.resize(m * N_, 1);
    

    /*---------------------*/
    /* 设置状态跟踪权重 Qx_ */
    /*--------------------*/
    // stage cost
    // every        1 0 0   0  
    //        小Q = 0 1 0   0
    //              0 0 rho 0
    //              0 0 0   0
    //那就是看你cost function怎么设计了，前面的两个1是x和y的位置误差，rho是对phi，对方向的跟踪？对v没有跟踪
    Qx_.resize(n * N_, n * N_);
    for (int i = 0; i < N_; ++i) 
    {
      Qx_.coeffRef(6*i + 0, 6*i + 0) = 10.0; // x 权重
      Qx_.coeffRef(6*i + 1, 6*i + 1) = 10.0; // y 权重
      Qx_.coeffRef(6*i + 2, 6*i + 2) = 1.0;  // v 权重
      Qx_.coeffRef(6*i + 3, 6*i + 3) = 5.0;  // theta 权重
      // a_prev 和 delta_prev 的跟踪权重设为0（不跟踪历史输入）
    }
    /*------------------*/
    /*------------------*/
    /*------------------*/
    
    /*-----------------
    设置输入增量权重 S_ 
    -------------------*/
    S_.resize(2 * N_, 2 * N_);
    for (int i = 0; i < N_; ++i) 
    {
      S_.coeffRef(2*i + 0, 2*i + 0) = 0.1; // Δa 权重
      S_.coeffRef(2*i + 1, 2*i + 1) = 0.1; // Δδ 权重
    }
    /*------------------*/
    /*------------------*/
    /*------------------*/

    int n_cons = 5;  // [v a da delta ddelta]，约束的维度。对每一个预测周期都需要约束
    // 控制约束矩阵的维度
    A_.resize(n_cons * N_, m * N_); // 32O, 160
    l_.resize(n_cons * N_, 1);
    u_.resize(n_cons * N_, 1);
    // [v] constrains
    Cx_.resize(1 * N_, n * N_);
    lx_.resize(1 * N_, 1);
    ux_.resize(1 * N_, 1);
    // [a delta ddelta] constrains
    Cu_.resize(3 * N_, m * N_);
    lu_.resize(3 * N_, 1);
    uu_.resize(3 * N_, 1);
    // set lower and upper boundaries
    for (int i = 0; i < N_; ++i) 
    {
      // TODO: set stage constraints of inputs (a, delta, ddelta)
      // -a_max <= a <= a_max for instance:
      Cu_.coeffRef(i * 3 + 0, i * m + 0) = 1;
      lu_.coeffRef(i * 3 + 0, 0) = -a_max_;
      uu_.coeffRef(i * 3 + 0, 0) = a_max_;
      // ...
      Cu_.coeffRef(i * 3 + 1, i * m + 1) = 1;
      lu_.coeffRef(i * 3 + 1, 0) = -delta_max_;
      uu_.coeffRef(i * 3 + 1, 0) = delta_max_;

      // TODO: 这里可能还有问题
      if(i > 0)
      {
        Cu_.coeffRef(i * 3 + 2, (i-1) * m + 1) = -1/dt_;
        Cu_.coeffRef(i * 3 + 2, i * m + 1) = 1/dt_;
        lu_.coeffRef(i * 3 + 2, 0) = -ddelta_max_;
        uu_.coeffRef(i * 3 + 2, 0) = ddelta_max_;        
      }
      else
      {
        Cu_.coeffRef(i * 3 + 2, i * m + 1) = 1;
      }
      // if(i > 0) Cu_.coeffRef(i * 3 + 2, (i-1) * m + 1) = -1;
      // Cu_.coeffRef(i * 3 + 2, i * m + 1) = 1;
      // lu_.coeffRef(i * 3 + 2, 0) = -ddelta_max_ * dt_;
      // uu_.coeffRef(i * 3 + 2, 0) = ddelta_max_ * dt_;  


      // TODO: set stage constraints of states (v)
      // -v_max <= v <= v_max
      Cx_.coeffRef(i, i * n + 3) = 1;
      lx_.coeffRef(i, 0) = -v_max_; 
      ux_.coeffRef(i, 0) = v_max_;
    }

    for (int i = 0; i < N_; ++i) {
      // 速度约束：v_min ≤ v_k ≤ v_max
      // 在扩展状态中，v 是第3个元素（索引2）
      Cx_.coeffRef(i, n_extended * i + 2) = 1.0; 
      lx_(i) = v_min_;
      ux_(i) = v_max_;
    }

    // set predict mats size
    predictState_.resize(N_);
    predictInput_.resize(N_);
    for (int i = 0; i < N_; ++i) {
      predictInput_[i].setZero();
    }
    for (int i = 0; i < history_length_; ++i) {
      historyInput_.emplace_back(0, 0);
    }
  }

  bool check_goal(const VectorX& x0)
  {
    double x, y, yaw, v_norm;
    x = x0(0); y = x0(1); yaw = x0(2); v_norm = x0(3);
    Eigen::Vector2d dxy = s_(s_.arcL(), 1);
    double phi = atan2(dxy.y(), dxy.x());
    if (path_direction_ == 0) 
      phi -= M_PI;

    /* 角度归一化 */
    if (phi - yaw > M_PI)
      phi -= 2 * M_PI;
    else if (phi - yaw < -M_PI)
      phi += 2 * M_PI;
    
    std::cout << "4" << std::endl;

    Eigen::Vector2d gxy = s_(s_.arcL(), 0); // 提取路径上的目标点位置
    double dx = gxy.x() - x;
    double dy = gxy.y() - y;
    std::cout << "dxy: " <<  (dx * dx + dy * dy) << ", v: " << std::abs(v_norm);
    std::cout << ", std::abs(phi - x0(2)): " << std::abs(phi - yaw) << std::endl;
    /* TODO: 在这里设置目标完成情况的参数的阈值 */
    if ((dx * dx + dy * dy) < 0.3 && 
        std::abs(v_norm) < 0.01 && 
        std::abs(phi - yaw) < 0.1) 
      return true;

    return false;
  }

  /* 只在类外调用 */
  int solveQP(const VectorX& x0_observe) 
  {
    x0_observe_ = x0_observe;
    if (!historyInput_.empty()) 
      historyInput_.pop_front();

//    historyInput_.pop_front();
    if (!predictInput_.empty()) 
      historyInput_.push_back(predictInput_.front());

    //historyInput_.push_back(predictInput_.front());
    //这是XX的约束，由预测出来的delta，结合ddelta_max来确定delta的上下界？
    lu_.coeffRef(2, 0) = predictInput_.front()(1) - ddelta_max_ * dt_;
    uu_.coeffRef(2, 0) = predictInput_.front()(1) + ddelta_max_ * dt_;
    // VectorX x0 = compensateDelay2(x0_observe_); // 当前应该的状态。如果是由延迟的情况下，那么x0_observe_是存在延迟的状态
    VectorX x0 = x0_observe_;
    if (check_goal(x0))
    {
      std::cout << "------------!!!!到达seg的终点!!!!-----------" << std::endl;
      std::cout << "到达终点时的状态为：" << x0.transpose() << std::endl;

      return 11;//表示已经到达终点了
    }

    X_ref_.clear();

    // set BB, AA, gg
    Eigen::MatrixXd BB, AA, gg;
    BB.setZero(n * N_, m * N_);
    AA.setZero(n * N_, n);
    gg.setZero(n * N_, 1);
    double s0 = s_.findS(x0.head(2));//这个是在整体路径上的进度，是一个长度值，不是百分比
    double phi, v, delta;
    double last_phi = x0(2);
    Eigen::SparseMatrix<double> qx; // q_x = -Q^T * X_ref
    qx.resize(n * N_, 1);
    for (int i = 0; i < N_; ++i) 
    {
      
      calLinPoint(s0, phi, v, delta); // 在 calLinPoint() 内改变的 phi, v, delta 的值
      if (phi - last_phi > M_PI) 
      {
        phi -= 2 * M_PI;
      } 
      else if (phi - last_phi < -M_PI) 
      {
        phi += 2 * M_PI;
      }
      last_phi = phi;

      /* 构建参考轨迹 X_ref_ */
      Eigen::VectorXd z_ref(6);
      Eigen::Vector2d xy_ref = s_(s0);
      z_ref << xy_ref(0), xy_ref(1), phi, v, 0.0, 0.0;
      X_ref_.push_back(z_ref);

      /* 线性化 */
      linearization(phi, v, delta);//这些量都是当前状态所处轨迹上的点

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
      /* 设置 BB AA gg 矩阵 */
      if (i == 0) 
      {
        BB.block(0, 0, n, m) = Bd_;
        AA.block(0, 0, n, n) = Ad_;
        gg.block(0, 0, n, 1) = gd_;
      } 
      else 
      {
        BB.block(n*i, m*i, n, m) = Bd_;
        for(int j = i - 1; j >= 0; --j)
        {
          BB.block(n * i, m * j, n, m) = Ad_ * BB.block(n*(i-1), m*j, n, m);
        }
        AA.block(n * i, 0, n, n) = Ad_ * AA.block(n*(i-1), 0, n, n);
        gg.block(n * i, 0, n, 1) = Ad_ * gg.block(n*(i-1), 0, n, 1) + gd_;
      }
      
      // TODO: 设置 qx 矩阵
      // Eigen::Vector2d xy = s_(s0);  // reference (x_r, y_r)

      // cost function should be represented as follows:
      /* *
       *           /  x1  \T       /  x1  \         /  x1  \
       *           |  x2  |        |  x2  |         |  x2  |
       *  J =  0.5 |  x3  |   Qx_  |  x3  | + qx^T  |  x3  | + const.
       *           | ...  |        | ...  |         | ...  |
       *           \  xN  /        \  xN  /         \  xN  /
       * */

      /* 按以下公式设置 qx 矩阵：
            参考公式为 q_x = -Q^T * x_ref
            qx = -Qx_.toDense().block(n * i, n * i, 4, 4) transpose() * VectorX(xy(0), xy(1), phi, 0); */
      // 每次循环设置 qx 的一个子矩阵
      // qx.coeffRef(n*i + 0, 0) = -Qx_.coeffRef(n * i + 0, n * i + 0) * xy(0);
      // qx.coeffRef(n*i + 1, 0) = -Qx_.coeffRef(n * i + 1, n * i + 1) * xy(1);
      // qx.coeffRef(n*i + 2, 0) = -Qx_.coeffRef(n * i + 2, n * i + 2) * phi;
      // qx.coeffRef(n*i + 3, 0) = -Qx_.coeffRef(n * i + 3, n * i + 3) * v;

      s0 += std::abs(desired_v_) * dt_; // 全部循环过后，得到期望进度
      s0 = s0 < s_.arcL() ? s0 : s_.arcL();
      if (i == 0)
        std::cout << "desired: " << xy.transpose() << ", " << phi << ", " << v << std::endl;
    }
    // .sparseView()方法：这个方法的作用是将一个已经存在的矩阵（无论是密集矩阵还是已经是稀疏矩阵）
      // 转换成一个稀疏矩阵视图（Eigen::SparseMatrix<double>类型）
    Eigen::SparseMatrix<double> BB_sparse = BB.sparseView();
    Eigen::SparseMatrix<double> AA_sparse = AA.sparseView();
    Eigen::SparseMatrix<double> gg_sparse = gg.sparseView();
    Eigen::SparseMatrix<double> x0_sparse = x0.sparseView();

    // state constrants propogate to input constraints using "X = BB * U + AA * x0 + gg"
    // 用状态约束“X = BB * U + AA * x0 + gg”表示输入约束
    /* *
     *               /  x1  \                              /  u0  \
     *               |  x2  |                              |  u1  |
     *  lx_ <=  Cx_  |  x3  |  <= ux_    ==>    lx <=  Cx  |  u2  |  <= ux
     *               | ...  |                              | ...  |
     *               \  xN  /                              \ uN-1 /
     * */
    /* 设置约束矩阵 */
    Eigen::SparseMatrix<double> Cx = Cx_ * BB_sparse; // N_*mN_
    Eigen::SparseMatrix<double> lx = lx_ - Cx_ * AA_sparse * x0_sparse - Cx_ * gg_sparse;//N_*1
    Eigen::SparseMatrix<double> ux = ux_ - Cx_ * AA_sparse * x0_sparse - Cx_ * gg_sparse;//N_*1

    /* *      / Cx  \       / lx  \       / ux  \
     *   A_ = \ Cu_ /, l_ = \ lu_ /, u_ = \ uu_ /
     * */
    
    //这里为什么要先转置一遍呀
    //A_的维度为4N_*mN_，所以优化变量的维度是mN_*1，还是控制量，
    //不是PPT24页里面把状态量和控制量放在一起优化，而是把对状态量的约束转换为了对控制量的约束
    Eigen::SparseMatrix<double> A_T = A_.transpose();
    A_T.middleCols(0, Cx.rows()) = Cx.transpose();
    A_T.middleCols(Cx.rows(), Cu_.rows()) = Cu_.transpose();
    A_ = A_T.transpose();
    // 也是拼接，只不过是换了一种计算形式
    for (int i = 0; i < lx.rows(); ++i) 
    {
      l_.coeffRef(i, 0) = lx.coeff(i, 0);
      u_.coeffRef(i, 0) = ux.coeff(i, 0);
    }
    for (int i = 0; i < lu_.rows(); ++i) 
    {
      l_.coeffRef(i + lx.rows(), 0) = lu_.coeff(i, 0);
      u_.coeffRef(i + lx.rows(), 0) = uu_.coeff(i, 0);
    }
    Eigen::SparseMatrix<double> BBT_sparse = BB_sparse.transpose();
    P_ = BBT_sparse * Qx_ * BB_sparse; // 参考 H = BB^T * Q * BB
    q_ = BBT_sparse * Qx_.transpose() * (AA_sparse * x0_sparse + gg_sparse) + BBT_sparse * qx; // 参考 f = BB^T * Q^T * (AA * x0 + gg) + BB^T * q_x

    /* 使用 OSQP 进行求解 */
    Eigen::VectorXd q_d = q_.toDense();
    Eigen::VectorXd l_d = l_.toDense();
    Eigen::VectorXd u_d = u_.toDense();
    qpSolver_.setMats(P_, q_d, A_, l_d, u_d);
    qpSolver_.solve();
    int ret = qpSolver_.getStatus();
    if (ret != 1) 
    {
      ROS_ERROR("fail to solve QP!");
      std::cout << "ret = " << ret << std::endl;
      return ret;
    }
    //将很长的向量映射为矩阵
    //qpSolver_求解的只是input u，而状态x是计算出来的
    Eigen::VectorXd sol = qpSolver_.getPrimalSol(); // 求解得到 U
    Eigen::MatrixXd solMat = Eigen::Map<const Eigen::MatrixXd>(sol.data(), m, N_);
    Eigen::VectorXd solState = BB * sol + AA * x0 + gg;
    Eigen::MatrixXd predictMat = Eigen::Map<const Eigen::MatrixXd>(solState.data(), n, N_);
    //然后将矩阵放到vector中去
    for (int i = 0; i < N_; ++i) 
    {
      predictInput_[i] = solMat.col(i);
      // std::cout << "predictInput_[i] = " << predictInput_[i] << std::endl;
      predictState_[i] = predictMat.col(i);
      // std::cout << "predictState_[i] = " << predictState_[i] << std::endl;
    }

    return ret;
  }

  int solveQP(const VectorX& x0_observe) 
  {
    x0_observe_ = x0_observe;
    
    /*-----------------------------------------------
    1. 增量控制输入处理
    ------------------------------------------------*/
    // 不再直接操作 predictInput_，而是通过历史输入和增量更新
    if (!historyInput_.empty()) 
      historyInput_.pop_front();
    
    // 若预测输入非空，记录上一时刻的实际输入
    if (!predictInput_.empty()) 
    {
      Eigen::Vector2d last_u = historyInput_.back();
      last_u += predictInput_.front(); // u_{k} = u_{k-1} + Δu_k
      historyInput_.push_back(last_u);
    }

    /*-----------------------------------------------
    2. 输入约束调整（基于增量Δu） TODO:
    ------------------------------------------------*/
    // 控制输入增量约束 Δa 和 Δδ 的上下界
    lu_.coeffRef(0, 0) = -da_max_ * dt_;   // Δa_min
    uu_.coeffRef(0, 0) = da_max_ * dt_;    // Δa_max
    lu_.coeffRef(1, 0) = -ddelta_max_ * dt_; // Δδ_min
    uu_.coeffRef(1, 0) = ddelta_max_ * dt_;  // Δδ_max

    /*-----------------------------------------------
    - 检查是否到终点
    ------------------------------------------------*/
    VectorX x0 = x0_observe_;
    if (check_goal(x0))
    {
      std::cout << "------------!!!!到达seg的终点!!!!-----------" << std::endl;
      std::cout << "到达终点时的状态为：" << x0.transpose() << std::endl;

      return 11;//表示已经到达终点了
    }

    /*-----------------------------------------------
    3. 状态预测矩阵构建（适配扩展状态）
    ------------------------------------------------*/
    const int n_extended = 6; // 扩展状态维度：6（原状态4 + 历史输入2）
    const int m = 2; // 控制输入维度 Δa, Δδ
    
    // 初始化矩阵维度
    Eigen::MatrixXd BB, AA, gg;
    BB.setZero(n_extended * N_, m * N_);
    AA.setZero(n_extended * N_, n_extended);
    gg.setZero(n_extended * N_, 1);

    double s0 = s_.findS(x0.head(2)); // 这个是在整体路径上的进度，是一个长度值，不是百分比
    double phi, v, delta;
    double last_phi = x0(2);

    // 获取当前扩展状态（包含历史输入）
    VectorX x0_extended(n_extended);
    x0_extended << x0_observe_, historyInput_.front(); // [x0; a_prev; δ_prev]

    /*-----------------------------------------------
    4. 构建增量控制的预测模型
    ------------------------------------------------*/
    for (int i = 0; i < N_; ++i) 
    {
      // 线性化点计算
      calLinPoint(s0, phi, v, delta); // 在 calLinPoint() 内改变的 phi, v, delta 的值
      if (phi - last_phi > M_PI) 
      {
        phi -= 2 * M_PI;
      } 
      else if (phi - last_phi < -M_PI) 
      {
        phi += 2 * M_PI;
      }
      last_phi = phi;

      /* 构建参考轨迹 X_ref_ */
      Eigen::VectorXd z_ref(6);
      Eigen::Vector2d xy_ref = s_(s0);
      z_ref << xy_ref(0), xy_ref(1), phi, v, 0.0, 0.0;
      X_ref_.push_back(z_ref);

      // 计算扩展后的 Ad_, Bd_（维度6x6和6x2）
      linearization(phi, v, delta);

      // 填充 BB 和 AA 矩阵
      if (i == 0) 
      {
        BB.block(0, 0, n_extended, m) = Bd_;
        AA.block(0, 0, n_extended, n_extended) = Ad_;
        gg.block(0, 0, n_extended, 1) = gd_;
      } 
      else 
      {
        // 递归计算 BB 的块
        BB.block(n_extended*i, m*i, n_extended, m) = Bd_;
        for (int j = i-1; j >= 0; --j) 
        {
          BB.block(n_extended*i, m*j, n_extended, m) = 
            Ad_ * BB.block(n_extended*(i-1), m*j, n_extended, m);
        }
        // 递归计算 AA 的块
        AA.block(n_extended*i, 0, n_extended, n_extended) = 
          Ad_ * AA.block(n_extended*(i-1), 0, n_extended, n_extended);
        gg.block(n_extended * i, 0, n_extended, 1) = Ad_ * gg.block(n_extended*(i-1), 0, n_extended, 1) + gd_;
      }
    }

    /*-----------------------------------------------
    5. 二次规划问题构建（适配增量Δu）
    ------------------------------------------------*/
    // 状态方程：X = BB * ΔU + AA * x0_extended
    Eigen::SparseMatrix<double> BB_sparse = BB.sparseView();
    Eigen::SparseMatrix<double> AA_sparse = AA.sparseView();
    Eigen::SparseMatrix<double> gg_sparse = gg.sparseView();
    
    // 目标函数：J = 0.5 * ΔU^T H ΔU + f^T ΔU
    Eigen::SparseMatrix<double> H = BB_sparse.transpose() * Qx_ * BB_sparse + S_; // S_ 为 Δu 的权重矩阵 TODO:
    Eigen::VectorXd f = BB_sparse.transpose() * Qx_ * (AA_sparse * x0_extended - X_ref_)
                        +BB_sparse.transpose() * Qx_ * gg_sparse; // TODO:

    // 输入幅值约束（通过历史输入和增量计算）
    Eigen::VectorXd u_min = ...; // 根据 a_prev + ΣΔa 和 δ_prev + ΣΔδ 计算
    Eigen::VectorXd u_max = ...;
    
    // 转换为 Δu 的约束
    Eigen::VectorXd delta_u_min = ...;
    Eigen::VectorXd delta_u_max = ...;

    // 更新 QP 求解器参数
    qpSolver_.setMats(H, f, A_constraints_, delta_u_min, delta_u_max);
    qpSolver_.solve();

    /*-----------------------------------------------
    6. 结果解析与输入更新
    ------------------------------------------------*/
    Eigen::VectorXd delta_u_opt = qpSolver_.getPrimalSol();
    Eigen::MatrixXd delta_u_sequence = Eigen::Map<const Eigen::MatrixXd>(delta_u_opt.data(), m, N_);

    // 预测输入为增量序列
    predictInput_.clear();
    for (int i = 0; i < N_; ++i) 
    {
      predictInput_.push_back(delta_u_sequence.col(i));
    }

    return qpSolver_.getStatus();
  }

  void getPredictXU(double t, VectorX& state, VectorU& input) 
  {
    if (t <= dt_) // dt_ 为 MPC 预测周期的时长
    {
      state = predictState_.front();
      input = predictInput_.front();
      return;
    }

    //输入的t一直都是0，那么下面这一段是用来干什么的呢?
    int horizon = std::floor(t / dt_);
    double dt = t - horizon * dt_;
    state = predictState_[horizon - 1];
    input = predictInput_[horizon - 1];
    double phi = state(2);
    double v = state(3);
    double a = input(0);
    double delta = input(1);
    state(0) += dt * v * cos(phi);
    state(1) += dt * v * sin(phi);
    state(2) += dt * v / ll_ * tan(delta);
    state(3) += dt * a;
  }

  // visualization
  // 在可视化参考轨迹，预测的轨迹，带延迟的预测轨迹。第一个是不变的，后面两个和运行的时刻有关
  void visualization() 
  {
    nav_msgs::Path msg;
    msg.header.frame_id = "map";
    msg.header.stamp = ros::Time::now();
    geometry_msgs::PoseStamped p;
    for (double s = 0; s < s_.arcL(); s += 0.01) 
    {
      p.pose.position.x = s_(s).x();
      p.pose.position.y = s_(s).y();
      p.pose.position.z = 0.0;
      msg.poses.push_back(p);
    }
    ref_pub_.publish(msg);

    msg.poses.clear();
    for (int i = 0; i < N_; ++i) 
    {
      p.pose.position.x = predictState_[i](0);
      p.pose.position.y = predictState_[i](1);
      p.pose.position.z = 0.0;
      msg.poses.push_back(p);
    }
    traj_pub_.publish(msg);

    if (useCompensate1 == 1)
    {
      msg.poses.clear();
      VectorX x0_delay = x0_observe_;
      double dt = 0.001;
      for (double t = delay_; t > 0; t -= dt) {
        int i = std::ceil(t / dt_);
        VectorU input = historyInput_[history_length_ - i];
        step(x0_delay, input, dt);
        p.pose.position.x = x0_delay(0);
        p.pose.position.y = x0_delay(1);
        p.pose.position.z = 0.0;
        msg.poses.push_back(p);
      }
      traj_delay_pub_.publish(msg);
    }
    else if(useCompensate2 == 1)
    {
      msg.poses.clear();
      for(int i = 0; i < compensateDelayX0_.size(); i++)
      {
        p.pose.position.x = compensateDelayX0_[i](0);
        p.pose.position.y = compensateDelayX0_[i](1);
        p.pose.position.z = 0.0;
        msg.poses.push_back(p);
      }
      //这个只是delay的那一段吧
      traj_delay_pub_.publish(msg);
    }

  }

  void setPath(const nav_msgs::Path::ConstPtr& pathMsg)
  {
    std::vector<double> track_points_x, track_points_y;
    for (int i = 0; i < pathMsg->poses.size(); ++i)
    {
      track_points_x.push_back(pathMsg->poses[i].pose.position.x);
      track_points_y.push_back(pathMsg->poses[i].pose.position.y);
    }
    s_.setWayPoints(track_points_x, track_points_y);
    std::cout << "Set new path!!!" << std::endl;
  }

  void setPath(const std::vector<Eigen::Vector2d> &path_seg, int path_direction)
  {
    std::vector<double> track_points_x, track_points_y;
    for (int i = 0; i < path_seg.size(); ++i)
    {
      track_points_x.push_back(path_seg[i](0));
      track_points_y.push_back(path_seg[i](1));
    }
    s_.setWayPoints(track_points_x, track_points_y);

    if (path_direction == 1)//向前
    {
      path_direction_ = 1;
      desired_v_ = abs(desired_v_);
    }
    else//向后
    {
      path_direction_ = 0;
      desired_v_ = -1 * abs(desired_v_);
    }
    std::cout << "Set new path seg!!! The desired_v is " << desired_v_ << std::endl;
  }
};

}  // namespace mpc_car
