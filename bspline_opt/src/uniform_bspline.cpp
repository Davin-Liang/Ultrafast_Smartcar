#include "bspline_opt/uniform_bspline.h"
#include <ros/ros.h>

namespace opt_planner
{

  UniformBspline::UniformBspline(const Eigen::MatrixXd &points, const int &order,
                                 const double &interval)
  {
    setUniformBspline(points, order, interval);
  }

  UniformBspline::~UniformBspline() {}

  void UniformBspline::setUniformBspline(const Eigen::MatrixXd &points, const int &order,
                                         const double &interval)
  {
    control_points_ = points;
    p_ = order;
    interval_ = interval;

    n_ = points.cols() - 1;
    m_ = n_ + p_ + 1;

    u_ = Eigen::VectorXd::Zero(m_ + 1);
    for (int i = 0; i <= m_; ++i)
    {

      if (i <= p_)
      {
        u_(i) = double(-p_ + i) * interval_;
      }
      else if (i > p_ && i <= m_ - p_)
      {
        u_(i) = u_(i - 1) + interval_;
      }
      else if (i > m_ - p_)
      {
        u_(i) = u_(i - 1) + interval_;
      }
    }
  }

  void UniformBspline::setKnot(const Eigen::VectorXd &knot) { this->u_ = knot; }

  Eigen::VectorXd UniformBspline::getKnot() { return this->u_; }

  bool UniformBspline::getTimeSpan(double &um, double &um_p)
  {
    if (p_ > u_.rows() || m_ - p_ > u_.rows())
      return false;

    um = u_(p_);
    um_p = u_(m_ - p_);

    return true;
  }

  Eigen::MatrixXd UniformBspline::getControlPoint() { return control_points_; }

  Eigen::Vector3d UniformBspline::evaluateDeBoor(const double &u)
  {

    double ub = min(max(u_(p_), u), u_(m_ - p_)); // 取节点中间段为有效段

    // determine which [ui,ui+1] lay in
    int k = p_; // 从阶数值的节点开始
    while (true)
    {
      if (u_(k + 1) >= ub)
        break;
      ++k;
    }

    /* 通过k找到与当前节点相关的控制点是k-p_到k */
    /* deBoor's alg */
    // 提取与当前区间相关的 p+1 个控制点
    vector<Eigen::Vector3d> d;
    for (int i = 0; i <= p_; ++i)
    {
      d.push_back(control_points_.col(k - p_ + i));
    }

    for (int r = 1; r <= p_; ++r)
    {
      for (int i = p_; i >= r; --i)
      {
        double alpha = (ub - u_[i + k - p_]) / (u_[i + 1 + k - r] - u_[i + k - p_]);
        // cout << "alpha: " << alpha << endl;
        d[i] = (1 - alpha) * d[i - 1] + alpha * d[i];
      }
    }

    // std::cout << "p_ = " << p_ << std::endl;
    // std::cout << "正在提取路径点" << d[0] << std::endl;
    // std::cout << "正在提取路径点" << d[1] << std::endl;
    // std::cout << "正在提取路径点" << d[2] << std::endl;
    return d[p_];
  }

  // Eigen::VectorXd UniformBspline::evaluateDeBoorT(const double& t) {
  //   return evaluateDeBoor(t + u_(p_));
  // }

  Eigen::MatrixXd UniformBspline::getDerivativeControlPoints()
  {
    // The derivative of a b-spline is also a b-spline, its order become p_-1
    // control point Qi = p_*(Pi+1-Pi)/(ui+p_+1-ui+1)
    Eigen::MatrixXd ctp(control_points_.rows(), control_points_.cols() - 1);
    for (int i = 0; i < ctp.cols(); ++i)
    {
      ctp.col(i) =
          p_ * (control_points_.col(i + 1) - control_points_.col(i)) / (u_(i + p_ + 1) - u_(i + 1));
    }
    return ctp;
  }

  UniformBspline UniformBspline::getDerivative()
  {
    Eigen::MatrixXd ctp = getDerivativeControlPoints();
    UniformBspline derivative(ctp, p_ - 1, interval_);

    /* cut the first and last knot */
    Eigen::VectorXd knot(u_.rows() - 2);
    knot = u_.segment(1, u_.rows() - 2);
    derivative.setKnot(knot);

    return derivative;
  }

  double UniformBspline::getInterval() { return interval_; }

  void UniformBspline::setPhysicalLimits(const double &vel, const double &acc, const double &tolerance)
  {
    limit_vel_ = vel;
    limit_acc_ = acc;
    limit_ratio_ = 1.1;
    feasibility_tolerance_ = tolerance;
  }

  bool UniformBspline::checkFeasibility(double &ratio, bool show)
  {
    bool fea = true;

    Eigen::MatrixXd P = control_points_;
    int dimension = control_points_.rows();

    /* check vel feasibility and insert points */
    double max_vel = -1.0;
    double enlarged_vel_lim = limit_vel_ * (1.0 + feasibility_tolerance_) + 1e-4;
    for (int i = 0; i < P.cols() - 1; ++i)
    {
      Eigen::VectorXd vel = p_ * (P.col(i + 1) - P.col(i)) / (u_(i + p_ + 1) - u_(i + 1));

      if (fabs(vel(0)) > enlarged_vel_lim || fabs(vel(1)) > enlarged_vel_lim ||
          fabs(vel(2)) > enlarged_vel_lim)
      {

        if (show)
          cout << "[Check]: Infeasible vel " << i << " :" << vel.transpose() << endl;
        fea = false;

        for (int j = 0; j < dimension; ++j)
        {
          max_vel = max(max_vel, fabs(vel(j)));
        }
      }
    }

    /* acc feasibility */
    double max_acc = -1.0;
    double enlarged_acc_lim = limit_acc_ * (1.0 + feasibility_tolerance_) + 1e-4;
    for (int i = 0; i < P.cols() - 2; ++i)
    {

      Eigen::VectorXd acc = p_ * (p_ - 1) *
                            ((P.col(i + 2) - P.col(i + 1)) / (u_(i + p_ + 2) - u_(i + 2)) -
                             (P.col(i + 1) - P.col(i)) / (u_(i + p_ + 1) - u_(i + 1))) /
                            (u_(i + p_ + 1) - u_(i + 2));

      if (fabs(acc(0)) > enlarged_acc_lim || fabs(acc(1)) > enlarged_acc_lim ||
          fabs(acc(2)) > enlarged_acc_lim)
      {

        if (show)
          cout << "[Check]: Infeasible acc " << i << " :" << acc.transpose() << endl;
        fea = false;

        for (int j = 0; j < dimension; ++j)
        {
          max_acc = max(max_acc, fabs(acc(j)));
        }
      }
    }

    ratio = max(max_vel / limit_vel_, sqrt(fabs(max_acc) / limit_acc_));

    return fea;
  }

  void UniformBspline::lengthenTime(const double &ratio)
  {
    int num1 = 5;
    int num2 = getKnot().rows() - 1 - 5;

    double delta_t = (ratio - 1.0) * (u_(num2) - u_(num1));
    double t_inc = delta_t / double(num2 - num1);
    for (int i = num1 + 1; i <= num2; ++i)
      u_(i) += double(i - num1) * t_inc;
    for (int i = num2 + 1; i < u_.rows(); ++i)
      u_(i) += delta_t;
  }

  // void UniformBspline::recomputeInit() {}
/*
  param：
    - ts:轨迹执行时间
    - point_set:原轨迹点集合
    - start_end_derivative:起点和终点的高阶约束
  output:
    - ctrl_pts:控制点矩阵
  fuction:
    - 将给定点集和起始/终止导数转换为B-Spline曲线的控制点矩阵，通过对原始轨迹的拟合得到B样条轨迹的控制点
*/
  void UniformBspline::parameterizeToBspline(const double &ts, 
                                             const vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> &point_set,
                                             const vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> &start_end_derivative,
                                             Eigen::MatrixXd *ctrl_pts)
  {
    /* 该部分b样条构建，Fast-planner 和 ego-planner 是一样，已进行对比过 */
    // 不同的地方在于后边的b样条优化
    /* B样条基函数的公式 */
    //B03 = 1/6(1-u)^3
    //B13 = 1/6(3u^3 -6u^2 +4)
    //B23 = 1/6(-3u^3 + 3u^2 + 3u + 1)
    //B33 = 1/6(u^3)
    //n+1个控制点控制k阶B样条曲线 4个控制点控制1个坐标点，共有K个坐标点，需要K+2控制点来控制
    /* 数据检查 */
    if (ts <= 0)
    {
      cout << "[B-spline]: time step error." << endl;
      return;
    }

    /* 确保离散路径点的数量不少于 4，否则无法拟合至少三阶 B 样条 */
    if (point_set.size() <= 3)
    {
      cout << "[B-spline]:point set have only " << point_set.size() << " points." << endl;
      return;
    }

    if (start_end_derivative.size() != 4)
    {
      cout << "[B-spline]:derivatives error." << endl;
    }

    int K = point_set.size();//initial points size

    /* 通过数学建模，我们可以将 B 样条拟合问题转换为一个线性方程组的问题 */
    /* 初始化 A 矩阵，该矩阵用于常规的三维 B 样条 */
    Eigen::Vector3d prow(3), vrow(3), arow(3);
    // Eigen::Vector3d prow(3), vrow(3), arow(3), inirow(3), endrow(3), secrow(3), lsecrow(3);
    // inirow <<    1,   0,   0;
    // secrow <<  1/2, 7/6, 1/3;
    // lsecrow << 1/3, 7/6, 1/2;
    // endrow <<    0,   0,   1;

    /* 
      M^4 = [
              1   4  1 0
             -3   0  3 0
              3  -6  3 0
             -1   3 -3 1
            ]
     */
    prow <<      1,   4,   1;
    vrow <<     -1,   0,   1;
    arow <<      1,  -2,   1;

    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(K + 4, K + 2);
    for (int i = 0; i < K; ++i)
      A.block(i, i, 1, 3) = (1 / 6.0) * prow.transpose();

    A.block(K, 0, 1, 3) = (1 / 2.0) * vrow.transpose();
    A.block(K + 1, K - 1, 1, 3) = (1 / 2.0) * vrow.transpose();
    A.block(K + 2, 0, 1, 3) =  (1 / 2.0) * arow.transpose();
    A.block(K + 3, K - 1, 1, 3) =  (1 / 2.0) * arow.transpose();

    /* 初始化 Axy 矩阵，用于处理每个维度的速度和加速度 */
    // Axy 的矩阵形式如下：

    Eigen::MatrixXd Axy = Eigen::MatrixXd::Zero(K + 4, K + 2); 
    // (k+4) 个等式约束，(k+2) 个控制点
    // (k+4) 个等式约束包括 k 个位置约束(来自轨迹点)，2个速度约束，2个加速度约束(来自初始位置和终点))
    // 对于 k 个轨迹点，每一个轨迹点对应的时间节点t都是 tm，代入 ((t-tm)/deltat) 都会得到 0，所以 s(t) 有很多项都为 0 
    /* 
                  |p          |
                  |  p        |
                  |    ...    |
                  |        p  |
      Axy =       |          p|
                  |v          |
                  |          v|
                  |a          |
                  |          a|
     */           
    for (int i = 0; i < K; ++i)
      Axy.block(i, i, 1, 3) = (1 / 6.0) * prow.transpose();

    Axy.block(K, 0, 1, 3)         = (1 / 2.0 / ts) * vrow.transpose();
    Axy.block(K + 1, K - 1, 1, 3) = (1 / 2.0 / ts) * vrow.transpose();
    Axy.block(K + 2, 0, 1, 3)     =  (1 / ts / ts) * arow.transpose();
    Axy.block(K + 3, K - 1, 1, 3) =  (1 / ts / ts) * arow.transpose();

    // write b
    /* bx, by, bz 是输入路径点的x、y、z坐标以及起终点的导数（速度和加速度） */
    Eigen::VectorXd bx(K + 4), by(K + 4), bz(K + 4);
    /* 分别存储路径点的x、y、z坐标 */
    for (int i = 0; i < K; ++i)
    {
      bx(i) = point_set[i](0);
      by(i) = point_set[i](1);
      bz(i) = point_set[i](2);
    }
    /* 将起终点的导数值添加到这些向量的末尾 */
    for (int i = 0; i < 4; ++i)
    {
      bx(K + i) = start_end_derivative[i](0);
      by(K + i) = start_end_derivative[i](1);
      bz(K + i) = start_end_derivative[i](2);
    }

    // solve Ax = b
    /* 解线性方程组，分别得到每个坐标轴上的控制点值：px, py, pz */
    Eigen::VectorXd px = Axy.colPivHouseholderQr().solve(bx);
    Eigen::VectorXd py = Axy.colPivHouseholderQr().solve(by);
    Eigen::VectorXd pz = A.colPivHouseholderQr().solve(bz);


    // convert to control pts
    /* 控制点矩阵 ctrl_pts 被填充成三行（分别对应x、y、z坐标），每行有 K+2 个控制点 */
    ctrl_pts->resize(3, K + 2); // 对于 K 个路径点，我们需要 K+2 个控制点来完全确定B样条曲线
    ctrl_pts->row(0) = px.transpose();
    ctrl_pts->row(1) = py.transpose();
    ctrl_pts->row(2) = pz.transpose();

    // cout << "[B-spline]: parameterization ok." << endl;
  }

  double UniformBspline::getTimeSum()
  {
    double tm, tmp;
    if (getTimeSpan(tm, tmp))
      return tmp - tm;
    else
      return -1.0;
  }

  double UniformBspline::getLength(const double &res)
  {
    double length = 0.0;
    double dur = getTimeSum();
    Eigen::VectorXd p_l = evaluateDeBoorT(0.0), p_n;
    for (double t = res; t <= dur + 1e-4; t += res)
    {
      p_n = evaluateDeBoorT(t);
      length += (p_n - p_l).norm();
      p_l = p_n;
    }
    return length;
  }

  double UniformBspline::getJerk()
  {
    UniformBspline jerk_traj = getDerivative().getDerivative().getDerivative();

    Eigen::VectorXd times = jerk_traj.getKnot();
    Eigen::MatrixXd ctrl_pts = jerk_traj.getControlPoint();
    int dimension = ctrl_pts.rows();

    double jerk = 0.0;
    for (int i = 0; i < ctrl_pts.cols(); ++i)
    {
      for (int j = 0; j < dimension; ++j)
      {
        jerk += (times(i + 1) - times(i)) * ctrl_pts(j, i) * ctrl_pts(j, i);
      }
    }

    return jerk;
  }

  void UniformBspline::getMeanAndMaxVel(double &mean_v, double &max_v)
  {
    UniformBspline vel = getDerivative();
    double tm, tmp;
    vel.getTimeSpan(tm, tmp);

    double max_vel = -1.0, mean_vel = 0.0;
    int num = 0;
    for (double t = tm; t <= tmp; t += 0.01)
    {
      Eigen::VectorXd vxd = vel.evaluateDeBoor(t);
      double vn = vxd.norm();

      mean_vel += vn;
      ++num;
      if (vn > max_vel)
      {
        max_vel = vn;
      }
    }

    mean_vel = mean_vel / double(num);
    mean_v = mean_vel;
    max_v = max_vel;
  }

  void UniformBspline::getMeanAndMaxAcc(double &mean_a, double &max_a)
  {
    UniformBspline acc = getDerivative().getDerivative();
    double tm, tmp;
    acc.getTimeSpan(tm, tmp);

    double max_acc = -1.0, mean_acc = 0.0;
    int num = 0;
    for (double t = tm; t <= tmp; t += 0.01)
    {
      Eigen::VectorXd axd = acc.evaluateDeBoor(t);
      double an = axd.norm();

      mean_acc += an;
      ++num;
      if (an > max_acc)
      {
        max_acc = an;
      }
    }

    mean_acc = mean_acc / double(num);
    mean_a = mean_acc;
    max_a = max_acc;
  }
} // namespace opt_planner
