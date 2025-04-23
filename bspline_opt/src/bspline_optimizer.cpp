#include "bspline_opt/bspline_optimizer.h"
#include "bspline_opt/gradient_descent_optimizer.h"
using namespace std;

namespace opt_planner
{
  BsplineOptimizer::~BsplineOptimizer() {std::cout<<"des BsplineOptimizer"<<std::endl;}

  // void BsplineOptimizer::setParam(ros::NodeHandle &nh, const shared_ptr<HybridAStarFlow> &obj)
  void BsplineOptimizer::setParam(ros::NodeHandle &nh)
  {
    // nh.param("optimization/lambda_smooth", lambda1_, -1.0);
    // nh.param("optimization/lambda_collision", lambda2_, -1.0);
    // nh.param("optimization/lambda_feasibility", lambda3_, -1.0);
    // nh.param("optimization/lambda_fitness", lambda4_, -1.0);

    // nh.param("optimization/dist0", dist0_, -1.0);
    // nh.param("optimization/max_vel", max_vel_, -1.0);
    // nh.param("optimization/max_acc", max_acc_, -1.0);
    // nh.param("optimization/max_curvature",max_curvature_, 0.1);
    // nh.param("optimization/order", order_, 3);
    lambda1_ = 5.0;
    lambda2_ = 400;
    lambda3_ = 0.1;
    lambda4_ = 1.6;

    dist0_ = 1.0;
    max_vel_ = 1.0;
    max_acc_ = 1.0;
    max_curvature_ = 0.1;
    order_ = 3;


    // hybrid_astar_flow_ = obj; // 指针关联，使其在 BsplineOptimizer 内部就可以获取 hybird A* 生成的路径
  }

//   void BsplineOptimizer::setEnvironment(const GridMap::Ptr &env)
//   {
//     this->grid_map_ = env;
//   }

//   void BsplineOptimizer::setControlPoints(const Eigen::MatrixXd &points)
//   {
//     cps_.points = points;
//   }

  void BsplineOptimizer::setBsplineInterval(const double &ts) 
  {
    bspline_interval_ = ts;
  }

  /* 将给定的走廊约束（corridor）设置到优化器中，并生成控制点的上下界约束（var_bdk）。
    这些约束会被用于B样条优化问题，以确保生成的轨迹在给定的安全走廊（走廊可以是障碍物的包围盒）内 */
  void BsplineOptimizer::setBsplineCorridor(const VecCube& corridor) 
  {
    var_bdk.clear(); // var_bdk: 存储每个控制点在每个维度（x 和 y）上的上下界
    var_bdk.resize(2);
    for (int k = 0; k < corridor.size(); k++) // 遍历每个 Cube（对应一个走廊段）
    {   
        Cube cube_     = corridor[k];
        double scale_k = 1;

        // 每个控制点的上下界表示它在对应轴（x 或 y）上的可取范围
        for(int i = 0; i < 2; i++ ) // 遍历 i = 0, 1，表示 x 和 y 两个轴
        {   
            for (int j = 0; j < order_; j ++) // 遍历每段轨迹的控制点
            {   
                pair<double, double> vb_x;

                // 这里将所有控制点都约束在对应段轨迹的box中,而没有单独地将两段轨迹的连接点约束在相邻两个box的公共区域内原因如下:
                // 前一段轨迹末控制点在前一个box的范围内,后一段轨迹末控制点在后一个box的范围内,再加上两个控制点相等这一条件,那么
                // 这个控制点就自动地被限定在相邻两个box的公共区域内了,不需要再另外设置约束了.(妙啊)
                // 连接点自动约束：
                //   注释部分解释了为什么不需要显式约束两段轨迹的连接点在相邻两个盒子的公共区域内：
                //   前一段的末尾控制点已经限制在前一个盒子范围内。
                //   后一段的起始控制点已经限制在后一个盒子范围内。
                //   如果两个控制点相等，那么它自然落在两个盒子的公共区域。
                double lo_bound, up_bound;
                
                // cube_.box[i]: 当前 Cube 在轴 i 上的上下界
                lo_bound = (cube_.box[i].first  ) / scale_k; // scale_k为缩放系数
                up_bound = (cube_.box[i].second ) / scale_k;
                
                

                // vb_x: 待优化变量的界限(此问题指分段多项式的系数,也就是贝塞尔曲线的控制点)
                vb_x  = make_pair( lo_bound, up_bound ); 

                var_bdk[i].push_back(vb_x);
            }
        } 
    }
  }


  // /* This function is very similar to check_collision_and_rebound(). 
  //  * It was written separately, just because I did it once and it has been running stably since March 2020.
  //  * But I will merge then someday.*/
  std::vector<std::vector<Eigen::Vector3d>> BsplineOptimizer::initControlPoints(Eigen::MatrixXd &init_points, bool flag_first_init /*= true*/)
  {

    if (flag_first_init)
    {
      cps_.clearance = dist0_;
      cps_.resize(init_points.cols());
      cps_.points = init_points;
    }

  //   // /*** a star search ***/
    vector<vector<Eigen::Vector3d>> a_star_pathes;

    // std::cout << "return a_star_pathes;" << std::endl;
    return a_star_pathes;
  }

  int BsplineOptimizer::earlyExit(void *func_data, const double *x, const double *g, const double fx, const double xnorm, const double gnorm, const double step, int n, int k, int ls)
  {
    BsplineOptimizer *opt = reinterpret_cast<BsplineOptimizer *>(func_data);
    // cout << "k=" << k << endl;
    // cout << "opt->flag_continue_to_optimize_=" << opt->flag_continue_to_optimize_ << endl;
    return (opt->force_stop_type_ == STOP_FOR_ERROR || opt->force_stop_type_ == STOP_FOR_REBOUND);
  }

  double BsplineOptimizer::costFunctionRebound(void *func_data, const double *x, double *grad, const int n)
  {
    BsplineOptimizer *opt = reinterpret_cast<BsplineOptimizer *>(func_data); // 将传入的 func_data 转换为 BsplineOptimizer 类型的指针，便于调用类内的成员函数和变量

    double cost;
    opt->combineCostRebound(x, grad, cost, n);

    opt->iter_num_ += 1;
    return cost;
  }

//   double BsplineOptimizer::costFunctionRefine(void *func_data, const double *x, double *grad, const int n)
//   {
//     BsplineOptimizer *opt = reinterpret_cast<BsplineOptimizer *>(func_data);

//     double cost;
//     opt->combineCostRefine(x, grad, cost, n);

//     opt->iter_num_ += 1;
//     return cost;
//   }

  // void BsplineOptimizer::calcDistanceCostRebound(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient, 
  //                                                                     const vector<vector<pair<double, double>>>& boxes)
  // {
  //   cost = 0.0;
  //   // double demarcation = cps_.clearance;
  //   double demarcation = 1;
  //   double a = 3 * demarcation, b = -3 * pow(demarcation, 2), c = pow(demarcation, 3);

  //   // force_stop_type_ = DONT_STOP;
  //   // if (iter_num > 3 && smoothness_cost / (cps_.size - 2 * order_) < 0.1) // 0.1 is an experimental value that indicates the trajectory is smooth enough.
  //   // {
  //   //   check_collision_and_rebound();
  //   // }

  //   /*** calculate distance cost and gradient ***/
  //   for(int k = 0; k < 2; k++){

  //     for(int i = 3; i < q.cols()-3 ; i++){
  //       double upbound = 0, lowbound = 0;
  //       if(0 == i){
  //         upbound = 
  //             var_bdk[k][ i * order_ + 0].second;
  //         lowbound = 
  //             var_bdk[k][ i * order_ + 0].first;
  //       }
  //       else if(1 == i){
  //         upbound = 
  //             min(var_bdk[k][ i * order_ + 0].second, var_bdk[k][ (i-1) * order_ + 1].second);
  //         lowbound = 
  //             max(var_bdk[k][ i * order_ + 0].first, var_bdk[k][ (i-1) * order_ + 1].first);
  //       }
  //       else if(q.cols()-2 == i){
  //         upbound = 
  //             min(var_bdk[k][ (i-1) * order_ + 1].second, var_bdk[k][ (i-2) * order_ + 2].second);
  //         lowbound = 
  //             max(var_bdk[k][ (i-1) * order_ + 1].first, var_bdk[k][ (i-2) * order_ + 2].first);
  //       }
  //       else if(q.cols()-1 == i){
  //         upbound = 
  //             var_bdk[k][ (i-2) * order_ + 2].second;
  //         lowbound = 
  //               var_bdk[k][ (i-2) * order_ + 2].first;
  //       }
  //       else{
  //         upbound = 
  //             min(var_bdk[k][ i * order_ + 0].second, min(var_bdk[k][ (i-1) * order_ + 1].second, var_bdk[k][ (i-2) * order_ + 2].second));
  //         lowbound = 
  //             max(var_bdk[k][ i * order_ + 0].first, max(var_bdk[k][ (i-1) * order_ + 1].first, var_bdk[k][ (i-2) * order_ + 2].first));
  //       }
  //       double boxup = upbound - q.col(i)(k);
  //       double boxlow = q.col(i)(k) - lowbound;
  //       double updist_err = demarcation - boxup;
  //       double lowdist_err = demarcation - boxlow;
  //       if(updist_err >= 0){
  //         if(updist_err < demarcation){
  //           cost += pow(updist_err, 3);
  //           gradient.col(i)(k) += 3.0 * updist_err * updist_err;
  //         }
  //         else{
  //           cost += a * updist_err * updist_err + b * updist_err + c;
  //           gradient.col(i)(k) += 2.0 * a * updist_err + b ;
  //         }
  //       //   std::cout<<"upbound " << upbound <<endl;
  //       // std::cout<<"lowbound " << lowbound <<endl;
  //       // std::cout<<"q.col(i)(k) " << q.col(i)(k) <<endl;
  //       }
  //       else if(lowdist_err >= 0){
  //         if(lowdist_err < demarcation){
  //           cost += pow(lowdist_err, 3);
  //           gradient.col(i)(k) += -3.0 * lowdist_err * lowdist_err;
  //         }
  //         else{
  //           cost += a * lowdist_err * lowdist_err + b * lowdist_err + c;
  //           gradient.col(i)(k) += -2.0 * a * lowdist_err + b ;
  //         }
  //       //   std::cout<<"upbound " << upbound <<endl;
  //       // std::cout<<"lowbound " << lowbound <<endl;
  //       // std::cout<<"q.col(i)(k) " << q.col(i)(k) <<endl;
  //       }

  //       // if(q.col(i)(k) > upbound){
  //       //   double temp = q.col(i)(k) - upbound;
  //       //   cost += pow(temp, 2);
  //       //   gradient.col(i)(k) += 2 * temp;
  //       // //   std::cout<<"upbound " << upbound <<endl;
  //       // // std::cout<<"lowbound " << lowbound <<endl;
  //       // // std::cout<<"q.col(i)(k) " << q.col(i)(k) <<endl;
  //       // }
  //       // else if(q.col(i)(k) < lowbound){
  //       //   double temp = q.col(i)(k) - lowbound;
  //       //   cost += pow(temp, 2);
  //       //   gradient.col(i)(k) += 2 * temp;
  //       // //   std::cout<<"upbound " << upbound <<endl;
  //       // // std::cout<<"lowbound " << lowbound <<endl;
  //       // // std::cout<<"q.col(i)(k) " << q.col(i)(k) <<endl;
  //       // }
  //     }
  //   }
    
    
  // }
  void BsplineOptimizer::calcDistanceCostRebound(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient, 
                                                                      const vector<vector<pair<double, double>>>& boxes)
  {
    cost = 0.0;
    double demarcation = cps_.clearance;
    double a = 3 * demarcation, b = -3 * pow(demarcation, 2), c = pow(demarcation, 3);

    /*** calculate distance cost and gradient ***/
    for (int k = 0; k < 2; k ++) // k 遍历 0 和 1，对应 x 和 y 两个坐标轴
    {
      for (int i = 0; i < q.cols()-2 ; i++)
      {
        for (int j = 0; j < order_; j++)
        {
          double upbound = 0, lowbound = 0;

          upbound = var_bdk[k][3*i + j].second;
          lowbound = var_bdk[k][3*i + j].first;
          /* boxup 和 boxlow 表示点到边界的距离 */
          double boxup = upbound - q.col(i+j)(k);
          double boxlow = q.col(i+j)(k) - lowbound;
          /* updist_err 和 lowdist_err 计算点到边界的剩余安全距离 */
          double updist_err = demarcation - boxup;
          double lowdist_err = demarcation - boxlow;
          if (updist_err >= 0)
          {
            if(updist_err < demarcation)
            {
              cost += pow(updist_err, 3);
              gradient.col(i+j)(k) += 3.0 * updist_err * updist_err;
            }
            else
            {
              cost += a * updist_err * updist_err + b * updist_err + c;
              gradient.col(i+j)(k) += 2.0 * a * updist_err + b ;
            }
            //   std::cout<<"upbound " << upbound <<endl;
            // std::cout<<"lowbound " << lowbound <<endl;
            // std::cout<<"q.col(i)(k) " << q.col(i)(k) <<endl;
          }
          if(lowdist_err >= 0)
          {
            if(lowdist_err < demarcation)
            {
              cost += pow(lowdist_err, 3);
              gradient.col(i+j)(k) += -3.0 * lowdist_err * lowdist_err;
            }
            else
            {
              cost += a * lowdist_err * lowdist_err + b * lowdist_err + c;
              gradient.col(i+j)(k) += -2.0 * a * lowdist_err - b ;
            }
          //   std::cout<<"upbound " << upbound <<endl;
          // std::cout<<"lowbound " << lowbound <<endl;
          // std::cout<<"q.col(i)(k) " << q.col(i)(k) <<endl;
          }
        }
    
      }
    }
    
    
  }

//   void BsplineOptimizer::calcFitnessCost(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient)
//   {

//     cost = 0.0;

//     int end_idx = q.cols() - order_;

//     // def: f = |x*v|^2/a^2 + |x×v|^2/b^2
//     double a2 = 25, b2 = 1;
//     for (auto i = order_ - 1; i < end_idx + 1; ++i)
//     {
//       Eigen::Vector3d x = (q.col(i - 1) + 4 * q.col(i) + q.col(i + 1)) / 6.0 - ref_pts_[i - 1];
//       Eigen::Vector3d v = (ref_pts_[i] - ref_pts_[i - 2]).normalized();

//       double xdotv = x.dot(v);
//       Eigen::Vector3d xcrossv = x.cross(v);

//       double f = pow((xdotv), 2) / a2 + pow(xcrossv.norm(), 2) / b2;
//       cost += f;

//       Eigen::Matrix3d m;
//       m << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
//       Eigen::Vector3d df_dx = 2 * xdotv / a2 * v + 2 / b2 * m * xcrossv;

//       gradient.col(i - 1) += df_dx / 6;
//       gradient.col(i) += 4 * df_dx / 6;
//       gradient.col(i + 1) += df_dx / 6;
//     }
//   }

// 计算B样条轨迹的平滑性成本（cost）以及相应的梯度（gradient）。平滑性通常通过控制加速度或加加速度（jerk，抖动）来衡量，选择哪种方式取决于参数 falg_use_jerk 的值。
// 当 falg_use_jerk 为真时，计算 jerk 的平滑性成本。
// 当 falg_use_jerk 为假时，计算加速度的平滑性成本。
  void BsplineOptimizer::calcSmoothnessCost(const Eigen::MatrixXd &q, double &cost,
                                            Eigen::MatrixXd &gradient, bool falg_use_jerk /* = true*/)
  {
    cost = 0.0;
    double ts, /*vm2, am2, */ ts_inv2, ts_inv3;
    // vm2 = max_vel_ * max_vel_;
    // am2 = max_acc_ * max_acc_;
    ts = bspline_interval_;
    ts_inv2 = 1 / ts / ts; // 1 / (ts^2)
    ts_inv3 = 1 / ts / ts /ts; // 1 / (ts^3)
    if (falg_use_jerk)
    {
      Eigen::Vector3d jerk, temp_j;

      for (int i = 0; i < q.cols() - 3; i++)
      {
        /* evaluate jerk */
        jerk = (q.col(i + 3) - 3 * q.col(i + 2) + 3 * q.col(i + 1) - q.col(i)) * ts_inv3;

        cost += jerk.squaredNorm();
        temp_j = 2.0 * jerk * ts_inv3; // TODO:
        /* jerk gradient */
        gradient.col(i + 0) += -temp_j;
        gradient.col(i + 1) += 3.0 * temp_j;
        gradient.col(i + 2) += -3.0 * temp_j;
        gradient.col(i + 3) += temp_j;
      }
    }
    else
    {
      Eigen::Vector3d acc, temp_acc;

      for (int i = 0; i < q.cols() - 2; i++)
      {
        /* evaluate acc */
        acc = (q.col(i + 2) - 2 * q.col(i + 1) + q.col(i))* ts_inv2;
        cost += acc.squaredNorm();
        temp_acc = 2.0 * acc * ts_inv2;
        /* acc gradient */
        gradient.col(i + 0) += temp_acc;
        gradient.col(i + 1) += -2.0 * temp_acc;
        gradient.col(i + 2) += temp_acc;
      }
    }
  }

  void BsplineOptimizer::calcFeasibilityCost(const Eigen::MatrixXd &q, double &cost,
                                             Eigen::MatrixXd &gradient)
  {

    //#define SECOND_DERIVATIVE_CONTINOUS

#ifdef SECOND_DERIVATIVE_CONTINOUS
    
    cost = 0.0;
    double demarcation = 1.0; // 1m/s, 1m/s/s
    double ar = 3 * demarcation, br = -3 * pow(demarcation, 2), cr = pow(demarcation, 3);
    double al = ar, bl = -br, cl = cr;

    /* abbreviation */
    double ts, ts_inv2, ts_inv3;
    ts = bspline_interval_;
    ts_inv2 = 1 / ts / ts;
    ts_inv3 = 1 / ts / ts / ts;

    /* velocity feasibility */
    for (int i = 0; i < q.cols() - 1; i++)
    {
      Eigen::Vector3d vi = (q.col(i + 1) - q.col(i)) / ts;

      for (int j = 0; j < 2; j++)
      {
        if (vi(j) > max_vel_ + demarcation)
        {
          double diff = vi(j) - max_vel_;
          cost += (ar * diff * diff + br * diff + cr) * ts_inv3; // multiply ts_inv3 to make vel and acc has similar magnitude

          double grad = (2.0 * ar * diff + br) / ts * ts_inv3;
          gradient(j, i + 0) += -grad;
          gradient(j, i + 1) += grad;
        }
        else if (vi(j) > max_vel_)
        {
          double diff = vi(j) - max_vel_;
          cost += pow(diff, 3) * ts_inv3;
          ;

          double grad = 3 * diff * diff / ts * ts_inv3;
          ;
          gradient(j, i + 0) += -grad;
          gradient(j, i + 1) += grad;
        }
        else if (vi(j) < -(max_vel_ + demarcation))
        {
          double diff = vi(j) + max_vel_;
          cost += (al * diff * diff + bl * diff + cl) * ts_inv3;

          double grad = (2.0 * al * diff + bl) / ts * ts_inv3;
          gradient(j, i + 0) += -grad;
          gradient(j, i + 1) += grad;
        }
        else if (vi(j) < -max_vel_)
        {
          double diff = vi(j) + max_vel_;
          cost += -pow(diff, 3) * ts_inv3;

          double grad = -3 * diff * diff / ts * ts_inv3;
          gradient(j, i + 0) += -grad;
          gradient(j, i + 1) += grad;
        }
        else
        {
          /* nothing happened */
        }
      }
    }

    /* acceleration feasibility */
    for (int i = 0; i < q.cols() - 2; i++)
    {
      Eigen::Vector3d ai = (q.col(i + 2) - 2 * q.col(i + 1) + q.col(i)) * ts_inv2;

      for (int j = 0; j < 2; j++)
      {
        if (ai(j) > max_acc_ + demarcation)
        {
          double diff = ai(j) - max_acc_;
          cost += ar * diff * diff + br * diff + cr;

          double grad = (2.0 * ar * diff + br) * ts_inv2;
          gradient(j, i + 0) += grad;
          gradient(j, i + 1) += -2 * grad;
          gradient(j, i + 2) += grad;
        }
        else if (ai(j) > max_acc_)
        {
          double diff = ai(j) - max_acc_;
          cost += pow(diff, 3);

          double grad = 3 * diff * diff * ts_inv2;
          gradient(j, i + 0) += grad;
          gradient(j, i + 1) += -2 * grad;
          gradient(j, i + 2) += grad;
        }
        else if (ai(j) < -(max_acc_ + demarcation))
        {
          double diff = ai(j) + max_acc_;
          cost += al * diff * diff + bl * diff + cl;

          double grad = (2.0 * al * diff + bl) * ts_inv2;
          gradient(j, i + 0) += grad;
          gradient(j, i + 1) += -2 * grad;
          gradient(j, i + 2) += grad;
        }
        else if (ai(j) < -max_acc_)
        {
          double diff = ai(j) + max_acc_;
          cost += -pow(diff, 3);

          double grad = -3 * diff * diff * ts_inv2;
          gradient(j, i + 0) += grad;
          gradient(j, i + 1) += -2 * grad;
          gradient(j, i + 2) += grad;
        }
        else
        {
          /* nothing happened */
        }
      }
    }

#else
    
    cost = 0.0;
    /* abbreviation */
    double ts, /*vm2, am2, */ ts_inv2;
    // vm2 = max_vel_ * max_vel_;
    // am2 = max_acc_ * max_acc_;

    ts = bspline_interval_;
    ts_inv2 = 1 / ts / ts;

    /* velocity feasibility */
    for (int i = 0; i < q.cols() - 1; i++)
    {
      Eigen::Vector3d vi = (q.col(i + 1) - q.col(i)) / ts;
      // 公式为 vi = (q(i+1) - qi) / deltat

      //cout << "temp_v * vi=" ;
      for (int j = 0; j < 3; j++)
      {
        if (vi(j) > max_vel_) // 若速度分量超出阈值（|v| > max_vel_），代价为平方惩罚
        {
          // cout << "fuck VEL" << endl;
          // cout << vi(j) << endl;
          cost += pow(vi(j) - max_vel_, 2) * ts_inv2; // multiply ts_inv3 to make vel and acc has similar magnitude

          gradient(j, i + 0) += -2 * (vi(j) - max_vel_) / ts * ts_inv2;
          gradient(j, i + 1) += 2 * (vi(j) - max_vel_) / ts * ts_inv2;
        }
        else if (vi(j) < -max_vel_)
        {
          cost += pow(vi(j) + max_vel_, 2) * ts_inv2;

          gradient(j, i + 0) += -2 * (vi(j) + max_vel_) / ts * ts_inv2;
          gradient(j, i + 1) += 2 * (vi(j) + max_vel_) / ts * ts_inv2;
        }
        else
        {
          /* code */
        }
      }
    }

    /* acceleration feasibility */
    for (int i = 0; i < q.cols() - 2; i++)
    {
      Eigen::Vector3d ai = (q.col(i + 2) - 2 * q.col(i + 1) + q.col(i)) * ts_inv2;

      //cout << "temp_a * ai=" ;
      for (int j = 0; j < 3; j++)
      {
        if (ai(j) > max_acc_)
        {
          // cout << "fuck ACC" << endl;
          // cout << ai(j) << endl;
          cost += pow(ai(j) - max_acc_, 2);

          gradient(j, i + 0) += 2 * (ai(j) - max_acc_) * ts_inv2;
          gradient(j, i + 1) += -4 * (ai(j) - max_acc_) * ts_inv2;
          gradient(j, i + 2) += 2 * (ai(j) - max_acc_) * ts_inv2;
        }
        else if (ai(j) < -max_acc_)
        {
          cost += pow(ai(j) + max_acc_, 2);

          gradient(j, i + 0) += 2 * (ai(j) + max_acc_) * ts_inv2;
          gradient(j, i + 1) += -4 * (ai(j) + max_acc_) * ts_inv2;
          gradient(j, i + 2) += 2 * (ai(j) + max_acc_) * ts_inv2;
        }
        else
        {
          /* code */
        }
      }
      //cout << endl;
    }

    // /* curvature feasibility */
    // for (int i = 0; i < q.cols() - 2; i++)
    // {
    //   Eigen::Vector2d ai = (q.col(i + 2).head(2) - 2 * q.col(i + 1).head(2) + q.col(i).head(2)) ;
    //   Eigen::Vector2d vi = (0 * q.col(i + 2) + q.col(i + 1).head(2) - q.col(i).head(2)) ;
    //   Eigen::Matrix2d B = Eigen::Matrix2d::Zero(2,2);
    //   double vi_norm = vi.norm();
    //   double vi_norm_3 = vi.norm()*vi.norm()*vi.norm();
    //   double vi_norm_5 = vi.norm()*vi.norm()*vi.norm()*vi.norm()*vi.norm()；
    //   B(0,1) = -1;
    //   B(1,0) = 1;
    //   double g_kl = 1/(vi_norm_3) * ai.transpose()*B*vi;
    //   if(g_kl > max_curvature_){
    //     cost += pow(g_kl - max_curvature_, 2);

    //   }
    //   else if(g_kl < -max_curvature_){
    //     cost += pow(g_kl + max_curvature_, 2);
    //   }
    //   for (int j = 0; j < 2; j++)
    //   {
    //     if (ai(j) > max_acc_)
    //     {
    //       // cout << "fuck ACC" << endl;
    //       // cout << ai(j) << endl;
    //       cost += pow(ai(j) - max_acc_, 2);

    //       gradient(j, i + 0) += 2 * (ai(j) - max_acc_) * ts_inv2;
    //       gradient(j, i + 1) += -4 * (ai(j) - max_acc_) * ts_inv2;
    //       gradient(j, i + 2) += 2 * (ai(j) - max_acc_) * ts_inv2;
    //     }
    //     else if (ai(j) < -max_acc_)
    //     {
    //       cost += pow(ai(j) + max_acc_, 2);

    //       gradient(j, i + 0) += 2 * (ai(j) + max_acc_) * ts_inv2;
    //       gradient(j, i + 1) += -4 * (ai(j) + max_acc_) * ts_inv2;
    //       gradient(j, i + 2) += 2 * (ai(j) + max_acc_) * ts_inv2;
    //     }
    //     else
    //     {
    //       /* code */
    //     }
    //   }
    //   //cout << endl;
    // }




    /*curvature constrain*/
    for(int i = 0; i < q.cols() - 2; i++){
        //控制点应该要写成路径点的形式

        Eigen::Vector2d temp_c =  (q.col(i).head(2) - 2 * q.col(i+1).head(2) + q.col(i+2).head(2) );

        double curvature_max_bound =  (temp_c.norm()-curvature_constraint_sqr);
        if(curvature_max_bound > 0.0){
          for(int j = 0; j < 2; j++){
            cost +=  curvature_max_bound * curvature_max_bound;
            gradient(j, i + 0) += 2 * (temp_c(j)) * (curvature_max_bound);
            gradient(j, i + 1) += -4 * (temp_c(j)) * (curvature_max_bound);
            gradient(j, i + 2) += 2 * (temp_c(j)) * (curvature_max_bound);
            
          }
        }
    }

#endif
  }

//   bool BsplineOptimizer::check_collision_and_rebound(void)
//   {

//     int end_idx = cps_.size - order_;

//     /*** Check and segment the initial trajectory according to obstacles ***/
//     int in_id, out_id;
//     vector<std::pair<int, int>> segment_ids;
//     bool flag_new_obs_valid = false;
//     int i_end = end_idx - (end_idx - order_) / 3;
//     for (int i = order_ - 1; i <= i_end; ++i)
//     {

//       bool occ = grid_map_->getInflateOccupancy(cps_.points.col(i));

//       /*** check if the new collision will be valid ***/
//       if (occ)
//       {
//         for (size_t k = 0; k < cps_.direction[i].size(); ++k)
//         {
//           cout.precision(2);
//           if ((cps_.points.col(i) - cps_.base_point[i][k]).dot(cps_.direction[i][k]) < 1 * grid_map_->getResolution()) // current point is outside all the collision_points.
//           {
//             occ = false; // Not really takes effect, just for better hunman understanding.
//             break;
//           }
//         }
//       }

//       if (occ)
//       {
//         flag_new_obs_valid = true;

//         int j;
//         for (j = i - 1; j >= 0; --j)
//         {
//           occ = grid_map_->getInflateOccupancy(cps_.points.col(j));
//           if (!occ)
//           {
//             in_id = j;
//             break;
//           }
//         }
//         if (j < 0) // fail to get the obs free point
//         {
//           ROS_ERROR("ERROR! the drone is in obstacle. This should not happen.");
//           in_id = 0;
//         }

//         for (j = i + 1; j < cps_.size; ++j)
//         {
//           occ = grid_map_->getInflateOccupancy(cps_.points.col(j));

//           if (!occ)
//           {
//             out_id = j;
//             break;
//           }
//         }
//         if (j >= cps_.size) // fail to get the obs free point
//         {
//           ROS_WARN("WARN! terminal point of the current trajectory is in obstacle, skip this planning.");

//           force_stop_type_ = STOP_FOR_ERROR;
//           return false;
//         }

//         i = j + 1;

//         segment_ids.push_back(std::pair<int, int>(in_id, out_id));
//       }
//     }

//     if (flag_new_obs_valid)
//     {
//       vector<vector<Eigen::Vector3d>> a_star_pathes;
//       for (size_t i = 0; i < segment_ids.size(); ++i)
//       {
//         /*** a star search ***/
//         Eigen::Vector3d in(cps_.points.col(segment_ids[i].first)), out(cps_.points.col(segment_ids[i].second));
//         if (a_star_->AstarSearch(/*(in-out).norm()/10+0.05*/ 0.1, in, out))
//         {
//           a_star_pathes.push_back(a_star_->getPath());
//         }
//         else
//         {
//           ROS_ERROR("a star error");
//           segment_ids.erase(segment_ids.begin() + i);
//           i--;
//         }
//       }

//       /*** Assign parameters to each segment ***/
//       for (size_t i = 0; i < segment_ids.size(); ++i)
//       {
//         // step 1
//         for (int j = segment_ids[i].first; j <= segment_ids[i].second; ++j)
//           cps_.flag_temp[j] = false;

//         // step 2
//         int got_intersection_id = -1;
//         for (int j = segment_ids[i].first + 1; j < segment_ids[i].second; ++j)
//         {
//           Eigen::Vector3d ctrl_pts_law(cps_.points.col(j + 1) - cps_.points.col(j - 1)), intersection_point;
//           int Astar_id = a_star_pathes[i].size() / 2, last_Astar_id; // Let "Astar_id = id_of_the_most_far_away_Astar_point" will be better, but it needs more computation
//           double val = (a_star_pathes[i][Astar_id] - cps_.points.col(j)).dot(ctrl_pts_law), last_val = val;
//           while (Astar_id >= 0 && Astar_id < (int)a_star_pathes[i].size())
//           {
//             last_Astar_id = Astar_id;

//             if (val >= 0)
//               --Astar_id;
//             else
//               ++Astar_id;

//             val = (a_star_pathes[i][Astar_id] - cps_.points.col(j)).dot(ctrl_pts_law);

//             // cout << val << endl;

//             if (val * last_val <= 0 && (abs(val) > 0 || abs(last_val) > 0)) // val = last_val = 0.0 is not allowed
//             {
//               intersection_point =
//                   a_star_pathes[i][Astar_id] +
//                   ((a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id]) *
//                    (ctrl_pts_law.dot(cps_.points.col(j) - a_star_pathes[i][Astar_id]) / ctrl_pts_law.dot(a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id])) // = t
//                   );

//               got_intersection_id = j;
//               break;
//             }
//           }

//           if (got_intersection_id >= 0)
//           {
//             cps_.flag_temp[j] = true;
//             double length = (intersection_point - cps_.points.col(j)).norm();
//             if (length > 1e-5)
//             {
//               for (double a = length; a >= 0.0; a -= grid_map_->getResolution())
//               {
//                 bool occ = grid_map_->getInflateOccupancy((a / length) * intersection_point + (1 - a / length) * cps_.points.col(j));

//                 if (occ || a < grid_map_->getResolution())
//                 {
//                   if (occ)
//                     a += grid_map_->getResolution();
//                   cps_.base_point[j].push_back((a / length) * intersection_point + (1 - a / length) * cps_.points.col(j));
//                   cps_.direction[j].push_back((intersection_point - cps_.points.col(j)).normalized());
//                   break;
//                 }
//               }
//             }
//             else
//             {
//               got_intersection_id = -1;
//             }
//           }
//         }

//         //step 3
//         if (got_intersection_id >= 0)
//         {
//           for (int j = got_intersection_id + 1; j <= segment_ids[i].second; ++j)
//             if (!cps_.flag_temp[j])
//             {
//               cps_.base_point[j].push_back(cps_.base_point[j - 1].back());
//               cps_.direction[j].push_back(cps_.direction[j - 1].back());
//             }

//           for (int j = got_intersection_id - 1; j >= segment_ids[i].first; --j)
//             if (!cps_.flag_temp[j])
//             {
//               cps_.base_point[j].push_back(cps_.base_point[j + 1].back());
//               cps_.direction[j].push_back(cps_.direction[j + 1].back());
//             }
//         }
//         else
//           ROS_WARN("Failed to generate direction. It doesn't matter.");
//       }

//       force_stop_type_ = STOP_FOR_REBOUND;
//       return true;
//     }

//     return false;
//   }

  bool BsplineOptimizer::BsplineOptimizeTrajRebound(Eigen::MatrixXd &optimal_points, double ts, const VecCube& corridor)
  {
    /* 设置时间区间间隔 */
    setBsplineInterval(ts);
    /* 设置障碍物约束 */
    setBsplineCorridor(corridor);
    bool flag_success = rebound_optimize();
    optimal_points = cps_.points;

    return flag_success;
  }

//   bool BsplineOptimizer::BsplineOptimizeTrajRefine(const Eigen::MatrixXd &init_points, const double ts, Eigen::MatrixXd &optimal_points)
//   {

//     setControlPoints(init_points);
//     setBsplineInterval(ts);

//     bool flag_success = refine_optimize();

//     optimal_points = cps_.points;

//     return flag_success;
//   }

  /* 用于优化 B 样条轨迹的函数。
  它结合 L-BFGS（Limited-memory Broyden-Fletcher-Goldfarb-Shanno）优化算法，
  通过多次尝试和重启，寻找最优的控制点配置，从而生成满足约束条件的轨迹 */
  // 目标：优化控制点使得生成的 B 样条轨迹满足路径规划需求（如避障、平滑性、以及其他目标函数约束）。
  // 优化方法：使用 L-BFGS 优化算法迭代调整控制点。
  // 退出条件：
  //  优化算法成功收敛。
  //  达到预定的迭代或重启次数限制。
  bool BsplineOptimizer::rebound_optimize()
  {
    iter_num_ = 0;
    /* 定义优化中参与调整的控制点索引范围 */
    int start_id = (order_) ;
    int end_id = this->cps_.size - (order_ );
    variable_num_ = 3 * (end_id - start_id); // 需要优化的变量总数（控制点数 × 3）

    double final_cost; // 最终优化结果的代价
    int iteration_count; // 记录优化的迭代次数
    double gradient_norm; // 梯度范数
    ros::Time t0 = ros::Time::now(), t1, t2; // 用于记录优化开始和结束的时间
    int restart_nums = 0, rebound_times = 0;
    
    bool flag_force_return, flag_occ, success, flag_found;
    new_lambda2_ = lambda2_; // 优化的代价函数中的权重参数
    constexpr int MAX_RESART_NUMS_SET = 3; // 最大重启次数
    // mu_ = Eigen::VectorXd::Zero(end_id + start_id - 2, 1);
    // double kkt_1 = 100.0;
    int loop_count = 0;
    do
    {
      /* ---------- prepare ---------- */
      min_cost_ = std::numeric_limits<double>::max(); // 在优化过程中记录的最小代价，初始化为最大值
      iter_num_ = 0;
      flag_force_return = false; // 是否被迫停止优化
      flag_occ = false; // 是否检测到约束冲突
      success = false; // 优化是否完成
      flag_found = false; // 是否找到符合约束的轨迹
      double q[variable_num_]; // 优化变量 q（控制点数据的拷贝）
      memcpy(q, cps_.points.data() + 3 * start_id, variable_num_ * sizeof(q[0]));
      
      lbfgs::lbfgs_parameter_t lbfgs_params;
      lbfgs::lbfgs_load_default_parameters(&lbfgs_params);
      lbfgs_params.mem_size = 16; // 优化器使用的内存大小
      lbfgs_params.max_iterations = 1000; //  最大迭代次数
      lbfgs_params.g_epsilon = 0.01; // 梯度收敛阈值
      lbfgs_params.past = 3; // 提前终止条件的历史窗口大小
      lbfgs_params.min_step = 1e-32; // 最小步长
      lbfgs_params.delta = 1e-6; // 提前终止的目标变化阈值
      lbfgs_params.line_search_type = 0;

      
      /* ---------- optimize ---------- */
      t1 = ros::Time::now();
      int result = lbfgs::lbfgs_optimize(variable_num_, 
                                          q, // 初始优化变量
                                          &final_cost, // 输出的优化代价
                                          BsplineOptimizer::costFunctionRebound, // 用户定义的代价函数
                                          NULL, 
                                          BsplineOptimizer::earlyExit, // 用户定义的早退出条件函数
                                          this, // 优化器对象
                                          &lbfgs_params, // 优化参数
                                          gradient_norm, // 输出梯度范数
                                          iteration_count); // 输出迭代次数
      t2 = ros::Time::now();
      double time_ms = (t2 - t1).toSec() * 1000;
      double total_time_ms = (t2 - t0).toSec() * 1000;

      if (result == lbfgs::LBFGS_CONVERGENCE ||
          result == lbfgs::LBFGS_ALREADY_MINIMIZED ||
          result == lbfgs::LBFGS_STOP) // 如果优化成功（收敛或达到最优），标记为成功并增加重启计数
      {
        // ROS_WARN("Solver error in planning!, return = %s", lbfgs::lbfgs_strerror(result));
        flag_force_return = false;
        flag_found = true;
        flag_occ = false;

        success = true;
        restart_nums++;
      }
      else if (result == lbfgs::LBFGSERR_CANCELED) // 如果优化被中断（可能因约束冲突），记录并重启优化
      {
        flag_force_return = true;
        rebound_times++;
        cout << "iter=" << iter_num_ << ",time(ms)=" << time_ms << ",rebound." << endl;
      }
      else // 如果出现其他错误，记录日志并可能强制退出循环
      {
        ROS_WARN("Solver error. Return = %d, %s. Skip this planning.", result, lbfgs::lbfgs_strerror(result));
        if(loop_count == 9 ) success = true;
        // while (ros::ok());
      }
    loop_count++;
    // } while ((!flag_found && (loop_count < 20))||(flag_occ && restart_nums < MAX_RESART_NUMS_SET) ||
    //          (flag_force_return && force_stop_type_ == STOP_FOR_REBOUND && rebound_times <= 20));
    }while (!flag_found && loop_count < 10);

    return success;
  }

//   bool BsplineOptimizer::refine_optimize()
//   {
//     iter_num_ = 0;
//     int start_id = order_;
//     int end_id = this->cps_.points.cols() - order_;
//     variable_num_ = 3 * (end_id - start_id);

//     double q[variable_num_];
//     double final_cost;

//     memcpy(q, cps_.points.data() + 3 * start_id, variable_num_ * sizeof(q[0]));

//     double origin_lambda4 = lambda4_;
//     bool flag_safe = true;
//     int iter_count = 0;
//     do
//     {
//       lbfgs::lbfgs_parameter_t lbfgs_params;
//       lbfgs::lbfgs_load_default_parameters(&lbfgs_params);
//       lbfgs_params.mem_size = 16;
//       lbfgs_params.max_iterations = 200;
//       lbfgs_params.g_epsilon = 0.001;

//       int result = lbfgs::lbfgs_optimize(variable_num_, q, &final_cost, BsplineOptimizer::costFunctionRefine, NULL, NULL, this, &lbfgs_params);
//       if (result == lbfgs::LBFGS_CONVERGENCE ||
//           result == lbfgs::LBFGSERR_MAXIMUMITERATION ||
//           result == lbfgs::LBFGS_ALREADY_MINIMIZED ||
//           result == lbfgs::LBFGS_STOP)
//       {
//         //pass
//       }
//       else
//       {
//         ROS_ERROR("Solver error in refining!, return = %d, %s", result, lbfgs::lbfgs_strerror(result));
//       }

//       UniformBspline traj = UniformBspline(cps_.points, 3, bspline_interval_);
//       double tm, tmp;
//       traj.getTimeSpan(tm, tmp);
//       double t_step = (tmp - tm) / ((traj.evaluateDeBoorT(tmp) - traj.evaluateDeBoorT(tm)).norm() / grid_map_->getResolution()); // Step size is defined as the maximum size that can passes throgth every gird.
//       for (double t = tm; t < tmp * 2 / 3; t += t_step)
//       {
//         if (grid_map_->getInflateOccupancy(traj.evaluateDeBoorT(t)))
//         {
//           // cout << "Refined traj hit_obs, t=" << t << " P=" << traj.evaluateDeBoorT(t).transpose() << endl;

//           Eigen::MatrixXd ref_pts(ref_pts_.size(), 3);
//           for (size_t i = 0; i < ref_pts_.size(); i++)
//           {
//             ref_pts.row(i) = ref_pts_[i].transpose();
//           }

//           flag_safe = false;
//           break;
//         }
//       }

//       if (!flag_safe)
//         lambda4_ *= 2;

//       iter_count++;
//     } while (!flag_safe && iter_count <= 0);

//     lambda4_ = origin_lambda4;

//     //cout << "iter_num_=" << iter_num_ << endl;

//     return flag_safe;
//   }

  /* 该函数用于计算当前优化变量的综合代价值 f_combine，并计算对应的梯度 grad。
      它结合了平滑性、距离约束、可行性等多个成本项，通过权重调整，得到最终的优化目标 */
  void BsplineOptimizer::combineCostRebound(const double *x, double *grad, double &f_combine, const int n)
  {
    // 将优化器提供的变量 x 复制到控制点容器 cps_.points 中
    // 控制点的更新从 3 * order_ 开始，跳过固定的前 order_ 个点，因为它们可能是边界条件，不需要优化
    memcpy(cps_.points.data() + 3 * (order_), x, n * sizeof(x[0]));
    // memcpy(cps_.points.data() , x, n * sizeof(x[0]));
    calcMeanDistance(); // 计算控制点之间的平均距离，可能用于归一化或成本项的计算
    /* ---------- evaluate cost and gradient ---------- */
    // f_smoothness: 平滑性成本
    // f_distance: 与障碍物距离相关的成本
    // f_feasibility: 可行性成本，可能与动态或运动学约束有关
    double f_smoothness{}, f_distance{}, f_feasibility{};

    // 定义并初始化三个梯度矩阵，对应三个成本项：
      // g_smoothness: 平滑性成本的梯度。
      // g_distance: 距离成本的梯度。
      // g_feasibility: 可行性成本的梯度。
      // 尺寸为 3 x cps_.size，表示每个控制点在三维空间（x, y, z）的梯度。
    Eigen::MatrixXd g_smoothness = Eigen::MatrixXd::Zero(3, cps_.size);
    Eigen::MatrixXd g_distance = Eigen::MatrixXd::Zero(3, cps_.size);
    Eigen::MatrixXd g_feasibility = Eigen::MatrixXd::Zero(3, cps_.size);

    calcSmoothnessCost(cps_.points, f_smoothness, g_smoothness); // 计算平滑项
    calcDistanceCostRebound(cps_.points, f_distance, g_distance, boxes); // 计算避障约束项
    calcFeasibilityCost(cps_.points, f_feasibility, g_feasibility); // 计算动力学约束项

    /* 计算综合代价 f_combine，通过线性组合三个成本项得到 */
    f_combine = lambda1_ * f_smoothness + new_lambda2_ * f_distance + lambda3_ * f_feasibility;

    /* 综合梯度 grad_3D 的计算 */
    Eigen::MatrixXd grad_3D = lambda1_ * g_smoothness + new_lambda2_ * g_distance + lambda3_ * g_feasibility;

    /* 将梯度 grad_3D 的数据拷贝到优化器提供的梯度数组 grad 中 */
    memcpy(grad, grad_3D.data() + 3 * (order_ ), n * sizeof(grad[0]));
    // memcpy(grad, grad_3D.data() , n * sizeof(grad[0]));
  }

  void BsplineOptimizer::calcMeanDistance( ){
    double average_interval_length_ = 0.0;
    double total_length = 0.0;
    Eigen::Vector2d pre_point = cps_.points.col(0).head(2), cur_point;
    for (int i = 1; i < cps_.points.cols(); ++i) {
      cur_point = cps_.points.col(i).head(2);
      total_length += std::sqrt((pre_point[0] - cur_point[0]) *
                                    (pre_point[0] - cur_point[0]) +
                                (pre_point[1] - cur_point[1]) *
                                    (pre_point[1]  - cur_point[1]));
      pre_point = cur_point;
    }
    average_interval_length_ = total_length / (cps_.points.cols() - 1);
    double interval_sqr = average_interval_length_ * average_interval_length_;
    curvature_constraint_sqr = (interval_sqr * max_curvature_) *
                                  (interval_sqr * max_curvature_);
  }
//   void BsplineOptimizer::combineCostRefine(const double *x, double *grad, double &f_combine, const int n)
//   {

//     memcpy(cps_.points.data() + 3 * order_, x, n * sizeof(x[0]));

//     /* ---------- evaluate cost and gradient ---------- */
//     double f_smoothness, f_fitness, f_feasibility;

//     Eigen::MatrixXd g_smoothness = Eigen::MatrixXd::Zero(3, cps_.points.cols());
//     Eigen::MatrixXd g_fitness = Eigen::MatrixXd::Zero(3, cps_.points.cols());
//     Eigen::MatrixXd g_feasibility = Eigen::MatrixXd::Zero(3, cps_.points.cols());

//     //time_satrt = ros::Time::now();

//     calcSmoothnessCost(cps_.points, f_smoothness, g_smoothness);
//     calcFitnessCost(cps_.points, f_fitness, g_fitness);
//     calcFeasibilityCost(cps_.points, f_feasibility, g_feasibility);

//     /* ---------- convert to solver format...---------- */
//     f_combine = lambda1_ * f_smoothness + lambda4_ * f_fitness + lambda3_ * f_feasibility;
//     // printf("origin %f %f %f %f\n", f_smoothness, f_fitness, f_feasibility, f_combine);

//     Eigen::MatrixXd grad_3D = lambda1_ * g_smoothness + lambda4_ * g_fitness + lambda3_ * g_feasibility;
//     memcpy(grad, grad_3D.data() + 3 * order_, n * sizeof(grad[0]));
//   }

} // namespace opt_planner