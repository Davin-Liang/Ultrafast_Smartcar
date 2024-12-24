#ifndef _BSPLINE_OPTIMIZER_H_
#define _BSPLINE_OPTIMIZER_H_

#include <Eigen/Eigen>
// #include <hybrid_a_star/hybrid_a_star_flow.h>
#include <bspline_opt/uniform_bspline.h>
// #include <plan_env/grid_map.h>
#include <ros/ros.h>
#include <bspline_opt/lbfgs.hpp>
#include <bspline_opt/type.h>
// #include <hybrid_a_star/hybrid_a_star_flow.h>
// Gradient and elasitc band optimization

// Input: a signed distance field and a sequence of points
// Output: the optimized sequence of points
// The format of points: N x 3 matrix, each row is a point
namespace opt_planner
{

  class ControlPoints
  {
  public:
    double clearance;
    int size;
    Eigen::MatrixXd points;
    std::vector<std::vector<Eigen::Vector3d>> base_point; // The point at the start of the direction vector (collision point)
    std::vector<std::vector<Eigen::Vector3d>> direction;  // Direction vector, must be normalized.
    std::vector<bool> flag_temp;                          // A flag that used in many places. Initialize it everytime before using it.
    // std::vector<bool> occupancy;

    void resize(const int size_set)
    {
      size = size_set;

      base_point.clear();
      direction.clear();
      flag_temp.clear();
      // occupancy.clear();

      points.resize(2, size_set);//二维平面
      base_point.resize(size);
      direction.resize(size);
      flag_temp.resize(size);
      // occupancy.resize(size);
    }
  };

  class BsplineOptimizer
  {

  public:
    BsplineOptimizer() {}
    ~BsplineOptimizer();

    /* main API */
    // void setEnvironment(const GridMap::Ptr &env);
    // void setParam(ros::NodeHandle &nh, const shared_ptr<HybridAStarFlow> &obj);
    void setParam(ros::NodeHandle &nh);
    Eigen::MatrixXd BsplineOptimizeTraj(const Eigen::MatrixXd &points, const double &ts,
                                        const int &cost_function, int max_num_id, int max_time_id);

    /* helper function */

    // required inputs
    void setControlPoints(const Eigen::MatrixXd &points);
    void setBsplineInterval(const double &ts);
    void setCostFunction(const int &cost_function);
    void setTerminateCond(const int &max_num_id, const int &max_time_id);
    void setBsplineCorridor(const VecCube& corridor);

    // optional inputs
    void setGuidePath(const vector<Eigen::Vector3d> &guide_pt);
    void setWaypoints(const vector<Eigen::Vector3d> &waypts,
                      const vector<int> &waypt_idx); // N-2 constraints at most

    void optimize();

    Eigen::MatrixXd getControlPoints();

    // AStar::Ptr a_star_;
    std::vector<Eigen::Vector3d> ref_pts_;

    std::vector<std::vector<Eigen::Vector3d>> initControlPoints(Eigen::MatrixXd &init_points, bool flag_first_init = true);
    bool BsplineOptimizeTrajRebound(Eigen::MatrixXd &optimal_points, double ts, const VecCube& corridor); // must be called after initControlPoints()
    bool BsplineOptimizeTrajRefine(const Eigen::MatrixXd &init_points, const double ts, Eigen::MatrixXd &optimal_points);

    inline int getOrder(void) { return order_; }

  private:
    bool flag_ini = true;
    int trajectory_size = 0;
    std::vector<std::vector< std::pair<double, double> >> boxes;
    vector<vector< pair<double, double> >> var_bdk; //xi, xi+1, ...
                                                    //yi, yi+1, ...
    // GridMap::Ptr grid_map_;
    // HybridAStar hybrid_astar_;
    // HybridAStar::Ptr hybrid_astar_ptr_;
    // HybridAStarFlow::Ptr hybrid_astar_flow_;
    enum FORCE_STOP_OPTIMIZE_TYPE
    {
      DONT_STOP,
      STOP_FOR_REBOUND,
      STOP_FOR_ERROR
    } force_stop_type_;

    // main input
    // Eigen::MatrixXd control_points_;     // B-spline control points, N x dim
    double bspline_interval_; // B-spline knot span B样条的节点间隔（knot span），即相邻的样条节点（控制点或参数化点）之间的间隔大小
    Eigen::Vector3d end_pt_;  // end of the trajectory
    // int             dim_;                // dimension of the B-spline
    //
    vector<Eigen::Vector3d> guide_pts_; // geometric guiding path points, N-6
    vector<Eigen::Vector3d> waypoints_; // waypts constraints
    vector<int> waypt_idx_;             // waypts constraints index
                                        //
    int max_num_id_, max_time_id_;      // stopping criteria
    int cost_function_;                 // used to determine objective function
    double start_time_;                 // global time for moving obstacles

    /* optimization parameters */
    int order_;                    // bspline degree
                                  //  B 样条的阶次 阶次越高，样条的曲线越平滑，但计算复杂度也会增加
    double lambda1_;               // jerk smoothness weight 
                                  // 平滑优化中的加加速度（jerk）权重
    double lambda2_, new_lambda2_; // distance weight 
                                  // 距离权重，用于惩罚样条控制点与目标距离的偏离
    double lambda3_;               // feasibility weight 
                                  // 用于惩罚违反约束的情况，如动态限制（速度、加速度）或路径可行性（无碰撞）
    double lambda4_;               // curve fitting
                                  // 曲线拟合权重，用于惩罚样条偏离原始路径的程度 在路径优化中，既希望样条平滑又希望其尽量接近初始路径

    int a;
    //
    double dist0_;             // safe distance
                              // 安全距离，用于确保样条路径与障碍物或其他动态物体之间保持最小距离。
    double max_vel_, max_acc_; // dynamic limits 动态限制参数
    double max_curvature_; //curvature constrain 
                          // 最大曲率约束，用于限制样条的曲率 曲率与轨迹的平滑性和车辆的物理特性（如转向角限制）有关

    int variable_num_;              // optimization variables
    int iter_num_;                  // iteration of the solver
    Eigen::VectorXd best_variable_; //
    double min_cost_;               //

    // int start_id = 0, end_id = 0;

    ControlPoints cps_;
    // for PHR
    double phr_rho_=1.0, phr_gamma_=1.0, phr_beta_=1000.0, phr_xi_=0.1;
    Eigen::VectorXd  mu_;
    double curvature_constraint_sqr = 0.0;
    /* cost function */
    /* calculate each part of cost function with control points q as input */

    static double costFunction(const std::vector<double> &x, std::vector<double> &grad, void *func_data);
    void combineCost(const std::vector<double> &x, vector<double> &grad, double &cost);

    // q contains all control points
    void calcSmoothnessCost(const Eigen::MatrixXd &q, double &cost,
                            Eigen::MatrixXd &gradient, bool falg_use_jerk = true);
    void calcFeasibilityCost(const Eigen::MatrixXd &q, double &cost,
                             Eigen::MatrixXd &gradient);
    void calcDistanceCostRebound(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient, const vector<vector<pair<double, double>>>& boxes);
    void calcFitnessCost(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient);
    bool check_collision_and_rebound(void);

    static int earlyExit(void *func_data, const double *x, const double *g, const double fx, const double xnorm, const double gnorm, const double step, int n, int k, int ls);
    static double costFunctionRebound(void *func_data, const double *x, double *grad, const int n);
    static double costFunctionRefine(void *func_data, const double *x, double *grad, const int n);

    bool rebound_optimize();
    bool refine_optimize();
    void combineCostRebound(const double *x, double *grad, double &f_combine, const int n);
    void combineCostRefine(const double *x, double *grad, double &f_combine, const int n);

    void calcMeanDistance();

    /* for benckmark evaluation only */
  public:
    typedef std::unique_ptr<BsplineOptimizer> Ptr;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

} // namespace opt_planner
#endif