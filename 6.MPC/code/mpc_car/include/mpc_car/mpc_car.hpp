#pragma once
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>

#include <arc_spline/arc_spline.hpp>
#include <deque>
#include <iosqp/iosqp.hpp>

namespace mpc_car {

static constexpr int n = 4;  // state x y phi v
static constexpr int m = 2;  // input a delta
typedef Eigen::Matrix<double, n, n> MatrixA;
typedef Eigen::Matrix<double, n, m> MatrixB;
typedef Eigen::Vector4d VectorG;
typedef Eigen::Vector4d VectorX;
typedef Eigen::Vector2d VectorU;

class MpcCar {
 private:
  ros::NodeHandle nh_;
  ros::Publisher ref_pub_, traj_pub_, traj_delay_pub_;

  double ll_;
  double dt_;
  double rho_;
  int N_;
  double rhoN_;

  double v_max_, a_max_, delta_max_, ddelta_max_;
  double delay_;

  arc_spline::ArcSpline s_;
  double desired_v_;

  osqp::IOSQP qpSolver_;

  std::vector<VectorX> predictState_;
  std::vector<VectorU> predictInput_;
  std::deque<VectorU> historyInput_;
  int history_length_;
  VectorX x0_observe_;

  // x_{k+1} = Ad * x_{k} + Bd * u_k + gd
  MatrixA Ad_;
  MatrixB Bd_;
  VectorG gd_;

  /**
   * osqp interface:
   * minimize     0.5 x^T P_ x + q_^T x
   * subject to   l_ <= A_ x <= u_
   **/
  Eigen::SparseMatrix<double> P_, q_, A_, l_, u_;

  /* *
   *               /  x1  \
   *               |  x2  |
   *  lx_ <=  Cx_  |  x3  |  <= ux_
   *               | ...  |
   *               \  xN  /
   * */
  Eigen::SparseMatrix<double> Cx_, lx_, ux_;  // p, v constrains
  /* *
   *               /  u0  \
   *               |  u1  |
   *  lu_ <=  Cu_  |  u2  |  <= uu_
   *               | ...  |
   *               \ uN-1 /
   * */
  Eigen::SparseMatrix<double> Cu_, lu_, uu_;  // a delta vs constrains
  Eigen::SparseMatrix<double> Qx_;

  void linearization(const double& phi,
                     const double& v,
                     const double& delta) {
    // x_{k+1} = Ad * x_{k} + Bd * u_k + gd
    // TODO: set values to Ad_, Bd_, gd_
    // ...
    // Ad_ << 0,0,-v*sin(phi),cos(phi),
    //        0,0,v*cos(phi),sin(phi),
    //        0,0,0,tan(delta)/ll_,
    //        0,0,0,0;

    // Bd_ << 0,0,
    //        0,0,
    //        0,v/(ll_*cos(delta)*cos(delta)),
    //        1,0;

    // gd_ << v*phi*sin(phi),
    //        -v*phi*cos(phi),
    //        -v*delta/(ll_*cos(delta)*cos(delta)),
    //        0;


    Ad_ << 1,0,-v*sin(phi)*dt_,cos(phi)*dt_,
           0,1,v*cos(phi)*dt_,sin(phi)*dt_,
           0,0,1,tan(delta)*dt_/ll_,
           0,0,0,1;

    Bd_ << 0,0,
           0,0,
           0,v*dt_/(ll_*cos(delta)*cos(delta)),
           1*dt_,0;

    gd_ << v*phi*sin(phi)*dt_,
           -v*phi*cos(phi)*dt_,
           -v*delta*dt_/(ll_*cos(delta)*cos(delta)),
           0;

    return;
  }

  void calLinPoint(const double& s0, double& phi, double& v, double& delta) {
    Eigen::Vector2d dxy = s_(s0, 1);
    Eigen::Vector2d ddxy = s_(s0, 2);
    double dx = dxy.x();
    double dy = dxy.y();
    double ddx = ddxy.x();
    double ddy = ddxy.y();
    double dphi = (ddy * dx - dy * ddx) / (dx * dx + dy * dy);
    phi = atan2(dy, dx);
    v = desired_v_;
    delta = atan2(ll_ * dphi, 1.0);
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

  VectorX compensateDelay(const VectorX& x0) {
    VectorX x0_delay = x0;
    // TODO: compensate delay
    // ...

    double cur_x = x0_delay(0);
    double cur_y = x0_delay(1);
    double cur_phi = x0_delay(2);
    double cur_v = x0_delay(3);

    x0_delay(0) += cur_v*cos(cur_phi)*dt_;
    x0_delay(1) += cur_v*sin(cur_phi)*dt_;

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
    history_length_ = std::ceil(delay_ / dt_);  // 向上取整

    ref_pub_ = nh.advertise<nav_msgs::Path>("reference_path", 1);
    traj_pub_ = nh.advertise<nav_msgs::Path>("traj", 1);
    traj_delay_pub_ = nh.advertise<nav_msgs::Path>("traj_delay", 1);

    // TODO: set initial value of Ad, Bd, gd
    Ad_.setIdentity();  // Ad for instance
    Bd_.setIdentity();  // ???
    gd_.setIdentity();
    // ...
    // set size of sparse matrices
    P_.resize(m * N_, m * N_);
    q_.resize(m * N_, 1);
    Qx_.resize(n * N_, n * N_);
    // stage cost
    Qx_.setIdentity();
    for (int i = 1; i < N_; ++i) {
      Qx_.coeffRef(i * n - 2, i * n - 2) = rho_;
      Qx_.coeffRef(i * n - 1, i * n - 1) = 0;
    }
    Qx_.coeffRef(N_ * n - 4, N_ * n - 4) = rhoN_;
    Qx_.coeffRef(N_ * n - 3, N_ * n - 3) = rhoN_;
    Qx_.coeffRef(N_ * n - 2, N_ * n - 2) = rhoN_ * rho_;
    int n_cons = 4;  // v a delta ddelta
    A_.resize(n_cons * N_, m * N_);
    l_.resize(n_cons * N_, 1);
    u_.resize(n_cons * N_, 1);
    // v constrains
    Cx_.resize(1 * N_, n * N_);
    lx_.resize(1 * N_, 1);
    ux_.resize(1 * N_, 1);
    // a delta constrains
    Cu_.resize(3 * N_, m * N_);
    lu_.resize(3 * N_, 1);
    uu_.resize(3 * N_, 1);
    // set lower and upper boundaries
    for (int i = 0; i < N_; ++i) {
      // TODO: set stage constraints of inputs (a, delta, ddelta)
      // -a_max <= a <= a_max for instance:
      Cu_.coeffRef(i * 3 + 0, i * m + 0) = 1;
      lu_.coeffRef(i * 3 + 0, 0) = -a_max_;
      uu_.coeffRef(i * 3 + 0, 0) = a_max_;

      Cu_.coeffRef(i * 3 + 1, i * m + 1) = 1;
      lu_.coeffRef(i * 3 + 1, 0) = -delta_max_;
      uu_.coeffRef(i * 3 + 1, 0) = delta_max_;

      if (i >= 1) {
        Cu_.coeffRef(i * 3 + 2, i * m + 1) = 1/dt_;
        Cu_.coeffRef(i * 3 + 2, (i-1) * m + 1) = -1/dt_;
        // Cu_.coeffRef(i * 3 + 2, i * m + 1) = 1;
        lu_.coeffRef(i * 3 + 2, 0) = -ddelta_max_;
        uu_.coeffRef(i * 3 + 2, 0) = ddelta_max_;
      } else {
        Cu_.coeffRef(i * 3 + 2, (i+1) * m + 1) = 1/dt_;
        Cu_.coeffRef(i * 3 + 2, (i) * m + 1) = -1/dt_;
        // Cu_.coeffRef(i * 3 + 2, i * m + 1) = 1;
        lu_.coeffRef(i * 3 + 2, 0) = -ddelta_max_;
        uu_.coeffRef(i * 3 + 2, 0) = ddelta_max_;
      }

      
      // ...

      // TODO: set stage constraints of states (v)
      // -v_max <= v <= v_max
      // Cx_.coeffRef( ...
      // lx_.coeffRef( ...
      // ux_.coeffRef( ...

      Cx_.coeffRef(i , i * n + 3) = 1;
      lx_.coeffRef(i, 0) = -v_max_;
      ux_.coeffRef(i, 0) = v_max_;
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

  int solveQP(const VectorX& x0_observe) {
    x0_observe_ = x0_observe;
    historyInput_.pop_front();
    historyInput_.push_back(predictInput_.front());
    lu_.coeffRef(2, 0) = predictInput_.front()(1) - ddelta_max_ * dt_;
    uu_.coeffRef(2, 0) = predictInput_.front()(1) + ddelta_max_ * dt_;
    VectorX x0 = compensateDelay(x0_observe_);
    // set BB, AA, gg
    Eigen::MatrixXd BB, AA, gg;
    BB.setZero(n * N_, m * N_);
    AA.setZero(n * N_, n);
    gg.setZero(n * N_, 1);
    double s0 = s_.findS(x0.head(2));
    double phi, v, delta;
    double last_phi = x0(2);
    Eigen::SparseMatrix<double> qx;
    qx.resize(n * N_, 1);
    for (int i = 0; i < N_; ++i) {
      calLinPoint(s0, phi, v, delta);
      if (phi - last_phi > M_PI) {
        phi -= 2 * M_PI;
      } else if (phi - last_phi < -M_PI) {
        phi += 2 * M_PI;
      }
      last_phi = phi;
      linearization(phi, v, delta);
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
      if (i == 0) {
        BB.block(0, 0, n, m) = Bd_;
        AA.block(0, 0, n, n) = Ad_;
        gg.block(0, 0, n, 1) = gd_;
      } else {
        // TODO: set BB AA gg
        AA.block(i*n, 0, n, n) = Ad_*AA.block((i-1)*n, 0, n, n);
        BB.block(i*n, i*m, n, m) = Bd_;
        for (int j = i - 1; j >= 0; j--) {
          BB.block(i*n, j*m, n, m) = Ad_*BB.block(i*n, (j+1)*m, n, m);
        }
        gg.block(i*n, 0, n, 1) = Ad_*gg.block((i - 1)*n, 0, n, 1)+gd_;
        // ...
      }
      // TODO: set qx
      Eigen::Vector2d xy = s_(s0);  // reference (x_r, y_r)

      // cost function should be represented as follows:
      /* *
       *           /  x1  \T       /  x1  \         /  x1  \
       *           |  x2  |        |  x2  |         |  x2  |
       *  J =  0.5 |  x3  |   Qx_  |  x3  | + qx^T  |  x3  | + const.
       *           | ...  |        | ...  |         | ...  |
       *           \  xN  /        \  xN  /         \  xN  /
       * */

      // qx.coeffRef(...
      // ...
      // for (int i = 1; i < N_; ++i) {
      //   qx.coeffRef(i * n - 2, 0) = rho_;
      //   qx.coeffRef(i * n - 1, 0) = 0;
      // }
      // qx.coeffRef()

      if (i == N_ - 1) {
        qx.coeffRef(i*n,0) = xy(0) * rhoN_;
        qx.coeffRef(i*n + 1,0) = -xy(1) * rhoN_;
        qx.coeffRef(i*n + 2,0) = -phi * rhoN_;
        // qx.coeffRef(i*n + 3,0) = -desired_v_ * rhoN_;
      }else {
        qx.coeffRef(i*n,0) = -xy(0);
        qx.coeffRef(i*n + 1,0) = -xy(1);
        qx.coeffRef(i*n + 2,0) = -phi;
        // qx.coeffRef(i*n + 3,0) = -desired_v_;
      }

      s0 += desired_v_ * dt_;
      s0 = s0 < s_.arcL() ? s0 : s_.arcL();
    }
    Eigen::SparseMatrix<double> BB_sparse = BB.sparseView();
    Eigen::SparseMatrix<double> AA_sparse = AA.sparseView();
    Eigen::SparseMatrix<double> gg_sparse = gg.sparseView();
    Eigen::SparseMatrix<double> x0_sparse = x0.sparseView();

    // state constrants propogate to input constraints using "X = BB * U + AA * x0 + gg"
    /* *
     *               /  x1  \                              /  u0  \
     *               |  x2  |                              |  u1  |
     *  lx_ <=  Cx_  |  x3  |  <= ux_    ==>    lx <=  Cx  |  u2  |  <= ux
     *               | ...  |                              | ...  |
     *               \  xN  /                              \ uN-1 /
     * */
    Eigen::SparseMatrix<double> Cx = Cx_ * BB_sparse;
    Eigen::SparseMatrix<double> lx = lx_ - Cx_ * AA_sparse * x0_sparse - Cx_ * gg_sparse;
    Eigen::SparseMatrix<double> ux = ux_ - Cx_ * AA_sparse * x0_sparse - Cx_ * gg_sparse;

    /* *      / Cx  \       / lx  \       / ux  \
     *   A_ = \ Cu_ /, l_ = \ lu_ /, u_ = \ uu_ /
     * */

    Eigen::SparseMatrix<double> A_T = A_.transpose();
    A_T.middleCols(0, Cx.rows()) = Cx.transpose();
    A_T.middleCols(Cx.rows(), Cu_.rows()) = Cu_.transpose();
    A_ = A_T.transpose();
    for (int i = 0; i < lx.rows(); ++i) {
      l_.coeffRef(i, 0) = lx.coeff(i, 0);
      u_.coeffRef(i, 0) = ux.coeff(i, 0);
    }
    for (int i = 0; i < lu_.rows(); ++i) {
      l_.coeffRef(i + lx.rows(), 0) = lu_.coeff(i, 0);
      u_.coeffRef(i + lx.rows(), 0) = uu_.coeff(i, 0);
    }
    Eigen::SparseMatrix<double> BBT_sparse = BB_sparse.transpose();
    P_ = BBT_sparse * Qx_ * BB_sparse;
    q_ = BBT_sparse * Qx_.transpose() * (AA_sparse * x0_sparse + gg_sparse) + BBT_sparse * qx;
    // osqp
    Eigen::VectorXd q_d = q_.toDense();
    Eigen::VectorXd l_d = l_.toDense();
    Eigen::VectorXd u_d = u_.toDense();
    qpSolver_.setMats(P_, q_d, A_, l_d, u_d);
    qpSolver_.solve();
    int ret = qpSolver_.getStatus();
    if (ret != 1) {
      ROS_ERROR("fail to solve QP!");
      return ret;
    }
    Eigen::VectorXd sol = qpSolver_.getPrimalSol();
    Eigen::MatrixXd solMat = Eigen::Map<const Eigen::MatrixXd>(sol.data(), m, N_);
    Eigen::VectorXd solState = BB * sol + AA * x0 + gg;
    Eigen::MatrixXd predictMat = Eigen::Map<const Eigen::MatrixXd>(solState.data(), n, N_);

    for (int i = 0; i < N_; ++i) {
      predictInput_[i] = solMat.col(i);
      predictState_[i] = predictMat.col(i);
    }
    return ret;
  }

  void getPredictXU(double t, VectorX& state, VectorU& input) {
    if (t <= dt_) {
      state = predictState_.front();
      input = predictInput_.front();
      return;
    }
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
  void visualization() {
    nav_msgs::Path msg;
    msg.header.frame_id = "world";
    msg.header.stamp = ros::Time::now();
    geometry_msgs::PoseStamped p;
    for (double s = 0; s < s_.arcL(); s += 0.01) {
      p.pose.position.x = s_(s).x();
      p.pose.position.y = s_(s).y();
      p.pose.position.z = 0.0;
      msg.poses.push_back(p);
    }
    ref_pub_.publish(msg);
    msg.poses.clear();
    for (int i = 0; i < N_; ++i) {
      p.pose.position.x = predictState_[i](0);
      p.pose.position.y = predictState_[i](1);
      p.pose.position.z = 0.0;
      msg.poses.push_back(p);
    }
    traj_pub_.publish(msg);
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
};

}  // namespace mpc_car