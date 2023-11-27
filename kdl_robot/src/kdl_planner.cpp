#include "kdl_ros_control/kdl_planner.h"

KDLPlanner::KDLPlanner(double _maxVel, double _maxAcc)
{
    velpref_ = new KDL::VelocityProfile_Trap(_maxVel,_maxAcc);
}

/*KDLPlanner::KDLPlanner(double _trajDuration, double _accDuration, Eigen::Vector3d _trajInit, Eigen::Vector3d _trajEnd)
{
    trajDuration_ = _trajDuration;
    accDuration_ = _accDuration;
    trajInit_ = _trajInit;
    trajEnd_ = _trajEnd;
}


KDLPlanner::KDLPlanner(double _trajDuration, Eigen::Vector3d _trajInit, double _trajRadius)
    : trajDuration_(_trajDuration), trajInit_(_trajInit), trajRadius_(_trajRadius) {
    trajInit_ = _trajInit;
    trajRadius_ = _trajRadius;
    trajDuration_ = _trajDuration;
    }*/
    
//linear trajectory with trapezoidal velocity
KDLPlanner::KDLPlanner(double _trajDuration, double _accDuration,
       Eigen::Vector3d _trajInit, Eigen::Vector3d _trajEnd){
        trajDuration_ = _trajDuration;
        accDuration_ = _accDuration;
        trajInit_ = _trajInit;
        trajEnd_ = _trajEnd;
        trajRadius_ = 0;
       }


//linear trajectory with cubic polinomial
KDLPlanner::KDLPlanner(double _trajDuration,
       Eigen::Vector3d _trajInit, Eigen::Vector3d _trajEnd){
        trajDuration_ = _trajDuration;
        accDuration_ = -1;
        trajInit_ = _trajInit;
        trajEnd_ = _trajEnd;
        trajRadius_ = 0;
       }

//circular trajectory with trapezoidal velocity
KDLPlanner::KDLPlanner(double _trajDuration, double _accDuration,
       Eigen::Vector3d _trajInit, double _trajRadius){
        trajDuration_ = _trajDuration;
        accDuration_ = _accDuration;
        trajInit_ = _trajInit;
        trajEnd_ = _trajInit;
        trajRadius_ = _trajRadius;
       }

//circular trajectory with cubic polinomial
KDLPlanner::KDLPlanner(double _trajDuration, 
Eigen::Vector3d _trajInit, double _trajRadius){
        trajDuration_ = _trajDuration;
        accDuration_ = -1;
        trajInit_ = _trajInit;
        trajEnd_ = _trajInit;
        trajRadius_ = _trajRadius;
        }
        
        
void KDLPlanner::trapezoidal_vel(double t, double tc, double &s, double &ds, double &dds)
{
    // s = distance
    // ds = velocity (first derivative)
    // dds == acceleration (second derivative)
    double tf = 2 * tc; // Total time

    if (t <= tc)
    {
        // Acceleration phase
        s = 0.5 * dds * t * t;
        ds = dds * t;
        dds = 1/tc;
    }
    else if (t < (tf - tc))
    {
        // Constant velocity phase
        double t_mid = tc / 2.0;
        s = 0.5 * dds * t_mid * t_mid + dds * t_mid * (t - tc / 2.0);
        ds = dds * t_mid;
        dds = 0.0;
    }
    else if (t <= tf)
    {
        // Deceleration phase
        double t_end = t - (tf - tc);
        s = 0.5 * dds * t_end * t_end + dds * t_end * (tf - tc) / 2.0;
        ds = dds * t_end;
        dds = -1/tc;
    }
    else
    {
        s = 0.0;
        ds = 0.0;
        dds = 0.0;
    }
}

void KDLPlanner::cubic_polinomial(double t, double& s, double& ds, double& dds) const {
      
        double a3 = -2/pow(trajDuration_, 3);
        double a2 = 3/pow(trajDuration_, 2); 
        double a1 = 0;
        double a0 = 0;  

      
        s = a3 * std::pow(t, 3) + a2 * std::pow(t, 2) + a1 * t + a0;

       
        ds = 3 * a3 * std::pow(t, 2) + 2 * a2 * t + a1;

     
        dds = 6 * a3 * t + 2 * a2;
    }

void KDLPlanner::CreateTrajectoryFromFrames(std::vector<KDL::Frame> &_frames,
                                            double _radius, double _eqRadius
                                            )
{
    path_ = new KDL::Path_RoundedComposite(_radius,_eqRadius,new KDL::RotationalInterpolation_SingleAxis());

    for (unsigned int i = 0; i < _frames.size(); i++)
    {
        path_->Add(_frames[i]);
    }
    path_->Finish();

    velpref_->SetProfile(0,path_->PathLength());
    traject_ = new KDL::Trajectory_Segment(path_, velpref_);
}

void KDLPlanner::createCircPath(KDL::Frame &_F_start,
                                KDL::Vector &_V_centre,
                                KDL::Vector& _V_base_p,
                                KDL::Rotation& _R_base_end,
                                double alpha,
                                double eqradius
                                )
{
    KDL::RotationalInterpolation_SingleAxis* otraj;
    otraj = new KDL::RotationalInterpolation_SingleAxis();
    otraj->SetStartEnd(_F_start.M,_R_base_end);
    path_circle_ = new KDL::Path_Circle(_F_start,
                                        _V_centre,
                                        _V_base_p,
                                        _R_base_end,
                                        alpha,
                                        otraj,
                                        eqradius);
    velpref_->SetProfile(0,path_circle_->PathLength());
    traject_ = new KDL::Trajectory_Segment(path_circle_, velpref_);
}

KDL::Trajectory* KDLPlanner::getTrajectory()
{
	return traject_;
}

/*trajectory_point KDLPlanner::compute_trajectory(double time)
{
  /* trapezoidal velocity profile with accDuration_ acceleration time period and trajDuration_ total duration.
     time = current time
     trajDuration_  = final time
     accDuration_   = acceleration time
     trajInit_ = trajectory initial point
     trajEnd_  = trajectory final point */

 /* trajectory_point traj;

  Eigen::Vector3d ddot_traj_c = -1.0/(std::pow(accDuration_,2)-trajDuration_*accDuration_)*(trajEnd_-trajInit_);

  if(time <= accDuration_)
  {
    traj.pos = trajInit_ + 0.5*ddot_traj_c*std::pow(time,2);
    traj.vel = ddot_traj_c*time;
    traj.acc = ddot_traj_c;
  }
  else if(time <= trajDuration_-accDuration_)
  {
    traj.pos = trajInit_ + ddot_traj_c*accDuration_*(time-accDuration_/2);
    traj.vel = ddot_traj_c*accDuration_;
    traj.acc = Eigen::Vector3d::Zero();
  }
  else
  {
    traj.pos = trajEnd_ - 0.5*ddot_traj_c*std::pow(trajDuration_-time,2);
    traj.vel = ddot_traj_c*(trajDuration_-time);
    traj.acc = -ddot_traj_c;
  }

  return traj;

}*/

trajectory_point KDLPlanner::compute_trajectory(double time){

  trajectory_point traj;
  if (trajRadius_ == 0 && accDuration_ < 0 ) {
  	traj = compute_trajectory_cubic_linear(time);
  }
  else if(trajRadius_ == 0 && accDuration_ >= 0){
  	traj = compute_trajectory_trap_linear(time);
  }
  else if (trajRadius_ != 0 && accDuration_ < 0) {
  	traj = compute_trajectory_trap_circular(time);
  }
  else if (trajRadius_ != 0 && accDuration_ >= 0){
  	traj = compute_trajectory_cubic_circular(time);
  }
  else {
    // Constructor error
  }
  return traj;
}

trajectory_point KDLPlanner::compute_trajectory_trap_circular(double time)
{
    trajectory_point traj;

    // Use trapezoidal_vel to get s, ds, and dds
    double s, ds, dds;
    trapezoidal_vel(time, accDuration_, s, ds, dds);

    // Apply circular trajectory equations
    traj.pos.x() = trajInit_.x() + 0.5 * dds * std::pow(time, 2);
    traj.pos.y() = trajInit_.y() - trajRadius_ * std::cos(2.0 * M_PI * s);
    traj.pos.z() = trajInit_.z() - trajRadius_ * std::sin(2.0 * M_PI * s);

    traj.vel.x() = dds * time;
    traj.vel.y() = 2.0 * M_PI * trajRadius_ * ds * std::sin(2.0 * M_PI * s);
    traj.vel.z() = -2.0 * M_PI * trajRadius_ * ds * std::cos(2.0 * M_PI * s);

    traj.acc.x() = dds;
    traj.acc.y() = (2.0 * M_PI * trajRadius_) * (dds - 2.0 * M_PI * trajRadius_ * ds * std::cos(2.0 * M_PI * s));
    traj.acc.z() = (2.0 * M_PI * trajRadius_) * (2.0 * M_PI * trajRadius_ * ds * std::sin(2.0 * M_PI * s));

    return traj;
}


trajectory_point KDLPlanner::compute_trajectory_cubic_circular(double time)
{

    trajectory_point traj;

    // Use cubic_polinomial to get s, ds, and dds
    double s, ds, dds;
    cubic_polinomial(time, s, ds, dds);

    // Map the curvilinear abscissa s to your trajectory
    traj.pos.x() = trajInit_.x();
    traj.pos.y() = trajInit_.y() - trajRadius_ * cos(2.0 * M_PI * s);
    traj.pos.z() = trajInit_.z() - trajRadius_ * sin(2.0 * M_PI * s);

    // Map the derivatives accordingly
    traj.vel.x() = 0.0; // Assuming constant x position
    traj.vel.y() = trajRadius_ * 2.0 * M_PI * ds * sin(2.0 * M_PI * s);
    traj.vel.z() = trajRadius_ * 2.0 * M_PI * ds * cos(2.0 * M_PI * s);

    traj.acc.x() = 0.0; // Assuming constant x position
    traj.acc.y() = trajRadius_ * (4.0 * M_PI * M_PI) * dds * cos(2.0 * M_PI * s);
    traj.acc.z() = -trajRadius_ * (4.0 * M_PI * M_PI) * dds * sin(2.0 * M_PI * s);

    return traj;

}

trajectory_point KDLPlanner::compute_trajectory_trap_linear(double time)
{
    trajectory_point traj;

    // Use trapezoidal_vel to get s, ds, and dds
    double s, ds, dds;
    trapezoidal_vel(time, accDuration_, s, ds, dds);

    // Apply linear trajectory equations
    traj.pos.x() = trajInit_.x() + 0.5 * dds * std::pow(time, 2);
    traj.pos.y() = trajInit_.y() + ds * time;
    traj.pos.z() = trajInit_.z();

    traj.vel.x() = dds * time;
    traj.vel.y() = ds;
    traj.vel.z() = 0.0;

    traj.acc.x() = dds;
    traj.acc.y() = 0.0;
    traj.acc.z() = 0.0;

    return traj;
}

trajectory_point KDLPlanner::compute_trajectory_cubic_linear(double time) {
    double s, ds, dds;

    // Use cubic_polinomial to compute s, ds, and dds
    cubic_polinomial(time, s, ds, dds);

    // Define linear path
    double x = trajInit_[0] + (trajEnd_[0] - trajInit_[0]) * s;
    double y = trajInit_[1] + (trajEnd_[1] - trajInit_[1]) * s;
    double z = trajInit_[2] + (trajEnd_[2] - trajInit_[2]) * s;

    // Fill in the trajectory_point fields
    trajectory_point traj;
    traj.pos = Eigen::Vector3d(x, y, z);
    traj.vel = Eigen::Vector3d(ds * (trajEnd_[0] - trajInit_[0]), ds * (trajEnd_[1] - trajInit_[1]), ds * (trajEnd_[2] - trajInit_[2]));
    traj.acc = Eigen::Vector3d(dds * (trajEnd_[0] - trajInit_[0]), dds * (trajEnd_[1] - trajInit_[1]), dds * (trajEnd_[2] - trajInit_[2]));

    return traj;
}

