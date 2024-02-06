#include "kdl_ros_control/kdl_planner.h"
#include <cmath>

KDLPlanner::KDLPlanner(double _maxVel, double _maxAcc)
{
    velpref_ = new KDL::VelocityProfile_Trap(_maxVel,_maxAcc);
}

KDLPlanner::KDLPlanner(double _trajDuration, double _accDuration, Eigen::Vector3d _trajInit, Eigen::Vector3d _trajEnd)
{
    trajDuration_ = _trajDuration;
    accDuration_ = _accDuration;
    trajInit_ = _trajInit;
    trajEnd_ = _trajEnd;
}

KDLPlanner::KDLPlanner(double _trajDuration, double _accDuration, Eigen::Vector3d _trajInit, Eigen::Vector3d _trajEnd,double _trajRadius)
{
    trajDuration_ = _trajDuration;
    accDuration_ = _accDuration;
    trajInit_ = _trajInit;
    trajEnd_ = _trajEnd;
    trajRadius_ = _trajRadius;
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

void KDLPlanner::trapezoidal_vel(double time, double &s, double &ds,double &dds)
{

  double ddot_c = -1.0/(std::pow(accDuration_,2)-trajDuration_*accDuration_);

  if(time <= accDuration_)
  {
    s = 0.5*ddot_c*std::pow(time,2);
    ds = ddot_c*time;
    dds = ddot_c;
  }
  else if(time <= trajDuration_-accDuration_)
  {
    s = ddot_c*accDuration_*(time-accDuration_/2);
    ds = ddot_c*accDuration_;
    dds = 0;
  }
  else
  {
    s = 1 - 0.5*ddot_c*std::pow(trajDuration_-time,2);
    ds = ddot_c*(trajDuration_-time);
    dds = -ddot_c;
  }

}

void KDLPlanner::cubic_polinomial(double t, double& s, double& ds, double& dds) const {
      
        double a3 = -2/pow(trajDuration_, 3);
        double a2 = 3*pow(trajDuration_, 2); 
        double a1 = 0;
        double a0 = 0;  

      
        s = a3 * std::pow(t, 3) + a2 * std::pow(t, 2) + a1 * t + a0;

       
        ds = 3 * a3 * std::pow(t, 2) + 2 * a2 * t + a1;

     
        dds = 6 * a3 * t + 2 * a2;
    }
    
    
    trajectory_point KDLPlanner::compute_trajectory_trap_linear(double time)
{
    trajectory_point traj;

    // Use trapezoidal_vel to get s, ds, and dds
    double s, ds, dds;
    
    trapezoidal_vel(time, s, ds, dds);

    // Apply linear trajectory equations
    traj.pos = trajInit_ + s*(trajEnd_ - trajInit_);
    

    traj.vel = ds * (trajEnd_ - trajInit_);
   

    traj.acc = dds*(trajEnd_ - trajInit_);
    

    return traj;
}

    trajectory_point KDLPlanner::compute_trajectory_cubic_linear(double time) {
    double s, ds, dds;

    // Use cubic_polinomial to compute s, ds, and dds
    cubic_polinomial(time, s, ds, dds);
    
    trajectory_point traj;
    
    traj.pos = trajInit_ + s*(trajEnd_ - trajInit_);
    

    traj.vel = ds * (trajEnd_ - trajInit_);
   

    traj.acc = dds*(trajEnd_ - trajInit_);
    

    return traj;
}

trajectory_point KDLPlanner::compute_trajectory_trap_circular(double time)
{
    trajectory_point traj;

    // Use trapezoidal_vel to get s, ds, and dds
    double s, ds, dds;
    trapezoidal_vel(time, s, ds, dds);

    // Apply circular trajectory equations
    traj.pos.x() = trajInit_.x();
    traj.pos.y() = trajInit_.y() - trajRadius_ * std::cos(2.0 * M_PI * s);
    traj.pos.z() = trajInit_.z() - trajRadius_ * std::sin(2.0 * M_PI * s);

    traj.vel.x() = 0;
    traj.vel.y() = 2.0 * M_PI * trajRadius_ * ds * std::sin(2.0 * M_PI * s);
    traj.vel.z() = -2.0 * M_PI * trajRadius_ * ds * std::cos(2.0 * M_PI * s);

    traj.acc.x() = 0;
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

    // Apply circular trajectory equations
    traj.pos.x() = trajInit_.x();
    traj.pos.y() = trajInit_.y() - trajRadius_ * std::cos(2.0 * M_PI * s);
    traj.pos.z() = trajInit_.z() - trajRadius_ * std::sin(2.0 * M_PI * s);

    traj.vel.x() = 0;
    traj.vel.y() = 2.0 * M_PI * trajRadius_ * ds * std::sin(2.0 * M_PI * s);
    traj.vel.z() = -2.0 * M_PI * trajRadius_ * ds * std::cos(2.0 * M_PI * s);

    traj.acc.x() = 0;
    traj.acc.y() = (2.0 * M_PI * trajRadius_) * (dds - 2.0 * M_PI * trajRadius_ * ds * std::cos(2.0 * M_PI * s));
    traj.acc.z() = (2.0 * M_PI * trajRadius_) * (2.0 * M_PI * trajRadius_ * ds * std::sin(2.0 * M_PI * s));
    
    return traj;

}
trajectory_point KDLPlanner::compute_trajectory(double time)
{
  /* trapezoidal velocity profile with accDuration_ acceleration time period and trajDuration_ total duration.
     time = current time
     trajDuration_  = final time
     accDuration_   = acceleration time
     trajInit_ = trajectory initial point
     trajEnd_  = trajectory final point */

  trajectory_point traj;

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

}
