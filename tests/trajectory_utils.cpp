#include "trajectory_utils.h"

using namespace trajectory_utils;

trajectory_generator::trajectory_generator(const double dt, const std::string& base_frame, const std::string& distal_frame):
    _dt(dt),
    _time(0.0),
    _eq_radius(0.01),
    _is_inited(false),
    _base_frame(base_frame),
    _distal_frame(distal_frame)
{
    _trj.reset(new KDL::Trajectory_Composite());
}

trajectory_generator::trajectory_generator(const double dt):
    _dt(dt),
    _time(0.0),
    _eq_radius(0.01),
    _is_inited(false),
    _base_frame(""),
    _distal_frame("")
{
    _trj.reset(new KDL::Trajectory_Composite());
}

void trajectory_generator::resetTrajectory()
{
    _time = 0.0;
    _is_inited = false;
    _trj.reset(new KDL::Trajectory_Composite());
}

bool trajectory_generator::addMinJerkTrj(const std::vector<KDL::Frame> &way_points, const double T)
{
    std::vector<double> Ts;
    for(unsigned int i = 0; i < way_points.size()-1; ++i)
        Ts.push_back(T);
    return addMinJerkTrj(way_points, Ts);
}

bool trajectory_generator::addMinJerkTrj(const KDL::Frame& start, const KDL::Frame& end, const double T)
{
    return addLineTrj(start, end, T, 0.0, 0.0, 0.0, 0.0);
}

bool trajectory_generator::addArcTrj(
               const KDL::Frame &start_pose, const KDL::Rotation &final_rotation,
               const double angle_of_rotation, const KDL::Vector &circle_center,
               const KDL::Vector &plane_normal, const double T,
               const double v0, const double v1, const double a0, const double a1)
{
    boost::shared_ptr<KDL::Path> _path = createArcPath(start_pose, final_rotation,
                                                       angle_of_rotation,
                                                       circle_center, plane_normal);
    boost::shared_ptr<KDL::VelocityProfile> _velocity_profile =
            createSplineVelProfile(_path->PathLength(), T, v0, v1, a0, a1);

    boost::shared_ptr<KDL::Trajectory_Segment> _trj_segment;
    _trj_segment.reset(new KDL::Trajectory_Segment(_path.get()->Clone(),
                                                   _velocity_profile.get()->Clone()));



    _trj->Add(_trj_segment.get()->Clone());

    _is_inited = true;
    _time = 0.0;

    return _is_inited;
}

bool trajectory_generator::addArcTrj(const KDL::Frame &start_pose, const KDL::Rotation &final_rotation,
               const double angle_of_rotation,
               const KDL::Vector &circle_center, const KDL::Vector &plane_normal,
               const double T)
{
    boost::shared_ptr<KDL::Path> _tmp_path = createArcPath(start_pose, final_rotation,
                                                           angle_of_rotation, circle_center,
                                                           plane_normal);
    double max_vel = 0.0;
    double max_acc = 0.0;

    computeMaxVelAndMaxAccForBCB(T, _tmp_path->PathLength(), max_vel, max_acc);

    return addArcTrj(start_pose, final_rotation, angle_of_rotation, circle_center,
                     plane_normal, max_vel, max_acc);
}

bool trajectory_generator::addArcTrj(const KDL::Frame& start_pose, const KDL::Rotation& final_rotation,
                                     const double angle_of_rotation,
                                     const KDL::Vector& circle_center, const KDL::Vector& plane_normal,
                                     const double max_vel, const double max_acc)
{
    boost::shared_ptr<KDL::Path> _path = createArcPath(start_pose, final_rotation, angle_of_rotation,
                                                       circle_center, plane_normal);

    boost::shared_ptr<KDL::VelocityProfile> _velocity_profile;
    if(!checkIfCoastPhaseExists(max_vel, max_acc, _path->PathLength()))
            return false;
    _velocity_profile = createTrapezoidalVelProfile(max_vel, max_acc, _path->PathLength());

    boost::shared_ptr<KDL::Trajectory_Segment> _trj_segment;
    _trj_segment.reset(new KDL::Trajectory_Segment(_path.get()->Clone(),
                                                   _velocity_profile.get()->Clone()));



    _trj->Add(_trj_segment.get()->Clone());

    _is_inited = true;
    _time = 0.0;

    return _is_inited;
}

bool trajectory_generator::addLineTrj(const std::vector<KDL::Frame>& way_points, double T)
{
    std::vector<double> Ts; Ts.reserve(way_points.size()-1);
    for(unsigned int i = 0; i < way_points.size()-1; ++i)
        Ts.push_back(T);
    return addLineTrj(way_points, Ts);
}

bool trajectory_generator::addLineTrj(const std::vector<KDL::Frame>& way_points, const std::vector<double> T)
{
    bool a = true;
    for(unsigned int i = 0; i < way_points.size()-1; ++i)
        a = a && addLineTrj(way_points[i], way_points[i+1], T[i]);
    _is_inited = a;
    return _is_inited;
}

bool trajectory_generator::addLineTrj(const std::vector<KDL::Frame>& way_points,
                 const double max_vel, const double max_acc)
{
    std::vector<double> max_vels; max_vels.reserve(way_points.size()-1);
    std::vector<double> max_accs; max_accs.reserve(way_points.size()-1);
    for(unsigned int i = 0; i < way_points.size()-1; ++i){
        max_vels.push_back(max_vel);
        max_accs.push_back(max_acc);}
    return addLineTrj(way_points, max_vels, max_accs);
}

bool trajectory_generator::addLineTrj(const std::vector<KDL::Frame>& way_points,
                 const std::vector<double> max_vels, const std::vector<double> max_accs)
{
    bool a = true;
    for(unsigned int i = 0; i < way_points.size()-1; ++i)
        a = a && addLineTrj(way_points[i], way_points[i+1], max_vels[i], max_accs[i]);
    _is_inited = a;
    return _is_inited;
}

bool trajectory_generator::addLineTrj(const KDL::Frame& start, const KDL::Frame& end, const double T)
{
    boost::shared_ptr<KDL::Path> _tmp_path = createLinePath(start, end);
    double max_vel = 0.0;
    double max_acc = 0.0;

    computeMaxVelAndMaxAccForBCB(T, _tmp_path->PathLength(), max_vel, max_acc);

    return addLineTrj(start, end, max_vel, max_acc);
}

bool trajectory_generator::addLineTrj(const KDL::Frame& start, const KDL::Frame& end,
                                      const double max_vel, const double max_acc)
{
    boost::shared_ptr<KDL::Path> _path = createLinePath(start, end);

    boost::shared_ptr<KDL::VelocityProfile> _velocity_profile;
    if(!checkIfCoastPhaseExists(max_vel, max_acc, _path->PathLength()))
            return false;
    _velocity_profile = createTrapezoidalVelProfile(max_vel, max_acc, _path->PathLength());

    boost::shared_ptr<KDL::Trajectory_Segment> _trj_segment;
    _trj_segment.reset(new KDL::Trajectory_Segment(_path.get()->Clone(),
                                                   _velocity_profile.get()->Clone()));

    _trj->Add(_trj_segment.get()->Clone());

    _is_inited = true;
    _time = 0.0;

    return _is_inited;
}

bool trajectory_generator::addMinJerkTrj(const std::vector<KDL::Frame> &way_points,
                                         const std::vector<double> Ts)
{
    bool a = true;
    for(unsigned int i = 0; i < way_points.size()-1; ++i)
        a = a && addMinJerkTrj(way_points[i], way_points[i+1], Ts[i]);
    _is_inited = a;
    return _is_inited;
}

bool trajectory_generator::addLineTrj(const KDL::Frame& start, const KDL::Frame& end,
                                      const double T,
                                      const double v0, const double v1,
                                      const double a0, const double a1)
{
    boost::shared_ptr<KDL::Path> _path = createLinePath(start, end);
    boost::shared_ptr<KDL::VelocityProfile> _velocity_profile =
            createSplineVelProfile(_path->PathLength(), T, v0, v1, a0, a1);

    boost::shared_ptr<KDL::Trajectory_Segment> _trj_segment;
    _trj_segment.reset(new KDL::Trajectory_Segment(_path.get()->Clone(),
                                                   _velocity_profile.get()->Clone()));

    _trj->Add(_trj_segment.get()->Clone());

    _is_inited = true;
    _time = 0.0;

    return _is_inited;
}

boost::shared_ptr<KDL::Trajectory_Composite> trajectory_generator::getTrajectory()
{
    return _trj;
}

boost::shared_ptr<KDL::Path> trajectory_generator::createLinePath(const KDL::Frame& start, const KDL::Frame& end)
{
    KDL::Frame _start = start; normalizeQuaternion(_start);
    KDL::Frame _end = end; normalizeQuaternion(_end);

    boost::shared_ptr<KDL::Path_Line> _linear_path;
    _linear_path.reset(new KDL::Path_Line(_start, _end, new KDL::RotationalInterpolation_SingleAxis(), _eq_radius));

    return _linear_path;
}

boost::shared_ptr<KDL::Path> trajectory_generator::createArcPath(const KDL::Frame& start_pose,
                                           const KDL::Rotation& final_rotation,
                                           const double angle_of_rotation,
                                           const KDL::Vector& circle_center,
                                           const KDL::Vector& plane_normal)
{

    KDL::Frame _start_pose = start_pose;
    normalizeQuaternion(_start_pose);
    KDL::Frame _final_rot; _final_rot.Identity();
    _final_rot.M = final_rotation;
    normalizeQuaternion(_final_rot);

    KDL::Vector x(start_pose.p - circle_center);
    double r = x.Normalize();
    KDL::Vector z(plane_normal);
    z.Normalize();
    KDL::Vector tmpv(z*x);
    KDL::Vector V_base_p(tmpv + circle_center);



    boost::shared_ptr<KDL::Path_Circle_Fix> _circle_path;
    _circle_path.reset(new KDL::Path_Circle_Fix(_start_pose, circle_center, V_base_p,
                       _final_rot.M, angle_of_rotation,
                       new KDL::RotationalInterpolation_SingleAxis(), _eq_radius));

    return _circle_path;
}

boost::shared_ptr<KDL::VelocityProfile> trajectory_generator::createTrapezoidalVelProfile(const double max_vel, const double max_acc,
                                                                                          const double L)
{
    boost::shared_ptr<KDL::VelocityProfile_Trap> _velocity_profile;
    _velocity_profile.reset(new KDL::VelocityProfile_Trap(max_vel, max_acc));
    _velocity_profile->SetProfile(0, L);

    return _velocity_profile;
}

boost::shared_ptr<KDL::VelocityProfile> trajectory_generator::createSplineVelProfile(const double L, const double T)
{
    boost::shared_ptr<KDL::VelocityProfile_Spline> _velocity_profile;
    _velocity_profile.reset(new KDL::VelocityProfile_Spline());
    _velocity_profile->SetProfileDuration(0, L, T);

    return _velocity_profile;
}

boost::shared_ptr<KDL::VelocityProfile> trajectory_generator::createSplineVelProfile(const double L, const double T,
                                                                                     const double v0, const double v1)
{
    boost::shared_ptr<KDL::VelocityProfile_Spline> _velocity_profile;
    _velocity_profile.reset(new KDL::VelocityProfile_Spline());
    _velocity_profile->SetProfileDuration(0, v0, L, v1, T);

    return _velocity_profile;
}

boost::shared_ptr<KDL::VelocityProfile> trajectory_generator::createSplineVelProfile(const double L, const double T,
                                                                                     const double v0, const double v1,
                                                                                     const double a0, const double a1)
{
    boost::shared_ptr<KDL::VelocityProfile_Spline> _velocity_profile;
    _velocity_profile.reset(new KDL::VelocityProfile_Spline());


    _velocity_profile->SetProfileDuration(0, v0, L, v1, T);
    _velocity_profile->SetProfileDuration(0, v0, a0, L, v1, a1, T); //<- I hope this is used...


    return _velocity_profile;
}

void trajectory_generator::normalizeQuaternion(KDL::Frame& F)
{
    double x,y,z,w;
    F.M.GetQuaternion(x,y,z,w);

    double d = sqrt(x*x + y*y + z*z + w*w);
    x = x/d;
    y = y/d;
    z = z/d;
    w = w/d;

    F.M = F.M.Quaternion(x, y,z,w);
}

bool trajectory_generator::checkIfCoastPhaseExists(const double max_vel, const double max_acc,
                                                   const double L)
{
    if(L <= (max_vel*max_vel)/max_acc){
        std::cout<<"Too fast trajectory, no Coast phase exists!"<<std::endl;
        return false;
    }
    return true;
}

void trajectory_generator::computeMaxVelAndMaxAccForBCB(const double T, const double L, double &max_vel, double &max_acc)
{
    max_vel = (3.*L)/(2.*T);
    max_acc = (9.*L)/(2.*T*T);
}

KDL::Frame trajectory_generator::Pos(double t){
    return _trj->Pos(t);
}

KDL::Twist trajectory_generator::Vel(double t){
    return _trj->Vel(t);
}

KDL::Twist trajectory_generator::Acc(double t){
    return _trj->Acc(t);
}

KDL::Frame trajectory_generator::Pos(){
    return _trj->Pos(_time);
}

KDL::Twist trajectory_generator::Vel(){
    return _trj->Vel(_time);
}

KDL::Twist trajectory_generator::Acc(){
    return _trj->Acc(_time);
}

void trajectory_generator::updateTrj(){
    _time += _dt;
}

void trajectory_generator::resetInternalTime(){
    _time = 0.0;
}

double trajectory_generator::getTime(){
    return _time;
}

double trajectory_generator::Duration(){
    return _trj->Duration();
}

bool trajectory_generator::isFinished(){
    return _time >= _trj->Duration();
}

bool trajectory_generator::isStarted(){
    return _time > 0.0;
}

bool trajectory_generator::isRunning(){
    return isStarted() && !isFinished();
}

bool trajectory_generator::isInited(){
    return _is_inited;
}

