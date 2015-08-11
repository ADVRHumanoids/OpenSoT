#include <KlamptController.h>
#include <utils.h>
#include <yarp/sig/all.h>
#include <boost/filesystem.hpp>
#include <boost/filesystem/convenience.hpp>

std::string getRobotName(std::string urdf_path)
{
    return boost::filesystem::path(urdf_path).stem().string();
}

std::string getSRDFPath(std::string urdf_path)
{
    return boost::filesystem::change_extension(urdf_path,"").string();
}

yarp::sig::Vector fromJntToiDyn(iDynUtils& model,
                                const KlamptController::JntPose &pose)
{
    yarp::sig::Vector q(model.iDyn3_model.getNrOfDOFs());

    for(KlamptController::JntPose::iterator i = pose.begin();
        i != pose.end(); ++i)
    {
        q[model.iDyn3_model.getDOFIndex(i->first)] = i->second;
    }

    return q;
}

KlamptController::JntPose fromiDynToJnt(iDynUtils& model,
                                        const yarp::sig::Vector &q)
{
    KlamptController::JntPose pose;
    for(std::vector<std::string>::const_iterator it =
        model.joint_names.begin();
        it != model.joint_names.end();
        ++it)
    {
        pose[*it]=q[model.iDyn3_model.getDOFIndex(it->first)];
    }

    return pose;
}

KlamptController::KlamptController(std::string urdf_path)
: robot(getRobotName(urdf_path), urdf_path, getSRDFPath(srdf_path))
{
}

KlamptController::~KlamptController()
{
    ;
}

KlamptController::JntPose KlamptController::getPose()
{
    return fromiDynToJntPose(model, model.iDyn3_model.getAng());
}


void KlamptController::setPose(const KlamptController::JntPose& pose)
{
    model.updateiDyn3Model(fromJntPoseToiDyn(model, pose), true);
}
