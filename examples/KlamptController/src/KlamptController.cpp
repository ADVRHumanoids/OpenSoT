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
    return boost::filesystem::change_extension(urdf_path,"srdf").string();
}

yarp::sig::Vector fromJntToiDyn(iDynUtils& model,
                                const KlamptController::JntPosition &posture)
{
    yarp::sig::Vector q(model.iDyn3_model.getNrOfDOFs());

    for(KlamptController::JntPosition::const_iterator i = posture.begin();
        i != posture.end(); ++i)
    {
        q[model.iDyn3_model.getDOFIndex(i->first)] = i->second;
    }

    return q;
}

KlamptController::JntPosition fromiDynToJnt(iDynUtils& model,
                                        const yarp::sig::Vector &q)
{
    KlamptController::JntPosition posture;
    for(std::vector<std::string>::const_iterator joint =
        model.getJointNames().begin();
        joint != model.getJointNames().end();
        ++joint)
    {
        posture[*joint]=q[model.iDyn3_model.getDOFIndex(*joint)];
    }

    return posture;
}

KlamptController::KlamptController(std::string urdf_path)
: model(getRobotName(urdf_path), urdf_path, getSRDFPath(urdf_path))
{
}

KlamptController::~KlamptController()
{
    ;
}

KlamptController::JntPosition KlamptController::getPosture()
{
    return fromiDynToJnt(model, model.iDyn3_model.getAng());
}


void KlamptController::setPosture(const KlamptController::JntPosition& posture)
{
    model.updateiDyn3Model(fromJntToiDyn(model, posture), true);
}
