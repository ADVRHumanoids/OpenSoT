#include <OpenSoT/utils/collision_utils.h>

#if ROS_VERSION_MINOR <= 12
#define STATIC_POINTER_CAST boost::static_pointer_cast
#define DYNAMIC_POINTER_CAST boost::dynamic_pointer_cast
#define SHARED_PTR boost::shared_ptr
#define MAKE_SHARED boost::make_shared
#else
#define STATIC_POINTER_CAST std::static_pointer_cast
#define DYNAMIC_POINTER_CAST std::dynamic_pointer_cast
#define SHARED_PTR std::shared_ptr
#define MAKE_SHARED std::make_shared
#endif

bool ComputeLinksDistance::globalToLinkCoordinates(const std::string& linkName,
                                                   const fcl::Transform3<double> &fcl_w_T_f,
                                                   KDL::Frame &link_T_f )
{

    fcl::Transform3<double> fcl_w_T_shape = collision_objects_[linkName]->getTransform();

    fcl::Transform3<double> fcl_shape_T_f = fcl_w_T_shape.inverse() *fcl_w_T_f;

    link_T_f = link_T_shape[linkName] * fcl2KDL ( fcl_shape_T_f );

    return true;
}

bool ComputeLinksDistance::shapeToLinkCoordinates(const std::string& linkName,
                                                  const fcl::Transform3<double> &fcl_shape_T_f,
                                                  KDL::Frame &link_T_f )
{

    link_T_f = link_T_shape[linkName] * fcl2KDL ( fcl_shape_T_f );

    return true;
}

bool ComputeLinksDistance::parseCollisionObjects ()
{
    std::vector<urdf::LinkSharedPtr> links;
    robot_urdf.getLinks(links);

    for(auto link : links)
    {
        if(!link->collision)
        {
            std::cout << "Collision not defined for link " << link->name << std::endl;
            continue;
        }

        if(link->collision->geometry->type != urdf::Geometry::CYLINDER &&
                link->collision->geometry->type != urdf::Geometry::SPHERE &&
                link->collision->geometry->type != urdf::Geometry::BOX &&
                link->collision->geometry->type != urdf::Geometry::MESH )
        {
            std::cout << "Collision type unknown for link " << link->name << std::endl;
            continue;
        }

        SHARED_PTR<fcl::CollisionGeometry<double>> shape;
        KDL::Frame shape_origin;

        if(link->collision->geometry->type == urdf::Geometry::CYLINDER)
        {
            std::cout << "adding capsule for " << link->name << std::endl;

            auto collisionGeometry =
                    DYNAMIC_POINTER_CAST<urdf::Cylinder>(link->collision->geometry);

            shape = MAKE_SHARED<fcl::Capsule<double>>(collisionGeometry->radius,
                                                      collisionGeometry->length);

            shape_origin = toKdl(link->collision->origin);

            shape_origin.p -= collisionGeometry->length/2.0 * shape_origin.M.UnitZ();

            custom_capsules_[link->name] =
                    boost::make_shared<ComputeLinksDistance::Capsule>(
                        shape_origin,
                        collisionGeometry->radius,
                        collisionGeometry->length
                        );

        }
        else if(link->collision->geometry->type == urdf::Geometry::SPHERE)
        {
            std::cout << "adding sphere for " << link->name << std::endl;

            auto collisionGeometry =
                    DYNAMIC_POINTER_CAST<urdf::Sphere>(link->collision->geometry);

            shape = MAKE_SHARED<fcl::Sphere<double>>(collisionGeometry->radius);
            shape_origin = toKdl ( link->collision->origin );
        }
        else if ( link->collision->geometry->type == urdf::Geometry::BOX )
        {
            std::cout << "adding box for " << link->name << std::endl;

            auto collisionGeometry =
                    DYNAMIC_POINTER_CAST<urdf::Box>(link->collision->geometry);

            shape = MAKE_SHARED<fcl::Box<double>>(collisionGeometry->dim.x,
                                                  collisionGeometry->dim.y,
                                                  collisionGeometry->dim.z);

            shape_origin = toKdl(link->collision->origin);

            std::cout << "Box has size " << collisionGeometry->dim.x <<
                         ", " << collisionGeometry->dim.y <<
                         ", " << collisionGeometry->dim.z << std::endl;

        }
        else if(link->collision->geometry->type == urdf::Geometry::MESH)
        {
            std::cout << "adding mesh for " << link->name << std::endl;

            auto collisionGeometry =
                    DYNAMIC_POINTER_CAST<urdf::Mesh>(link->collision->geometry);

            auto mesh = shapes::createMeshFromResource(collisionGeometry->filename);

            if(!mesh)
            {
                std::cout << "Error loading mesh for link " << link->name << std::endl;
                continue;
            }

            std::vector<fcl::Vector3<double>> vertices;
            std::vector<fcl::Triangle> triangles;

            for(unsigned int i = 0; i < mesh->vertex_count; ++i)
            {
                fcl::Vector3<double> v(mesh->vertices[3*i]*collisionGeometry->scale.x,
                        mesh->vertices[3*i + 1]*collisionGeometry->scale.y,
                        mesh->vertices[3*i + 2]*collisionGeometry->scale.z);

                vertices.push_back(v);
            }

            for(unsigned int i = 0; i< mesh->triangle_count; ++i)
            {
                fcl::Triangle t(mesh->triangles[3*i],
                        mesh->triangles[3*i + 1],
                        mesh->triangles[3*i + 2]);

                triangles.push_back(t);
            }

            // add the mesh data into the BVHModel structure
            auto bvhModel = MAKE_SHARED<fcl::BVHModel<fcl::OBBRSS<double>>>();
            shape = bvhModel;
            bvhModel->beginModel();
            bvhModel->addSubModel(vertices, triangles);
            bvhModel->endModel();

            shape_origin = toKdl ( link->collision->origin );
        }

        auto collision_object = boost::make_shared<fcl::CollisionObject<double>>(shape);

        collision_objects_[link->name] = collision_object;
        shapes_[link->name] = shape;

        /* Store the transformation of the CollisionShape from URDF
         * that is, we store link_T_shape for the actual link */
        link_T_shape[link->name] = shape_origin;

    }

    return true;
}

bool ComputeLinksDistance::updateCollisionObjects()
{
    for(auto link_name : linksToUpdate)
    {
        // link pose
        KDL::Frame w_T_link, w_T_shape;
        model.getPose(link_name, w_T_link);

        // shape pose
        w_T_shape = w_T_link * link_T_shape[link_name];

        // set pose to fcl shape
        fcl::Transform3<double> fcl_w_T_shape = KDL2fcl(w_T_shape);
        fcl::CollisionObject<double>* collObj_shape = collision_objects_[link_name].get();
        collObj_shape->setTransform(fcl_w_T_shape);
    }

    return true;
}


fcl::Transform3<double> ComputeLinksDistance::KDL2fcl(const KDL::Frame& in)
{
    fcl::Transform3<double> out;
    double x, y, z, w;
    in.M.GetQuaternion(x, y, z, w);
    fcl::Quaternion<double> q(w, x, y, z);
    out.translation() << in.p[0], in.p[1], in.p[2];
    out.linear() = q.toRotationMatrix();
    return out;
}

KDL::Frame ComputeLinksDistance::fcl2KDL(const fcl::Transform3<double>& in)
{

    fcl::Quaternion<double> q (in.linear());
    fcl::Vector3<double> t = in.translation();

    KDL::Frame f;
    f.p = KDL::Vector (t[0], t[1], t[2]);
    f.M = KDL::Rotation::Quaternion(q.x(), q.y(), q.z(), q.w());

    return f;
}

void ComputeLinksDistance::generateLinksToUpdate()
{
    // get all link pairs that can possibly collide
    std::vector<std::string> entries;
    allowed_collision_matrix->getAllEntryNames(entries);

    linksToUpdate.clear();

    // take all link pairs that (i) can possibly collide, (ii) are not supposed
    // to collide
    for(auto it_A = entries.begin(); it_A != entries.end(); ++it_A)
    {
        for(auto it_B = it_A + 1; it_B != entries.end(); ++it_B)
        {
            collision_detection::AllowedCollision::Type collisionType;

            // check if collision of this AB pair is never supposed to happpen
            if(allowed_collision_matrix->getAllowedCollision(*it_A, *it_B, collisionType) &&
                 collisionType == collision_detection::AllowedCollision::NEVER)
            {
                linksToUpdate.insert(*it_A);
                linksToUpdate.insert(*it_B);
            }
        }
    }
}

void ComputeLinksDistance::generatePairsToCheck()
{
    pairsToCheck.clear();

    std::vector<std::string> entries;
    allowed_collision_matrix->getAllEntryNames(entries);

    for(auto it_A = entries.begin(); it_A != entries.end(); ++it_A)
    {
        for(auto it_B = it_A + 1; it_B != entries.end(); ++it_B)
        {
            collision_detection::AllowedCollision::Type collisionType;

            if(allowed_collision_matrix->getAllowedCollision(*it_A, *it_B, collisionType ) &&
                 collisionType == collision_detection::AllowedCollision::NEVER)
            {
                pairsToCheck.emplace_back(this, *it_A, *it_B);
            }
        }
    }

    std::cout << "Checking " << pairsToCheck.size() << " pairs for collision" << std::endl;
}

ComputeLinksDistance::ComputeLinksDistance(XBot::ModelInterface& _model):
    model(_model)
{
    // construct moveit's robot model
    auto urdf_model_ptr = MAKE_SHARED<urdf::Model>();
    urdf_model_ptr->initString(model.getUrdfString());

    auto srdf_model_ptr = MAKE_SHARED<srdf::Model>();
    srdf_model_ptr->initString(*urdf_model_ptr, model.getSrdfString());

    moveit_robot_model = MAKE_SHARED<robot_model::RobotModel>(urdf_model_ptr, srdf_model_ptr);

    // check if _capsules urdf files exist
    boost::filesystem::path original_urdf(model.getUrdfPath());
    std::string capsule_model_urdf_filename = original_urdf.stem().c_str() +
                                              std::string("_capsules.urdf");
    boost::filesystem::path capsule_urdf(original_urdf.parent_path() /
                                         capsule_model_urdf_filename );

    boost::filesystem::path original_srdf(model.getSrdfPath());
    std::string capsule_model_srdf_filename = original_srdf.stem().c_str() +
                                              std::string("_capsules.srdf");
    boost::filesystem::path capsule_srdf(original_srdf.parent_path() /
                                         capsule_model_srdf_filename );

    // load the proper files
    std::string urdf_to_load, srdf_to_load;

    if(boost::filesystem::exists(capsule_urdf))
    {
        urdf_to_load = capsule_urdf.c_str();
    }
    else
    {
        urdf_to_load = original_urdf.c_str();
    }

    if(boost::filesystem::exists(capsule_srdf))
    {
        srdf_to_load = capsule_srdf.c_str();
    }
    else
    {
        srdf_to_load = original_srdf.c_str();
    }

    if(!urdf_to_load.empty())
    {
        std::cout << "urdf_to_load: " << urdf_to_load << std::endl;
        robot_urdf.initFile(urdf_to_load);
    }
    else
    {
        robot_urdf.initString(model.getUrdfString());
    }

    if(!srdf_to_load.empty())
    {
        std::cout << "srdf_to_load: " << srdf_to_load << std::endl;
        robot_srdf.initFile(robot_urdf, srdf_to_load);
    }
    else
    {
        robot_srdf.initString(robot_urdf, model.getSrdfString());
    }

    parseCollisionObjects();

    setCollisionBlackList({});
}

std::list<LinkPairDistance> ComputeLinksDistance::getLinkDistances(double detectionThreshold)
{
    std::list<LinkPairDistance> results;

    // set transforms to all shapes given model state
    updateCollisionObjects();

    for(auto pair : pairsToCheck)
    {
        std::string linkA = pair.linkA;
        std::string linkB = pair.linkB;

        fcl::CollisionObject<double>* collObj_shapeA = pair.collisionObjectA.get();
        fcl::CollisionObject<double>* collObj_shapeB = pair.collisionObjectB.get();

        fcl::DistanceRequest<double> request;
#if FCL_MINOR_VERSION > 2
        request.gjk_solver_type = fcl::GST_INDEP;
#endif
        request.enable_nearest_points = true;

        // result will be returned via the collision result structure
        fcl::DistanceResult<double> result;

        // perform distance test
        fcl::distance(collObj_shapeA, collObj_shapeB, request, result);

        // nearest points must be transformed to world frame
        KDL::Frame linkA_pA, linkB_pB;

        fcl::Transform3<double> world_pA, world_pB;
        world_pA.linear().setIdentity();
        world_pA.translation() = result.nearest_points[0];
        world_pB.linear().setIdentity();
        world_pB.translation() = result.nearest_points[1];

        globalToLinkCoordinates(linkA, world_pA, linkA_pA);
        globalToLinkCoordinates(linkB, world_pB, linkB_pB);

        // if distance is below the threshold, add to result
        if(result.min_distance < detectionThreshold)
        {
            results.emplace_back(linkA, linkB, linkA_pA, linkB_pB, result.min_distance);
        }
    }

    results.sort();

    return results;
}

bool ComputeLinksDistance::setCollisionWhiteList(std::list<LinkPairDistance::LinksPair> whiteList)
{
    allowed_collision_matrix = MAKE_SHARED<collision_detection::AllowedCollisionMatrix>(
                    moveit_robot_model->getLinkModelNamesWithCollisionGeometry(), true);

    // iterate over whit list
    for(auto it = whiteList.begin(); it != whiteList.end(); ++it)
    {
        // check pair exists and has collision info associated with it
        if(collision_objects_.count(it->first) > 0 &&
              collision_objects_.count(it->second) > 0 )
        {
            // set collision pair to 'not allowed', i.e. it will always be checked
            allowed_collision_matrix->setEntry(it->first, it->second, false);
        }
        else // print error
        {

            std::string link_not_found;

            if(collision_objects_.count(it->first) == 0)
            {
                link_not_found = it->first;
            }

            std::cout << "Error: could not find link " << it->first << " specified in whitelist, "
                      << "or link does not have collision geometry information" << std::endl;

            if(collision_objects_.count(it->second) == 0)
            {
                if(link_not_found == "")
                {
                    link_not_found = it->second;
                }
                else
                {
                    link_not_found += " , " + it->second;
                }
            }

            std::cout << "Error: could not find link " << link_not_found << " specified in whitelist, "
                      << "or link does not have collision geometry information" << std::endl;
        }
    }

    loadDisabledCollisionsFromSRDF ( this->robot_srdf, allowed_collision_matrix );

    generateLinksToUpdate();
    generatePairsToCheck();

    //allowed_collision_matrix->print(std::cout);
    return true;
}

bool ComputeLinksDistance::setCollisionBlackList(std::list<LinkPairDistance::LinksPair> blackList)
{
    allowed_collision_matrix = MAKE_SHARED<collision_detection::AllowedCollisionMatrix>(
                    moveit_robot_model->getLinkModelNamesWithCollisionGeometry(), true);

    std::vector<std::string> linksWithCollisionObjects;

    for(auto pair : collision_objects_)
    {
        linksWithCollisionObjects.push_back(pair.first);
    }

    // set all pairs to not allowed (all are checked)
    allowed_collision_matrix->setEntry(linksWithCollisionObjects, linksWithCollisionObjects, false);

    // don't check pairs from black list
    for(auto pair : blackList)
    {
        allowed_collision_matrix->setEntry(pair.first, pair.second, true);
    }

    // don't check disabled pairs from srdf
    loadDisabledCollisionsFromSRDF(model.getSrdf(), allowed_collision_matrix);

    generateLinksToUpdate();
    generatePairsToCheck();

    return true;
}

void ComputeLinksDistance::loadDisabledCollisionsFromSRDF(const srdf_advr::Model& srdf,
                                                          collision_detection::AllowedCollisionMatrixPtr acm )
{
    for(const auto& dc : srdf.getDisabledCollisionPairs())
    {
        acm->setEntry(dc.link1_, dc.link2_, true);
    }
}



LinkPairDistance::LinkPairDistance(const std::string &link1, const std::string &link2,
                                     const KDL::Frame &link1_T_closestPoint1,
                                     const KDL::Frame &link2_T_closestPoint2,
                                     const double &distance ) :
    linksPair(link1, link2),
    link_T_closestPoint(link1_T_closestPoint1, link2_T_closestPoint2),
    distance(distance)
{

}

const double &LinkPairDistance::getDistance() const
{
    return distance;
}

const std::pair<KDL::Frame, KDL::Frame> &LinkPairDistance::getLink_T_closestPoint() const
{
    return link_T_closestPoint;
}

const std::pair<std::string, std::string> &LinkPairDistance::getLinkNames() const
{
    return linksPair;
}

bool LinkPairDistance::operator<(const LinkPairDistance& second) const
{
    if(distance < second.distance)
    {
        return true;
    }
    else
    {
        return linksPair.first < second.linksPair.first;
    }
}
