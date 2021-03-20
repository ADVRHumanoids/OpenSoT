#include <OpenSoT/utils/collision_utils.h>
#include <octomap_msgs/conversions.h>
#include <eigen_conversions/eigen_msg.h>

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

    fcl::Transform3<double> fcl_w_T_shape = _collision_obj[linkName]->getTransform();

    fcl::Transform3<double> fcl_shape_T_f = fcl_w_T_shape.inverse() *fcl_w_T_f;

    link_T_f = _link_T_shape[linkName] * fcl2KDL ( fcl_shape_T_f );

    return true;
}

bool ComputeLinksDistance::shapeToLinkCoordinates(const std::string& linkName,
                                                  const fcl::Transform3<double> &fcl_shape_T_f,
                                                  KDL::Frame &link_T_f )
{

    link_T_f = _link_T_shape[linkName] * fcl2KDL ( fcl_shape_T_f );

    return true;
}

namespace
{
/**
 * @brief capsule_from_collision checks if the link collision is
 * formed by a cylinder and two spheres, and it returns the cylinder
 * collision shared pointer if that is true (nullptr otherwise)
 */
urdf::CollisionConstSharedPtr capsule_from_collision(urdf::Link& l)
{
    if(l.collision_array.size() != 3)
    {
        return nullptr;
    }

    int num_spheres = 0;
    urdf::CollisionConstSharedPtr cylinder;

    for(auto c : l.collision_array)
    {
        if(c->geometry->type == urdf::Geometry::CYLINDER)
        {
            cylinder = c;
        }
        else if(c->geometry->type == urdf::Geometry::SPHERE)
        {
            num_spheres++;
        }
    }

    if(cylinder && num_spheres == 2)
    {
        return cylinder;
    }

    return nullptr;
}
}

bool ComputeLinksDistance::parseCollisionObjects()
{
    // get urdf links
    std::vector<urdf::LinkSharedPtr> links;
    _urdf->getLinks(links);

    // loop over links
    for(auto link : links)
    {
        // no collision defined, skip
        if(!link->collision)
        {
            std::cout << "Collision not defined for link " << link->name << std::endl;
            continue;
        }

        // convert urdf collision to fcl shape
        std::shared_ptr<fcl::CollisionGeometryd> shape;
        KDL::Frame shape_origin;

        if(auto cylinder = capsule_from_collision(*link))
        {
            std::cout << "adding capsule for " << link->name << std::endl;

            auto collisionGeometry =
                    DYNAMIC_POINTER_CAST<urdf::Cylinder>(cylinder->geometry);

            shape = std::make_shared<fcl::Capsuled>(collisionGeometry->radius,
                                                    collisionGeometry->length);

            shape_origin = toKdl(cylinder->origin);

            // note: why this? it looks wrong from simulations..
            //shape_origin.p -= collisionGeometry->length/2.0 * shape_origin.M.UnitZ();
            //shape_origin.p += collisionGeometry->length/2.0 * shape_origin.M.UnitZ();
            //shape_origin.p += (collisionGeometry->length/2.0+collisionGeometry->radius) * shape_origin.M.UnitZ();

            _custom_capsules[link->name] =
                                           boost::make_shared<ComputeLinksDistance::Capsule>(
                                               shape_origin,
                                               collisionGeometry->radius,
                                               collisionGeometry->length
                                               );



        }
        else if(link->collision->geometry->type == urdf::Geometry::CYLINDER)
        {
            std::cout << "adding cylinder for " << link->name << std::endl;

            auto collisionGeometry =
                    DYNAMIC_POINTER_CAST<urdf::Cylinder>(link->collision->geometry);

            shape = std::make_shared<fcl::Cylinderd>(collisionGeometry->radius,
                                                     collisionGeometry->length);

            shape_origin = toKdl(link->collision->origin);

            // note: check following line (for capsules it looks to
            // generate wrong results)
            // shape_origin.p -= collisionGeometry->length/2.0 * shape_origin.M.UnitZ();

        }
        else if(link->collision->geometry->type == urdf::Geometry::SPHERE)
        {
            std::cout << "adding sphere for " << link->name << std::endl;

            auto collisionGeometry =
                    DYNAMIC_POINTER_CAST<urdf::Sphere>(link->collision->geometry);

            shape = std::make_shared<fcl::Sphered>(collisionGeometry->radius);
            shape_origin = toKdl(link->collision->origin);
        }
        else if ( link->collision->geometry->type == urdf::Geometry::BOX )
        {
            std::cout << "adding box for " << link->name << std::endl;

            auto collisionGeometry =
                    DYNAMIC_POINTER_CAST<urdf::Box>(link->collision->geometry);

            shape = std::make_shared<fcl::Boxd>(collisionGeometry->dim.x,
                                                collisionGeometry->dim.y,
                                                collisionGeometry->dim.z);

            shape_origin = toKdl(link->collision->origin);

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

            std::vector<fcl::Vector3d> vertices;
            std::vector<fcl::Triangle> triangles;

            for(unsigned int i = 0; i < mesh->vertex_count; ++i)
            {
                fcl::Vector3d v(mesh->vertices[3*i]*collisionGeometry->scale.x,
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
            auto bvhModel = std::make_shared<fcl::BVHModel<fcl::OBBRSSd>>();
            shape = bvhModel;
            bvhModel->beginModel();
            bvhModel->addSubModel(vertices, triangles);
            bvhModel->endModel();

            shape_origin = toKdl(link->collision->origin);
        }


        if(!shape)
        {
            std::cout << "Collision type unknown for link " << link->name << std::endl;
            continue;
        }

        auto collision_object = boost::make_shared<fcl::CollisionObjectd>(shape);

        _collision_obj[link->name] = collision_object;

        /* Store the transformation of the CollisionShape from URDF
         * that is, we store link_T_shape for the actual link */
        _link_T_shape[link->name] = shape_origin;

        // add link name to list of links colliding with environment
        _links_vs_environment.insert(link->name);

    }

    return true;
}

bool ComputeLinksDistance::updateCollisionObjects()
{
    _tmp_links_list.clear();

    _tmp_links_list.insert(_links_to_update.begin(), _links_to_update.end());
    _tmp_links_list.insert(_links_vs_environment.begin(), _links_vs_environment.end());

    for(auto link_name : _tmp_links_list)
    {
        // link pose
        KDL::Frame w_T_link, w_T_shape;
        _model.getPose(link_name, w_T_link);

        // shape pose
        w_T_shape = w_T_link * _link_T_shape.at(link_name);

        // set pose to fcl shape
        fcl::Transform3<double> fcl_w_T_shape = KDL2fcl(w_T_shape);
        fcl::CollisionObject<double>* collObj_shape = _collision_obj[link_name].get();
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
    _acm->getAllEntryNames(entries);

    _links_to_update.clear();

    // take all link pairs that (i) can possibly collide, (ii) are not supposed
    // to collide
    for(auto it_A = entries.begin(); it_A != entries.end(); ++it_A)
    {
        for(auto it_B = it_A + 1; it_B != entries.end(); ++it_B)
        {
            collision_detection::AllowedCollision::Type collisionType;

            // check if collision of this AB pair is never supposed to happpen
            if(_acm->getAllowedCollision(*it_A,
                                         *it_B,
                                         collisionType) &&
                    collisionType == collision_detection::AllowedCollision::NEVER)
            {
                _links_to_update.insert(*it_A);
                _links_to_update.insert(*it_B);
            }
        }
    }
}

void ComputeLinksDistance::generatePairsToCheck()
{
    _pairs_to_check.clear();

    // get all link pairs that can possibly collide
    std::vector<std::string> entries;
    _acm->getAllEntryNames(entries);

    // take all link pairs that (i) can possibly collide, (ii) are not supposed
    // to collide
    for(auto it_A = entries.begin(); it_A != entries.end(); ++it_A)
    {
        for(auto it_B = it_A + 1; it_B != entries.end(); ++it_B)
        {
            collision_detection::AllowedCollision::Type collisionType;

            if(_acm->getAllowedCollision(*it_A,
                                         *it_B,
                                         collisionType) &&
                    collisionType == collision_detection::AllowedCollision::NEVER)
            {
                _pairs_to_check.emplace_back(this, *it_A, *it_B);
            }
        }
    }

    // now, add all link-environment pairs
    for(auto envobj : _env_obj_names)
    {
        for(auto linkobj: _links_vs_environment)
        {
            // note: env always as second entry!
            _pairs_to_check.emplace_back(this, linkobj, envobj);

        }
    }

    std::cout << "Checking " << _pairs_to_check.size() << " pairs for collision" << std::endl;
}

ComputeLinksDistance::ComputeLinksDistance(const XBot::ModelInterface& _model,
                                           urdf::ModelConstSharedPtr collision_urdf,
                                           srdf::ModelConstSharedPtr collision_srdf):
    _model(_model)
{
    // user-provided urdf to override collision information
    if(collision_urdf)
    {
        _urdf = MAKE_SHARED<urdf::Model>(*collision_urdf);
    }
    else  // none was given, use default from model ifc
    {
        _urdf = MAKE_SHARED<urdf::Model>();
        _urdf->initString(_model.getUrdfString());
    }

    // user-provided srdf to override acm information
    if(collision_srdf)
    {
        _srdf = MAKE_SHARED<srdf::Model>(*collision_srdf);
    }
    else  // none was given, use default from model ifc
    {
        _srdf = MAKE_SHARED<srdf::Model>();
        _srdf->initString(*_urdf, _model.getSrdfString());
    }

    _moveit_model = std::make_shared<robot_model::RobotModel>(_urdf,
                                                              _srdf);

    parseCollisionObjects();

    setCollisionBlackList({});
}

std::list<LinkPairDistance> ComputeLinksDistance::getLinkDistances(double detectionThreshold)
{
    // return value
    std::list<LinkPairDistance> results;

    // set transforms to all shapes given model state
    updateCollisionObjects();

    // loop over pairs to check
    for(auto pair : _pairs_to_check)
    {
        // object names
        // note: could be either link names or world objects
        std::string linkA = pair.linkA;
        std::string linkB = pair.linkB;

        // fcl collisions
        auto coll_A = pair.collisionObjectA.get();
        auto coll_B = pair.collisionObjectB.get();

        // if distance between bounding spheres (easy to compute)
        // is too large, skip
        auto c_A = coll_A->getTranslation();
        auto c_B = coll_B->getTranslation();
        double r_A = coll_A->collisionGeometry()->aabb_radius;
        double r_B = coll_B->collisionGeometry()->aabb_radius;
        double bounding_sphere_dist = (c_A - c_B).norm() - r_A - r_B;

        if(bounding_sphere_dist > detectionThreshold)
        {
            continue;
        }

        // set request for distance computation
        fcl::DistanceRequest<double> request;
#if FCL_MINOR_VERSION > 2
        request.gjk_solver_type = fcl::GST_INDEP; // fcl::GST_LIBCCD;
#endif
        request.enable_nearest_points = true;

        // result will be returned via the collision result structure
        fcl::DistanceResult<double> result;

        // perform distance test
        fcl::distance(coll_A, coll_B, request, result);

        // we return nearest points
        fcl::Transform3<double> world_pA;
        world_pA.linear().setIdentity();
        world_pA.translation() = result.nearest_points[0];

        fcl::Transform3<double> world_pB;
        world_pB.linear().setIdentity();
        world_pB.translation() = result.nearest_points[1];

        // if distance is below the threshold, add to result
        if(result.min_distance < detectionThreshold)
        {
            results.emplace_back(linkA, linkB,
                                 fcl2KDL(world_pA), fcl2KDL(world_pB),
                                 result.min_distance);
        }
    }

    results.sort();

    return results;
}

bool ComputeLinksDistance::setCollisionWhiteList(std::list<LinkPairDistance::LinksPair> whiteList)
{
    _acm = std::make_shared<collision_detection::AllowedCollisionMatrix>(
               _moveit_model->getLinkModelNamesWithCollisionGeometry(), true);

    // iterate over white list
    for(auto it = whiteList.begin(); it != whiteList.end(); ++it)
    {
        // check pair exists and has collision info associated with it
        if(_collision_obj.count(it->first) > 0 &&
                _collision_obj.count(it->second) > 0 )
        {
            // set collision pair to 'not allowed', i.e. it will always be checked
            std::cout << "will check collision " << it->first << " - " << it->second << std::endl;
            _acm->setEntry(it->first, it->second, false);
        }
        else // print error
        {

            std::string link_not_found;

            if(_collision_obj.count(it->first) == 0)
            {
                link_not_found = it->first;
            }

            std::cout << "Error: could not find link " << it->first << " specified in whitelist, "
                      << "or link does not have collision geometry information" << std::endl;

            if(_collision_obj.count(it->second) == 0)
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

    loadDisabledCollisionsFromSRDF(*_srdf, _acm);

    generateLinksToUpdate();
    generatePairsToCheck();

    return true;
}

bool ComputeLinksDistance::setCollisionBlackList(std::list<LinkPairDistance::LinksPair> blackList)
{
    _acm = std::make_shared<collision_detection::AllowedCollisionMatrix>(
               _moveit_model->getLinkModelNamesWithCollisionGeometry(), true);

    std::vector<std::string> linksWithCollisionObjects;

    for(auto pair : _collision_obj)
    {
        linksWithCollisionObjects.push_back(pair.first);
    }

    // set all pairs to not allowed (all are checked)
    _acm->setEntry(linksWithCollisionObjects, linksWithCollisionObjects, false);

    // don't check pairs from black list
    for(auto pair : blackList)
    {
        _acm->setEntry(pair.first, pair.second, true);
    }

    // don't check disabled pairs from srdf
    loadDisabledCollisionsFromSRDF(*_srdf, _acm);

    generateLinksToUpdate();
    generatePairsToCheck();

    return true;
}

namespace
{

boost::shared_ptr<fcl::CollisionObjectd> fcl_from_primitive(
        const shape_msgs::SolidPrimitive& shape,
        const geometry_msgs::Pose& pose)
{
    std::shared_ptr<fcl::CollisionGeometryd> fcl_shape;

    // parse shapes
    if(shape.type == shape.BOX)
    {
        fcl_shape = std::make_shared<fcl::Boxd>(shape.dimensions[0],
                shape.dimensions[1],
                shape.dimensions[2]);

    }
    else if(shape.type == shape.SPHERE)
    {
        fcl_shape = std::make_shared<fcl::Sphered>(shape.dimensions[0]);
    }
    else
    {
        fprintf(stderr, "unsupported shape type \n");
        return nullptr;
    }

    // create collision object
    auto co = boost::make_shared<fcl::CollisionObjectd>(fcl_shape);

    // set transform
    fcl::Transform3d w_T_octo;
    tf::poseMsgToEigen(pose, w_T_octo);
    co->setTransform(w_T_octo);

    return co;
}

}

bool ComputeLinksDistance::setWorldCollisions(const moveit_msgs::PlanningSceneWorld& wc)
{

    bool ret = true;

    for(const auto& co : wc.collision_objects)
    {

        // handle remove action
        if(co.operation == co.REMOVE)
        {
            ret = removeWorldCollision(co.id) && ret;
            continue;
        }

        // only support collisions specified w.r.t. world
        if(!co.header.frame_id.empty() &&
                co.header.frame_id != "world")
        {
            fprintf(stderr, "invalid frame id '%s' \n",
                    co.header.frame_id.c_str());
            ret = false;
            continue;
        }

        // for now, don't support array of primitives
        if(co.primitives.size() > 1)
        {
            fprintf(stderr, "no support for primitive arrays \n");
            ret = false;
            continue;
        }

        // primitive case
        if(!co.primitives.empty())
        {
            // generate fcl collision
            auto fcl_collision = fcl_from_primitive(co.primitives[0],
                    co.primitive_poses[0]);

            if(!fcl_collision)
            {
                fprintf(stderr, "could not parse primitive for '%s' \n",
                        co.id.c_str());
                ret = false;
                continue;
            }

            printf("adding collision '%s' to world \n",
                   co.id.c_str());

            addWorldCollision(co.id, fcl_collision);

        }

    }

    // octree unavailable
    if(wc.octomap.octomap.data.empty())
    {
        return ret;
    }

    // abstract octree pointer
    auto abs_octree = std::shared_ptr<octomap::AbstractOcTree>(
                          octomap_msgs::msgToMap(wc.octomap.octomap));

    if(!abs_octree)
    {
        return false;
    }

    // we need it to be of type OcTree to construct the fcl class
    auto octree = std::dynamic_pointer_cast<octomap::OcTree>(abs_octree);

    if(!octree)
    {
        return false;
    }

    // fcl geometry
    auto fcl_octree = std::make_shared<fcl::OcTreed>(octree);

    // fcl collision object
    auto coll_obj = boost::make_shared<fcl::CollisionObjectd>(fcl_octree);

    // set transform
    fcl::Transform3d w_T_octo;
    tf::poseMsgToEigen(wc.octomap.origin, w_T_octo);
    coll_obj->setTransform(w_T_octo);

    // save collision object
    addWorldCollision("octomap", coll_obj);

    return ret;

}

namespace
{
    std::string world_obj_name(std::string name)
    {
        return "world/" + name;
    }
}

bool ComputeLinksDistance::addWorldCollision(const std::string &id,
                                             boost::shared_ptr<fcl::CollisionObjectd> fcl_obj)
{
    // empty name invalid
    if(id.empty())
    {
        fprintf(stderr, "invalid world collision '%s' \n",
                id.c_str());

        return false;
    }

    // collision name (to remove ambiguity with link names)
    auto coll_name = world_obj_name(id);

    // add to data structs
    _collision_obj[coll_name] = fcl_obj;
    _env_obj_names.insert(coll_name);

    // re-generate pairs to check
    generatePairsToCheck();

    // note: should it return false if object already exists?
    return true;
}

bool ComputeLinksDistance::removeWorldCollision(const std::string &id)
{
    auto coll_name = world_obj_name(id);

    // find id to remove
    auto it = _collision_obj.find(coll_name);

    // does not exist
    if(it == _collision_obj.end())
    {
        fprintf(stderr, "invalid world collision '%s' \n",
                id.c_str());

        return false;
    }

    // exists, delete it
    _collision_obj.erase(it);
    _env_obj_names.erase(coll_name);

    // re-generate pairs to check
    generatePairsToCheck();

    return true;
}

bool ComputeLinksDistance::moveWorldCollision(const std::string &id,
                                              KDL::Frame new_pose)
{
    auto it = _collision_obj.find(world_obj_name(id));

    if(it == _collision_obj.end())
    {
        return false;
    }

    it->second->setTransform(KDL2fcl(new_pose));

    return true;
}

void ComputeLinksDistance::loadDisabledCollisionsFromSRDF(const srdf::Model& srdf,
                                                          collision_detection::AllowedCollisionMatrixPtr acm )
{
    for(const auto& dc : srdf.getDisabledCollisionPairs())
    {
        acm->setEntry(dc.link1_, dc.link2_, true);
    }
}



LinkPairDistance::LinkPairDistance(const std::string& link1,
                                   const std::string& link2,
                                   const KDL::Frame& w_T_closestPoint1,
                                   const KDL::Frame& w_T_closestPoint2,
                                   const double& distance ) :
    _link_pair(link1, link2),
    _closest_points(w_T_closestPoint1, w_T_closestPoint2),
    distance(distance)
{

}

bool LinkPairDistance::isLink2WorldObject() const
{
    // true if starts with 'world/'
    const auto prefix = "world/";
    const auto prefix_len = strlen(prefix);
    return _link_pair.second.length() > prefix_len &&
            _link_pair.second.substr(0, prefix_len) == prefix;
}

const double &LinkPairDistance::getDistance() const
{
    return distance;
}

const std::pair<KDL::Frame, KDL::Frame> &LinkPairDistance::getClosestPoints() const
{
    return _closest_points;
}

const std::pair<std::string, std::string> &LinkPairDistance::getLinkNames() const
{
    return _link_pair;
}

bool LinkPairDistance::operator<(const LinkPairDistance& second) const
{
    if(distance < second.distance)
    {
        return true;
    }
    else
    {
        return _link_pair.first < second._link_pair.first;
    }
}
