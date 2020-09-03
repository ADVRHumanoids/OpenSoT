// #include <boost/filesystem.hpp>
#include <OpenSoT/utils/collision_utils.h>
// #include <kdl_parser/kdl_parser.hpp>
// #include <fcl/BV/OBBRSS.h>
// #include <fcl/BVH/BVH_model.h>
// #include <fcl/narrowphase/distance.h>
// #include <fcl/shape/geometric_shapes.h>
// #include <geometric_shapes/shapes.h>
// #include <geometric_shapes/shape_operations.h>
// #include <boost/make_shared.hpp>

bool ComputeLinksDistance::globalToLinkCoordinates ( const std::string& linkName,
        const fcl::Transform3<double> &fcl_w_T_f,
        KDL::Frame &link_T_f )
{

    fcl::Transform3<double> fcl_w_T_shape = collision_objects_[linkName]->getTransform();

//     fcl::Transform3<double> fcl_shape_T_f = fcl_w_T_shape.inverseTimes(fcl_w_T_f);
    fcl::Transform3<double> fcl_shape_T_f = fcl_w_T_shape.inverse() *fcl_w_T_f;

    link_T_f = link_T_shape[linkName] * fcl2KDL ( fcl_shape_T_f );

    return true;
}

bool ComputeLinksDistance::shapeToLinkCoordinates ( const std::string& linkName,
        const fcl::Transform3<double> &fcl_shape_T_f,
        KDL::Frame &link_T_f )
{

    link_T_f = link_T_shape[linkName] * fcl2KDL ( fcl_shape_T_f );

    return true;
}

bool ComputeLinksDistance::parseCollisionObjects ()
{
    std::vector<boost::shared_ptr<urdf::Link> > links;
    robot_urdf.getLinks ( links );
    typedef std::vector<boost::shared_ptr<urdf::Link> >::iterator it_type;

    for ( it_type iterator = links.begin();
            iterator != links.end(); iterator++ ) {

        boost::shared_ptr<urdf::Link> link = *iterator;

        if ( link->collision ) {
            if ( link->collision->geometry->type == urdf::Geometry::CYLINDER ||
                    link->collision->geometry->type == urdf::Geometry::SPHERE   ||
                    link->collision->geometry->type == urdf::Geometry::BOX      ||
                    link->collision->geometry->type == urdf::Geometry::MESH ) {


                shared_ptr_type<fcl::CollisionGeometry<double>> shape;
                KDL::Frame shape_origin;

                if ( link->collision->geometry->type == urdf::Geometry::CYLINDER ) {
                    std::cout << "adding capsule for " << link->name << std::endl;

                    boost::shared_ptr<urdf::Cylinder> collisionGeometry =
                        boost::dynamic_pointer_cast<urdf::Cylinder> (
                            link->collision->geometry );

                    shape.reset ( new fcl::Capsule<double> ( collisionGeometry->radius,
                                  collisionGeometry->length ) );

                    shape_origin = toKdl ( link->collision->origin );
                    shape_origin.p -= collisionGeometry->length/2.0 * shape_origin.M.UnitZ();

                    custom_capsules_[link->name] =
                        boost::shared_ptr<ComputeLinksDistance::Capsule> (
                            new ComputeLinksDistance::Capsule ( shape_origin,
                                    collisionGeometry->radius,
                                    collisionGeometry->length ) );
                } else if ( link->collision->geometry->type == urdf::Geometry::SPHERE ) {
                    std::cout << "adding sphere for " << link->name << std::endl;

                    boost::shared_ptr<urdf::Sphere> collisionGeometry =
                        boost::dynamic_pointer_cast<urdf::Sphere> (
                            link->collision->geometry );

                    shape.reset ( new fcl::Sphere<double> ( collisionGeometry->radius ) );
                    shape_origin = toKdl ( link->collision->origin );
                } else if ( link->collision->geometry->type == urdf::Geometry::BOX ) {
                    std::cout << "adding box for " << link->name << std::endl;

                    boost::shared_ptr<urdf::Box> collisionGeometry =
                        boost::dynamic_pointer_cast<urdf::Box> (
                            link->collision->geometry );

                    shape.reset ( new fcl::Box<double> ( collisionGeometry->dim.x,
                                                         collisionGeometry->dim.y,
                                                         collisionGeometry->dim.z ) );
                    shape_origin = toKdl ( link->collision->origin );
                    std::cout << "Box has size " << collisionGeometry->dim.x <<
                              ", " << collisionGeometry->dim.y <<
                              ", " << collisionGeometry->dim.z << std::endl;
                } else if ( link->collision->geometry->type == urdf::Geometry::MESH ) {
                    std::cout << "adding mesh for " << link->name << std::endl;

                    boost::shared_ptr< ::urdf::Mesh> collisionGeometry = boost::dynamic_pointer_cast< ::urdf::Mesh> ( link->collision->geometry );

                    shapes::Mesh *mesh = shapes::createMeshFromResource ( collisionGeometry->filename );
                    if ( mesh == NULL ) {
                        std::cout << "Error loading mesh for link " << link->name << std::endl;
                        continue;
                    }

                    std::vector<fcl::Vector3<double>> vertices;
                    std::vector<fcl::Triangle> triangles;

                    for ( unsigned int i=0; i < mesh->vertex_count; ++i ) {
                        fcl::Vector3<double> v ( mesh->vertices[3*i]*collisionGeometry->scale.x,
                                                 mesh->vertices[3*i + 1]*collisionGeometry->scale.y,
                                                 mesh->vertices[3*i + 2]*collisionGeometry->scale.z );

                        vertices.push_back ( v );
                    }

                    for ( unsigned int i=0; i< mesh->triangle_count; ++i ) {
                        fcl::Triangle t ( mesh->triangles[3*i],
                                          mesh->triangles[3*i + 1],
                                          mesh->triangles[3*i + 2] );
                        triangles.push_back ( t );
                    }

                    // add the mesh data into the BVHModel structure
                    shape.reset ( new fcl::BVHModel<fcl::OBBRSS<double>> );
                    fcl::BVHModel<fcl::OBBRSS<double>>* bvhModel = ( fcl::BVHModel<fcl::OBBRSS<double>>* ) shape.get();
                    bvhModel->beginModel();
                    bvhModel->addSubModel ( vertices, triangles );
                    bvhModel->endModel();

                    shape_origin = toKdl ( link->collision->origin );
                }

                boost::shared_ptr<fcl::CollisionObject<double>> collision_object (
                            new fcl::CollisionObject<double> ( shape ) );


                collision_objects_[link->name] = collision_object;
                shapes_[link->name] = shape;

                /* Store the transformation of the CollisionShape from URDF
                 * that is, we store link_T_shape for the actual link */
                link_T_shape[link->name] = shape_origin;
            } else {
                std::cout << "Collision type unknown for link " << link->name << std::endl;
            }
        } else {
            std::cout << "Collision not defined for link " << link->name << std::endl;
        }
    }
    return true;
}

bool ComputeLinksDistance::updateCollisionObjects()
{
    typedef std::set<std::string>::iterator it_links;
    typedef std::map<std::string,boost::shared_ptr<fcl::CollisionObject<double>> >::iterator it_co;

//    for(it_type it = collision_objects_.begin();
//        it != collision_objects_.end(); ++it)
    for ( it_links it = linksToUpdate.begin();
            it != linksToUpdate.end(); ++it ) {
//        std::string link_name = it->first;
        std::string link_name = *it;
        KDL::Frame w_T_link, w_T_shape;
        model.getPose ( link_name, w_T_link );
        w_T_shape = w_T_link * link_T_shape[link_name];

        fcl::Transform3<double> fcl_w_T_shape = KDL2fcl ( w_T_shape );
        fcl::CollisionObject<double>* collObj_shape = collision_objects_[link_name].get();
        collObj_shape->setTransform ( fcl_w_T_shape );
    }
    return true;
}


fcl::Transform3<double> ComputeLinksDistance::KDL2fcl ( const KDL::Frame &in )
{
    fcl::Transform3<double> out;
    double x,y,z,w;
    in.M.GetQuaternion ( x, y, z, w );
    fcl::Quaternion<double> q ( w, x, y, z );
//     fcl::Vector3<double> t(in.p[0], in.p[1], in.p[2]);
//     out.setQuatRotation(q);
//     out.setTranslation(t);
    out.translation() << in.p[0], in.p[1], in.p[2];
    out.linear() = q.toRotationMatrix();
    return out;
}

KDL::Frame ComputeLinksDistance::fcl2KDL ( const fcl::Transform3<double> &in )
{
//     fcl::Quaternion<double> q = in.getQuatRotation();
//     fcl::Vector3<double> t = in.getTranslation();

    fcl::Quaternion<double> q ( in.linear() );
    fcl::Vector3<double> t = in.translation();

    KDL::Frame f;
    f.p = KDL::Vector ( t[0],t[1],t[2] );
//     f.M = KDL::Rotation::Quaternion(q.getX(), q.getY(), q.getZ(), q.getW());
    f.M = KDL::Rotation::Quaternion ( q.x(), q.y(), q.z(), q.w() );

    return f;
}

void ComputeLinksDistance::generateLinksToUpdate()
{
    linksToUpdate.clear();
    std::vector<std::string> collisionEntries;
    // TODO isn't the result of
    // model.moveit_robot_model->getLinkModelNamesWithCollisionGeometry()
    // exactly what we are looking for?
    allowed_collision_matrix->getAllEntryNames ( collisionEntries );
    typedef std::vector<std::string>::iterator iter_link;
    typedef std::list<std::pair<std::string,std::string> >::iterator iter_pair;

    for ( iter_link it_A = collisionEntries.begin();
            it_A != collisionEntries.end();
            ++it_A ) {
        for ( iter_link it_B = collisionEntries.begin();
                it_B != collisionEntries.end();
                ++it_B ) {
            if ( it_B <= it_A ) {
                continue;
            } else {
                collision_detection::AllowedCollision::Type collisionType;
                if ( allowed_collision_matrix->getAllowedCollision ( *it_A,*it_B,collisionType ) &&
                        collisionType == collision_detection::AllowedCollision::NEVER ) {
                    linksToUpdate.insert ( *it_A );
                    linksToUpdate.insert ( *it_B );
                }
            }
        }
    }
}

void ComputeLinksDistance::generatePairsToCheck()
{
    pairsToCheck.clear();
    std::vector<std::string> collisionEntries;
    allowed_collision_matrix->getAllEntryNames ( collisionEntries );
    typedef std::vector<std::string>::iterator iter_link;
    typedef std::list<std::pair<std::string,std::string> >::iterator iter_pair;

    for ( iter_link it_A = collisionEntries.begin();
            it_A != collisionEntries.end();
            ++it_A ) {
        for ( iter_link it_B = collisionEntries.begin();
                it_B != collisionEntries.end();
                ++it_B ) {
            if ( it_B <= it_A ) {
                continue;
            } else {
                collision_detection::AllowedCollision::Type collisionType;
                if ( allowed_collision_matrix->getAllowedCollision ( *it_A,*it_B,collisionType ) &&
                        collisionType == collision_detection::AllowedCollision::NEVER ) {
                    pairsToCheck.push_back ( ComputeLinksDistance::LinksPair ( this,*it_A,*it_B ) );
                }
            }
        }
    }
    std::cout << "Checking " << pairsToCheck.size() << " pairs for collision" << std::endl;
}

ComputeLinksDistance::ComputeLinksDistance ( XBot::ModelInterface &model ) : model ( model )
{
    boost::shared_ptr<urdf::Model> urdf_model_ptr =
        boost::shared_ptr<urdf::Model> ( new urdf::Model() );
    urdf_model_ptr->initString ( model.getUrdfString() );

    boost::shared_ptr<srdf::Model> srdf_model_ptr =
        boost::shared_ptr<srdf::Model> ( new srdf::Model() );
    srdf_model_ptr->initString ( *urdf_model_ptr, model.getSrdfString() );

    moveit_robot_model.reset ( new robot_model::RobotModel ( urdf_model_ptr, srdf_model_ptr ) );

    boost::filesystem::path original_urdf ( model.getUrdfPath() );
    std::string capsule_model_urdf_filename = std::string ( original_urdf.stem().c_str() ) + std::string ( "_capsules.urdf" );
    boost::filesystem::path capsule_urdf ( original_urdf.parent_path() /
                                           capsule_model_urdf_filename );

    boost::filesystem::path original_srdf ( model.getSrdfPath() );
    std::string capsule_model_srdf_filename = std::string ( original_srdf.stem().c_str() ) + std::string ( "_capsules.srdf" );
    boost::filesystem::path capsule_srdf ( original_srdf.parent_path() /
                                           capsule_model_srdf_filename );

    std::string urdf_to_load, srdf_to_load;

    if ( boost::filesystem::exists ( capsule_urdf ) ) {
        urdf_to_load = capsule_urdf.c_str();
    } else {
        urdf_to_load = original_urdf.c_str();
    }

    if ( boost::filesystem::exists ( capsule_srdf ) ) {
        srdf_to_load = capsule_srdf.c_str();
    } else {
        srdf_to_load = original_srdf.c_str();
    }

    if(!urdf_to_load.empty())
      {
        std::cout<<"urdf_to_load: "<<urdf_to_load<<std::endl;
        robot_urdf.initFile ( urdf_to_load );
      }
      else
        robot_urdf.initString(model.getUrdfString());

       if(!srdf_to_load.empty())
      {
        std::cout<<"srdf_to_load: "<<srdf_to_load<<std::endl;
        robot_srdf.initFile ( robot_urdf, srdf_to_load );
      }
      else
        robot_srdf.initString(robot_urdf, model.getSrdfString());


    this->parseCollisionObjects ();

    this->setCollisionBlackList ( std::list<LinkPairDistance::LinksPair>() );
}

std::list<LinkPairDistance> ComputeLinksDistance::getLinkDistances ( double detectionThreshold )
{
    std::list<LinkPairDistance> results;

    updateCollisionObjects();

    typedef std::list< ComputeLinksDistance::LinksPair >::iterator iter_pair;

    for ( iter_pair it = pairsToCheck.begin();
            it != pairsToCheck.end();
            ++it ) {
        std::string linkA = it->linkA;
        std::string linkB = it->linkB;

        fcl::CollisionObject<double>* collObj_shapeA = it->collisionObjectA.get();
        fcl::CollisionObject<double>* collObj_shapeB = it->collisionObjectB.get();

        fcl::DistanceRequest<double> request;
#if FCL_MINOR_VERSION > 2
        request.gjk_solver_type = fcl::GST_INDEP;
#endif
        request.enable_nearest_points = true;

        // result will be returned via the collision result structure
        fcl::DistanceResult<double> result;

        // perform distance test
        fcl::distance ( collObj_shapeA, collObj_shapeB, request, result );

        // p1Homo, p2Homo newly computed points by FCL
        // absolutely computed w.r.t. base-frame
        KDL::Frame linkA_pA, linkB_pB;

        fcl::Transform3<double> world_pA, world_pB;
        world_pA.linear().setIdentity();
        world_pA.translation() = result.nearest_points[0];
        world_pB.linear().setIdentity();
        world_pB.translation() = result.nearest_points[1];

        globalToLinkCoordinates ( linkA, world_pA, linkA_pA );
        globalToLinkCoordinates ( linkB, world_pB, linkB_pB );

//         if ( collObj_shapeA->getNodeType() == fcl::GEOM_CAPSULE &&
//                 collObj_shapeB->getNodeType() == fcl::GEOM_CAPSULE ) {
//             globalToLinkCoordinates ( linkA, result.nearest_points[0], linkA_pA );
//             globalToLinkCoordinates ( linkB, result.nearest_points[1], linkB_pB );
//         } else {
//             shapeToLinkCoordinates ( linkA, result.nearest_points[0], linkA_pA );
//             shapeToLinkCoordinates ( linkB, result.nearest_points[1], linkB_pB );
//         }

        if ( result.min_distance < detectionThreshold )
            results.push_back ( LinkPairDistance ( linkA, linkB,
                                                   linkA_pA, linkB_pB,
                                                   result.min_distance ) );
    }

    results.sort();

    return results;
}

bool ComputeLinksDistance::setCollisionWhiteList ( std::list<LinkPairDistance::LinksPair> whiteList )
{
    allowed_collision_matrix.reset (
        new collision_detection::AllowedCollisionMatrix (
            moveit_robot_model->getLinkModelNamesWithCollisionGeometry(), true ) );

    typedef std::list<LinkPairDistance::LinksPair>::iterator iter_pairs;
    for ( iter_pairs it = whiteList.begin(); it != whiteList.end(); ++it ) {
        if ( collision_objects_.count ( it->first ) > 0 &&
                collision_objects_.count ( it->second ) > 0 ) {
            allowed_collision_matrix->setEntry ( it->first, it->second, false );
        } else {
            std::string link_not_found;
            if ( collision_objects_.count ( it->first ) == 0 ) {
                link_not_found = it->first;
            }
            std::cout << "Error: could not find link " << it->first << " specified in whitelist, "
                      << "or link does not have collision geometry information" << std::endl;
            if ( collision_objects_.count ( it->second ) == 0 ) {
                if ( link_not_found == "" ) {
                    link_not_found = it->second;
                } else {
                    link_not_found += " , " + it->second;
                }
            }

            std::cout << "Error: could not find link " << link_not_found << " specified in whitelist, "
                      << "or link does not have collision geometry information" << std::endl;
        }
    }

    loadDisabledCollisionsFromSRDF ( this->robot_srdf, allowed_collision_matrix );

    this->generateLinksToUpdate();
    this->generatePairsToCheck();

    //allowed_collision_matrix->print(std::cout);
    return true;
}

bool ComputeLinksDistance::setCollisionBlackList ( std::list<LinkPairDistance::LinksPair> blackList )
{
    allowed_collision_matrix.reset (
        new collision_detection::AllowedCollisionMatrix (
            moveit_robot_model->getLinkModelNamesWithCollisionGeometry(), true ) );

    std::vector<std::string> linksWithCollisionObjects;

    typedef std::map<std::string,boost::shared_ptr<fcl::CollisionObject<double>> >::iterator iter_collision;
    for ( iter_collision it = collision_objects_.begin(); it != collision_objects_.end(); ++it ) {
        linksWithCollisionObjects.push_back ( it->first );
    }

    allowed_collision_matrix->setEntry ( linksWithCollisionObjects, linksWithCollisionObjects, false );

    typedef std::list<LinkPairDistance::LinksPair>::iterator iter_pairs;
    for ( iter_pairs it = blackList.begin(); it != blackList.end(); ++it ) {
        allowed_collision_matrix->setEntry ( it->first, it->second, true );
    }

    loadDisabledCollisionsFromSRDF ( model.getSrdf(),allowed_collision_matrix );

    this->generateLinksToUpdate();
    this->generatePairsToCheck();

    //allowed_collision_matrix->print(std::cout);
    return true;
}

void ComputeLinksDistance::loadDisabledCollisionsFromSRDF ( const srdf_advr::Model& srdf,
        collision_detection::AllowedCollisionMatrixPtr acm )
{
    for ( std::vector<srdf_advr::Model::DisabledCollision>::const_iterator dc = srdf.getDisabledCollisionPairs().begin();
            dc != srdf.getDisabledCollisionPairs().end();
            ++dc ) {
        acm->setEntry ( dc->link1_, dc->link2_, true );
    }
}



LinkPairDistance::LinkPairDistance ( const std::string &link1, const std::string &link2,
                                     const KDL::Frame &link1_T_closestPoint1,
                                     const KDL::Frame &link2_T_closestPoint2,
                                     const double &distance ) :
//     linksPair ( link1 < link2 ? link1:link2,
//                 link1 < link2 ? link2:link1 ),
//     link_T_closestPoint ( link1 < link2 ? link1_T_closestPoint1:link2_T_closestPoint2,
//                           link1 < link2 ? link2_T_closestPoint2 :link1_T_closestPoint1 ),
    linksPair ( link1, link2 ),
    link_T_closestPoint ( link1_T_closestPoint1, link2_T_closestPoint2 ),
    distance ( distance )
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

bool LinkPairDistance::operator < ( const LinkPairDistance &second ) const
{
    if ( this->distance < second.distance ) {
        return true;
    } else {
        return ( this->linksPair.first < second.linksPair.first );
    }
}
