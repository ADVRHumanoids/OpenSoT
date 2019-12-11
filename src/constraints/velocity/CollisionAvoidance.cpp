/*
 * Copyright (C) 2014 Walkman
 * Author: Yangwei YOU
 * email:  yangwei.you@foxmail.com
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU Lesser General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

#include <OpenSoT/constraints/velocity/CollisionAvoidance.h>
// #include <boost/filesystem.hpp>
// #include <kdl_parser/kdl_parser.hpp>
// #include <geometric_shapes/shapes.h>
// #include <geometric_shapes/shape_operations.h>
// #include <boost/make_shared.hpp>

// #include <OpenSoT/utils/collision_utils.h>
// #include <fcl/config.h>
// #include <fcl/BV/OBBRSS.h>
// #include <fcl/BVH/BVH_model.h>
// #include <fcl/distance.h>
// #include <fcl/shape/geometric_shapes.h>

// local version of vectorKDLToEigen since oldest versions are bogous.
// To use instead of:
// #include <eigen_conversions/eigen_kdl.h>
// tf::vectorKDLToEigen
// void vectorKDLToEigen ( const KDL::Vector &k, Eigen::Matrix<double, 3, 1> &e )
// {
//     for ( int i = 0; i < 3; ++i ) {
//         e[i] = k[i];
//     }
// }

using namespace OpenSoT::constraints::velocity;

using namespace Eigen;

// fcl::Transform3f KDL2fcl ( const KDL::Frame &in )
// {
//     fcl::Transform3f out;
//     double x,y,z,w;
//     in.M.GetQuaternion ( x, y, z, w );
//     fcl::Vec3f t ( in.p[0], in.p[1], in.p[2] );
//     fcl::Quaternion3f q ( w, x, y, z );
//     out.setQuatRotation ( q );
//     out.setTranslation ( t );
//     return out;
// }
// 
// KDL::Frame fcl2KDL ( const fcl::Transform3f &in )
// {
//     fcl::Quaternion3f q = in.getQuatRotation();
//     fcl::Vec3f t = in.getTranslation();
// 
//     KDL::Frame f;
//     f.p = KDL::Vector ( t[0],t[1],t[2] );
//     f.M = KDL::Rotation::Quaternion ( q.getX(), q.getY(), q.getZ(), q.getW() );
// 
//     return f;
// }

bool CollisionAvoidance::globalToLinkCoordinates ( const std::string& linkName,
        const fcl::Transform3<double> &fcl_w_T_f,
        KDL::Frame &link_T_f )
{

    fcl::Transform3d fcl_w_T_shape = collision_objects_[linkName]->getTransform();

//     fcl::Transform3<double> fcl_shape_T_f = fcl_w_T_shape.inverseTimes(fcl_w_T_f);
    fcl::Transform3d fcl_shape_T_f = fcl_w_T_shape.inverse() *fcl_w_T_f;

    link_T_f = link_T_shape[linkName] * ComputeLinksDistance::fcl2KDL ( fcl_shape_T_f );

    return true;
}

bool CollisionAvoidance::shapeToLinkCoordinates ( const std::string& linkName,
        const fcl::Transform3<double> &fcl_shape_T_f,
        KDL::Frame &link_T_f )
{

    link_T_f = link_T_shape[linkName] * ComputeLinksDistance::fcl2KDL ( fcl_shape_T_f );

    return true;
}

CollisionAvoidance::CollisionAvoidance ( const Eigen::VectorXd& x,
        XBot::ModelInterface &robot,
        std::string& base_link,
        const std::vector<std::string> &interested_robot_links,
        const std::map<std::string, boost::shared_ptr<fcl::CollisionObjectd>> &envionment_collision_objects,
        const double &detection_threshold,
        const double &linkPair_threshold,
        const double &boundScaling ) :
    Constraint ( "collision_avoidance", x.size() ),
    _interested_links(interested_robot_links),
    _detection_threshold ( detection_threshold ),
    _linkPair_threshold ( linkPair_threshold ),
//     computeLinksDistance ( robot ),
    robot_col ( robot ),
    _x_cache ( x ),
    _boundScaling ( boundScaling ),
    base_name ( base_link )
{
    _J_transform.setZero ( 3,6 );

    for ( auto &it1:_interested_links ) {
        for ( auto &it2:envionment_collision_objects ) {
            LinkPairDistance::LinksPair link_pair ( it1, it2.first );
//             _environment_link.push_back ( it2.first );
            _interested_link_pairs.push_back ( link_pair );
        }
    }

    collision_objects_ = envionment_collision_objects;
    for ( auto &it:collision_objects_ ) {
        link_T_shape[it.first] = KDL::Frame();
    }

    parseCollisionObjects();
    update ( x );
}

bool CollisionAvoidance::parseCollisionObjects()
{
    boost::shared_ptr<urdf::Model> urdf_model_ptr =
        boost::shared_ptr<urdf::Model> ( new urdf::Model() );
    urdf_model_ptr->initString ( robot_col.getUrdfString() );

    boost::shared_ptr<srdf::Model> srdf_model_ptr =
        boost::shared_ptr<srdf::Model> ( new srdf::Model() );
    srdf_model_ptr->initString ( *urdf_model_ptr, robot_col.getSrdfString() );

//     moveit_robot_model.reset(new robot_model::RobotModel(urdf_model_ptr, srdf_model_ptr));

    boost::filesystem::path original_urdf ( robot_col.getUrdfPath() );
    std::string capsule_model_urdf_filename = std::string ( original_urdf.stem().c_str() ) + std::string ( "_capsules.urdf" );
    boost::filesystem::path capsule_urdf ( original_urdf.parent_path() /
                                           capsule_model_urdf_filename );

    boost::filesystem::path original_srdf ( robot_col.getSrdfPath() );
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


    std::cout<<"srdf_to_load: "<<srdf_to_load<<std::endl;
    std::cout<<"urdf_to_load: "<<urdf_to_load<<std::endl;

    urdf::Model robot_urdf;
    srdf::Model robot_srdf;
    robot_urdf.initFile ( urdf_to_load );
    robot_srdf.initFile ( robot_urdf, srdf_to_load );

    std::vector<boost::shared_ptr<urdf::Link> > links;
    for ( auto &it:_interested_links ) {
        boost::shared_ptr<urdf::Link> link;
        robot_urdf.getLink ( it, link );
        links.push_back ( link );
    }

//     linksToUpdate.clear();
//     std::vector<boost::shared_ptr<urdf::Link> > links;
//     for ( auto &it:_interested_link_pairs ) {
//         boost::shared_ptr<urdf::Link> link;
//         linksToUpdate.insert ( it.first );
//         robot_urdf.getLink ( it.first, link );
//         links.push_back ( link );
//     }
    for ( auto &link:links ) {
        if ( link->collision ) {
            if ( link->collision->geometry->type == urdf::Geometry::CYLINDER ||
                    link->collision->geometry->type == urdf::Geometry::SPHERE   ||
                    link->collision->geometry->type == urdf::Geometry::BOX      ||
                    link->collision->geometry->type == urdf::Geometry::MESH ) {

                std::shared_ptr<fcl::CollisionGeometryd> shape;
                KDL::Frame shape_origin;

                if ( link->collision->geometry->type == urdf::Geometry::CYLINDER ) {
                    std::cout << "adding capsule for " << link->name << std::endl;

                    boost::shared_ptr<urdf::Cylinder> collisionGeometry =
                        boost::dynamic_pointer_cast<urdf::Cylinder> (
                            link->collision->geometry );

                    shape.reset ( new fcl::Capsuled ( collisionGeometry->radius,
                                                     collisionGeometry->length ) );

                    shape_origin = toKdl ( link->collision->origin );
                    shape_origin.p -= collisionGeometry->length/2.0 * shape_origin.M.UnitZ();

                } else if ( link->collision->geometry->type == urdf::Geometry::SPHERE ) {
                    std::cout << "adding sphere for " << link->name << std::endl;

                    boost::shared_ptr<urdf::Sphere> collisionGeometry =
                        boost::dynamic_pointer_cast<urdf::Sphere> (
                            link->collision->geometry );

                    shape.reset ( new fcl::Sphered ( collisionGeometry->radius ) );
                    shape_origin = toKdl ( link->collision->origin );
                } else if ( link->collision->geometry->type == urdf::Geometry::BOX ) {
                    std::cout << "adding box for " << link->name << std::endl;

                    boost::shared_ptr<urdf::Box> collisionGeometry =
                        boost::dynamic_pointer_cast<urdf::Box> (
                            link->collision->geometry );

                    shape.reset ( new fcl::Boxd ( collisionGeometry->dim.x,
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

                    std::vector<fcl::Vector3d> vertices;
                    std::vector<fcl::Triangle> triangles;

                    for ( unsigned int i=0; i < mesh->vertex_count; ++i ) {
                        fcl::Vector3d v ( mesh->vertices[3*i]*collisionGeometry->scale.x,
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
                    shape.reset ( new fcl::BVHModel<fcl::OBBRSSd> );
                    fcl::BVHModel<fcl::OBBRSSd>* bvhModel = ( fcl::BVHModel<fcl::OBBRSSd>* ) shape.get();
                    bvhModel->beginModel();
                    bvhModel->addSubModel ( vertices, triangles );
                    bvhModel->endModel();

                    shape_origin = toKdl ( link->collision->origin );
                }

                boost::shared_ptr<fcl::CollisionObjectd> collision_object (
                            new fcl::CollisionObjectd ( shape ) );


                collision_objects_[link->name] = collision_object;
//                 shapes_[link->name] = shape;


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

bool CollisionAvoidance::updateCollisionObjects()
{
    for ( auto &it:_interested_links ) {
        std::string link_name = it;
        KDL::Frame w_T_link, w_T_shape;
        robot_col.getPose ( link_name, w_T_link );
        w_T_shape = w_T_link * link_T_shape[link_name];

        fcl::Transform3d fcl_w_T_shape = ComputeLinksDistance::KDL2fcl ( w_T_shape );
        fcl::CollisionObjectd* collObj_shape = collision_objects_[link_name].get();
        collObj_shape->setTransform ( fcl_w_T_shape );
    }
    return true;
}

bool CollisionAvoidance::updateEnvironmentCollisionObjects ( const std::map<std::string, KDL::Frame> &envionment_collision_frames )
{
    for ( auto &it:envionment_collision_frames ) {
        fcl::Transform3d fcl_w_T_shape = ComputeLinksDistance::KDL2fcl ( it.second );
        collision_objects_[it.first]->setTransform ( fcl_w_T_shape );
    }

    return true;
}

double CollisionAvoidance::getLinkPairThreshold()
{
    return _linkPair_threshold;
}

double CollisionAvoidance::getDetectionThreshold()
{
    return _detection_threshold;
}


void CollisionAvoidance::setLinkPairThreshold ( const double &linkPair_threshold )
{
    _linkPair_threshold = std::fabs ( linkPair_threshold );
    //this->update();
}

void CollisionAvoidance::setDetectionThreshold ( const double &detection_threshold )
{
    _detection_threshold = std::fabs ( detection_threshold );
    //this->update();
}

void CollisionAvoidance::update ( const Eigen::VectorXd &x )
{
    // we update _Aineq and _bupperBound only if x has changed
    //if(!(x == _x_cache)) {
    _x_cache = x;
    calculate_Aineq_bUpperB ( _Aineq, _bUpperBound );
    _bLowerBound = -1.0e20*_bLowerBound.setOnes ( _bUpperBound.size() );

    //}
//    std::cout << "_Aineq" << _Aineq.toString() << std::endl << std::endl;
    //    std::cout << "_bUpperBound" << _bUpperBound.toString() << std::endl << std::endl;
}

void CollisionAvoidance::skewSymmetricOperator ( const Eigen::Vector3d & r_cp, Eigen::MatrixXd& J_transform )
{
    if ( J_transform.rows() != 3 || J_transform.cols() != 6 ) {
        J_transform.setZero ( 3,6 );
    }

    J_transform.block ( 0,0,3,3 ) = Eigen::Matrix3d::Identity();
    J_transform.block ( 0,3,3,3 ) <<       0,  r_cp ( 2 ), -r_cp ( 1 ),
                      -r_cp ( 2 ),        0,  r_cp ( 0 ),
                      r_cp ( 1 ), -r_cp ( 0 ),        0;
}

std::list<LinkPairDistance> CollisionAvoidance::getLinkDistances ( const double &detectionThreshold )
{
    std::list<LinkPairDistance> results;

    updateCollisionObjects();

    for ( auto &it:_interested_link_pairs ) {
        std::string linkA = it.first;
        std::string linkB = it.second;

        fcl::CollisionObjectd* collObj_shapeA = collision_objects_[linkA].get();
        fcl::CollisionObjectd* collObj_shapeB = collision_objects_[linkB].get();

        fcl::DistanceRequestd request;
#if FCL_MINOR_VERSION > 2
        request.gjk_solver_type = fcl::GST_INDEP;
#endif
        request.enable_nearest_points = true;

        // result will be returned via the collision result structure
        fcl::DistanceResultd result;

        // perform distance test
        fcl::distance ( collObj_shapeA, collObj_shapeB, request, result );

        // p1Homo, p2Homo newly computed points by FCL
        // absolutely computed w.r.t. base-frame
        KDL::Frame linkA_pA, linkB_pB;

        fcl::Transform3d world_pA, world_pB;
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
//         std::cout << "min_distance: " << result.min_distance << std::endl;
// 	std::cout << "nearest_points1: " << result.nearest_points[0][0]  << ", " << result.nearest_points[0][1] << ", " << result.nearest_points[0][2] << std::endl;
// 	std::cout << "nearest_points2: " << result.nearest_points[1][0]  << ", " << result.nearest_points[1][1] << ", " << result.nearest_points[1][2] << std::endl;

        if ( result.min_distance < detectionThreshold )
            results.push_back ( LinkPairDistance ( linkA, linkB,
                                                   linkA_pA, linkB_pB,
                                                   result.min_distance ) );
    }

    results.sort();

    return results;
}

void CollisionAvoidance::calculate_Aineq_bUpperB ( Eigen::MatrixXd & Aineq_fc,
        Eigen::VectorXd & bUpperB_fc )
{
    std::list<LinkPairDistance> interested_LinkPairs;
    interested_LinkPairs = getLinkDistances ( _detection_threshold );

    /*//////////////////////////////////////////////////////////*/
// std::cout << "interested_LinkPairs size: " << interested_LinkPairs.size() << std::endl;
    MatrixXd Aineq_fc_Eigen ( interested_LinkPairs.size(), robot_col.getJointNum() );
    VectorXd bUpperB_fc_Eigen ( interested_LinkPairs.size() );

    double Dm_LinkPair;
    KDL::Frame Link1_T_CP,Link2_T_CP;
    std::string Link1_name, Link2_name;

    int Link1_index, Link2_index;

    KDL::Frame Waist_T_Link1, Waist_T_Link2, Waist_T_Link1_CP, Waist_T_Link2_CP;
    KDL::Vector Link1_origin_kdl, Link2_origin_kdl, Link1_CP_kdl, Link2_CP_kdl;
    Eigen::Matrix<double, 3, 1> Link1_origin, Link2_origin, Link1_CP, Link2_CP;

    Vector3d closepoint_dir;

    MatrixXd Link1_CP_Jaco, Link2_CP_Jaco;

    Affine3d Waist_frame_world_Eigen;
    robot_col.getPose ( base_name, Waist_frame_world_Eigen );
    KDL::Frame World_T_Waist;
    tf::transformEigenToKDL ( Waist_frame_world_Eigen, World_T_Waist );
    Waist_frame_world_Eigen = Waist_frame_world_Eigen.inverse();
    KDL::Frame Waist_T_World;
    tf::transformEigenToKDL ( Waist_frame_world_Eigen, Waist_T_World );

    Matrix3d Waist_frame_world_Eigen_Ro = Waist_frame_world_Eigen.matrix().block ( 0,0,3,3 );
    MatrixXd temp_trans_matrix ( 6,6 );
    temp_trans_matrix.setZero ( 6,6 );
    temp_trans_matrix.block ( 0,0,3,3 ) = Waist_frame_world_Eigen_Ro;
    temp_trans_matrix.block ( 3,3,3,3 ) = Waist_frame_world_Eigen_Ro;

    int linkPairIndex = 0;
    std::list<LinkPairDistance>::iterator j;
    for ( auto &linkPair:interested_LinkPairs ) {

        Dm_LinkPair = linkPair.getDistance();
        Link1_T_CP = linkPair.getLink_T_closestPoint().first;
        Link2_T_CP = linkPair.getLink_T_closestPoint().second;
        Link1_name = linkPair.getLinkNames().first;
        Link2_name = linkPair.getLinkNames().second;

        robot_col.getPose ( Link1_name, base_name, Waist_T_Link1 );
//         robot_col.getPose ( Link2_name, base_name, Waist_T_Link2 );

        fcl::Transform3d fcl_w_T_shape2 = collision_objects_[Link2_name]->getTransform();
        KDL::Frame World_T_Shape2 = ComputeLinksDistance::fcl2KDL ( fcl_w_T_shape2 );
        KDL::Frame Shape2_T_Link2 = link_T_shape[Link2_name].Inverse();
        Waist_T_Link2 = Waist_T_World*World_T_Shape2*Shape2_T_Link2;

//         std::cout << "Waist_T_Link1: " << Waist_T_Link1.p.x()  << ", " << Waist_T_Link1.p.y() << ", " << Waist_T_Link1.p.z() << std::endl;
// 	std::cout << "Waist_T_Link2: " << Waist_T_Link2.p.x()  << ", " << Waist_T_Link2.p.y() << ", " << Waist_T_Link2.p.z() << std::endl;

        Waist_T_Link1_CP = Waist_T_Link1 * Link1_T_CP;
        Waist_T_Link2_CP = Waist_T_Link2 * Link2_T_CP;
        Link1_origin_kdl = Waist_T_Link1.p;
        Link2_origin_kdl = Waist_T_Link2.p;
        Link1_CP_kdl = Waist_T_Link1_CP.p;
        Link2_CP_kdl = Waist_T_Link2_CP.p;

        tf::vectorKDLToEigen ( Link1_origin_kdl, Link1_origin );
        tf::vectorKDLToEigen ( Link2_origin_kdl, Link2_origin );
        tf::vectorKDLToEigen ( Link1_CP_kdl, Link1_CP );
        tf::vectorKDLToEigen ( Link2_CP_kdl, Link2_CP );


        closepoint_dir = Link2_CP - Link1_CP;
        closepoint_dir = closepoint_dir / Dm_LinkPair;
// 	std::cout << "Link1_CP: " << Link1_CP.transpose() << std::endl;
// 	std::cout << "Link2_CP: " << Link2_CP.transpose() << std::endl;
// 	std::cout << "Link1_T_CP: " << Link1_T_CP.p.x()  << ", " << Link1_T_CP.p.y() << ", " << Link1_T_CP.p.z() << std::endl;
// 	std::cout << "Link2_T_CP: " << Link2_T_CP.p.x()  << ", " << Link2_T_CP.p.y() << ", " << Link2_T_CP.p.z() << std::endl;
//         std::cout << "dirction: " << closepoint_dir.transpose() << std::endl;
        robot_col.getRelativeJacobian ( Link1_name, base_name,Link1_CP_Jaco );

        Link1_CP_Jaco = temp_trans_matrix * Link1_CP_Jaco;
        skewSymmetricOperator ( Link1_CP - Link1_origin,_J_transform );
        Link1_CP_Jaco = _J_transform * Link1_CP_Jaco;


//         robot_col.getRelativeJacobian ( Link2_name, base_name, Link2_CP_Jaco );
//         Link2_CP_Jaco = temp_trans_matrix * Link2_CP_Jaco;
//         skewSymmetricOperator ( Link2_CP - Link2_origin,_J_transform );
//         Link2_CP_Jaco = _J_transform * Link2_CP_Jaco;
        Link2_CP_Jaco = MatrixXd::Zero ( Link1_CP_Jaco.rows(), Link1_CP_Jaco.cols() );


        Aineq_fc_Eigen.row ( linkPairIndex ) = closepoint_dir.transpose() * ( Link1_CP_Jaco - Link2_CP_Jaco );
        bUpperB_fc_Eigen ( linkPairIndex ) = ( Dm_LinkPair - _linkPair_threshold ) * _boundScaling;

        ++linkPairIndex;

    }

    Aineq_fc = Aineq_fc_Eigen;
    bUpperB_fc = bUpperB_fc_Eigen;

}

void CollisionAvoidance::setBoundScaling ( const double &boundScaling )
{
    _boundScaling = boundScaling;
}
