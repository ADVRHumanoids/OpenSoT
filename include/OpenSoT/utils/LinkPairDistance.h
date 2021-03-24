#ifndef OPENSOT_LINKPAIRDISTANCE_H_
#define OPENSOT_LINKPAIRDISTANCE_H_

/**
 * @brief The LinkPairDistance class represents the minimum distance information
 * between two links.
 * The two links must have shape (i.e. collision) information.
 * Instances of this class will contain the name of both links in the pair,
 * the actual minimum distance between the two, and the homogeneous transforms
 * in the respective link frame reference system of the minimum distance point
 * in each shape.
 */
class LinkPairDistance
{

public:

    typedef std::pair<std::string, std::string> LinksPair;

    /**
     * @brief LinkPairDistance creates an instance of a link pair distance data structure
     * @param link1 the first link name
     * @param link2 the second link name
     * @param w_T_closestPoint1 the transform from the world frame
     * to the closest point on its shape
     * @param w_T_closesPoint2 the transform from the world frame
     * to the closest point on its shape
     * @param distance the distance between the two minimum distance points
     */
    LinkPairDistance(const std::string& link1,
                     const std::string& link2,
                     const KDL::Frame& w_T_closestPoint1,
                     const KDL::Frame& w_T_closestPoint2,
                     const double& distance);

    /**
     * @brief isLink2WorldObject returns true if link2 refers
     * to an environment object, and not a robot link. On the other hand,
     * link1 is always a robot link
     */
    bool isLink2WorldObject() const;

    /**
     * @brief getDistance returns the minimum distance between the two link shapes
     * @return a double representing the minimum distance
     */
    const double& getDistance() const;

    /**
     * @brief getLink_T_closestPoint returns a pair of homogeneous transformation
     * from a link reference frame to the closest point on the respective link shape
     * @return a pair of homogeneous transformations
     */
    const std::pair<KDL::Frame, KDL::Frame>& getClosestPoints() const;

    /**
     * @brief getLinkNames returns the pair of links between which we want
     * express the distance information
     * @return a pair of strings
     */
    const std::pair<std::string, std::string>& getLinkNames() const;

    /**
     * @brief operator< is the comparison operator between two LinkDIstancePairs.
     * It returns true if the first link pair is closer than the second pair.
     * If the distance is exactly the same, the pairs will be sorted by
     * alphabetic order using the first link name in each pair
     * @param second the second link pair
     * @return true if first pair is closer than second pair
     */
    bool operator <(const LinkPairDistance& second) const;

private:
    /**
     * @brief _link_pair the pair of link names in alphabetic order
     */
    LinksPair _link_pair;

    /**
     * @brief link_T_closestPoint is a pair of homogeneous transformations.
     *        The first transformation will express the pose of the closes point on the first link shape
     *        in the link reference frame, i.e. link1_T_closestPoint1.
     *        The second transformation will express the pose of the closes point on the second link shape
     *        in the link reference frame, i.e. link2_T_closestPoint2.
     */
    std::pair<KDL::Frame, KDL::Frame> _closest_points;

    /**
     * @brief distance the minimum distance between the two link shapes, i.e.
     * ||w_T_closestPoint1.p - w_T_closesPoint2.p||
     */
    double distance;

};

#endif OPENSOT_LINKPAIRDISTANCE_H_

