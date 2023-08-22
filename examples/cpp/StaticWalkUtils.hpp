#include <tf_conversions/tf_eigen.h>
#include <tf_conversions/tf_kdl.h>


/**
 * @brief The manipulation_trajectories class creates a simple linear trajectory for the arm and the com during
 * simple manipulation task
 */
class manipulation_trajectories{
public:
    manipulation_trajectories(const KDL::Frame& com_init,
                              const KDL::Frame& r_wrist_init):
        com_trj(0.01, "world", "com"),
        r_wrist_trj(0.01, "DWYTorso", "r_wrist")
    {
        T_trj = 1.;
        double h_com = 0.1;
        double arm_forward = 0.1;

        KDL::Frame com_wp = com_init;
        KDL::Frame r_wrist_wp = r_wrist_init;

        std::vector<KDL::Frame> com_waypoints, r_wrist_waypoints;
        //1. CoM goes down a little
        com_waypoints.push_back(com_wp);
        com_wp.p.z(com_wp.p.z()-h_com);
        com_waypoints.push_back(com_wp);

        r_wrist_waypoints.push_back(r_wrist_wp);
        r_wrist_waypoints.push_back(r_wrist_wp);
        //2. right arm move forward
        com_waypoints.push_back(com_wp);

        r_wrist_wp.p.x(r_wrist_wp.p.x()+arm_forward);
        r_wrist_waypoints.push_back(r_wrist_wp);
        //3. right arm move backward
        com_waypoints.push_back(com_wp);

        r_wrist_wp.p.x(r_wrist_wp.p.x()-arm_forward);
        r_wrist_waypoints.push_back(r_wrist_wp);
        //4. com goes back
        com_wp.p.z(com_wp.p.z()+h_com);
        com_waypoints.push_back(com_wp);
        r_wrist_waypoints.push_back(r_wrist_wp);

        com_trj.addMinJerkTrj(com_waypoints, T_trj);
        r_wrist_trj.addMinJerkTrj(r_wrist_waypoints, T_trj);
    }

    trajectory_utils::trajectory_generator com_trj, r_wrist_trj;
    double T_trj;
};

/**
 * @brief The walking_pattern_generator class generate Cartesian trajectories
 * for COM, left/right feet for a "static walk" (the CoM is always inside the
 * support polygon)
 * We consider 3 steps (hardcoded) and we always start with the right foot.
 */
class walking_pattern_generator{
public:
    walking_pattern_generator(const KDL::Frame& com_init,
                              const KDL::Frame& l_sole_init,
                              const KDL::Frame& r_sole_init):
        com_trj(0.01, "world", "com"),
        l_sole_trj(0.01, "world", "l_sole"),
        r_sole_trj(0.01, "world", "r_sole")
    {
        T_com = 3.;
        T_foot = 1.;

        step_lenght = 0.1;

        KDL::Vector plane_normal; plane_normal.Zero();
        plane_normal.y(1.0);

        KDL::Frame com_wp = com_init;

        KDL::Frame r_sole_wp, l_sole_wp;

        std::vector<double> com_time;
        std::vector<KDL::Frame> com_waypoints;
        com_waypoints.push_back(com_init);

        //1. We assume the CoM is in the middle of the feet and it
        //moves on top of the left feet (keeping the same height),
        //left and right feet keep the same pose:
        com_wp.p.x(l_sole_init.p.x());
        com_wp.p.y(l_sole_init.p.y());
        com_waypoints.push_back(com_wp);
        com_time.push_back(T_com);

        l_sole_trj.addMinJerkTrj(l_sole_init, l_sole_init, T_com);
        r_sole_trj.addMinJerkTrj(r_sole_init, r_sole_init, T_com);
        //2. Now the CoM is on the left foot, the right foot move forward
        // while the left foot keep the position:
        com_waypoints.push_back(com_wp);
        com_time.push_back(T_foot);
        l_sole_trj.addMinJerkTrj(l_sole_init, l_sole_init, T_foot);

        KDL::Vector arc_center = r_sole_init.p;
        arc_center.x(arc_center.x() + step_lenght/2.);
        r_sole_trj.addArcTrj(r_sole_init, r_sole_init.M, M_PI, arc_center, plane_normal, T_foot);
        //3. CoM pass from left to right foot, feet remains in the
        // same position
        r_sole_wp = r_sole_trj.Pos(r_sole_trj.Duration());
        com_wp.p.x(r_sole_wp.p.x());
        com_wp.p.y(r_sole_wp.p.y());
        com_waypoints.push_back(com_wp);
        com_time.push_back(T_com);

        l_sole_trj.addMinJerkTrj(l_sole_init, l_sole_init, T_com);
        r_sole_trj.addMinJerkTrj(r_sole_wp, r_sole_wp, T_com);
        //4. Now the CoM is on the right foot, the left foot move forward
        // while the right foot keep the position:
        com_waypoints.push_back(com_wp);
        com_time.push_back(T_foot);
        r_sole_trj.addMinJerkTrj(r_sole_wp, r_sole_wp, T_foot);

        arc_center = l_sole_init.p;
        arc_center.x(arc_center.x() + step_lenght);
        l_sole_trj.addArcTrj(l_sole_init, l_sole_init.M, M_PI, arc_center, plane_normal, T_foot);
        //5. CoM pass from right to left foot, feet remains in the
        // same position
        l_sole_wp = l_sole_trj.Pos(l_sole_trj.Duration());
        com_wp.p.x(l_sole_wp.p.x());
        com_wp.p.y(l_sole_wp.p.y());
        com_waypoints.push_back(com_wp);
        com_time.push_back(T_com);

        l_sole_trj.addMinJerkTrj(l_sole_wp, l_sole_wp, T_com);
        r_sole_trj.addMinJerkTrj(r_sole_wp, r_sole_wp, T_com);
        //6. Now the CoM is on the left foot, the right foot move forward
        // while the left foot keep the position:
        com_waypoints.push_back(com_wp);
        com_time.push_back(T_foot);
        l_sole_trj.addMinJerkTrj(l_sole_wp, l_sole_wp, T_foot);

        arc_center = r_sole_wp.p;
        arc_center.x(arc_center.x() + step_lenght);
        r_sole_trj.addArcTrj(r_sole_wp, r_sole_wp.M, M_PI, arc_center, plane_normal, T_foot);
        //7. CoM pass from left to right foot, feet remains in the
        // same position
        r_sole_wp = r_sole_trj.Pos(r_sole_trj.Duration());
        com_wp.p.x(r_sole_wp.p.x());
        com_wp.p.y(r_sole_wp.p.y());
        com_waypoints.push_back(com_wp);
        com_time.push_back(T_com);

        l_sole_trj.addMinJerkTrj(l_sole_wp, l_sole_wp, T_com);
        r_sole_trj.addMinJerkTrj(r_sole_wp, r_sole_wp, T_com);
        //8. Now the CoM is on the right foot, the left foot move forward
        // while the right foot keep the position:
        com_waypoints.push_back(com_wp);
        com_time.push_back(T_foot);
        r_sole_trj.addMinJerkTrj(r_sole_wp, r_sole_wp, T_foot);

        arc_center = l_sole_wp.p;
        arc_center.x(arc_center.x() + step_lenght/2.);
        l_sole_trj.addArcTrj(l_sole_wp, l_sole_wp.M, M_PI, arc_center, plane_normal, T_foot);
        //9. The CoM goes back in the middle of the feet. Feet remain
        // in the same position
        com_wp.p.y(com_init.p.y());
        com_waypoints.push_back(com_wp);
        com_time.push_back(T_com);

        l_sole_wp = l_sole_trj.Pos(l_sole_trj.Duration());
        l_sole_trj.addMinJerkTrj(l_sole_wp, l_sole_wp, T_com);
        r_sole_trj.addMinJerkTrj(r_sole_wp, r_sole_wp, T_com);

        com_trj.addMinJerkTrj(com_waypoints,com_time);
    }

    /**
     * @brief getAnchor return the hardcoded anchor foot based on the time
     * @param t time
     * @return anchor string
     */
    std::string getAnchor(const double t)
    {
        double phase = T_com+T_foot;
        if(t < phase)
            return "l_sole";
        else if(t < 2.*phase)
            return "r_sole";
        else if (t < 3*phase)
            return "l_sole";
        else
            return "r_sole";
    }

    /**
     * @brief T_COM trajectory time for the CoM
     */
    double T_com;
    /**
     * @brief T_foot trajectory time for the feet
     */
    double T_foot;

    /**
     * @brief step_lenght
     */
    double step_lenght;

    trajectory_utils::trajectory_generator com_trj, l_sole_trj, r_sole_trj;
};
