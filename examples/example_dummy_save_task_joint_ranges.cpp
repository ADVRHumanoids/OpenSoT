/**
 @author Alessio Rocchi
 */

#include <idynutils/RobotUtils.h>
#include <yarp/os/Time.h>
#include <fstream>
#include <yarp/os/Network.h>

int main(int argc, char** argv)
{
    yarp::os::Network::init();

    RobotUtils walkman( "save_task_joint_limits", "bigman",
                     std::string(OPENSOT_TESTS_ROBOTS_DIR)+"bigman/bigman.urdf",
                     std::string(OPENSOT_TESTS_ROBOTS_DIR)+"bigman/bigman.srdf");
    yarp::os::Time::delay(1.0);
    yarp::sig::Vector q = walkman.sensePosition();
    yarp::sig::Vector q_lower = q;
    yarp::sig::Vector q_upper = q;

    unsigned int counter = 0;

    std::ofstream j_lims("task_joint_limits.txt");

    while(true)
    {
        q = walkman.sensePosition();
        for(unsigned int i = 0; i < q.size(); ++i)
        {
            if(q[i] < q_lower[i]) q_lower[i] = q[i];
            if(q[i] > q_upper[i]) q_upper[i] = q[i];
        }

        if(counter == 0)
        {
            j_lims.seekp(0);
            j_lims << "#copy paste into code" << std::endl;
            std::string q_lower_str = q_lower.toString();
            std::replace(q_lower_str.begin(), q_lower_str.end(), '\t',',');
            j_lims << "double q_l[] = {" << q_lower_str << "};" << std::endl;
            std::string q_upper_str = q_upper.toString();
            std::replace(q_upper_str.begin(),q_upper_str.end(),'\t',',');
            j_lims << "double q_u[] = {" << q_upper_str << "};" << std::endl;
            j_lims << "yarp::sig::Vector q_lower(sizeof(q_l)/sizeof(double),q_l);" << std::endl;
            j_lims << "yarp::sig::Vector q_upper(sizeof(q_u)/sizeof(double),q_u);" << std::endl;
            for(unsigned int i = 0; i < 64; ++i)
                j_lims << " ";
            j_lims << std::endl;
            j_lims.flush();
        }

        yarp::os::SystemClock::delaySystem(0.001);
        counter++;
        counter = counter%500;
    }


    return 1;
}

