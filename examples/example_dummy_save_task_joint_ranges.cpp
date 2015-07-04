/**
 @author Alessio Rocchi
 */

#include <idynutils/WalkmanUtils.h>
#include <yarp/os/Time.h>
#include <fstream>

int main(int argc, char** argv)
{

    WalkmanUtils walkman("save_task_joint_limits");
    yarp::sig::Vector q = walkman.sensePosition();
    yarp::sig::Vector q_lower = q;
    yarp::sig::Vector q_upper = q;

    unsigned int counter = 0;

    std::ofstream j_lims("task_joint_limits.csv");

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
            j_lims << "q_l[] = {" << q_lower_str << "};" << std::endl;
            std::string q_upper_str = q_upper.toString();
            std::replace(q_upper_str.begin(),q_upper_str.end(),'\t',',');
            j_lims << "q_u[] = {" << q_upper_str << "};" << std::endl;
            j_lims << "yarp::sig::Vector q_lower(sizeof(q_l),q_l);" << std::endl;
            j_lims << "yarp::sig::Vector q_upper(sizeof(q_u),q_u);" << std::endl;
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

