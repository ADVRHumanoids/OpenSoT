#ifndef _HCOD_WRAPPER_
#define _HCOD_WRAPPER_

#include <memory>
#include "visibility.h"


namespace soth{

class HCOD;
class Bound;

class SOTH_API HCOD_wrapper{
public:
    HCOD_wrapper(const unsigned int sizeProblem, const unsigned int nbStage = 0);
    ~HCOD_wrapper();

    void pushBackStage(const unsigned int nr, const double* Jdata, const Bound* bdata);
    void setNameByOrder(const std::string root);
    void setDamping(const double& d);
    void setInitialActiveSet();
    void activeSearch(double* u);

    bool updateStage(const unsigned int i, const double* Jdata, const Bound* bdata);



private:
    std::shared_ptr<HCOD> _hcod;
    int _problem_size;

};

}

#endif
