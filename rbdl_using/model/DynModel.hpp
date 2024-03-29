#ifndef MERCURY_DYN_MODEL
#define MERCURY_DYN_MODEL

#include "utils/common_convert_calcu.hpp"
#include <rbdl/rbdl.h>

class DynModel{
public:

    DynModel(RigidBodyDynamics::Model* model);
    ~DynModel(void);

    bool getMassInertia(dynacore::Matrix & a);
    bool getInverseMassInertia(dynacore::Matrix & ainv);
    bool getGravity(dynacore::Vector &  grav);
    bool getCoriolis(dynacore::Vector & coriolis);

    void UpdateDynamics(const dynacore::Vector & q, const dynacore::Vector & qdot);

protected:
    dynacore::Matrix A_;
    dynacore::Matrix Ainv_;
    dynacore::Vector grav_;
    dynacore::Vector coriolis_;

    RigidBodyDynamics::Model* model_;
};

#endif
