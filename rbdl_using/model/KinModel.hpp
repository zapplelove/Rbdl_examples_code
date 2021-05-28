#ifndef KIN_MODEL
#define KIN_MODEL

#include "utils/common_convert_calcu.hpp"
#include "RobotDefinition.hpp"
#include <rbdl/rbdl.h>
#include <iostream>

class KinModel{
    public:
        KinModel(RigidBodyDynamics::Model* model);
        ~KinModel(void);
        //in world frame
        void getPos(int link_id, dynacore::Vect3 & pos);
        void getOri(int link_id, dynacore::Vect3 & rpy);
        void getLinearVel(int link_id, dynacore::Vect3 & vel);
        void getAngularVel(int link_id, dynacore::Vect3 & ang_vel);
        void getJacobian(int link_id, dynacore::Matrix & J);
        void getJDotQdot(int link_id, dynacore::Vector & JDotQdot);
        void getCoMJacobian  (dynacore::Matrix & J) const;
        void getCoMPos  (dynacore::Vect3 & com_pos) const;
        void getCoMVel (dynacore::Vect3 & com_vel) const;
        void getCentroidInertia(dynacore::Matrix & Icent){ Icent = Ig_; }
        void getCentroidJacobian(dynacore::Matrix & Jcent){ Jcent = Jg_; }
        void getCentroidVelocity(dynacore::Vector & centroid_vel);
        //matrix from world frame to body frame
        void getWorldToBodyMatrix(dynacore::Mat3 & BodyMatrix); 

        void UpdateKinematics(const dynacore::Vector & q, const dynacore::Vector & qdot);

        unsigned int _find_body_idx(int id) const;
        unsigned int find_body_id(const char* link_name) const;
        void displaylinks();

        dynacore::Vect3 com_pos_;
        dynacore::Vector centroid_vel_;
    protected:
        double gravity_;
        void _UpdateCentroidFrame(const dynacore::Vector & q, const dynacore::Vector & qdot);
        dynacore::Matrix Ig_;
        dynacore::Matrix Jg_;

        RigidBodyDynamics::Model* model_;
};

#endif
