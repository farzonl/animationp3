#ifndef _MYWORLD_
#define _MYWORLD_

#include <vector>
#include <Eigen/Dense>
#include<Eigen/StdVector>
#include "dart/dart.h"

class RigidBody;
struct RigidContact;

class CollisionInterface;

class MyWorld {
 public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    MyWorld();

	virtual ~MyWorld();

	void initializePinata();
    
	int getNumRigidBodies() {
		return mRigidBodies.size();
	}

	RigidBody* getRigidBody(int _index) {
		return mRigidBodies[_index];
	}
   
	// TODO: your simulation and collision handling code goes here
	void simulate();
	void collisionHandling();
        
	CollisionInterface* getCollisionDetector() {
		return mCollisionDetector;
	}

	int getSimFrames() const { 
		return mFrame; 
	}

	void setDartWorld(dart::simulation::WorldPtr _dartWorld) {
		mPinataWorld = _dartWorld;
	}

	dart::simulation::WorldPtr getPinataWorld() {
		return mPinataWorld;
	}
    
    void updateMomentumWithImpulse(RigidBody* rBody, RigidContact &contact,
                                            double j, Eigen::Vector3d &rDelta);
    static Eigen::Vector3d computeRDelta(RigidBody * rBody, RigidContact &contact);
    static void updateInertialTensor(RigidBody* rBody);
    static void updateAngularVelocity(RigidBody* rBody);
    void updateQuaternion(RigidBody* rBody);
    void updateAngularMomentum(RigidBody* rBody);
    static double computeEpsilonDevisorbyRb(RigidBody * rBody, RigidContact &contact, Eigen::Vector3d &rDelta);
    static Eigen::Vector3d computeAngularMomentum(RigidBody* rBody, const Eigen::Vector3d &acceleration);
    static Eigen::Vector3d computePdot(RigidBody* rBody, Eigen::Vector3d &rDelta);
	double getTimeStep() {
		return mTimeStep;
	}

	void setExtForce(int _dir, double _mag) {
		mForce[_dir] = _mag;
	}
  
 protected:
	int mFrame;
	double mTimeStep;
	Eigen::Vector3d mGravity;
	std::vector<RigidBody*> mRigidBodies;
	CollisionInterface* mCollisionDetector; // Access to collision detection information
	dart::simulation::WorldPtr mPinataWorld;
	Eigen::Vector3d mForce;
};

#endif
