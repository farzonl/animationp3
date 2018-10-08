#include "MyWorld.h"
#include "RigidBody.h"
#include "CollisionInterface.h"
#include <iostream>

using namespace Eigen;
using namespace std;

MyWorld::MyWorld() {
    mFrame = 0;
    mTimeStep = 0.001;
    mGravity = Vector3d(0.0, -9.8, 0.0);
    mForce.setZero();
    // Create a collision detector
    mCollisionDetector = new CollisionInterface();
    
    // Create and intialize two default rigid bodies
    RigidBody *rb1 = new RigidBody(dart::dynamics::Shape::BOX, Vector3d(0.05, 0.05, 0.05));
    mCollisionDetector->addRigidBody(rb1, "box"); // Put rb1 in collision detector
    rb1->mPosition[0] = -0.3;
    rb1->mPosition[1] = -0.5;
    
    rb1->mAngMomentum = Vector3d(0.0, 0.01, 0.0);
    mRigidBodies.push_back(rb1);
    
    RigidBody *rb2 = new RigidBody(dart::dynamics::Shape::ELLIPSOID, Vector3d(0.06, 0.06, 0.06));
    mCollisionDetector->addRigidBody(rb2, "ellipse"); // Put rb2 in collision detector
    rb2->mPosition[0] = 0.3;
    rb2->mPosition[1] = -0.5;
    rb2->mAngMomentum = Vector3d(0.01, 0.0, 0.0);
    rb2->mColor = Vector4d(0.2, 0.8, 0.2, 1.0); // Blue
    mRigidBodies.push_back(rb2);
}

void MyWorld::removeRigidBodies() {
    if(mRigidBodies.size() > 2) {
        mCollisionDetector->removeRigidBody(mRigidBodies.back());
        mRigidBodies.pop_back();
    }
}

float MyWorld::genRandNum() {
    return 0.1f + (rand() % 90)/100.0f;
}

void MyWorld::addRigidBodies() {
    std::string shapeStr = "box";
    RigidBody *rb = nullptr;
    if (rand() % 2 == 0) {
        shapeStr = "ellipse";
        rb = new RigidBody(dart::dynamics::Shape::ELLIPSOID, Vector3d(0.06, 0.06, 0.06));
    } else {
        rb = new RigidBody(dart::dynamics::Shape::BOX, Vector3d(0.05, 0.05, 0.05));
    }
    mCollisionDetector->addRigidBody(rb, shapeStr); // Put rb in collision detector
    rb->mPosition[0] = -.5 + genRandNum();
    rb->mPosition[1] = -.1 - genRandNum();
    rb->mAngMomentum = Vector3d(rand()%1/10.f, rand()%1/10.f, rand()%1/10.f);
    rb->mColor = Vector4d(genRandNum(), genRandNum(),
                          genRandNum(), genRandNum());
    mRigidBodies.push_back(rb);
}

void MyWorld::initializePinata() {
    // Add pinata to the collison detector
    mCollisionDetector->addSkeleton(mPinataWorld->getSkeleton(0));
    
    // Add some damping in the Pinata joints
    int nJoints = mPinataWorld->getSkeleton(0)->getNumBodyNodes();
    for (int i = 0; i < nJoints; i++) {
        int nDofs = mPinataWorld->getSkeleton(0)->getJoint(i)->getNumDofs();
        for (int j = 0; j < nDofs; j++)
        mPinataWorld->getSkeleton(0)->getJoint(i)->setDampingCoefficient(j, 1.0);
    }
    
    // Weld two seems to make a box
    dart::dynamics::BodyNode* top = mPinataWorld->getSkeleton(0)->getBodyNode("top");
    dart::dynamics::BodyNode* front = mPinataWorld->getSkeleton(0)->getBodyNode("front");
    dart::dynamics::BodyNode* back = mPinataWorld->getSkeleton(0)->getBodyNode("back");
    dart::constraint::WeldJointConstraint *joint1 = new dart::constraint::WeldJointConstraint(top, front);
    dart::constraint::WeldJointConstraint *joint2 = new dart::constraint::WeldJointConstraint(top, back);
    mPinataWorld->getConstraintSolver()->addConstraint(joint1);
    mPinataWorld->getConstraintSolver()->addConstraint(joint2);
}

MyWorld::~MyWorld() {
    for (int i = 0; i < mRigidBodies.size(); i++)
    delete mRigidBodies[i];
    mRigidBodies.clear();
    if (mCollisionDetector)
    delete mCollisionDetector;
}

void MyWorld::simulate() {
    mFrame++;
    
    // TODO: The skeleton code has provided the integration of position and linear momentum,
    // your first job is to fill in the integration of orientation and angular momentum.
    for (int i = 0; i < mRigidBodies.size(); i++) {
        // derivative of position and linear momentum
        Eigen::Vector3d dPos = mRigidBodies[i]->mLinMomentum / mRigidBodies[i]->mMass;
        Eigen::Vector3d dLinMom = mRigidBodies[i]->mMass * mGravity + mRigidBodies[i]->mAccumulatedForce;
        
        updateInertialTensor(mRigidBodies[i]);
        updateAngularVelocity(mRigidBodies[i]);
        updateQuaternion(mRigidBodies[i]); // call after updating angV
        updateAngularMomentum(mRigidBodies[i]);
        // update position and linear momentum
        mRigidBodies[i]->mPosition += dPos * mTimeStep;
        mRigidBodies[i]->mLinMomentum += mTimeStep * dLinMom;
    }
    
    // Reset accumulated force and torque to be zero after a complete integration
    for (int i = 0; i < mRigidBodies.size(); i++) {
        mRigidBodies[i]->mAccumulatedForce.setZero();
        mRigidBodies[i]->mAccumulatedTorque.setZero();
    }
    
    // Apply external force to the pinata
    mPinataWorld->getSkeleton(0)->getBodyNode("bottom")->addExtForce(mForce);
    mForce.setZero();
    
    // Simulate Pinata using DART
    mPinataWorld->step();
    
    // Run collision detector
    mCollisionDetector->checkCollision();
    
    // TODO: implement a collision handler
    collisionHandling();
    
    // Break the pinata if it has enough momentum
    if (mPinataWorld->getSkeleton(0)->getCOMLinearVelocity().norm() > 0.6)
    mPinataWorld->getConstraintSolver()->removeAllConstraints();
}

/*
 The collision detector provided here will return an array of contacts at every time step. Here is a list of APIs you might need:
 
 // Get the number of contacts at the current time step
 int nContacts = mCollisionDetector->getNumContacts();
 
 // Get the world coordinate of the first contact point at the current time step
 Vector3d point = mCollisionDetector->getContact(0).point;
 
 // Get the normal vector of the third contact point at the current time step. The vector is expressed in the world space.
 Vector3d normal = mCollisionDetector->getContact(2).normal;
 
 // Get the pointer to the rigid body A involved in the second contact. If A is a null pointer, that means it is the pinata. Otherwise, it will return a non-null pointer.
 RigidBody* A = mCollisionDetector->getContact(1).rb1;
 
 // Get the pointer to the rigid body B involved in the second contact. If B is a null pointer, that means it is the pinata. Otherwise, it will return a non-null pointer.
 RigidBody* B = mCollisionDetector->getContact(1).rb2;
 
 // Get the velocity of the colliding point on the pinata in the world space. If neither rb1 nor rb2 is the pinata, it will return (0, 0, 0).
 Vector3d pVelocity = mCollisionDetector->getContact(0).pinataVelocity;
 
 */

// TODO: fill in the collision handling function
void MyWorld::collisionHandling() {
    // restitution coefficient
    double epsilon = 0.8;
    
    // TODO: handle the collision events
    
    int nContacts = mCollisionDetector->getNumContacts(); // Get all the contacts so we can loop over them
    //std::cout << "n# contacts: " << nContacts << std::endl;
    for (int i = 0; i < nContacts; i++) {
        // Get the ith contact
        RigidContact currContact = mCollisionDetector->getContact(i);
        RigidBody* A = currContact.rb1;
        RigidBody* B = currContact.rb2;
        bool AisNotPinata = A? true : false;
        bool BisNotPinata = B? true : false;
        
        if(!AisNotPinata && !BisNotPinata) {
            continue;
        }
        
        double aDivisor = 0.0;
        double bDivisor = 0.0;
        Eigen::Vector3d pdotA = currContact.pinataVelocity;
        Eigen::Vector3d pdotB = currContact.pinataVelocity;
        Eigen::Vector3d rDeltaA;
        Eigen::Vector3d rDeltaB;
        if(AisNotPinata) {
            rDeltaA = computeRDelta(A, currContact);
            aDivisor = computeEpsilonDevisorbyRb(A, currContact, rDeltaA);
            pdotA = computePdot(A, rDeltaA);
        }
        
        if(BisNotPinata) {
            rDeltaB = computeRDelta(B, currContact);
            bDivisor = computeEpsilonDevisorbyRb(B, currContact, rDeltaB);
            pdotB = computePdot(B, rDeltaB);
        }

        //vr=ˆn(tc) dot (pdotA(t0) − ̇pdotB(t0))
        double vr = currContact.normal.dot(pdotA - pdotB);
        double j = -(1 + epsilon) * vr / (aDivisor + bDivisor);
        
        updateMomentumWithImpulse(A, currContact.normal, j, rDeltaA);
        updateMomentumWithImpulse(B, -1 * currContact.normal, j, rDeltaB);
    }
}

void MyWorld::updateInertialTensor(RigidBody* rBody){
    // I(t) = R(t) * Ibody * R(t)^T
    rBody->mInertia_tensor = rBody->mOrientation *
        rBody->mI_body * (rBody->mOrientation.transpose());
}

void MyWorld::updateAngularMomentum(RigidBody* rBody) {
    rBody->mAngMomentum += mTimeStep * computeAngularMomentum(rBody,mGravity);;
}

Eigen::Vector3d MyWorld::computeAngularMomentum(RigidBody* rBody, const Eigen::Vector3d &acceleration) {
    // L(t) = I(t) * w(t)
    return rBody->mInertia_tensor * acceleration + rBody->mAccumulatedTorque;
}

void MyWorld::updateAngularVelocity(RigidBody* rBody) {
    //w(t) = I(t)^-1 * L(t)
    rBody->mAngVelocity = rBody->mInertia_tensor.inverse() * rBody->mAngMomentum;
}

// compute the impulse
//compute j
// Ia = Impulse of rigidBody_a
// Ib = Impulse of rigidBody_b
// 1/Ma + 1/Mb + [ˆn(t_c) dot (Ia^-1(t_c)(ra cross ^n(tc))) cross ra] + [^n(tc) dot (Ib^-1(t_c)(rb cross ^n(tc))) cross rb +^n(tc)]
//do j computation in  for (int i = 0; i < nContacts; i++)
// break computation into pieces, a  dot product section and cross product section
double MyWorld::computeEpsilonDevisorbyRb(RigidBody * rBody, RigidContact &contact, Eigen::Vector3d &rDelta) {
    return (1/rBody->mMass) + contact.normal.dot(rBody->mInertia_tensor.inverse() * rDelta.cross(contact.normal).cross(rDelta));
}

// used to computed:
//ra = pa(tc) − xa(tc)
//rb = pb(tc) − xb(tc)
Eigen::Vector3d MyWorld::computeRDelta(RigidBody * rBody, RigidContact &contact) {
    return contact.point - rBody->mPosition;
}

//compute pdota and pdotb
// v−r=ˆn(tc)·( ̇p−a(t0)− ̇p−b(t0))
// v+r=ˆn(tc)·( ̇p+a(t0)− ̇p+b(t0))
Eigen::Vector3d MyWorld::computePdot(RigidBody* rBody, Eigen::Vector3d &rDelta) { //need to derrive the Pdot
    rBody->mLinVelocity = rBody->mLinMomentum / rBody->mMass;
    updateInertialTensor(rBody);
    updateAngularVelocity(rBody);
    return rBody->mLinVelocity + rBody->mAngVelocity.cross(rDelta);
}

void MyWorld::updateMomentumWithImpulse(RigidBody* rBody, Eigen::Vector3d normal,
                               double j, Eigen::Vector3d &rDelta) {
    if (rBody) {
        // J: Impulse ConstrRigid.pdf pg. 34
        Eigen::Vector3d J = j * normal;
        Eigen::Vector3d torqueJ = rDelta.cross(J); //torque impulse
        //Finally, apply the change in linear momentum and angular momentum to the current state
        rBody->mLinMomentum += J;
        rBody->mAngMomentum += torqueJ;
    }
}

void MyWorld::updateQuaternion(RigidBody* rBody) {
    Eigen::Quaterniond qOmega;
    qOmega.w() = 0.0;
    //q_dot(t) = 1/2 * w(t) * q(t)
    qOmega.vec() = 0.5 * rBody->mAngVelocity;
    Eigen::Quaterniond q_dot = qOmega * rBody->mQuatOrient;
    rBody->mQuatOrient.w() += mTimeStep * q_dot.w();
    rBody->mQuatOrient.vec() += mTimeStep * q_dot.vec();
    rBody->mQuatOrient.normalize();
    rBody->mOrientation = (rBody->mQuatOrient).toRotationMatrix();
}




