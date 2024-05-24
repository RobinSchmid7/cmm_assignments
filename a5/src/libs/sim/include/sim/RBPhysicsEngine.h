#pragma once

#include <sim/RB.h>
#include <sim/RBRenderer.h>
#include <sim/RBSpring.h>
#include <sim/RBUtils.h>

namespace crl {

// subfunctions
Quaternion updateRotationGivenAngularVelocity(const Quaternion &q,
                                              const V3D &angularVelocity,
                                              double dt) {
    double angularVelocityMagnitude = angularVelocity.norm();
    // avoid divide by close to zero...
    if (angularVelocityMagnitude > 1e-10) {
        // TODO: Ex.1 Integration
        // implement quaternion update logic
        // q_p = rot(w, dt) * q
        Eigen::AngleAxis dAxis = Eigen::AngleAxis(dt * angularVelocityMagnitude, angularVelocity.unit());
        auto dq = Quaternion(dAxis);

        // Alternative method for comparison
        // Eigen::Quaternion q_p = dq * q;
//        double w = cos(0.5*dt*angularVelocityMagnitude);
//        V3D vec = sin(0.5*dt*angularVelocityMagnitude) * angularVelocity/angularVelocityMagnitude;
//        Eigen::Quaternion dq_alt = Quaternion(w,vec.x(),vec.y(),vec.z());
//        Eigen::Quaternion q_p_alt = dq_alt*q;
        // std::cout << "1" << "\n" << q_p.w() << "\n" << q_p.vec() << std::endl;
        // std::cout << "2" << "\n" << q_p_alt.w() << "\n" << q_p_alt.vec() << std::endl;

        return dq * q;
    }
    return q;
}

/**
 * our simulation world governed by the rigid-body dynamics
 */
class RBPhysicsEngine {
public:
    // constructor
    RBPhysicsEngine() {}

    // desctructor
    ~RBPhysicsEngine() {
        // we are going to delete every rigid body when simulation world is
        // destroyed.
        for (uint i = 0; i < rbs.size(); i++) delete rbs[i];
        rbs.clear();
        for (uint i = 0; i < springs.size(); i++) delete springs[i];
        springs.clear();
    }

    /**
     * add rigid body to simulation world.
     * note that we assume this is a cube block with approx. 0.24 x 0.24 x 0.24. 
     */
    RB *addRigidBodyToEngine() {
        rbs.push_back(new RB());
        // we use default mass = 100 kg
        rbs.back()->rbProps.mass = 100;

        rbs.back()->rbProps.collision = false;

        rbs.back()->rbProps.id = rbs.size() - 1;
        // add force and torque
        f_ext.push_back(V3D());
        tau_ext.push_back(V3D());

        return rbs.back();
    }

    /**

     * add rigid body with collision to simulation world.
     * note that we assume this is a sphere with radius = 0.1. 
     */
    RB *addCollidingRigidBodyToEngine() {
        double i = 0.4 * 100 * 0.1 * 0.1;
        rbs.push_back(new RB());
        // we use default mass = 100 kg
        rbs.back()->rbProps.mass = 100;
        rbs.back()->rbProps.setMOI(i, i, i, 0, 0, 0);
        rbs.back()->rbProps.collision = true;
        rbs.back()->rbProps.id = rbs.size() - 1;
        // add force and torque
        f_ext.push_back(V3D());
        tau_ext.push_back(V3D());
        return rbs.back();
    }

    /**

     * add spring to simulation world.
     */
    RBSpring *addSpringToEngine(RB *parent, RB *child, P3D pJPos, P3D cJPos) {
        springs.push_back(new RBSpring());
        springs.back()->parent = parent;
        springs.back()->child = child;
        // local position of attached point from parent/child frames
        springs.back()->pJPos = pJPos;
        springs.back()->cJPos = cJPos;
        // we use default spring constant = 2000;
        springs.back()->k = 10000;

        // default rest length is the distance between attaching points when
        // the spring is added.
        if (parent == nullptr) {
            // TODO: Ex.2-1
            // implement your logic for a spring which is attached to world
            // TODO: Fix the following line
            springs.back()->l0 = V3D(child->state.getWorldCoordinates(cJPos),pJPos).norm();
        } else {
            // TODO: Ex.2-2
            // implement your logic for a spring where both ends are attached
            // to rigid bodies
            // TODO: Fix the following line
            springs.back()->l0 = V3D(child->state.getWorldCoordinates(cJPos),parent->state.getWorldCoordinates(pJPos)).norm();
        }
        return springs.back();
    }

    /**
     * apply external force (no spring force, no gravity. Force comes from 
     * third sources) to rigid body.
     */
    void applyForceTo(RB *rb, const V3D &f, const P3D &p) {
        // add force only if rb is in rbs
        for (uint i = 0; i < rbs.size(); i++) {
            if (rbs[i] == rb) {
                f_ext[i] += f;
                V3D r = rb->state.getWorldCoordinates(V3D(p));
                tau_ext[i] += r.cross(f);
            }
        }
    }

    /**
     * simulation stepping logic. advance one simulation timestep with dt.
     * the unit of dt is second. 
     */
    void step(double dt) {
        // external force and torque
        for (uint i = 0; i < rbs.size(); i++) {
            // force and torque by gravity
            f_ext[i] += rbs[i]->rbProps.mass * V3D(0, RBGlobals::g, 0);
        }

        // force and torque by springs
        for (RBSpring *spring : springs) {
            // TODO: Ex.2 Spring force
            // compute spring force f_spring = -kx and torque tau_spring and
            // add them to f_ext and tau_ext
            //
            // Hint:
            // - spring->l0 is the rest length and spring->k is the spring constant
            // - you can retrieve index of rb in this->rbs list from rb->rbProps.id

            if (spring->parent == nullptr) {
                // TODO: Ex.2-1
                // implement your logic for a spring which is attached to world
                // Spring force
                V3D springConnection = V3D(spring->child->state.getWorldCoordinates(spring->cJPos), spring->pJPos);
                V3D f_spring = spring->k * (springConnection.norm() - spring->l0) * springConnection.unit();
                f_ext[spring->child->rbProps.id] += f_spring;

                // Torque
                P3D rbPos = rbs[spring->child->rbProps.id]->state.pos;
                P3D springPos = spring->child->state.getWorldCoordinates(spring->cJPos);
                V3D tau_spring = V3D(rbPos, springPos).cross(f_spring);
                tau_ext[spring->child->rbProps.id] += tau_spring;

            } else {
                // TODO: Ex.2-2
                // implement your logic for a spring where both ends are attached
                // to rigid bodies.
                // Spring force
                V3D springConnection = V3D(spring->child->state.getWorldCoordinates(spring->cJPos), spring->parent->state.getWorldCoordinates(spring->pJPos));
                V3D f_spring = spring->k * (springConnection.norm() - spring->l0) * springConnection.unit();
                f_ext[spring->child->rbProps.id] += f_spring;
                f_ext[spring->parent->rbProps.id] += -f_spring;

                // Torque
                P3D rbPosC = rbs[spring->child->rbProps.id]->state.pos;
                P3D rbPosP = rbs[spring->parent->rbProps.id]->state.pos;
                P3D springPosC = spring->child->state.getWorldCoordinates(spring->cJPos);
                P3D springPosP = spring->parent->state.getWorldCoordinates(spring->pJPos);
                V3D tau_springC = V3D(rbPosC, springPosC).cross(f_spring);
                V3D tau_springP = V3D(rbPosP, springPosP).cross(-f_spring);
                tau_ext[spring->child->rbProps.id] += tau_springC;
                tau_ext[spring->parent->rbProps.id] += tau_springP;
            }
        }

        for (uint i = 0; i < rbs.size(); i++) {
            RB *rb = rbs[i];
            // retrieve saved force and tau
            V3D f = f_ext[i];
            V3D tau = tau_ext[i];

            // TODO: Ex.1 Integration
            // implement forward (explicit) Euler integration scheme for computing velocity and pose.
            //
            // Hint:
            // - recall, you need to compute 3x3 moment of inertia matrix expressed in world frame
            //
            // implement your logic here.

            // Rotation matrix from local to world frame
            Matrix3x3 R_wl = rb->state.orientation.normalized().toRotationMatrix();
            // Mass of inertia in local frame
            Matrix3x3 MOI_local = rb->rbProps.MOI_local;
            // Mass of inertia in world frame
            Matrix3x3 MOI_world = R_wl * MOI_local * R_wl.transpose();
            // Inverse of mass of inertia in world frame
            Matrix3x3 MOI_world_inv = R_wl * MOI_local.inverse() * R_wl.transpose();

            // Update states, use forward Euler
//            rb->state.pos = rb->state.pos + dt * rb->state.velocity;
//            rb->state.velocity = rb->state.velocity + dt * f/(rb->rbProps.mass);
//            rb->state.orientation = updateRotationGivenAngularVelocity(rb->state.orientation, rb->state.angularVelocity, dt);
//            rb->state.angularVelocity = rb->state.angularVelocity + dt * MOI_world_inv * (tau - rb->state.angularVelocity.cross(V3D(MOI_world * rb->state.angularVelocity)));

            // TODO: Ex.3 Stable Simulation
            // why our simulation is blown up? let's make it more stable!
            //
            // comment out (do not erase!) your logic for Ex.1 and implement Ex.3 here.

            // Update states, use sympletic Euler
//            rb->state.velocity = rb->state.velocity + dt * f/(rb->rbProps.mass);
//            rb->state.angularVelocity = rb->state.angularVelocity + dt * MOI_world_inv * (tau - rb->state.angularVelocity.cross(V3D(MOI_world * rb->state.angularVelocity)));
//            rb->state.pos = rb->state.pos + dt * rb->state.velocity;
//            rb->state.orientation = updateRotationGivenAngularVelocity(rb->state.orientation,rb->state.angularVelocity,dt);

            // Impulse, post processing of linear and angular velocity
            rb->state.velocity = rb->state.velocity + dt * f/(rb->rbProps.mass);
            rb->state.angularVelocity = rb->state.angularVelocity + dt * MOI_world_inv * (tau - rb->state.angularVelocity.cross(V3D(MOI_world * rb->state.angularVelocity)));
            if (simulateCollisions && rb->rbProps.collision) {
                // TODO: Ex.4 Impulse-based Collisions
                // we will simulate collisions between a spherical rigidbody and
                // the ground plane. implement impulse-based collisions here. use
                // coefficient of restituation "epsilon". (it's a member variable 
                // of this class). we assume the friction is infinite.
                // note that we ignore collisions between rigidbodies.
                //
                // Steps:
                // 0. read the material "ImpulseBasedCollisions" on CMM21 website
                // carefully.
                // 1. update linear and angular velocity (not pose yet!!) of the 
                // rigidbody by external force and torque before this if statement 
                // block
                // 2. if rb->rbProps.collision == true, we will assume this rb is
                // a spherical object, and collide with the ground. (if not, 
                // it's a cube and it does not collide with any other objects.)
                // 3. compute impulse
                // 4. update linear and angular velocity with impulse
                // 5. now update pose of the rigid body (by integrating velocity)
                //
                // Hint:
                // - the radius of the sphere is 0.1 m
                // - detect collision if 1) the y coordinate of the point at the
                // bottom of the sphere < 0 and 2) the y component of linear
                // velocity of the point at the botton < 0.
                // - we will assume that a collision only happens at the bottom
                // points.
                // - we will assume there's only one contact between a sphere
                // and the ground

                // If below the ground and moving towards it
                if (rb->state.pos.y - 0.1 < 0 && rb->state.velocity.y() < 0) {
                    V3D r = V3D(rb->state.pos - P3D(0,0.1,0));
                    // Relaltive velocity
                    V3D u = rb->state.velocity + rb->state.angularVelocity.cross(r);

                    V3D N = V3D(0,1,0);
                    Matrix3x3 K = Matrix3x3::Identity()/rb->rbProps.mass - getSkewSymmetricMatrix(r) * MOI_world_inv * getSkewSymmetricMatrix(r);
                    // Collision impulse
                    V3D J = V3D(K.inverse()*(-u - eps * u.getComponentAlong(N)*N));

                    rb->state.velocity = rb->state.velocity + J/rb->rbProps.mass;
                    rb->state.angularVelocity = rb->state.angularVelocity + MOI_world_inv*(r.cross(J));
                }
            }

            rb->state.pos = rb->state.pos + dt * rb->state.velocity;
            rb->state.orientation = updateRotationGivenAngularVelocity(rb->state.orientation, rb->state.angularVelocity, dt);
        }

        // clean up
        for (uint i = 0; i < f_ext.size(); i++) {
            f_ext[i] = V3D();
            tau_ext[i] = V3D();
        }
    }

    /**
     * draw every rigid body belongs to world.
     */
    inline void draw(const gui::Shader &rbShader) {
        // draw moi boxes
        for (uint i = 0; i < this->rbs.size(); i++) {

            if (!this->rbs[i]->rbProps.fixed) {
                if (this->rbs[i]->rbProps.collision)
                    crl::RBRenderer::drawCollisionRB(this->rbs[i], rbShader);
                else
                    crl::RBRenderer::drawMOI(this->rbs[i], rbShader);
            }
        }

        // draw springs
        for (uint i = 0; i < this->springs.size(); i++) {
            P3D start, end;
            if (this->springs[i]->parent == nullptr) {
                start = this->springs[i]->pJPos;
                end = this->springs[i]->child->state.getWorldCoordinates(
                    this->springs[i]->cJPos);
            } else {
                start = this->springs[i]->parent->state.getWorldCoordinates(
                    this->springs[i]->pJPos);
                end = this->springs[i]->child->state.getWorldCoordinates(
                    this->springs[i]->cJPos);
            }
            drawCylinder(start, end, 0.05, rbShader);
        }


        // and now coordinate frames
        if (showCoordFrame) {
            for (uint i = 0; i < this->rbs.size(); i++)
                crl::RBRenderer::drawCoordFrame(this->rbs[i], rbShader);
        }
    }

    /**
     * returns NULL if no RBs are hit by the ray...
     */
    RB *getFirstRBHitByRay(const Ray &ray, P3D &intersectionPoint) {
        RB *selectedRB = nullptr;
        double t = DBL_MAX;
        P3D tmpIntersectionPoint = P3D(0, 0, 0);

        for (uint i = 0; i < rbs.size(); i++) {
            if (rbs[i]->getRayIntersectionPoint(ray, tmpIntersectionPoint)) {
                double tTmp = ray.getRayParameterFor(tmpIntersectionPoint);
                if (tTmp < t) {
                    selectedRB = rbs[i];
                    t = tTmp;
                    intersectionPoint = tmpIntersectionPoint;
                }
            }
        }
        return selectedRB;
    }



public:
    // this is a list of all rigid bodies and springs belong to the world.
    std::vector<RB *> rbs;
    std::vector<RBSpring *> springs;

    // coefficients
    float eps = 0.0;  // restitution

    // drawing flags
    bool showCoordFrame = true;

    // options
    bool simulateCollisions = false;

private:
    // list of force and tau applied to rigid bodies
    std::vector<V3D> f_ext;
    std::vector<V3D> tau_ext;
};
}  // namespace crl