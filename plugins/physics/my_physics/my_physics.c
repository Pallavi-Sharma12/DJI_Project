/*
 * File:
 * Date:
 * Description:
 * Author:
 * Modifications:
 */

#include <ode/ode.h>
#include <plugins/physics.h>
#include <webots/physics/Physics.h>
#include <webots/physics/PhysicsSolid.h>
#include <webots/physics/Solid.h>

// Wind parameters
#define WIND_FORCE 5.0  // Adjust the wind force as desired

// Function called at each simulation step
static void simulationStepCallback(void *data, void *userData) {
  PhysicsSolid *solid = (PhysicsSolid *)data;
  
  // Get the solid's mass and apply a force due to wind
  const double mass = wb_physics_solid_get_mass(solid);
  const WbVector3 windForce = {0.0, WIND_FORCE, 0.0};  // Adjust the wind direction as desired
  const WbVector3 windAcceleration = wb_vector3_scale(windForce, 1.0 / mass);
  
  // Apply the wind force to the solid
  wb_physics_solid_add_force(solid, windAcceleration);
}

int main(int argc, char **argv) {
  wb_robot_init();
  
  // Enable the physics and get the physics simulation object
  wb_physics_init();
  WbPhysicsPhysics *physics = wb_physics_physics_get();
  
  // Get the root solid
  WbSolidRef rootSolid = wb_physics_physics_get_root(physics);
  
  // Set the simulation step callback for all solids
  wb_physics_solid_foreach(simulationStepCallback, NULL, rootSolid);
  
  while (wb_robot_step(64) != -1)
    ;
  
  wb_robot_cleanup();
  
  return 0;
}

// static pthread_mutex_t mutex; // needed to run with multi-threaded version of ODE

// /*
 // * Note: This plugin will become operational only after it was compiled and associated with the current world (.wbt).
 // * To associate this plugin with the world follow these steps:
 // *  1. In the Scene Tree, expand the "WorldInfo" node and select its "physics" field
 // *  2. Then hit the [Select] button at the bottom of the Scene Tree
 // *  3. In the list choose the name of this plugin (same as this file without the extention)
 // *  4. Then save the .wbt by hitting the "Save" button in the toolbar of the 3D view
 // *  5. Then reload the world: the plugin should now load and execute with the current simulation
 // */

// void webots_physics_init() {
  // pthread_mutex_init(&mutex, NULL);
  // /*
   // * Get ODE object from the .wbt model, e.g.
   // *   dBodyID body1 = dWebotsGetBodyFromDEF("MY_ROBOT");
   // *   dBodyID body2 = dWebotsGetBodyFromDEF("MY_SOLID");
   // *   dGeomID geom2 = dWebotsGetGeomFromDEF("MY_SOLID");
   // * If an object is not found in the .wbt world, the function returns NULL.
   // * Your code should correcly handle the NULL cases because otherwise a segmentation fault will crash Webots.
   // *
   // * This function is also often used to add joints to the simulation, e.g.
   // *   dWorldID world = dBodyGetWorld(body1);
   // *   pthread_mutex_lock(&mutex);
   // *   dJointID joint = dJointCreateBall(world, 0);
   // *   dJointAttach(joint, body1, body2);
   // *   pthread_mutex_unlock(&mutex);
   // *   ...
   // */
// }

// void webots_physics_step() {
  // /*
   // * Do here what needs to be done at every time step, e.g. add forces to bodies
   // *   dBodyAddForce(body1, f[0], f[1], f[2]);
   // *   ...
   // */
// }

// int webots_physics_collide(dGeomID g1, dGeomID g2) {
  // /*
   // * This function needs to be implemented if you want to overide Webots collision detection.
   // * It must return 1 if the collision was handled and 0 otherwise.
   // * Note that contact joints should be added to the contact_joint_group which can change over the time, e.g.
   // *   n = dCollide(g1, g2, MAX_CONTACTS, &contact[0].geom, sizeof(dContact));
   // *   dJointGroupID contact_joint_group = dWebotsGetContactJointGroup();
   // *   dWorldID world = dBodyGetWorld(body1);
   // *   ...
   // *   pthread_mutex_lock(&mutex);
   // *   dJointCreateContact(world, contact_joint_group, &contact[i])
   // *   dJointAttach(contact_joint, body1, body2);
   // *   pthread_mutex_unlock(&mutex);
   // *   ...
   // */
  // return 0;
// }

// void webots_physics_cleanup() {
  // /*
   // * Here you need to free any memory you allocated in above, close files, etc.
   // * You do not need to free any ODE object, they will be freed by Webots.
   // */
  // pthread_mutex_destroy(&mutex);
// }
