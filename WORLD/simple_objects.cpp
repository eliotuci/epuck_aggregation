#include "simple_objects.h"


/* ------------------------------------------------------------ */


SIMPLE_Objects::SIMPLE_Objects( btDynamicsWorld* _world ) : World_Entity ( )
{
  rot.assign    (3, 0.0);
  dim.assign    (3, 0.0);
  colour.assign (3, 0.0);
  world = _world;
  //collision_counter = 0;
  removed = false;
}

/* ------------------------------------------------------------ */

void SIMPLE_Objects::set_mass(double _mass){

    this->mass = _mass;
    btTransform trans;
    btVector3 inertia(0.0,0.0,0.0);
    body[object_id]->getMotionState()->getWorldTransform(trans);
    world->removeRigidBody(body[object_id]);
    body[object_id]->getCollisionShape()->calculateLocalInertia(_mass, inertia);
    body[object_id]->setActivationState(DISABLE_DEACTIVATION);
    body[object_id]->setMassProps(_mass, inertia);
    body[object_id]->setAngularVelocity(btVector3(0,0,0));
    body[object_id]->setLinearVelocity(btVector3(0,0,0));
    body[object_id]->setLinearFactor(btVector3(1,1,1));
    body[object_id]->setAngularFactor(btVector3(1,1,1));
    body[object_id]->updateInertiaTensor();
    body[object_id]->clearForces();
    body[object_id]->getCollisionShape()->setLocalScaling(btVector3(1,1,1));
    body[object_id]->setActivationState(1);
    world->addRigidBody(body[object_id]);

}

/* ------------------------------------------------------------ */

void SIMPLE_Objects::set_size(double _length){
    world->removeRigidBody(body[object_id]);
    this->dim[0] = _length;
    if(_length == 0.30){
        world->addRigidBody(body[0]);
        object_id = 0;
    }
    else{
        world->addRigidBody(body[1]);
        object_id = 1;
    }
}

/* ------------------------------------------------------------ */
/* ------------------------------------------------------------ */

void SIMPLE_Objects::remove_object(){
  world->removeRigidBody(this->body[object_id]);
  this->removed = true;
}

/* ------------------------------------------------------------ */

void SIMPLE_Objects::add_object(){
  world->addRigidBody(this->body[object_id]);
  this->removed = false;
 }

/* ------------------------------------------------------------ */
/* ------------------------------------------------------------ */

const vector <double> SIMPLE_Objects::get_pos     ( void ){
    pos[0] = body[this->object_id]->getWorldTransform().getOrigin().x();
    pos[1] = body[this->object_id]->getWorldTransform().getOrigin().y();
    pos[2] = body[this->object_id]->getWorldTransform().getOrigin().z();
    return pos;

}

/* ------------------------------------------------------------ */
/* ------------------------------------------------------------ */

const vector <double> SIMPLE_Objects::get_rot     ( void ){
    btMatrix3x3 rotMatrix = btMatrix3x3(body[this->object_id]->getWorldTransform().getRotation());
    float rotX,rotY,rotZ;
    rotMatrix.getEulerZYX(rotZ,rotY,rotX);
    rot[0] = rotX;
    rot[1] = rotY;
    rot[2] = rotZ;
    return rot;
}

/* ------------------------------------------------------------ */

void SIMPLE_Objects::set_rot ( const vector <double> &_rot ){
/*rot[0] = _rot[0];
rot[1] = _rot[1];
rot[2] = _rot[2]; */

        btQuaternion rotation;
        rotation.setEulerZYX( 0.0, _rot[1], 0.0);
        btTransform trans = body[this->object_id]->getWorldTransform();
        trans.setRotation(rotation);
        body[this->object_id]->setWorldTransform(trans);
}

/* ------------------------------------------------------------ */

void SIMPLE_Objects::set_pos ( const vector <double> &_pos ){

     btTransform trans = body[this->object_id]->getWorldTransform();
     trans.setOrigin(btVector3(_pos[0],_pos[1] + this->dim[1] * 0.5 ,_pos[2]));
     body[this->object_id]->setWorldTransform(trans);
}
/* ------------------------------------------------------------- */
/* PLANE   PLANE   PLANE   PLANE   PLANE   PLANE   PLANE   PLANE */
/* ------------------------------------------------------------ */
SIMPLE_Plane::SIMPLE_Plane( btDynamicsWorld* world ) : SIMPLE_Objects( world )
{
    type_id = PLANE;
    addPlane();
}

/* ------------------------------------------------------------- */
void SIMPLE_Plane::addPlane()
{
    btCollisionShape* groundShape = new btBoxShape(btVector3(btScalar(5.0),btScalar(0.25),btScalar(5.0)));
    btTransform groundTransform;
    groundTransform.setIdentity();
    groundTransform.setOrigin(btVector3(0.0,-0.25,0.0));
    mass = 0.0;
    btVector3 Inertia(0.0,0.0,0.0);
    btDefaultMotionState* groundMotionState = new btDefaultMotionState(groundTransform);
    btRigidBody::btRigidBodyConstructionInfo info(mass,groundMotionState,groundShape,Inertia);
    info.m_restitution = 0.0f;
    info.m_friction = 0.5f;
    //info.m_rollingFriction = 0.1;
    body[0]=new btRigidBody(info);
    world->addRigidBody(body[0]);
    object_id = 0;

 }
/* ------------------------------------------------------------ */
#ifdef _GRAPHICS_

void SIMPLE_Plane::render ( )
{

    if(body[object_id]->getCollisionShape()->getShapeType()!=BOX_SHAPE_PROXYTYPE)
        return;
    glColor3f(0.5,0.5,0.5);
    btVector3 extent=((btBoxShape*)body[object_id]->getCollisionShape())->getHalfExtentsWithMargin();
    btTransform t;
    body[object_id]->getMotionState()->getWorldTransform(t);
    float mat[16];
    t.getOpenGLMatrix(mat);
    glPushMatrix();
    glMultMatrixf(mat);     //translation,rotation
                   glBegin(GL_QUADS);
                           glVertex3f(-extent.x(),extent.y(),-extent.z());
                           glVertex3f(-extent.x(),-extent.y(),-extent.z());
                           glVertex3f(-extent.x(),-extent.y(),extent.z());
                           glVertex3f(-extent.x(),extent.y(),extent.z());
                   glEnd();
                   glBegin(GL_QUADS);
                           glVertex3f(extent.x(),extent.y(),-extent.z());
                           glVertex3f(extent.x(),-extent.y(),-extent.z());
                           glVertex3f(extent.x(),-extent.y(),extent.z());
                           glVertex3f(extent.x(),extent.y(),extent.z());
                   glEnd();
                   glBegin(GL_QUADS);
                           glVertex3f(-extent.x(),extent.y(),extent.z());
                           glVertex3f(-extent.x(),-extent.y(),extent.z());
                           glVertex3f(extent.x(),-extent.y(),extent.z());
                           glVertex3f(extent.x(),extent.y(),extent.z());
                   glEnd();
                   glBegin(GL_QUADS);
                           glVertex3f(-extent.x(),extent.y(),-extent.z());
                           glVertex3f(-extent.x(),-extent.y(),-extent.z());
                           glVertex3f(extent.x(),-extent.y(),-extent.z());
                           glVertex3f(extent.x(),extent.y(),-extent.z());
                   glEnd();
                   glBegin(GL_QUADS);
                           glVertex3f(-extent.x(),extent.y(),-extent.z());
                           glVertex3f(-extent.x(),extent.y(),extent.z());
                           glVertex3f(extent.x(),extent.y(),extent.z());
                           glVertex3f(extent.x(),extent.y(),-extent.z());
                   glEnd();
                   glBegin(GL_QUADS);
                           glVertex3f(-extent.x(),-extent.y(),-extent.z());
                           glVertex3f(-extent.x(),-extent.y(),extent.z());
                           glVertex3f(extent.x(),-extent.y(),extent.z());
                           glVertex3f(extent.x(),-extent.y(),-extent.z());
                   glEnd();
           glPopMatrix();
    glPopMatrix();
}
#endif
/* ------------------------------------------------------------- */
/* BRICK   BRICK   BRICK   BRICK   BRICK   BRICK   BRICK   BRICK */
/* ------------------------------------------------------------- */

SIMPLE_Brick::SIMPLE_Brick(int ind, const vector <double> & data, btDynamicsWorld* world) : SIMPLE_Objects( world )
{
  
  index = ind;
  pos.assign    (9, 0.0);
  start_pos.assign(3,0.0);
  //corners.assign (4,0.0);
  
  pos[0] = start_pos[0] = data[0];//X position
  pos[1] = start_pos[1] = data[1];//Y position
  pos[2] = start_pos[2] = data[2];//Z position
  
  dim[0] = data[3]; //Length on the X axis
  dim[1] = data[5]; //Height on the Y axis
  dim[2] = data[4]; //Width on the Z axis
  
  rot[1] = data[6]; //Rotation on the XZ plain
  
  angle = rot[1];//(rot[1]*PI)/180.0;
  
  pos[3] = pos[0] + ((dim[0]*0.5 * cos(angle)));
  pos[4] = pos[2] + ((dim[0]*0.5 * sin(angle)));
  pos[5] = pos[1];
  //corners[3] = pos[0] + ((dim[0]*0.5 * cos(angle)));
  //corners[4] = pos[2] + ((dim[0]*0.5 * sin(angle)));
  
  pos[6] = pos[0] + (-(dim[0]*0.5 * cos(angle)));
  pos[7] = pos[2] + (-(dim[0]*0.5 * sin(angle)));
  pos[8] = pos[1];
  //printf("pos[3] = %lf pos[4] = %lf pos[6] = %lf ")
  //corners[6] = pos[0] + (-(dim[0]*0.5 * cos(angle)));
  //corners[7] = pos[2] + (-(dim[0]*0.5 * sin(angle)));
  
  colour[0] = data[7]; //RED
  colour[1] = data[8]; //GREEN
  colour[2] = data[9]; //BLUE
  
  type_id  = BRICK;
  mass = data[10];
  rotation.setEulerZYX( 0.0, angle, 0.0);
  // for(int obj =0;obj<2;obj++){
  //   dim[0] +=0.10;
  btTransform t;
  t.setIdentity();
  t.setOrigin(btVector3(pos[0],pos[1]+dim[1]/2.0,pos[2]));
  t.setRotation(rotation);
  btBoxShape* brick=new btBoxShape(btVector3(dim[0]*0.5,dim[1]*0.5,dim[2]*0.5));
  brick->setMargin(0.005);
  btVector3 inertia(0,0,0);
  if(mass!=0.0)
    brick->calculateLocalInertia(mass,inertia);
  btMotionState* motion=new btDefaultMotionState(t);
  btRigidBody::btRigidBodyConstructionInfo info(mass,motion,brick,inertia);
  info.m_restitution = data[11];
  info.m_friction = data[12];
  info.m_rollingFriction = data[13];
  body[0]=new btRigidBody(info);
  body[0]->setDamping(data[14],data[15]);
  //body[0]->setCollisionFlags(body[0]->getCollisionFlags() | btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK );//Remove this is not collision recorded
  world->addRigidBody(body[0]);
  object_id = 0;
  
}

/* ------------------------------------------------------------ */

/* ------------------------------------------------------------ */

void SIMPLE_Brick::reset_pos(){
  btTransform trans = body[object_id]->getWorldTransform();
  trans.setOrigin(btVector3(start_pos[0],start_pos[1]+dim[1]/2.0,start_pos[2]));
  //trans.setOrigin(btVector3(0.0,start_pos[1]+dim[1]/2.0,0.0));
  //    angle = (gsl_rng_uniform_pos( GSL_randon_generator::r_rand )*2.0*PI)-PI;
  //    rotation.setEulerZYX( 0.0, angle, 0.0);
  trans.setRotation(rotation);
  body[object_id]->setWorldTransform(trans);
}


/* ------------------------------------------------------------ */
#ifdef _GRAPHICS_
void SIMPLE_Brick::render ( )
{
  if(body[object_id]->getCollisionShape()->getShapeType()!=BOX_SHAPE_PROXYTYPE)
    return;
  glColor3f(colour[0], colour[1], colour[2]);
  btVector3 extent=((btBoxShape*)body[object_id]->getCollisionShape())->getHalfExtentsWithMargin();
  btTransform t;
  body[object_id]->getMotionState()->getWorldTransform(t);
  float mat[16];
  t.getOpenGLMatrix(mat);
  glPushMatrix();
  glMultMatrixf(mat);     //translation,rotation
  glBegin(GL_QUADS);
  glVertex3f(-extent.x(),extent.y(),-extent.z());
  glVertex3f(-extent.x(),-extent.y(),-extent.z());
  glVertex3f(-extent.x(),-extent.y(),extent.z());
  glVertex3f(-extent.x(),extent.y(),extent.z());
  glEnd();
  glBegin(GL_QUADS);
  glVertex3f(extent.x(),extent.y(),-extent.z());
  glVertex3f(extent.x(),-extent.y(),-extent.z());
  glVertex3f(extent.x(),-extent.y(),extent.z());
  glVertex3f(extent.x(),extent.y(),extent.z());
  glEnd();
  glBegin(GL_QUADS);
  glVertex3f(-extent.x(),extent.y(),extent.z());
  glVertex3f(-extent.x(),-extent.y(),extent.z());
  glVertex3f(extent.x(),-extent.y(),extent.z());
  glVertex3f(extent.x(),extent.y(),extent.z());
  glEnd();
  glBegin(GL_QUADS);
  glVertex3f(-extent.x(),extent.y(),-extent.z());
  glVertex3f(-extent.x(),-extent.y(),-extent.z());
  glVertex3f(extent.x(),-extent.y(),-extent.z());
  glVertex3f(extent.x(),extent.y(),-extent.z());
  glEnd();
  glBegin(GL_QUADS);
  glVertex3f(-extent.x(),extent.y(),-extent.z());
  glVertex3f(-extent.x(),extent.y(),extent.z());
  glVertex3f(extent.x(),extent.y(),extent.z());
  glVertex3f(extent.x(),extent.y(),-extent.z());
  glEnd();
  glBegin(GL_QUADS);
  glVertex3f(-extent.x(),-extent.y(),-extent.z());
  glVertex3f(-extent.x(),-extent.y(),extent.z());
  glVertex3f(extent.x(),-extent.y(),extent.z());
  glVertex3f(extent.x(),-extent.y(),-extent.z());
  glEnd();
  glPopMatrix();
}
#endif

/* ------------------------------------------------------------ */
/* CYLINDER    CYLINDER    CYLINDER    CYLINDER    CYLINDER     */
/* ------------------------------------------------------------ */

SIMPLE_Cylinder::SIMPLE_Cylinder(int ind, const vector <double> & data,  btDynamicsWorld* world ) : SIMPLE_Objects( world  )
{
  index = ind;
  pos.assign    (3, 0.0);
  start_pos.assign(3,0.0);
  pos[0] = start_pos[0] = data[0];//X position
  pos[1] = start_pos[1] = data[1];//Y position
  pos[2] = start_pos[2] = data[2];//Z position
  
  dim[0] = data[4]; //width = dim[0] = radius
  dim[1] = data[5]; //height = dim[1] = height
  dim[2] = data[3];
  
  rot[1] = data[6]; // no need for rotation to Cylinder
  
  colour[0] = data[7];
  colour[1] = data[8];
  colour[2] = data[9];
  
  type_id  = CYLINDER;
  mass = data[10];
  btTransform t;
  t.setIdentity();
  t.setOrigin(btVector3(pos[0],pos[1] + dim[1]*0.5,pos[2]));
  btCylinderShape* cylinder=new btCylinderShape(btVector3(dim[0]*0.5,dim[1]*0.5,dim[0]*0.5));
  cylinder->setMargin(dim[1]*0.5);
  btVector3 inertia(0,0,0);
  if(mass!=0.0)
    cylinder->calculateLocalInertia(mass,inertia);
  
  btMotionState* motion=new btDefaultMotionState(t);
  btRigidBody::btRigidBodyConstructionInfo info(mass,motion,cylinder,inertia);
  info.m_restitution = data[11];
  info.m_friction = data[12];
  info.m_rollingFriction = data[13];
  body[0]=new btRigidBody(info);
  body[0]->setDamping(data[14],data[15]);
  //body[0]->setCollisionFlags(body[0]->getCollisionFlags() | btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK ); //Remove this is not collision recorded
  world->addRigidBody(body[0]);
  object_id = 0;
}

/* ------------------------------------------------------------ */

/* ------------------------------------------------------------ */

void SIMPLE_Cylinder::reset_pos(){

  btTransform trans = body[object_id]->getWorldTransform();
  trans.setOrigin(btVector3(pos[0],pos[1]+dim[1]/2.0,pos[2]));
  body[object_id]->setWorldTransform(trans);
  
}


/* ------------------------------------------------------------ */
#ifdef _GRAPHICS_

void SIMPLE_Cylinder::render ( )
{
  if(body[object_id]->getCollisionShape()->getShapeType()!=CYLINDER_SHAPE_PROXYTYPE)
    return;
  GLUquadric* quad = gluNewQuadric();
  glColor3f(colour[0], colour[1], colour[2]);
  btVector3 extent=((btCylinderShape*)body[object_id]->getCollisionShape())->getHalfExtentsWithMargin();
  btTransform t;
  body[object_id]->getMotionState()->getWorldTransform(t);
  float mat[16];
  t.getOpenGLMatrix(mat);
  glPushMatrix();
  glMultMatrixf(mat);
  //translation,rotation
  glTranslatef(0,extent.y(),0);
  glRotatef(90,1,0,0);
  gluCylinder(quad,extent.x(),extent.x(),extent.y()*2.0,100, 1);
  gluDisk(quad, 0, extent.x(), 100, 1);
  glTranslated(0,0,extent.y()*2.0);
  gluDisk(quad, 0, extent.x(), 100, 1);
  
  gluDeleteQuadric(quad);
  glPopMatrix();
}
#endif

/* ------------------------------------------------------------ */
/* SPHERE    SPHERE    SPHERE    SPHERE    SPHERE     SPHERE    */
/* ------------------------------------------------------------ */

SIMPLE_Sphere::SIMPLE_Sphere(int ind, const vector <double> & data, btDynamicsWorld* world ) : SIMPLE_Objects( world )
{
  index = ind;
  pos.assign    (3, 0.0);
  start_pos.assign(3,0.0);
  pos[0] = start_pos[0] = data[0];//X position
  pos[1] = start_pos[1] = data[1];//Y position
  pos[2] = start_pos[2] = data[2];//Z position
  
  dim[0] = data[3]; // only radius is required for sphere
  dim[1] = data[4];
  dim[2] = data[5];
  
  rot[1] = data[6]; // sphere have no orientation for rotation
  
  colour[0] = data[7];
  colour[1] = data[8];
  colour[2] = data[9];
  
  type_id  = SPHERE;
  mass = data[10];
  btTransform t;	//position and rotation
  t.setIdentity();
  t.setOrigin(btVector3(pos[0],pos[1]+dim[0],pos[2]));	//put it to x,y,z coordinates
  btSphereShape* sphere=new btSphereShape(dim[0]); //it's a sphere, so use sphereshape
  //sphere->setMargin(dim[0]);
  btVector3 inertia(0,0,0);	//inertia is 0,0,0 for static object, else
  if(mass!=0.0)
      sphere->calculateLocalInertia(mass,inertia);	//it can be determined by this function (for all kind of shapes)

  btMotionState* motion=new btDefaultMotionState(t);	//set the position (and motion)
  btRigidBody::btRigidBodyConstructionInfo info(mass,motion,sphere,inertia);	//create the constructioninfo, you can create multiple bodies with the same info
  info.m_restitution = data[11];
  info.m_friction = data[12];
  info.m_rollingFriction = data[13];
  body[0]=new btRigidBody(info);
  body[0]->setDamping(data[14],data[15]);
  //body[0]->setCollisionFlags(body[0]->getCollisionFlags() | btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK );//Remove this is not collision recorded
  world->addRigidBody(body[0]);	//and let the world know about it
  object_id = 0;
}

/* ------------------------------------------------------------ */

/* ------------------------------------------------------------ */

void SIMPLE_Sphere::reset_pos(){
  
  btTransform trans = body[object_id]->getWorldTransform();
  trans.setOrigin(btVector3(pos[0],pos[1]+dim[0],pos[2]));
  body[object_id]->setWorldTransform(trans);
  
}


/* ------------------------------------------------------------ */
#ifdef _GRAPHICS_
void SIMPLE_Sphere::render ( )
{
  if(body[object_id]->getCollisionShape()->getShapeType()!=SPHERE_SHAPE_PROXYTYPE)	//only render, if it's a sphere
    return;
  glColor3f(colour[0], colour[1], colour[2]);
  GLUquadric* quad = gluNewQuadric();
  float r=((btSphereShape*)body[object_id]->getCollisionShape())->getRadius();
  btTransform t;
  body[object_id]->getMotionState()->getWorldTransform(t);	//get the transform
  float mat[16];
  t.getOpenGLMatrix(mat);	//OpenGL matrix stores the rotation and orientation
  glPushMatrix();
  glMultMatrixf(mat);	//multiplying the current matrix with it moves the object in place
  gluSphere(quad,r,100,100);
  gluDeleteQuadric(quad);
  glPopMatrix();
}
#endif
/* ------------------------------------------------------------ */

/* ------------------------------------------------------------ */
/* ------------------------------------------------------------ */

