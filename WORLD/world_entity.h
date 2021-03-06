#ifndef _WORLD_ENTITY_
#define _WORLD_ENTITY_

#include <map>
#include <cmath>
#include <btBulletDynamicsCommon.h>
#include <btBulletCollisionCommon.h>
#include <bullet/btBulletDynamicsCommon.h>
#ifdef _GRAPHICS_
#include <GL/glut.h>
#endif
#include "../MISC/general.h"

enum {WALL = 0, CYLINDER=1, SPHERE=2, LIGHT=3, BRICK = 4, ROBOT=5, PLANE = 6};

#define NOISE_LEVEL 1.0

using namespace std;

class World_Entity {
  
 protected:
  vector <double> pos;
  vector <double> rot;
  vector <double> dim;
  vector <double> colour;
  
  int    index;
  int    type_id;
  bool   crashed;
  
  
 public:
  template<class T>
  struct Collisions
  {
    T index;
    T id;
  };
  template<class T> Collisions<T>
  MakeIndexWithId(const T _index, const T _id )
    {
      Collisions<T> ret;
      ret.index = _index;
      ret.id = _id;
      return ret; 
    }
  vector <Collisions <int> > collision_memory;
  
  double mass;
  static double noise_level;
  
  World_Entity(){}
  virtual ~World_Entity(){}
  
  inline const int     get_index    ( void ) { return index;}
  inline const bool    is_crashed   ( void ) { return crashed;}
  inline const double  get_mass     ( void ) { return mass;}
  inline const int     get_type_id  ( void ) { return type_id;}

  inline void          set_crashed  ( bool val ) { crashed = val;}
  
  
  virtual void  set_pos                     ( const vector <double> &_pos ) = 0;
  virtual void  set_rot                     ( const vector <double> &_rot ) = 0;
  virtual const vector <double> get_pos     ( void ) = 0;
  virtual const vector <double> get_rot     ( void ) = 0;
  virtual const vector <double> get_dim     ( void ) = 0;
  virtual const vector <double> get_colour  ( void ) = 0;
  
  
  /* ------------------------------------------------------------ */
  inline void set_colour( const vector <double> &c ) { 
    colour[0] = c[0];
    colour[1] = c[1];
    colour[2] = c[2];
  }
  /* ------------------------------------------------------------ */
  
};

#endif


