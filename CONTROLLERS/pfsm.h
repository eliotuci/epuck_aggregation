#ifndef _PFSM_
#define _PFSM_

#include "controller.h"

class Pfsm : public Controller
{
  
 private:
  double rho; //rho = 1 zero turning angle - rho = 0 angle drawn from uniform distribution in [0, PI]
  int forward_movement_lenght;
  int count_step_forward;
  int turning_lenght;
  int count_step_turn;
  bool turning_angle_already_computed;
  bool turning_clockwise;
  
  
  void copy(const Pfsm &other);
  void allocate( );
  void destroy();
  
 public:
  Pfsm(  );
  Pfsm(const Pfsm& other);
  virtual ~Pfsm();
  
  
  /* -------------------------------------------------------------------------------------------------- */
  /*                                             VIRTUAL FUNCTIONS                                      */
  /* -------------------------------------------------------------------------------------------------- */
  void init                          ( const vector <chromosome_type> &genes );
  void step                          ( const vector <double> &input_array, vector <double> &output_array);
  void reset                         ( void );
  void read_from_file                ( void );
  void compute_genotype_length       ( void );
  /* -------------------------------------------------------------------------------------------------- */
  /* -------------------------------------------------------------------------------------------------- */

  int random_turn_lenght             ( void );
  void move_forward                  ( const vector <double> &input_array, vector <double> &output_array);
  void turn_on_spot                  ( const vector <double> &input_array, vector <double> &output_array);
  
  Pfsm& operator=(const Pfsm &other);
};

#endif
