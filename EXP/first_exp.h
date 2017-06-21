#ifndef _FIRST_EXPERIMENT_
#define _FIRST_EXPERIMENT_

#include "experiment.h"

class First_experiment : public Experiment
{
 private:
  vector < vector < double > > init_agents_pos;
  vector < vector < double > > init_agents_rot;
  vector < vector < double > > inputs;      // sensory inputs 
  vector < vector < double > > outputs;     // outputs generated by the controller, used to move robots
  vector < vector < double > > camera_sector_readings;
  vector < vector < double > > optic_flow_sensor_readings;


  /* ------------------------------ */
  vector <double> m_init_agent_dist_to_object;
  double m_max_obj_displacement;
  double m_obj_displacement;
  double m_time_taken_to_succeed;
  /* ------------------------------ */
    
 public:
  First_experiment( Parameters* params );
  virtual ~First_experiment();

  void evaluate_solutions                 ( void );
  void init_evaluations_loop              ( void );
  void init_single_evaluation             ( void );
  void adv                                ( void );
  bool stop_iterations_loop               ( void );
  bool stop_evaluations_loop              ( void );
  void finalise_single_evaluation         ( void );
  void finalise_evaluation_loop           ( void );
  void set_agents_position                ( void );

  void update_sensors                     ( void );
  void update_controllers                 ( void );
  void update_actuators                   ( void );
  void update_world                       ( void );
  void update_optic_flow_sensors          ( void );
  void manage_collisions                  ( void );
  void compute_fitness                    ( void );
  
};
#endif