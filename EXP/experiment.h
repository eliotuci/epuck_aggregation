#ifndef _EXPERIMENT_
#define _EXPERIMENT_

#include "../MISC/parameters.h"

//Evolutionary Algorithms
#include "../EA/population.h"
#include "../EA/roulette_wheel.h"

// Controllers
#include "../CONTROLLERS/controller.h"
#include "../CONTROLLERS/ctrnn3Layers.h"
#include "../CONTROLLERS/ctrnn3LayersHebb.h"
#include "../CONTROLLERS/ctrnnFullyRecurrent.h"
#include "../CONTROLLERS/perceptron.h"
#include "../CONTROLLERS/simple3Layers.h"
#include "../CONTROLLERS/elman.h"
#include "../CONTROLLERS/pfsm.h"

//World
#include "../WORLD/world_entity.h"
#include "../WORLD/simple_agents.h"
#include "../WORLD/simple_objects.h"


class Experiment {
  
 private:
 protected:
  unsigned int m_evaluation;
  unsigned int m_iteration;
  double m_physicsStep;
  static const int NbPhysicsWorldUpdates = 6;
  static const int m_id_population       = 1;
  
  Parameters *m_parameters;
  std::vector< Controller* >     m_networks;
  std::vector< Population* >     m_population;
  std::vector< SIMPLE_Agents* >  m_agents;
  std::vector< SIMPLE_Objects*>  m_plane;
  std::vector< SIMPLE_Objects*>  m_objects;
  
  unsigned int m_root;
  unsigned int m_NbRepetitions;
  unsigned int m_NbAlleles;
  unsigned int m_NbBasesPerAllele;
  unsigned int m_NbGenesPerInd;
  unsigned int m_count_rep;
  unsigned int m_start;
  unsigned int m_end;
  vector <chromosome_type> genes;
  vector <double> fitness;
  
  /* ---------------------------------------------- */
  vector < int > m_pop_one_solutions;
  vector < int > m_pop_solutions_received;
  vector< vector <int> > m_teamTuples;
  vector <double> m_tmp_get_fitness;
  vector< vector <int> > m_listOfTuplesPerProcess;
  
  /* ---------------------------------------------- */
  
  unsigned int m_nbActiveAgents;
  
  //World with Bullet
  btCollisionConfiguration*  m_collisionConfig;
  btDispatcher*              m_dispatcher;
  btBroadphaseInterface*     m_broadphase;
  btConstraintSolver*        m_solver;
  btDynamicsWorld*           m_world;
    
 public:
  Experiment( Parameters* params );
  ~Experiment();

  virtual void evaluate_solutions         ( void ) = 0;
  virtual void init_evaluations_loop      ( void ) = 0;
  virtual void init_single_evaluation     ( void ) = 0;
  virtual void set_agents_position        ( void ) = 0;
  virtual void adv                        ( void ) = 0;
  virtual bool stop_evaluations_loop      ( void ) = 0;
  
  inline unsigned int getEvaluation       ( void ) const { return m_evaluation; }
  inline unsigned int getIteration        ( void ) const { return m_iteration; }
  
  //virtual unsigned int getEvaluation      ( void ) = 0;
  //virtual unsigned int getIteration       ( void ) = 0;
  
  void initAgentsController                 ( void );
  void initEvolutionaryAlgorithm            ( void );
  void send_new_population_to_all_processes ( void );
  void send_solutions_to_all_processes      ( void );
  void receive_fitness_from_all_processes   ( void );

  void runOneGeneration                     ( void );

  //Bullet world 
  void init_physics_param                     ( void );
  void resetPhysicsState                      ( void );
  void init_agents                            ( void );
  void init_objects                           ( void );
  void add_agent                              ( SIMPLE_Agents* agent);
  void remove_agent                           ( SIMPLE_Agents *agent);
  void from_genome_to_controllers             ( vector < vector <chromosome_type> > &genes, const int which_genotype );
  void defineHeteroTeamsRandomlyAssembled     ( void );
  void defineHeteroTeamsSequentiallyAssembled ( void );
  void defineHomogeneousTeams                 ( void );
  void draw_arena                             ( void );
  
  inline std::vector< Controller* >     getNetworks ()   { return m_networks;}
  inline std::vector< Population* >     getPopulation()  { return m_population;}
  inline std::vector< SIMPLE_Agents* >  getAgents()      { return m_agents;}
  inline std::vector< SIMPLE_Objects*>  getPlane()       { return m_plane;}
  inline std::vector< SIMPLE_Objects*>  getObjects()     { return m_objects;}
  
};
#endif
