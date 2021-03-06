#include "experiment.h"

/* ----------------------------------------------------------------------------------- */

Experiment::Experiment( Parameters* params ){
  m_root        = 0;
  m_parameters  = params;

  // m_teamTuples is the vector in which I store the solutions to be evaluated by each node
  if ( m_parameters->AreGroupsHeteroGroups() ){//Heterogeneous groups
    if( m_parameters->getNbMatches() > 1 ){
      m_teamTuples.resize ( (m_parameters->getNbGroups()/m_parameters->getNbAgentsPerGroup() ) * m_parameters->getNbMatches() );
      if ( NbRank == m_root ) defineHeteroTeamsRandomlyAssembled();
    }
    else if( m_parameters->getNbMatches() == 1 ){
      m_teamTuples.resize ( m_parameters->getNbGroups() );
      if ( NbRank == m_root ) defineHeteroTeamsSequentiallyAssembled( );
    }
  }
  else{//Homogeneous groups
    m_teamTuples.resize ( m_parameters->getNbGroups() );
    if( m_parameters->IsSelectionIndividual() ){
      if ( NbRank == m_root ) defineHeteroTeamsSequentiallyAssembled( );
    }
    else{
      if ( NbRank == m_root ) defineHomogeneousTeams();
    }
  }
  
  if( !( m_teamTuples.size() % NbTotProcesses) ) {
    m_NbRepetitions = m_teamTuples.size() / NbTotProcesses;
    m_pop_solutions_received.resize ( m_parameters->getNbAgentsPerGroup() );
    
    if(NbRank == m_root ){
      m_listOfTuplesPerProcess.resize( NbTotProcesses );
      int countTuples = 0;
      for(int i = 0; i < NbTotProcesses; i++){
	for(int j = 0; j < m_NbRepetitions; j++){
	  m_listOfTuplesPerProcess[i].push_back( countTuples++ );
	  //cerr << " m_listOfTuplesPerProcess["<<i<<"]["<<j<<"] = " <<m_listOfTuplesPerProcess[i][j] << endl; 
	}
      }
    }
  }
  else {
    if(NbRank == m_root ){
      cerr <<" In Experiment.cpp constructor " << "\n The number of teamTuples is: " << m_teamTuples.size() 
	   << " \n The number of computer node is: " << NbTotProcesses
	   << " \n Remember that (nb of teamTuples % nb of computer node) = 0 "
	   << " \n Please check Experiment.cpp constructor to fix the problem by changing either the number of groups, or the number of agents per group, or the number of matches."
	   << endl;
    }
    exit(EXIT_FAILURE);
  }

  //Init Agents Controllers
  initAgentsController( );
    
  //Init Agents' World
  init_physics_param();
  init_objects();
  init_agents();
  
  m_physicsStep = m_parameters->getSimulationTimeStep()/(double)(NbPhysicsWorldUpdates);
      
  //Here you need to determine the number of parameters per solution
  m_NbAlleles        = m_networks[0]->get_genotype_length();
  m_NbBasesPerAllele = 1;
  m_NbGenesPerInd    = m_NbAlleles * m_NbBasesPerAllele;

  //fitness is the vector in which each process store the fitness of individuals/groups
  fitness.erase  ( fitness.begin(), fitness.begin() + fitness.size() );
  fitness.clear();
  if( !m_parameters->IsSelectionIndividual() ){  //Group selection
    if ( m_parameters->AreGroupsHeteroGroups() ){//Heterogeneous groups
      fitness.assign ( (m_parameters->getNbGroups() * m_parameters->getNbAgentsPerGroup()), 0.0 );
    }
    else{//Homogeneous Groups
      fitness.assign ( m_parameters->getNbGroups(), 0.0 );
    }
  }
  else{//Individual selection
    if ( m_parameters->AreGroupsHeteroGroups() ){//Heterogeneous groups
      if( m_parameters->getNbMatches() == 1 ) {
	fitness.assign      ( m_parameters->getNbGroups() * m_parameters->getNbAgentsPerGroup(), 0.0 );
      }
      else if( m_parameters->getNbMatches() > 1 ) {
	fitness.assign ( m_parameters->getNbGroups(), 0.0 );
      }
      else{
	cerr <<" In experiment.cpp : number of matches has to be greater or equal to 1 \n" << endl;
	exit(EXIT_FAILURE);
      }
    }
    else {//Groups are homogeneous
      fitness.assign ( (m_parameters->getNbGroups() * m_parameters->getNbAgentsPerGroup()), 0.0 );
    }
  }

  //genes is the vector in which each process store the genes of the individuals to evaluate
  genes.erase ( genes.begin(), genes.begin() + genes.size() );
  genes.clear();
  if ( m_parameters->AreGroupsHeteroGroups() ){//Heterogeneous groups
    if( m_parameters->getNbMatches() == 1 ){
      genes.assign ( (m_NbGenesPerInd * m_parameters->getNbGroups() * m_parameters->getNbAgentsPerGroup()), 0.0 );
    }
    else if( m_parameters->getNbMatches() > 1 ){
      genes.assign ( (m_NbGenesPerInd * m_parameters->getNbGroups() ), 0.0 );
    }
  }
  else{ //Homogeneous groups
    if( m_parameters->IsSelectionIndividual() ){
      genes.assign ( (m_NbGenesPerInd * m_parameters->getNbGroups() * m_parameters->getNbAgentsPerGroup()), 0.0 );
    }
    else{
      genes.assign ( (m_NbGenesPerInd * m_parameters->getNbGroups() ), 0.0 );
    }
  }
  
  if( m_parameters->isModeEvolution() ){
    //init GAs
    if(NbRank == m_root ){
      initEvolutionaryAlgorithm ( );
    }
  }
}

/* ----------------------------------------------------------------------------------- */

Experiment::~Experiment( void ){
  // Delete network objects
  for(int n = 0; n < m_networks.size() ; n++)
    delete m_networks[n];
  m_networks.clear();

  // Delete agents
  for(int n = 0; n < m_agents.size() ; n++)
    delete m_agents[n];
  m_agents.clear();

  // Delete objects
  for(int n = 0; n < m_objects.size() ; n++)
    delete m_objects[n];
  m_objects.clear();

  // Delete plane
  for(int n = 0; n < m_plane.size() ; n++)
    delete m_plane[n];
  m_plane.clear();
  
  if( NbRank == m_root ) {
    for(int n = 0; n < m_population.size(); n++)
      delete m_population[n];
    m_population.clear();
  }

  //delete m_collisionConfig;
  //delete m_dispatcher;
  //delete m_broadphase;
  //delete m_solver;
  //delete m_world;
  
}

/* ----------------------------------------------------------------------------------- */

void Experiment::initAgentsController( void ){
  std::string typeOfController   = m_parameters->getTypeOfController();

  if ( typeOfController == "ctrnn_three_layers" )
    {
      for(int r = 0; r < m_parameters->getNbAgentsPerGroup (); r++)
	m_networks.push_back(new Ctrnn3Layers());
    }
  else if ( typeOfController == "ctrnn_three_layers_hebb" )
    {
      for(int r = 0; r < m_parameters->getNbAgentsPerGroup (); r++)
	m_networks.push_back(new Ctrnn3LayersHebb());
    }
  else if ( typeOfController == "simple_three_layers" )
    {
      for(int r = 0; r < m_parameters->getNbAgentsPerGroup (); r++)
	m_networks.push_back(new Simple3Layers());
    }
  else if ( typeOfController == "perceptron" )
    {
      for(int r = 0; r < m_parameters->getNbAgentsPerGroup (); r++)
	m_networks.push_back(new Perceptron());
    }
  else if ( typeOfController == "elman" )
    {
      for(int r = 0; r < m_parameters->getNbAgentsPerGroup (); r++)
	m_networks.push_back(new Elman());
    }
  else if ( typeOfController == "ctrnn_fully_recurrent" )
    {
      for(int r = 0; r < m_parameters->getNbAgentsPerGroup (); r++)
	m_networks.push_back(new CtrnnFullyRecurrent());
    }
  else if ( typeOfController == "pfsm" )//Prababilistic finite state machine
    {
      for(int r = 0; r < m_parameters->getNbAgentsPerGroup (); r++)
	m_networks.push_back(new Pfsm());
    }
  else
    {
      cerr << "In Experiment::initControllers - Controller type not found " << endl;
      exit(EXIT_FAILURE);
    }
}

/* ----------------------------------------------------------------------------------- */

void Experiment::initEvolutionaryAlgorithm ( void ){
  std::string typeOfGA = m_parameters->getTypeOfGA();
  if ( typeOfGA == "roulette_wheel" )
    {
      for(int r = 0; r < m_parameters->getNbEvolvingPopulations(); r++)
	m_population.push_back( new Roulette_wheel( r, m_parameters, m_NbAlleles, m_NbBasesPerAllele ) );
    }
  else
    {
      cerr << "In Experiment::initEvolutionaryAlgorithm - GAs type not found " << endl;
      exit(EXIT_FAILURE);
    }     
}

/* ----------------------------------------------------------------------------------- */

void Experiment::resetPhysicsState( void )
{
  int numObjects = 0;
  int i;
  
  if (m_world)
    {
      for (i=0; i<m_world->getNumConstraints(); i++)
        {
	  m_world->getConstraint(0)->setEnabled(true);
        }
      numObjects = m_world->getNumCollisionObjects();
      
      ///create a copy of the array, not a reference!
      btCollisionObjectArray copyArray = m_world->getCollisionObjectArray();
      for (i=0;i<numObjects;i++)
        {
	  btCollisionObject* colObj = copyArray[i];
	  btRigidBody* body = btRigidBody::upcast(colObj);
	  if (body)
            {
	      if (body->getMotionState())
                {
		  btDefaultMotionState* myMotionState = (btDefaultMotionState*)body->getMotionState();
		  myMotionState->m_graphicsWorldTrans = myMotionState->m_startWorldTrans;
		  body->setCenterOfMassTransform( myMotionState->m_graphicsWorldTrans );
		  colObj->setInterpolationWorldTransform( myMotionState->m_startWorldTrans );
		  colObj->forceActivationState(ACTIVE_TAG);
		  colObj->activate();
		  colObj->setDeactivationTime(0);
		  //colObj->setActivationState(WANTS_DEACTIVATION);
                }
	      //removed cached contact points (this is not necessary if all objects have been removed from the dynamics world)
	      if (m_world->getBroadphase()->getOverlappingPairCache())
		m_world->getBroadphase()->getOverlappingPairCache()->cleanProxyFromPairs(colObj->getBroadphaseHandle(), m_dispatcher);
	      
	      btRigidBody* body = btRigidBody::upcast(colObj);
	      if (body && !body->isStaticObject())
                {
		  btRigidBody::upcast(colObj)->setLinearVelocity(btVector3(0,0,0));
		  btRigidBody::upcast(colObj)->setAngularVelocity(btVector3(0,0,0));
                }
            }
	  
        }
      
      ///reset some internal cached data in the broadphase
      m_world->getBroadphase()->resetPool(m_dispatcher);
      m_world->getConstraintSolver()->reset();
    }
}

/* ------------------------------------------------------------------------ */

bool callbackFunc(btManifoldPoint& cp, const btCollisionObjectWrapper* obj1, int id1, int index1, const btCollisionObjectWrapper* obj2, int id2, int index2);

/* ------------------------------------------------------------------------ */

bool callbackFunc(btManifoldPoint& cp, const btCollisionObjectWrapper* obj1, int id1, int index1, const btCollisionObjectWrapper* obj2, int id2, int index2)
{
  //cerr << " callback " << endl;
  // World_Entity* object1 = (World_Entity*)obj1->getCollisionObject()->getUserPointer();
  // World_Entity* object2 = (World_Entity*)obj2->getCollisionObject()->getUserPointer();

  // if( object1->get_type_id() == ROBOT ){
  //   object1->collision_memory.push_back( object1->MakeIndexWithId ( object2->get_index(), object2->get_type_id() ) );
  // }

  // if( object2->get_type_id() == ROBOT ){
  //   object2->collision_memory.push_back( object2->MakeIndexWithId( object1->get_index(), object1->get_type_id() ) );
  // }
//   if(object1->get_type_id() != PLANE && object2->get_type_id() != PLANE ){  
//     object1->collision_objects[object1->collision_counter] = new int[2];
//     object1->collision_objects[object1->collision_counter][0] = object2->get_index();
//     object1->collision_objects[object1->collision_counter][1] = object2->get_type_id();
//     object1->collision_counter++;
    
//     object2->collision_objects[object2->collision_counter] = new int[2];
//     object2->collision_objects[object2->collision_counter][0] = object1->get_index();
//     object2->collision_objects[object2->collision_counter][1] = object1->get_type_id();
//     object2->collision_counter++;
//   }
  return false;
}

/* ----------------------------------------------------------------------------------- */

void Experiment::init_physics_param( void ){
  this->m_collisionConfig = new btDefaultCollisionConfiguration();
  this->m_dispatcher      = new btCollisionDispatcher(m_collisionConfig);
  btVector3 worldMin(-5,-1,-5);
  btVector3 worldMax(5,1,5);
  this->m_broadphase      = new btAxisSweep3(worldMin,worldMax);
  this->m_solver          = new btSequentialImpulseConstraintSolver();
  this->m_world           = new btDiscreteDynamicsWorld(m_dispatcher,m_broadphase,m_solver,m_collisionConfig);
  this->m_world->setGravity(btVector3(0.0,-9.81,0.0));  //gravity on Earth
  this->m_world->getSolverInfo().m_numIterations = 110;
  m_world->getSolverInfo().m_solverMode |= SOLVER_USE_2_FRICTION_DIRECTIONS  | SOLVER_SIMD;// | SOLVER_RANDMIZE_ORDER;
  //gContactAddedCallback = callbackFunc; //Comment this function in case you do not need to record collisions.
}

/* ----------------------------------------------------------------------------------- */

void Experiment::init_agents( void ){
  vector <double> colour;
  colour.assign(3, 1.0);
  m_nbActiveAgents = 0;
  for (int r = 0; r < m_parameters->getNbAgentsPerGroup(); r++){
    m_agents.push_back( new SIMPLE_Agents ( r, m_parameters->getSimulationTimeStep(), m_world) );
    m_agents.back()->set_crashed(false);
    m_agents.back()->body->setUserPointer( m_agents.back() );
    m_agents.back()->left_wheel->setUserPointer( m_agents.back() );
    m_agents.back()->right_wheel->setUserPointer( m_agents.back() );
    colour[0] = 1.0;
    colour[1] = 0.0;
    colour[2] = 0.0;
    m_agents.back()->set_colour( colour );
    m_nbActiveAgents++;
  }
}

/* ----------------------------------------------------------------------------------- */

void Experiment::add_agent(SIMPLE_Agents* agent){
  vector <double> pos;
  pos.assign(3,0.0);
  vector <double> rot;
  rot.assign(3,0.0);
  pos[0] = 4.80;
  pos[2] = 4.80;
  agent->set_robot_pos_rot( pos, rot );
  agent->right_hinge->setLimit(1,-1,1.0,0.3,1);
  agent->left_hinge->setLimit(1,-1,1.0,0.3,1);
  agent->body->setActivationState(1);
  agent->right_wheel->setActivationState(1);
  agent->left_wheel->setActivationState(1);
  agent->set_removed(false);
  m_nbActiveAgents++;
}

/* ----------------------------------------------------------------------------------- */

void Experiment::remove_agent(SIMPLE_Agents *agent){
    vector <double> pos;
    pos.assign(3,0.0);
    vector <double> rot;
    rot.assign(3,0.0);
    pos[0] = 5.8;
    pos[2] = 5.8;
    agent->set_robot_pos_rot( pos, rot );
    agent->right_hinge->setLimit(0.0, 0.0);
    agent->left_hinge->setLimit(0.0, 0.0);
    agent->body->setActivationState(0);
    agent->right_wheel->setActivationState(0);
    agent->left_wheel->setActivationState(0);
    agent->set_removed(true);
    m_nbActiveAgents--;
}

/* ----------------------------------------------------------------------------------- */

void Experiment::draw_arena ( void ){
  double radius =  (2.0 * m_parameters->getObjectsDatas()[0][3])/(sqrt(6)-sqrt(2));
  double apotema = (radius/4.0) * (sqrt(6)+sqrt(2));
  double x_pos = apotema;
  double z_pos = 0.0;
  double angle = PI/6;
  double xz_rot = PI/2;  
  
  for(int ob = 0; ob < 12; ob++){
    m_parameters->setObjectsDatas (0, 0, x_pos );
    m_parameters->setObjectsDatas (0, 2, z_pos );
    m_parameters->setObjectsDatas (0, 6, xz_rot );
    
    m_objects.push_back( new SIMPLE_Brick( ob, m_parameters->getObjectsDatas()[0], m_world ) );
    m_objects.back()->body[0]->setUserPointer( m_objects.back() );
    m_objects.back()->set_crashed(false);
    x_pos = apotema * cos(angle*(ob+1));
    z_pos = apotema * sin(angle*(ob+1));
    xz_rot += -PI/6;
  }
}

/* ----------------------------------------------------------------------------------- */

void Experiment::init_objects( void ){
  m_plane.push_back( new SIMPLE_Plane( m_world ) );
  m_plane[0]->body[0]->setUserPointer( m_plane[0] );
  
  if( m_parameters->getNbObjects() )
    {
      draw_arena();
      
      /*       unsigned int count = 0, ob = 0;
        while( count < m_parameters->getNbBricks() && ob < m_parameters->getNbObjects() )
        {
    	  m_objects.push_back( new SIMPLE_Brick( ob, m_parameters->getObjectsDatas()[ob], m_world ) );
    	  //body->setUserPointer(bodies[bodies.size()-1]);
    	  m_objects.back()->body[0]->setUserPointer( m_objects.back() );
    	  //m_objects.back()->body[1]->setUserPointer( m_objects.back() );
    	  //m_objects.back()->body[2]->setUserPointer( m_objects.back() );
    	  m_objects.back()->set_crashed(false);
    	  ob++;
    	  count++;
        }
      count = 0;
      while( count < m_parameters->getNbCylinders() && ob < m_parameters->getNbObjects() )
        {
    	  m_objects.push_back( new SIMPLE_Cylinder ( ob, m_parameters->getObjectsDatas()[ob], m_world ) );
    	  m_objects.back()->body[0]->setUserPointer(m_objects.back() );
    	  m_objects.back()->set_crashed(false);
    	  ob++;
    	  count++;
        }
      count = 0;
      while( count < m_parameters->getNbSpheres() && ob < m_parameters->getNbObjects() )
    	{
    	  m_objects.push_back( new SIMPLE_Sphere ( ob, m_parameters->getObjectsDatas()[ob], m_world) ); 
    	  m_objects.back()->body[0]->setUserPointer( m_objects.back() );
    	  m_objects.back()->set_crashed(false);
    	  ob++;
    	  count++;
	  }*/
    } 
}

/* ----------------------------------------------------------------------------------- */

void Experiment::from_genome_to_controllers( vector < vector <chromosome_type> > &genes, const int which_group )
{  
  for(int ind=0; ind < m_agents.size(); ind++)
    {
      cerr << " Ind = " << ind << endl;
      // //m_networks[ind]->set_genotype_length ( genes[(which_group*m_agents.size())+ind].size() );

      // m_networks[ind]->set_genotype_length ( genes[which_group+ind].size() );

      // //m_networks[ind]->init( genes[(which_group*m_agents.size())+ind] );
      // m_networks[ind]->init( genes[which_group+ind] );

      //for (int j = 0; j < genes[which_group].size(); j++){
      //cerr << " genes["<<j<<"] = " << genes[which_group][j];
      //}
      //cerr  << endl;
    }
}


/* ----------------------------------------------------------------------------------- */

void Experiment::send_new_population_to_all_processes( void ){
  //All processes must stop here and wait for the others
  MPI_Barrier(MPI_COMM_WORLD);
  
  if( NbRank == m_root ){
    vector <chromosome_type> tmp_genes;
    tmp_genes.erase(tmp_genes.begin(), tmp_genes.begin() + tmp_genes.size() );
    tmp_genes.clear();
    
    for(int g = 0; g < m_parameters->getNbGroups(); g++){
      for(int ind = 0; ind < m_population[0]->get_nb_agents_per_group(); ind++){
	//cerr << " Nb Genes = " << m_population[0]->get_solution (g, 0).size() << endl;
	//getchar();
	//for(int ind = 0; ind < m_population[0]->get_solution (g, 0).size(); ind++){
	//cerr << " Pop["<<g<<"].Genes["<<ind<<"] = " << m_population[0]->get_solution (g, 0)[ind] << endl;
	//tmp_genes.push_back( m_population[0]->get_solution (g, 0)[ind] );
	//} 
	tmp_genes.insert ( std::end(tmp_genes), std::begin(m_population[0]->get_solution (g, ind)), std::end(m_population[0]->get_solution (g, ind)) );
      }
    }
    /*
      int g = 0;
      for(int i = 0; i < (m_parameters->getNbGroups()*m_population[0]->get_nb_agents_per_group()); i++){
      double tsum = 0.0;
      for(int j = 0 ; j < m_NbGenesPerInd; j++){
      tsum += tmp_genes[g++];
      }
      cerr << " In Exp.cpp tsum["<<i<<"] = " << tsum/(double)(m_NbGenesPerInd) << endl;
      getchar();
      } */
    
    /* for(int i = 0; i < tmp_genes.size(); i++){
      cerr << " tmp_genes["<<i<<"] = " << tmp_genes[i]
      << " NbGroups = " << m_parameters->getNbGroups() << " NbGenesPerInd = " << m_NbGenesPerInd << endl;
      }
      cerr << " send solution to node " << endl;
      getchar(); */
    
    for (int toRank = 0; toRank < NbTotProcesses; toRank++)
      {
	//This is the solution assigned to core m_root
	if( toRank == m_root ){
	  genes = tmp_genes;
	}
	else{
	  /* send values in tmp_vec to process with rank i, with i equal to the i^th individual */
#ifdef DOUBLE_GENES_TYPE
	  MPI_Send(&tmp_genes[0], tmp_genes.size(), MPI_DOUBLE, toRank, 0, MPI_COMM_WORLD);
#endif
#ifdef INT_GENES_TYPE
	  MPI_Send(&tmp_genes[0], tmp_genes.size(), MPI_INT, toRank, 0, MPI_COMM_WORLD);
#endif
	}
      }
  }
  else {
#ifdef DOUBLE_GENES_TYPE
    MPI_Recv(&genes[0], genes.size(), MPI_DOUBLE, 0, 0, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
#endif
#ifdef INT_GENES_TYPE
    MPI_Recv(&genes[0], genes.size(), MPI_INT, 0, 0, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
#endif    
  }
  
  /*
    if( NbRank == 3){
    for(int i = 0; i < genes.size(); i++){
    cerr << " genes["<<i<<"] = " << genes[i] << endl;
    }
    }
    if( NbRank == 0 ) getchar(); */
}


/* ----------------------------------------------------------------------------------- */

void Experiment::send_solutions_to_all_processes( void ){
  //All processes must stop here and wait for the others
  MPI_Barrier(MPI_COMM_WORLD);

  if( NbRank == m_root ){
    for (int i = m_start; i < m_end; i++)
      {
	int toRank = i % NbTotProcesses;
	vector <int> solution;
	solution.erase( solution.begin(), solution.begin() + solution.size() );
	solution.clear();
	
	for(int j = 0; j < m_parameters->getNbAgentsPerGroup(); j++){
	  solution.push_back( m_teamTuples[m_listOfTuplesPerProcess[toRank][m_count_rep]][j] );
	  //cerr << " solution["<<j<<"] = " << solution[j] << endl; 
	}
	
	/* cerr << " m_count_rep = " << m_count_rep << " rank = " << toRank
	     << " m_listTuples = " << m_listOfTuplesPerProcess[toRank][m_count_rep]<< "\n";
	for(int z = 0; z < solution.size(); z++)
	  cerr << " sol["<<z<<"]= " << solution[z] << endl;
	  getchar(); */
	  
	if( toRank == m_root ){
	  //for(int j = 0; j < solution.size(); j++){
	  //m_pop_solutions_received[j] = solution[j];
	  //}
	  m_pop_solutions_received = solution;
	}
	else{
	  //Root process sends to other cores indeces of the solutions to be evaluated 
	  MPI_Send(&solution[0], solution.size(), MPI_INT, toRank, 0, MPI_COMM_WORLD);
	}
      }
  }
  else {
    //Every core receives from root the indeces of the solutions 
    MPI_Recv(&m_pop_solutions_received[0], m_pop_solutions_received.size(), MPI_INT, 0, 0, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
  }
  
  /*  if( NbRank == 1 ){
      cerr << " rank = " << NbRank;
      for(int j = 0; j < m_parameters->getNbAgentsPerGroup(); j++){
      cerr << " S"<<j<<" = " << m_pop_solutions_received[j];
      }
      cerr << endl;
      }
      if( NbRank == m_root )  getchar(); */
}

/* ----------------------------------------------------------------------------------- */

void Experiment::receive_fitness_from_all_processes(  ){
  //Stop and wait for everyone to finalise the evalution
  MPI_Barrier(MPI_COMM_WORLD);
  
  if (NbRank == m_root){
    m_tmp_get_fitness.erase(m_tmp_get_fitness.begin(), m_tmp_get_fitness.begin() + m_tmp_get_fitness.size() );
    m_tmp_get_fitness.resize( fitness.size() * NbTotProcesses );
    MPI_Gather(&fitness[0], fitness.size(), MPI_DOUBLE, &m_tmp_get_fitness[0], fitness.size(), MPI_DOUBLE, 0, MPI_COMM_WORLD);
    
    for(int i = 0; i < NbTotProcesses; i++){
      for(int j = 0; j < m_population[0]->getStoreFitness().size(); j++){
	m_population[0]->addAtStoreFitness( j, m_tmp_get_fitness[j + (i*m_population[0]->getStoreFitness().size()) ] );
	//cerr << " m_tmp_get_fitness["<<j + (i*m_parameters->getNbGroups())<<"] = " << m_tmp_get_fitness[j + (i*m_parameters->getNbGroups()) ] << endl;
	//cerr << "fitness["<< (j + (i*m_parameters->getNbGroups()))%m_parameters->getNbGroups() << "] = " << m_population[0]->getStoreFitness()[(j + (i*m_parameters->getNbGroups()))%m_parameters->getNbGroups()] << endl;
      }
      //getchar();
    }
    //cerr << " receive fitness from all processes " << endl;
    //getchar();
  }
  else {
    //Each process send to process rank 0 the fitness
    MPI_Gather(&fitness[0], fitness.size(), MPI_DOUBLE, NULL, 1, MPI_DOUBLE, 0, MPI_COMM_WORLD);
  }
  
  for(int i = 0; i < fitness.size(); i++)
    fitness[i] = 0.0;
}

/* ------------------------------------------------------------------------------------------------- */

void Experiment::defineHeteroTeamsRandomlyAssembled( void ){
  for(int i = 0; i < m_teamTuples.size(); i++ ){
    m_teamTuples[i].erase( m_teamTuples[i].begin(), m_teamTuples[i].begin() + m_teamTuples[i].size() );
  }
  
  int count_ext = 0;
  int count_int = 0;
  do{
    m_pop_one_solutions.erase(m_pop_one_solutions.begin(), m_pop_one_solutions.begin() + m_pop_one_solutions.size() );
    for(int i = 0; i < m_parameters->getNbGroups(); i++){
      m_pop_one_solutions.push_back( i );
    }
    do{
      for(int i = 0; i < m_parameters->getNbAgentsPerGroup(); i++){
	int position = gsl_rng_uniform_int(GSL_randon_generator::r_rand, m_pop_one_solutions.size() );
	m_teamTuples[count_int].push_back( m_pop_one_solutions[position] );
	m_pop_one_solutions.erase( m_pop_one_solutions.begin() + position );
      }
      count_int++;
    }while( !m_pop_one_solutions.empty() );
    count_ext++;
  }while( count_ext < m_parameters->getNbMatches() );
  /*
    int count = 0;
    for(int i = 0; i < m_teamTuples.size(); i++){
    cerr << " " << count;
    for(int j = 0; j < m_teamTuples[i].size(); j++){
    cerr << " m_teamTuples["<<i<<"]["<<j<<"] = " << m_teamTuples[i][j];
    }
    count++;
    cerr << endl;
    }
    getchar(); */
}

/* ------------------------------------------------------------------------------------------------- */

void Experiment::defineHeteroTeamsSequentiallyAssembled( void ){
  for(int i = 0; i < m_teamTuples.size(); i++ ){
    m_teamTuples[i].erase( m_teamTuples[i].begin(), m_teamTuples[i].begin() + m_teamTuples[i].size() );
  }
  int count = 0;
  for(int i = 0; i < m_teamTuples.size(); i++){
    for(int j = 0; j < m_parameters->getNbAgentsPerGroup(); j++){
      m_teamTuples[i].push_back( count++ );
    }
  }
  /*
    for(int i = 0; i < m_teamTuples.size(); i++){
      for(int j = 0; j < m_teamTuples[i].size(); j++){
      cerr << " m_teamTuples["<<i<<"]["<<j<<"] = " << m_teamTuples[i][j];
      }
      cerr << endl;
      }
      getchar();*/ 
}

/* ------------------------------------------------------------------------------------------------- */

void Experiment::defineHomogeneousTeams( void ){
  for(int i = 0; i < m_teamTuples.size(); i++ ){
    m_teamTuples[i].erase( m_teamTuples[i].begin(), m_teamTuples[i].begin() + m_teamTuples[i].size() );
    m_teamTuples[i].clear();
  }
  for(int i = 0; i < m_teamTuples.size(); i++){
    for(int j = 0; j < m_parameters->getNbAgentsPerGroup(); j++){
      m_teamTuples[i].push_back( i );
    }
  }
  /*
  for(int i = 0; i < m_teamTuples.size(); i++){
    for(int j = 0; j < m_teamTuples[i].size(); j++){
      cerr << " m_teamTuples["<<i<<"]["<<j<<"] = " << m_teamTuples[i][j];
    }
    cerr << endl;
  }
  getchar(); */
}

/* ------------------------------------------------------------------------------------------------- */

void Experiment::runOneGeneration( void ){  
  m_count_rep = 0;
  m_start     = 0;
  m_end       = m_start + NbTotProcesses;

  send_new_population_to_all_processes( );
 
  do{
    //if(NbRank == 0)
    //cerr << " Repetition = " << m_count_rep << endl;
    send_solutions_to_all_processes    ();
    evaluate_solutions                 ();
    receive_fitness_from_all_processes ();
    m_start = m_end;
    m_end += NbTotProcesses;
    //if( m_end > m_parameters->getNbGroups() ) m_end = m_parameters->getNbGroups();
    m_count_rep++;
  }while( m_count_rep < m_NbRepetitions );

  
  if (NbRank == m_root) {
    if ( m_parameters->AreGroupsHeteroGroups() ){
      if( m_parameters->getNbMatches() > 1){
	m_population[0]->avStoreFitness( (double) ( m_parameters->getNbMatches() ) );
	defineHeteroTeamsRandomlyAssembled();
      }
    }

    /*    for(int i = 0; i < m_population[0]->getStoreFitness().size(); i++ )
	  cerr << " Final Fitness["<<i<<"] = " <<  m_population[0]->getStoreFitness()[i] << endl;
	  getchar(); */
    
    for(int i = 0; i < m_population.size(); i++){
      m_population[i]->assign_fitness();
      m_population[i]->dumpStatsGenome();
      m_population[i]->breeding();
      m_population[i]->resetStoreFitness();
    }
  }
}

/* ------------------------------------------------------------------------------------------------- */
