#include "first_exp.h"

/* ----------------------------------------------------------------------------------- */

First_experiment::First_experiment( Parameters* params ) : Experiment ( params ) {

  //Resize and init the input and output array for each robot
  inputs.resize                      ( m_agents.size() );
  outputs.resize                     ( m_agents.size() );
  camera_sector_readings.resize      ( m_agents.size() );
  optic_flow_sensor_readings.resize  ( m_agents.size() );
  
  for (int r = 0; r < m_agents.size(); r++) {
    inputs[r].assign                     ( m_networks[r]->get_num_input(),  0.0 );
    outputs[r].assign                    ( m_networks[r]->get_num_output(), 0.0 );
    camera_sector_readings[r].assign     ( m_agents[r]->num_camera_sectors, 0.0 );
    optic_flow_sensor_readings[r].assign ( m_agents[r]->num_mouse_parameters, 0.0 );
  }

  m_init_agent_dist_to_object.resize ( m_agents.size() );
  m_max_obj_displacement = 1.5;
  
}

/* ----------------------------------------------------------------------------------- */

First_experiment::~First_experiment() {}

/* ----------------------------------------------------------------------------------- */

void First_experiment::evaluate_solutions( void ) {
  init_evaluations_loop( );
  do{//Loop for evaluations for single trial
    init_single_evaluation( );
    do{//This is the loop for the iterations mean simulation steps
      adv();
    }while( stop_iterations_loop( ) );//Untill the last iteration
  }while( stop_evaluations_loop( ) );//Until the last evaluation
}

/* ----------------------------------------------------------------------------------- */

void First_experiment::init_evaluations_loop( ){ 
  m_evaluation = 0;//This is the counter for the evalutions
  set_agents_position();
  fill( fitness.begin(), fitness.end(), 0.0 );
	
  if( m_parameters->isModeEvolution() ){
    int NbGenesPerInd = 0;
    if ( m_parameters->AreGroupsHeteroGroups() ){//Heterogeneous groups
      if( m_parameters->getNbMatches() == 1 ){
	NbGenesPerInd = genes.size()/(m_parameters->getNbGroups() * m_parameters->getNbAgentsPerGroup());
      }
      else if( m_parameters->getNbMatches() > 1 ){
	NbGenesPerInd = genes.size()/(m_parameters->getNbGroups());
      }
    }
    else{//Heterogeneous groups
      if( m_parameters->IsSelectionIndividual() ){
	NbGenesPerInd = genes.size()/(m_parameters->getNbGroups() * m_parameters->getNbAgentsPerGroup());
      }
      else{
	NbGenesPerInd = genes.size()/(m_parameters->getNbGroups());
      }
    }
    
    vector < chromosome_type > geneSet;
    
    for(int i = 0; i < m_agents.size(); i++){
      for( int j = 0; j < NbGenesPerInd; j++){
	geneSet.push_back( genes[m_pop_solutions_received[i]*(NbGenesPerInd) + j] );
      }
      //      vector < chromosome_type > geneSet (genes.begin() + (m_pop_solutions_received[i]*(NbGenesPerInd)),
      //				  genes.end()-((m_parameters->getNbGroups()-m_pop_solutions_received[i]-2)*(NbGenesPerInd)));
      //m_networks[i]->init( geneSet );

      /*
	if( NbRank == 3){
	for(int i = 0; i < geneSet.size(); i++){
	cerr << " geneSet["<<i<<"] = " << geneSet[i] << " sol Received = " << m_pop_solutions_received[i] << endl;
	}
	}
	if( NbRank == 0 ) getchar();
      */
      
      geneSet.erase(geneSet.begin(), geneSet.begin() + geneSet.size() );
      geneSet.clear();
    }
  }
  /*if( NbRank == 0 ) {
    cerr << " init evaluation loop " << endl;
    getchar();
    }*/
}

/* ----------------------------------------------------------------------------------- */

void First_experiment::init_single_evaluation( void ){ 
  m_iteration = 0;
  set_agents_position();
  
  for(int r = 0; r < m_agents.size(); r++) {
    if( !m_agents[r]->is_removed() ){
      	
      //reset the network
      m_networks[r]->reset();
      
      //reset output values to zero
      fill( outputs[r].begin(), outputs[r].end(), 0.0);

      //reset the camera_sector_readings to zero
      fill( camera_sector_readings[r].begin(), camera_sector_readings[r].end(), 0.0);
      
      //reset the optic_flow_sensor_readings to zero
      fill( optic_flow_sensor_readings[r].begin(), optic_flow_sensor_readings[r].end(), 0.0);

      //init distance to object to be transported for fitness
      //m_init_agent_dist_to_object[r] = find_distance <double > ( m_objects[0]->get_pos(), m_agents[r]->get_pos() );
      
    }
  }
  
  m_obj_displacement      = 0.0;
  m_time_taken_to_succeed = (double)(m_parameters->getNbMaxIterations());
}

/* ----------------------------------------------------------------------------------- */

void First_experiment::set_agents_position( void ){
  resetPhysicsState();
  for(int obj=0; obj<m_objects.size(); obj++){
    if( m_objects[obj]->get_mass() != 0.0){
      m_objects[obj]->reset_pos();
      m_objects[obj]->body[m_objects[obj]->object_id]->clearForces();
      m_objects[obj]->body[m_objects[obj]->object_id]->setLinearVelocity(btVector3(0.0,0.0,0.0));
      m_objects[obj]->body[m_objects[obj]->object_id]->setAngularVelocity(btVector3(0.0,0.0,0.0));
    }
    //m_objects[obj]->add_object();
  }

  init_agents_pos.erase(init_agents_pos.begin(), init_agents_pos.begin() + init_agents_pos.size() );
  init_agents_pos.clear();
  init_agents_rot.erase(init_agents_rot.begin(), init_agents_rot.begin() + init_agents_rot.size() );
  init_agents_rot.clear();
  init_agents_pos.resize( m_nbActiveAgents );
  init_agents_rot.resize( m_nbActiveAgents );
  
  double min_dist    = 0.10; 
  bool flag          = false;
  double alfa_sector = TWO_PI/(double)(m_nbActiveAgents);
  for(int r = 0; r < m_nbActiveAgents; r++){
    if(!m_agents[r]->is_removed())
      {
	init_agents_pos[r].assign(3, 0.0);
	init_agents_rot[r].assign(3, 0.0);
	do{
	  flag = false;
	  double angle = (alfa_sector*r) + (gsl_rng_uniform_pos(GSL_randon_generator::r_rand) * alfa_sector );
	  init_agents_pos[r][0] = (cos(angle) * (gsl_rng_uniform_pos(GSL_randon_generator::r_rand) * 1) );
	  init_agents_pos[r][2] = (sin(angle) * (gsl_rng_uniform_pos(GSL_randon_generator::r_rand) * 1) );
	  for(int ar = 0; ar < r; ar++){
	    if ( find_distance <double > ( init_agents_pos[r], init_agents_pos[ar] ) < min_dist ) {
	      flag = true;
	      break;
	    }
	  }
        } while( flag );
	
	init_agents_rot[r][1] = gsl_rng_uniform_pos( GSL_randon_generator::r_rand )*TWO_PI;
	  //atan2(init_agents_pos[r][0], init_agents_pos[r][2]) + (PI*0.5) + (gsl_rng_uniform_pos( GSL_randon_generator::r_rand )*PI/5.0 - (PI/10.0) );
	if( init_agents_rot[r][1] < 0.0) init_agents_rot[r][1] = ( TWO_PI + init_agents_rot[r][1]);
	else if ( init_agents_rot[r][1] > TWO_PI) init_agents_rot[r][1] = (init_agents_rot[r][1] - TWO_PI);
	
	m_agents[r]->set_robot_pos_rot( init_agents_pos[r], init_agents_rot[r] );
      }
  }
}

/* ----------------------------------------------------------------------------------- */

void First_experiment::adv ( void ){

  if( m_parameters->isModeViewing() ) {
    cerr << " Gen = " << m_parameters->getCurrentGeneration() << " Eval = " << m_evaluation << " iter = " << m_iteration << endl;
    if( !stop_iterations_loop( ) ){
      exit(0);
    }
  }

  update_sensors( );
  update_controllers ( );
  update_actuators();

  for(int i=0; i < NbPhysicsWorldUpdates; i++){
    update_world();
    m_world->stepSimulation( m_physicsStep );
  }
  
  update_optic_flow_sensors();
  manage_collisions ();
  compute_fitness();
 
  m_iteration++;
}

/* ----------------------------------------------------------------------------------- */

void First_experiment::update_sensors( void ){
  
  for(int r=0; r < m_agents.size(); r++){
    if(!m_agents[r]->is_removed()){
      
      //Clean and update the inputs vector with the 8 IR readings
      inputs[r].erase(inputs[r].begin(), inputs[r].begin() + inputs[r].size() );
      m_agents[r]->get_IR_reading( inputs[r] );
      m_agents[r]->add_noise( inputs[r] );

      //Update input vector with the 3 camera readings
      //fill( camera_sector_readings[r].begin(), camera_sector_readings[r].end(), 0.0 ); 
      //m_agents[r]->get_camera_reading( camera_sector_readings[r] );
      //for( int i = 0; i < camera_sector_readings[r].size(); i++){
      //inputs[r].push_back( camera_sector_readings[r][i] );
      //}

      //Copy (and then reset) previous 4 outputs into input vector
      //for(int i = 0; i < outputs[r].size(); i++){
      //inputs[r].push_back( outputs[r][i] );
      //outputs[r][i] = 0.0;
      //}

      //Copy (and then reset) the 4 optic flow sensor readings into input vector
      //for(int i = 0; i < optic_flow_sensor_readings[r].size(); i++){
      //inputs[r].push_back( optic_flow_sensor_readings[r][i] );
      //optic_flow_sensor_readings[r][i] = 0.0;
      //}
    }
    
    //for(int i = 0; i < inputs[r].size(); i++)
    //cerr << " inputs["<<r<<"]["<<i<< "] = " << inputs[r][i] << endl;
  }
}

/* ----------------------------------------------------------------------------------- */

void First_experiment::update_controllers(void)
{
  // Update robot controllers
  // Give the inputs values to the network, and compute outputs values
  for(int r = 0 ; r < m_agents.size() ; r++) {
    if( !m_agents[r]->is_removed() ) { 
      m_networks[r]->step( inputs[r], outputs[r] );
    }
  }
}

/* ----------------------------------------------------------------------------------- */

void First_experiment::update_actuators( void ){
  /* vector <double> outputs;
     outputs.resize(4);
     outputs[0] = 1.0; // right wheel
     outputs[1] = 0.0;
     outputs[2] = 0.0; //left wheel
     outputs[3] = 1.0;  */
     
  for(int r=0; r < m_agents.size(); r++){
    //update robot wheels velocity
    if( !m_agents[r]->is_removed() ){
      m_agents[r]->set_vel( outputs[r] );
    }
  }
}

/* ----------------------------------------------------------------------------------- */

void First_experiment::update_world( void ){
  for(int r=0; r < m_agents.size(); r++){
    if(!m_agents[r]->is_removed()){
      //update robot position and rotation
      m_agents[r]->update_pos_rot( );
    }
  }
}

/* ----------------------------------------------------------------------------------- */

void First_experiment::update_optic_flow_sensors( void ){
  for(int r=0; r < m_agents.size(); r++){
    if( !m_agents[r]->is_removed() ){
      m_agents[r]->get_mouse_reading( optic_flow_sensor_readings[r] );
    }
  }
}

/* ----------------------------------------------------------------------------------- */

void First_experiment::manage_collisions (void ){
  // for(int r=0; r < m_agents.size(); r++){
  //   cerr << " nb collisions = " << m_agents[r]->collision_memory.size() << endl;
  //   if( !m_agents[r]->collision_memory.empty() ){
  //     cerr << " agent["<<r<<"] removed" << endl;
  //     remove_agent( m_agents[r] );
  //     m_agents[r]->collision_memory.erase(m_agents[r]->collision_memory.begin(),
  // 					  m_agents[r]->collision_memory.begin() + m_agents[r]->collision_memory.size() );
  //     m_agents[r]->collision_memory.clear();
  //   }
  // }
}

/* ----------------------------------------------------------------------------------- */

void First_experiment::compute_fitness( void ){

  //m_objects[0]->remove_object();
  //  set_removed(true);
  
  // m_iteration = m_parameters->getNbMaxIterations();
  
  // for(int obj = 0; obj < m_objects.size(); obj++){
  //   double obj_displ = find_distance <double> (m_objects[obj]->get_pos(), m_objects[obj]->start_pos);
  //   if( obj_displ > m_max_obj_displacement ){
  //     for(int r=0; r < m_agents.size(); r++){
  // 	if( !m_agents[r]->is_removed() ){
  // 	  if( find_distance <double > ( m_objects[obj]->get_pos(), m_agents[r]->get_pos() ) < (2.0*m_agents[r]->get_radius()) ){
  // 	    m_agents[r]->set_removed(true);
  // 	    m_objects[obj]->remove_object( );
  // 	    m_obj_displacement[r]      = 1.0;
  // 	    m_time_taken_to_succeed[r] = (double)(m_iteration);
  // 	  }
  // }
  // else{
  //   if( obj_displ < 0.0000001 )
  //     m_obj_displacement = 0.0;
  //   else
  //     m_obj_displacement = obj_displ/m_max_obj_displacement;
  // }
}

/* ----------------------------------------------------------------------------------- */

bool First_experiment::stop_iterations_loop(void)
{
  if(m_iteration >= m_parameters->getNbMaxIterations() )
    {
      finalise_single_evaluation( );
      m_iteration = 0;
      if( m_parameters->isModeEvolution() || m_parameters->isModeEvaluation() ){
	return false;
      }
      else if ( m_parameters->isModeViewing() ) {
      	if ( stop_evaluations_loop( ) ){
	  return true;
	}
	else{
	  return false;
	}
      }
    }
  else
    return true;
}

/* ----------------------------------------------------------------------------------- */

bool First_experiment::stop_evaluations_loop(void)
{
  m_evaluation++;
  if( m_evaluation >= m_parameters->getNbMaxEvaluations() )
    {
      m_evaluation = 0;
      finalise_evaluation_loop( );
      return false;
    }
  else
    return true;
}

/* ----------------------------------------------------------------------------------- */

void First_experiment::finalise_single_evaluation( void ){

  // vector <double> current_dist;
  // for(int r=0; r < m_agents.size(); r++){
  //   if( !m_agents[r]->is_removed() ){
  //     current_dist.push_back( find_distance <double > ( m_objects[0]->get_pos(), m_agents[r]->get_pos() ) );
  //     if(current_dist.back() < 0.20) current_dist.back() = 0.0;
  //     else if ( current_dist.back() > m_init_agent_dist_to_object[r] ) current_dist.back() = m_init_agent_dist_to_object[r];
  //     current_dist.back() = 1.0 - ( current_dist.back() / m_init_agent_dist_to_object[r] );
  //   }
  // }
  
  // if ( m_parameters->AreGroupsHeteroGroups() || (!m_parameters->AreGroupsHeteroGroups() && m_parameters->IsSelectionIndividual())  ) {
  //   double av_current_dist = 0.0;
  //   for(int i = 0; i < m_pop_solutions_received.size(); i++)
  //     av_current_dist += current_dist[i];
  //   av_current_dist /= (double)(m_pop_solutions_received.size());

  //   for(int i = 0; i < m_pop_solutions_received.size(); i++){
  //     fitness[m_pop_solutions_received[i]] += av_current_dist + m_obj_displacement +
  // 	(1.0 - (m_time_taken_to_succeed/(double)(m_parameters->getNbMaxIterations())));
  //   }
    
  //   /*    for(int i = 0; i < m_pop_solutions_received.size(); i++){
  //     fitness[m_pop_solutions_received[i]] += m_obj_displacement +
  // 	(1.0 - (m_time_taken_to_succeed/(double)(m_parameters->getNbMaxIterations()))) +
  // 	current_dist[i];
  // 	} */
  // }
  // else{
  //   double av_current_dist = 0.0;
  //   for(int i = 0; i < m_pop_solutions_received.size(); i++)
  //     av_current_dist += current_dist[i];
  //   av_current_dist /= (double)(m_pop_solutions_received.size());
    
  //   fitness[m_pop_solutions_received[0]] += av_current_dist + m_obj_displacement +
  //     (1.0 - (m_time_taken_to_succeed/(double)(m_parameters->getNbMaxIterations())));
  // }
  
  // current_dist.erase(current_dist.begin(), current_dist.begin() + current_dist.size() );
  // current_dist.clear();
  
  // /*
  //   cerr << " obj movement = " <<  m_obj_displacement
  //   << " av_dist " << av_distances_to_object
  //   << " time = " << (1.0 - (m_time_taken_to_succeed/(double)(m_parameters->getNbMaxIterations())))
  //   << endl;
  // */
  
  // /*
  //   for(int i = 0; i < m_pop_solutions_received.size(); i++){
  //   double fit_sum = 0.0;
  //   for(int j = 0; j < m_NbGenesPerInd; j++)
  //   fit_sum += genes[(m_pop_solutions_received[i]*m_NbGenesPerInd)+j];
  //   fitness[m_pop_solutions_received[i]] += (fit_sum/(double)(m_NbGenesPerInd));
  //   //cerr << " fitness["<<m_pop_solutions_received[i]<< "] = " << fitness[m_pop_solutions_received[i]] << endl;
  //   } */
  
  // /*
  //   if( NbRank == 1 ){
  //   for(int i = 0; i < m_pop_one_solutions_received.size(); i++){
  //   cerr << " In node 1 - m_pop_one_solutions_received["<<i<<"] = " << m_pop_one_solutions_received[i] << endl;
  //   }
  //   getchar();
  //   }*/
}

/* ----------------------------------------------------------------------------------- */

void First_experiment::finalise_evaluation_loop( void ){
  // if ( m_parameters->AreGroupsHeteroGroups() ) {
  //   if( m_parameters->IsSelectionIndividual() ) {
  //     for(int i = 0; i < m_pop_solutions_received.size(); i++){
  // 	//fitness[m_pop_solutions_received[i]] /= (double)(m_parameters->getNbMaxEvaluations());
	
  // 	double fit_sum = 0.0;
  // 	for(int j = 0; j < m_NbGenesPerInd; j++)
  // 	  fit_sum += genes[(m_pop_solutions_received[i]*m_NbGenesPerInd)+j];
  // 	fitness[m_pop_solutions_received[i]] = (fit_sum/(double)(m_NbGenesPerInd));
  // 	//cerr << " fitness["<<m_pop_solutions_received[i]<< "] = " << fitness[m_pop_solutions_received[i]] << endl;
  //     }
  //   }
  //   else{
  //     double fit_sum = 0.0;
  //     for(int i = 0; i < m_pop_solutions_received.size(); i++){
  // 	//fitness[m_pop_solutions_received[i]] /= (double)(m_parameters->getNbMaxEvaluations());
  // 	for(int j = 0; j < m_NbGenesPerInd; j++)
  // 	  fit_sum += genes[(m_pop_solutions_received[i]*m_NbGenesPerInd)+j];
  //     }
  //     fit_sum /= (m_pop_solutions_received.size() * (double)(m_NbGenesPerInd));
      
  //     for(int i = 0; i < m_pop_solutions_received.size(); i++){
  // 	fitness[m_pop_solutions_received[i]] = fit_sum;
  // 	//if (NbRank == 3) { cerr << " fitness["<<m_pop_solutions_received[i]<< "] = " << fitness[m_pop_solutions_received[i]] << endl;}
  //     }
  //     //if (NbRank == m_root) {getchar();}
  //   }
  // }
  // else{
  //   if( m_parameters->IsSelectionIndividual() ) {
  //     for(int i = 0; i < m_pop_solutions_received.size(); i++){
  // 	//fitness[m_pop_solutions_received[i]] /= (double)(m_parameters->getNbMaxEvaluations());
	
  // 	double fit_sum = 0.0;
  // 	for(int j = 0; j < m_NbGenesPerInd; j++)
  // 	  fit_sum += genes[(m_pop_solutions_received[i]*m_NbGenesPerInd)+j];
  // 	fitness[m_pop_solutions_received[i]] = (fit_sum/(double)(m_NbGenesPerInd));
  // 	//cerr << " fitness["<<m_pop_solutions_received[i]<< "] = " << fitness[m_pop_solutions_received[i]] << endl;
  //     }
  //   }
  //   else{
      
  //     //fitness[m_pop_solutions_received[0]] /= (double)(m_parameters->getNbMaxEvaluations());
      
  //     // //for(int i = 0; i < m_pop_solutions_received.size(); i++){
  //     double fit_sum = 0.0;
  //     for(int j = 0; j < m_NbGenesPerInd; j++)
  // 	fit_sum += genes[(m_pop_solutions_received[0/*i*/]*m_NbGenesPerInd)+j];
  //     fitness[m_pop_solutions_received[0/*i*/]] = (fit_sum/(double)(m_NbGenesPerInd));
  //     // //cerr << " fitness["<<m_pop_solutions_received[i]<< "] = " << fitness[m_pop_solutions_received[i]] << endl;
  //     // //}
  //   }
  // }
}

/* ----------------------------------------------------------------------------------- */
