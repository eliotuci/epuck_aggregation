#include "roulette_wheel.h"


/* -------------------------------------------------------------- */

Roulette_wheel::Roulette_wheel ( unsigned int id, const Parameters* params, const int nb_alleles, const int nb_bases_per_allele ) : Population ( id, params, nb_alleles, nb_bases_per_allele ){


  //Chose type of breeding
  if( !m_parameters->IsSelectionIndividual() ){   //Group selection
    if ( m_parameters->AreGroupsHeteroGroups() ){//Heterogeneous groups
      //breeding_HeteroGroups();
      breeding_fn_ptr = &Roulette_wheel::breeding_HeteroGroupSelection;
    }
    else{//Homogeneous Groups
      //breeding_HomoGroups();
      breeding_fn_ptr = &Roulette_wheel::breeding_HomoGroupSelection;
    }
  }
  else{//Individual selection
    if ( m_parameters->AreGroupsHeteroGroups() ){//Heterogeneous groups
      if( m_parameters->getNbMatches() == 1 ) {
	//breeding_HeteroGroups();
	breeding_fn_ptr = &Roulette_wheel::breeding_HeteroIndividualSelection;
      }
      else if( m_parameters->getNbMatches() > 1 ) {
	//breeding_HomoGroups();
	breeding_fn_ptr = &Roulette_wheel::breeding_HomoGroupSelection;
      }
    }
    else {//Groups are homogeneous
      breeding_fn_ptr = &Roulette_wheel::breeding_HomoIndividualSelection;
    }
  }
}

/* -------------------------------------------------------------- */

Roulette_wheel::~Roulette_wheel ( void ){}

/* -------------------------------------------------------------- */

void Roulette_wheel::breeding ( void ) {
  
  compute_probabilities_of_selection();
    
  //Clear and resize m_parents vector
  m_parents.erase(m_parents.begin(), m_parents.begin() + m_parents.size() );
  m_parents.clear();
  (this->*breeding_fn_ptr)();
}

/* -------------------------------------------------------------- */

void Roulette_wheel::compute_probabilities_of_selection ( void ){
  //transform all fitness values in positive numbers
  if( fitness[0].value < 0.0 ){
    for(int i = 0; i < fitness.size(); i++){
      fitness[i].value += fabs ( fitness[0].value );
    }
  }
  proporational_fitness_selection();
  
  //if( m_parameters->getCurrentGeneration() == 0 )
  //rank_based_selection( );
}

/* -------------------------------------------------------------- */

void Roulette_wheel::proporational_fitness_selection( void ){
  ProbSelection.erase(ProbSelection.begin(), ProbSelection.begin() + ProbSelection.size() );
  ProbSelection.clear();
  ProbSelection.push_back( fitness[0].value / m_sum_of_fitness );
  
  /*  cerr << " ProbSelection[ "<< 0 <<"] = " << ProbSelection[0]
   << " Fit = " << fitness[0].value
   << endl; */
  for(int i = 1; i < fitness.size(); i++){
    ProbSelection.push_back( ProbSelection[i-1] + ( fitness[i].value / m_sum_of_fitness) );

    /*   cerr << " ProbSelection[ "<< i <<"] = " << ProbSelection[i]
    	 << " diff = " << ProbSelection.back() - ProbSelection[i-1]
    	 << " Fit = " << fitness[i].value
    	 << endl; */
  }
  //getchar();
}

/* -------------------------------------------------------------- */

void Roulette_wheel::rank_based_selection( void ){
  double selective_pressure = 2.0;
  
  ProbSelection.erase(ProbSelection.begin(), ProbSelection.begin() + ProbSelection.size() );
  ProbSelection.clear();
  for(int i = 0; i < fitness.size(); i++){
    ProbSelection.push_back( (2.0 - selective_pressure) + 
			     2.0 * (selective_pressure - 1.0)*
			     ((double)(i)/(double)(fitness.size()-1)) );
   
  }
  for(int i = 0; i < fitness.size(); i++){
    ProbSelection[i] /= ProbSelection.back();
    //cerr << " ProbSelection[ "<< i <<"] = " << ProbSelection[i] << endl;
  }
  //getchar();
}

/* -------------------------------------------------------------- */

void Roulette_wheel::breeding_HomoGroupSelection( void ){

  //Copy the elite
  for(int g = 0; g < m_parameters->getNbGroups(); g++){
    if( g < m_parameters->getNbElite()  ){
      //for(int a = 0; a < m_nb_agents_per_group; a++){
      tmp_chromosome[g][0] = chromosome[fitness[fitness.size()-1-g].index][0];
      m_parents.push_back( MakeMumAndDad( fitness[fitness.size()-1-g].index, 9999 ));
      //}
    }
    else{//breed
      int mum = 9999, dad = 9999;
      //for(int a = 0; a < m_nb_agents_per_group; a++){
      select_groups( &mum, &dad );
      //int first_ind = gsl_rng_uniform_int (GSL_randon_generator::r_rand, m_nb_agents_per_group );

      if( gsl_rng_uniform(GSL_randon_generator::r_rand) < m_parameters->getProbCrossOver() ) {
	//int second_ind = gsl_rng_uniform_int (GSL_randon_generator::r_rand, m_nb_agents_per_group );
	tmp_chromosome[g][0].create_with_cross_over_and_mutate_operators ( chromosome[fitness[mum].index][0], 
									   chromosome[fitness[dad].index][0],
									   m_parameters->getProbMutation() );
	m_parents.push_back( MakeMumAndDad( fitness[mum].index, fitness[dad].index ));
      }
      else{
	tmp_chromosome[g][0].create_with_mutate_operator ( chromosome[fitness[mum].index][0], m_parameters->getProbMutation() );
	m_parents.push_back( MakeMumAndDad( fitness[mum].index, 9999 ));
      }
	//}
    }
  }
  copy_new_genotypes_in_working_array ();
}

/* -------------------------------------------------------------- */

void Roulette_wheel::breeding_HomoIndividualSelection ( void ){
  
  int count_ind = 0;
  for(int g = 0; g < m_parameters->getNbGroups(); g++){
    if( g < m_parameters->getNbElite()  ){
      for(int ind = 0; ind < m_nb_agents_per_group; ind++){
	if( ind == 0 ){
	  int selected_mum_group = (int)(floor( (double)(fitness[fitness.size()-1-count_ind].index)/(double)(m_nb_agents_per_group)));
	  int selected_ind       = (int)(fitness[fitness.size()-1-count_ind].index % m_nb_agents_per_group);
	  tmp_chromosome[g][ind] = chromosome[selected_mum_group][selected_ind];
	  m_parents.push_back( MakeMumAndDad( fitness[fitness.size()-1-count_ind].index, 9999 ));
	}
	else{
	  tmp_chromosome[g][ind] = tmp_chromosome[g][0]; 
	  m_parents.push_back( m_parents.back() );
	}
      }
      count_ind++;
    }
    else{
      for(int ind = 0; ind < m_nb_agents_per_group; ind++){
	int mum = 9999, dad = 9999,  first_ind = 9999, second_ind = 9999;
	
	if( ind == 0 ){
	  select_groups( &mum, &dad );
	  int selected_mum_group = (int)(floor( (double)(fitness[mum].index)/(double)(m_nb_agents_per_group)));
	  first_ind              = (int)(fitness[mum].index % m_nb_agents_per_group);
	  
	  if( gsl_rng_uniform(GSL_randon_generator::r_rand) < m_parameters->getProbCrossOver() ) {
	    int selected_dad_group  = (int)(floor( (double)(fitness[dad].index)/(double)(m_nb_agents_per_group)));
	    second_ind              = (int)(fitness[dad].index % m_nb_agents_per_group);
	    tmp_chromosome[g][ind].create_with_cross_over_and_mutate_operators ( chromosome[selected_mum_group][first_ind], 
										 chromosome[selected_dad_group][second_ind],
										 m_parameters->getProbMutation() );
	    m_parents.push_back( MakeMumAndDad(fitness[mum].index, fitness[dad].index));
	  }
	  else{
	    tmp_chromosome[g][ind].create_with_mutate_operator ( chromosome[selected_mum_group][first_ind],
								 m_parameters->getProbMutation() );
	    m_parents.push_back( MakeMumAndDad(fitness[mum].index, 9999 ));
	  }
	}
	else{
	  tmp_chromosome[g][ind] = tmp_chromosome[g][0];
	  m_parents.push_back( m_parents.back() );
	}
      }
    }
  }
  copy_new_genotypes_in_working_array ();
}

/* -------------------------------------------------------------- */

void Roulette_wheel::breeding_HeteroGroupSelection ( void ){
  int count_ind = 0;
  for(int g = 0; g < m_parameters->getNbGroups(); g++){
    if( g < m_parameters->getNbElite()  ){
      for(int ind = 0; ind < m_nb_agents_per_group; ind++){
	int selected_mum_group = (int)(floor( (double)(fitness[fitness.size()-1-count_ind].index)/(double)(m_nb_agents_per_group)));

	//cerr <<" ind = " << ind << " count_ind = " << count_ind << " selected_mum_group = " << selected_mum_group << " index = " << fitness[fitness.size()-1-count_ind].index << endl;
	//getchar();
	
	tmp_chromosome[g][ind] = chromosome[selected_mum_group][ind];
	m_parents.push_back( MakeMumAndDad( fitness[fitness.size()-1-count_ind].index, 9999 ));
	count_ind++;
      }
    }
    else{
      for(int ind = 0; ind < m_nb_agents_per_group; ind++){
	int mum = 9999, dad = 9999,  first_ind = 9999, second_ind = 9999;
	select_groups( &mum, &dad );
	int selected_mum_group = (int)(floor( (double)(fitness[mum].index)/(double)(m_nb_agents_per_group)));
	first_ind              = (int)(gsl_rng_uniform_int (GSL_randon_generator::r_rand, m_nb_agents_per_group ));
	
	if( gsl_rng_uniform(GSL_randon_generator::r_rand) < m_parameters->getProbCrossOver() ) {
	  int selected_dad_group  = (int)(floor( (double)(fitness[dad].index)/(double)(m_nb_agents_per_group)));
	  second_ind              = (int)(gsl_rng_uniform_int (GSL_randon_generator::r_rand, m_nb_agents_per_group ));
	  tmp_chromosome[g][ind].create_with_cross_over_and_mutate_operators ( chromosome[selected_mum_group][first_ind], 
									       chromosome[selected_dad_group][second_ind],
									       m_parameters->getProbMutation() );
	  m_parents.push_back( MakeMumAndDad(fitness[mum].index, fitness[dad].index));
	}
	else{
	  tmp_chromosome[g][ind].create_with_mutate_operator ( chromosome[selected_mum_group][first_ind],
							       m_parameters->getProbMutation() );
	  m_parents.push_back( MakeMumAndDad(fitness[mum].index, 9999 ));
	}
      }
    }
  }
  copy_new_genotypes_in_working_array ();
}

/* -------------------------------------------------------------- */

void Roulette_wheel::breeding_HeteroIndividualSelection ( void ){
  int count_solutions = 0;
  vector <int> d_vector;
  m_parents.resize( m_parameters->get_popSize() );
  for(int i = 0; i < m_parameters->get_popSize(); i++){
    d_vector.push_back(i);
  }
  
  do{
    //for(int i = 0; i < d_vector.size(); i++)
    //cerr << " Before - v["<<i<<"]= " << d_vector[i] << endl;
    
    int index_d_vector    = gsl_rng_uniform_int (GSL_randon_generator::r_rand, d_vector.size() );
    int element_d_vector  = d_vector[index_d_vector];
    int d_group           = (int)(floor((double)(element_d_vector)/(double)(m_nb_agents_per_group)));
    int d_ind             = (int)(element_d_vector % m_nb_agents_per_group);
    
    /* cerr << " index_d_vector = " << index_d_vector
       << " element_d_vector = " << element_d_vector
       << " d_group = " << d_group
       << " d_ind = " << d_ind
       << " c_sol = " << count_solutions
       << endl; */
    
    
    d_vector.erase       ( d_vector.begin() + index_d_vector );
    
    /* for(int i = 0; i < d_vector.size(); i++)
       cerr << " After - v["<<i<<"]= " << d_vector[i] << endl;
       getchar(); */
      
    if( count_solutions < m_parameters->getNbElite()  ){
      int selected_mum_group = (int)(floor( (double)(fitness[fitness.size()-1-count_solutions].index)/(double)(m_nb_agents_per_group)));
      int selected_ind       = (int)(fitness[fitness.size()-1-count_solutions].index % m_nb_agents_per_group);
      tmp_chromosome[d_group][d_ind] = chromosome[selected_mum_group][selected_ind];
      m_parents[element_d_vector] = MakeMumAndDad( fitness[fitness.size()-1-count_solutions].index, 9999 );
      count_solutions++;
    }
    else{
      int mum = 9999, dad = 9999,  first_ind = 9999, second_ind = 9999;
      select_groups( &mum, &dad );
      int selected_mum_group = (int)(floor( (double)(fitness[mum].index)/(double)(m_nb_agents_per_group)));
      first_ind              = (int)(fitness[mum].index % m_nb_agents_per_group);
      if( gsl_rng_uniform(GSL_randon_generator::r_rand) < m_parameters->getProbCrossOver() ) {
	int selected_dad_group  = (int)(floor( (double)(fitness[dad].index)/(double)(m_nb_agents_per_group)));
	second_ind              = (int)(fitness[dad].index % m_nb_agents_per_group);
	tmp_chromosome[d_group][d_ind] .create_with_cross_over_and_mutate_operators (
										     chromosome[selected_mum_group][first_ind], 
										     chromosome[selected_dad_group][second_ind],
										     m_parameters->getProbMutation() );
	m_parents[element_d_vector] = MakeMumAndDad( fitness[mum].index, fitness[dad].index );
      }
      else{
	tmp_chromosome[d_group][d_ind].create_with_mutate_operator ( chromosome[selected_mum_group][first_ind],
								     m_parameters->getProbMutation() );
	m_parents[element_d_vector] = MakeMumAndDad( fitness[mum].index, 9999 );
      }
    }
  }while( !d_vector.empty() );
  copy_new_genotypes_in_working_array ();
}

/* -------------------------------------------------------------- */

void Roulette_wheel::select_groups( int *mum, int *dad ){
  //Select mum
  double NbRnd = 0.0;
  if( m_parameters->getNbSolutionsTruncated() ){
    NbRnd = ProbSelection[m_parameters->getNbSolutionsTruncated()-1] +
      (gsl_rng_uniform (GSL_randon_generator::r_rand) * (ProbSelection.back() -
							    ProbSelection[m_parameters->getNbSolutionsTruncated()-1]) );
    if( NbRnd > 1.0 ){
      cerr <<" In Roulette_wheele.cpp : select group wrong  " << endl;
      exit(EXIT_FAILURE);
    }
  }
  else{
    NbRnd = gsl_rng_uniform (GSL_randon_generator::r_rand) * ProbSelection.back();
  }

 for (int i = m_parameters->getNbSolutionsTruncated(); i < ProbSelection.size(); i++){
    if( NbRnd < ProbSelection[i] ){
      *mum = i;
      break;
    }
 }

 /* cerr << " " << m_parameters->getNbSolutionsTruncated() << " Prob mum = " << NbRnd << " mum = " << *mum
      << " trunc = " << ProbSelection[m_parameters->getNbSolutionsTruncated()-1]
      << " last = " << ProbSelection.back()
      << endl; */
 
 //Select dad different from mum
 unsigned int iter = 0;
 *dad = *mum;
 do{
   if( m_parameters->getNbSolutionsTruncated()){
     NbRnd = ProbSelection[m_parameters->getNbSolutionsTruncated()-1] +
       (gsl_rng_uniform (GSL_randon_generator::r_rand) * (ProbSelection.back() -
							     ProbSelection[m_parameters->getNbSolutionsTruncated()-1]) );
     if( NbRnd > 1.0 ){
       cerr <<" In Roulette_wheele.cpp : select group wrong  " << endl;
       exit(EXIT_FAILURE);
     }
   }
   else{
     NbRnd = gsl_rng_uniform (GSL_randon_generator::r_rand) * ProbSelection.back();
   }
   
   
   for (int i = m_parameters->getNbSolutionsTruncated(); i < ProbSelection.size(); i++){
     if( NbRnd < ProbSelection[i]){
       *dad = i;
       break;
     }
   }
   iter++;
 }while( *mum == *dad && iter < 10);

 /* cerr << " " << m_parameters->getNbSolutionsTruncated() << " Prob dad = " << NbRnd << " dad = " << *dad
      << " trunc = " << ProbSelection[m_parameters->getNbSolutionsTruncated()-1]
      << " last = " << ProbSelection.back()
      << endl;
      getchar(); */
}

/* -------------------------------------------------------------- */

void Roulette_wheel::copy_new_genotypes_in_working_array ( void ){
  for(int g = 0; g < m_parameters->getNbGroups(); g++){
    for(int ind = 0; ind < m_nb_agents_per_group; ind++){
      chromosome[g][ind] = tmp_chromosome[g][ind];
    }
  }
}

/* -------------------------------------------------------------- */


void Roulette_wheel::dumpStats( void )
{
  char fname_statistics[500];
  sprintf(fname_statistics, "%s%s%d.data", m_parameters->getStatsFileDir().c_str(), m_parameters->getRunName().c_str(), process_id ); 
  ofstream outfile;
  outfile.open (fname_statistics, ios::app);
  outfile.setf(ios::fixed);
  
  outfile << " " << m_parameters->getCurrentGeneration()
	  << " " << fitness[fitness.size()-1].value //The fitness of the best
	  << " " << m_sum_of_fitness/(double)((m_parameters->getNbGroups() * m_nb_agents_per_group)) // the mean fitness
	  << " " << fitness[0].value //the fitness of the worst
	  << endl;
  outfile.close();
}

/* ---------------------------------------- */

void Roulette_wheel::dumpGenome( void )
{
  char fname_genome[500];
  sprintf(fname_genome, "%s%s%d_pop_G%d.geno", m_parameters->getGenomeFileDir().c_str(), m_parameters->getRunName().c_str(), process_id, m_parameters->getCurrentGeneration() );
  ofstream out ( fname_genome );
  out.setf( ios::fixed );
  
  for(unsigned int p=0; p<m_parameters->get_popSize(); p++) {
    int group_id, ind_id;
    
    //Chose type of breeding
    if( m_parameters->get_popSize() == (m_parameters->getNbGroups()* m_parameters->getNbAgentsPerGroup()) ){
      group_id  = (int)(floor( (double)(fitness[fitness.size()-1-p].index)/(double)( m_nb_agents_per_group )));
      ind_id    = (int)(fitness[fitness.size()-1-p].index % m_nb_agents_per_group);
    }
    else if( m_parameters->get_popSize() == m_parameters->getNbGroups() ){
      group_id  = p;
      ind_id    = 0;
    }
    else{
      cerr <<" In Roulette_wheele.cpp (dumpGenome) : select group/ind wrong  " << endl;
      exit(EXIT_FAILURE);
    }
    
    out << "" << chromosome[group_id][ind_id].get_allele_values().size();
    for(int n=0; n<chromosome[group_id][ind_id].get_allele_values().size(); n++){
      out << " " << setprecision(15) << chromosome[group_id][ind_id].get_allele_values()[n]; 
    }
    out << " " << fitness[fitness.size()-1-p].index;
      
    if( m_parameters->getCurrentGeneration() ){
      out << " " << m_parents[fitness[fitness.size()-1-p].index].mum_index
	  << " " << m_parents[fitness[fitness.size()-1-p].index].dad_index;
    }
    
    // //This is its group ID and its individual ID
    out << " " << fitness[fitness.size()-1-p].value << endl;
  }
  out.close();
}

/* ---------------------------------------- */
