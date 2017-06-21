#include "population.h"

/* -------------------------------------------------------------- */

Population::Population ( unsigned int id, const Parameters* params, const int nb_alleles, const int nb_bases_per_allele ) {
  process_id    = id;
  m_parameters  = params;
  resetStoreFitness          ();
  initChromosomes            ( nb_alleles, nb_bases_per_allele );
}

/* -------------------------------------------------------------- */

Population::~Population (){
  for (int g = 0; g < m_parameters->getNbGroups(); g++){
    delete[] chromosome[g];
    delete[] tmp_chromosome[g];
  }
  delete[] chromosome;
  delete[] tmp_chromosome;
}

/* -------------------------------------------------------------- */

void Population::initChromosomes ( const int nb_alleles, const int nb_bases_per_allele ) {
  int count_solutions = 0;    
  chromosome        = new Chromosome <chromosome_type> *[m_parameters->getNbGroups()];
  tmp_chromosome    = new Chromosome <chromosome_type> *[m_parameters->getNbGroups()];
  for (int g = 0; g < m_parameters->getNbGroups(); g++){
    chromosome[g]     = new Chromosome <chromosome_type> [m_nb_agents_per_group];
    tmp_chromosome[g] = new Chromosome <chromosome_type> [m_nb_agents_per_group];


    for(int ind = 0; ind < m_nb_agents_per_group; ind++){
      if( m_parameters->AreSolutionDiploid() ){
	chromosome[g][ind].set_diploid     ( true );
	tmp_chromosome[g][ind].set_diploid ( true );
      }
      else{
	chromosome[g][ind].set_diploid     ( false );
	tmp_chromosome[g][ind].set_diploid ( false );
      }

      if( ind && !m_parameters->AreGroupsHeteroGroups() && m_parameters->IsSelectionIndividual() ) {
	chromosome[g][ind] = chromosome[g][0]; 
      }
      else{
	chromosome[g][ind].init_allele_values ( nb_alleles, nb_bases_per_allele );
      }
      count_solutions++;
  
      /*double tsum = 0;
	for(int h = 0; h < chromosome[g][ind].allele.size(); h++){
	//cerr << " chromosome["<<g<<"]["<<ind<<"].allele["<<h<<"]= "<< chromosome[g][ind].allele[h] << endl;
	tsum += chromosome[g][ind].get_allele_values()[h];
	}
	cerr << " tsum["<<g<<"]["<<ind<<"] = " << tsum/(double)(chromosome[g][ind].get_allele_values().size()) << endl;
	getchar();*/
    }  
  }
  if(count_solutions != m_parameters->get_popSize()){
    cerr <<" In population.cpp (constructor) : pop size is wrong " << endl;
    exit(EXIT_FAILURE);
  }
}

/* -------------------------------------------------------------- */

void Population::assign_fitness( void ){
  m_sum_of_fitness = 0.0;
  fitness.erase(fitness.begin(), fitness.begin() + fitness.size() );
  for(int i = 0; i < store_fitness.size(); i++){
    fitness.push_back( MakeValueWithIndex(store_fitness[i], i) );
    m_sum_of_fitness += store_fitness[i];
  }
  sort(fitness.begin(), fitness.end());

  /*
    for(int i = 0; i < fitness.size(); i++){
    cerr << " " << fitness[i].value << " " << fitness[i].index << endl;
    }
    
    double tsum = 0.0;
    int selected_mum_group  = (int)(floor( (double)(63)/(double)( m_nb_agents_per_group )));
    for(int h = 0; h < 3; h++){
    for(int f = 0; f < chromosome[selected_mum_group][h].allele.size(); f++){
    tsum += chromosome[selected_mum_group][h].get_allele_values()[f];
    //cerr << " " << chromosome[selected_mum_group][0].get_allele_values()[f] << endl;
    }
    }
    cerr << " Population.cpp tsum = " << tsum/(double)(chromosome[0][0].get_allele_values().size()*3) << endl;
    getchar(); */
}

/* -------------------------------------------------------------- */

void Population::dumpStatsGenome       ( void ){
  
  if( !(m_parameters->getCurrentGeneration() % m_parameters->getDumpStatsEvery() ) )
    dumpStats();
  if( !(m_parameters->getCurrentGeneration() % m_parameters->getDumpGenomeEvery() ) ){
    dumpGenome();
  }
}

/* -------------------------------------------------------------- */

void Population::resetStoreFitness( void ){
  store_fitness.erase(store_fitness.begin(), store_fitness.begin() + store_fitness.size() );
  store_fitness.clear();
  store_fitness.assign( m_parameters->get_popSize(), 0.0);
  
  if( !m_parameters->IsSelectionIndividual() ){  //Group selection
    if ( m_parameters->AreGroupsHeteroGroups() ){//Heterogeneous groups
      m_nb_agents_per_group = (int)(m_parameters->getNbAgentsPerGroup());
    }
    else{//Homogeneous Groups
      m_nb_agents_per_group = 1;
    }
  }
  else{//Individual selection
    if ( m_parameters->AreGroupsHeteroGroups() ){//Heterogeneous groups
      if( m_parameters->getNbMatches() == 1 ) {
	m_nb_agents_per_group = (int)(m_parameters->getNbAgentsPerGroup());
      }
      else if( m_parameters->getNbMatches() > 1 ) {
	m_nb_agents_per_group           = 1;
      }
    }
    else {//Groups are homogeneous
      m_nb_agents_per_group = (int)(m_parameters->getNbAgentsPerGroup());
    }
  }
}

/* -------------------------------------------------------------- */
