#ifndef _POPULATION_
#define _POPULATION_

#include "../MISC/general.h"
#include "../MISC/parameters.h"
#include "chromosome.h"

using namespace std;

template<class T>
struct ValueWithIndex
{
  T value;
  int index;
};
template<class T>
bool operator < (const ValueWithIndex<T>& v1, const ValueWithIndex<T>& v2)
{
  return v1.value < v2.value;
}
template<class T> ValueWithIndex<T>
MakeValueWithIndex(const T& value, int index)
{
  ValueWithIndex<T> ret;
  ret.value = value;
  ret.index = index;
  return ret; 
}


class Population
{
 protected:
  const Parameters *m_parameters;
  Chromosome <chromosome_type> **chromosome;
  Chromosome <chromosome_type> **tmp_chromosome;
  int m_nb_agents_per_group;
  vector <double> store_fitness;

  double m_sum_of_fitness;
  vector< ValueWithIndex <double> > fitness;
  unsigned int process_id;
  
 public:
  Population ( unsigned int id, const Parameters* params, const int nb_alleles, const int nb_bases_per_allele );     
  virtual ~Population();
  
  inline vector <chromosome_type> & get_solution            ( const int g, const int ind )  { return chromosome[g][ind].allele; }
  inline Chromosome <chromosome_type> get_chromosome        ( const int g, const int ind )  { return chromosome[g][ind]; }
  inline unsigned int               get_nb_agents_per_group ( void )                        { return m_nb_agents_per_group;}
  inline vector <double> &          getStoreFitness         ( void )                        { return store_fitness; }
  inline void                       setStoreFitness         ( const int i, const double v ) { store_fitness[i] = v; }
  inline void                       addAtStoreFitness       ( const int i, const double v ) { store_fitness[i] += v; }
  inline void                       avStoreFitness          ( double div ) {
    for(int i = 0; i < store_fitness.size(); i++)
      store_fitness[i] /= div;
  }
  inline vector < ValueWithIndex <double> > getFitness      ( void )                        { return fitness; }
    
  void initChromosomes            ( const int nb_alleles, const int nb_bases_per_allele );
  void assign_fitness             ( void );
  void resetStoreFitness          ( void );
  void dumpStatsGenome            ( void );
  virtual void dumpStats          ( void ) = 0;
  virtual void dumpGenome         ( void ) = 0;
  virtual void breeding           ( void ) = 0;
  
};

#endif

