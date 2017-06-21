#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <iostream>
#include <string>
#include <algorithm>
#include <map>

#include "../MISC/general.h"

using namespace std;

class Controller {

protected:
  double delta_t;
  int   num_input;
  int   num_output;
  int   genotype_length;
  
  void copy(const Controller& other)
  {
    delta_t = other.delta_t;
    num_input = other.num_input;
    num_output = other.num_output;
    genotype_length = other.genotype_length;
  }
  void destroy(){}

 public:
  Controller() {}
  Controller(const Controller& other)
    {
      copy(other);
    }
  virtual ~Controller()
    {
      destroy();
    }
  
  int         get_num_input           ( void )          { return num_input; }
  int         get_num_output          ( void )          { return num_output; }
  inline int  get_genotype_length     ( void )          { return genotype_length; }
  inline void set_genotype_length     ( const int val ) { genotype_length = val; }
  
  virtual void init                     ( const vector <chromosome_type> &genes ) = 0;
  virtual void step                     ( const vector <double> &input_array, vector <double> &output_array) = 0;
  virtual void reset                    ( void ) = 0;
  virtual void read_from_file           ( void ) = 0;
  virtual void compute_genotype_length  ( void ) = 0;
  
  Controller& operator=(const Controller &other)
    {
      if(this != &other)
        {
	  destroy();
	  copy(other);
        }
      return *this;
    }
  
#ifdef _DIPLOID_
  inline double get_value ( const vector <double> &genes, const int counter ){
    return ((genes[counter]+genes[genotype_length+counter])/2.0);
  }
  inline double get_value ( const vector <int> &genes, const int counter ){
    return ((double)(genes[counter]+genes[genotype_length+counter])/DOUBLE_MAX_INT);
  }
#else
  inline double get_value ( const vector <double> &genes, const int counter){
    return genes[counter];
  }
  inline double get_value ( const vector <int> &genes, const int counter){
    return ((double)(genes[counter])/MAX_INT);
  }
#endif
};

#endif

