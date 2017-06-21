#include "perceptron.h"

/* -------------------------------------------------------------------------------------- */

void  Perceptron::read_from_file ( void ) { 
  ifstream I ("../CONTROLLERS/perceptron");
  if(!I) {
    cerr << "File with network structure not found" <<endl;
    exit(0);
  }
  
  num_input               = getInt      ('=', I);
  num_output              = getInt      ('=', I);
  low_bound_Wts           = getDouble   ('=', I);
  upper_bound_Wts         = getDouble   ('=', I);
  
  int check = getInt('=',I);
  I.close();
  if( check != 999 ){
    cerr << " In ../CONTROLLERS/perceptron specifications' file --- End check was not 999" << endl;
    exit(0);
  }
}

/* -------------------------------------------------------------------------------------- */

void Perceptron::compute_genotype_length ( void ){
  genotype_length = 0;
  //weights input-output
  for(int i = 0; i < num_input; i++)
    for(int out = 0; out < num_output; out++)
      genotype_length++;
  
  //weigths bias-output
  for(int out = 0; out < num_output; out++)
    genotype_length++;
}

/* -------------------------------------------------------------------------------------- */

Perceptron::Perceptron( )
  : Controller()
{
  // read from file net structure
  read_from_file ( );
  
  //Set genotyope length given net structure
  compute_genotype_length();
  
  //Init. the input layers with connections
  allocate(num_input, num_output);
}

/* -------------------------------------------------------------------------------------- */

Perceptron::Perceptron( const Perceptron& other)
{
    allocate(other.num_input, other.num_output);
    copy(other);
}

/* -------------------------------------------------------------------------------------- */

void Perceptron::copy(const Perceptron &other)
{
    Controller::copy(other);

    biasLayer = other.biasLayer;
    inputLayer = other.inputLayer;

    low_bound_Wts = other.low_bound_Wts;
    upper_bound_Wts = other.upper_bound_Wts;
}

/* -------------------------------------------------------------------------------------- */

void Perceptron::allocate(int numInput, int numOutput)
{
  inputLayer.resize(numInput);
  for(int i = 0; i < numInput ; i++)
    inputLayer[i].weightsOut.assign(numOutput, 0.0);
  
  biasLayer.resize(1);
  biasLayer[0].weightsOut.assign(numOutput, 0.0);
}

/* -------------------------------------------------------------------------------------- */

void Perceptron::destroy()
{
  Controller::destroy();
  
  for(int i = 0; i < num_input ; i++)
    inputLayer[i].weightsOut.clear();
  inputLayer.clear();
  
  for(int i = 0; i < num_output ; i++)
    biasLayer[i].weightsOut.clear();
  biasLayer.clear();
}

/* -------------------------------------------------------------------------------------- */

Perceptron::~Perceptron()
{
    destroy();
}

/* -------------------------------------------------------------------------------------- */
void Perceptron::init ( const vector <chromosome_type> &genes )
{

  //for(int i = 0; i < genes.size(); i++ ){
  //cerr << " genes["<<i<<"]= " << genes[i] << endl;
  //}

  int    counter     = 0;
  
  /* ------------------ INPUT_LAYER -------------------- */
  for( int i=0; i<num_input; i++){
    inputLayer[i].state        = 0.0;
    for(int j=0; j< num_output ; j++){
      inputLayer[i].weightsOut[j] = get_value(genes, counter)*(upper_bound_Wts - low_bound_Wts) + low_bound_Wts;
      counter++;
    }
  }
  
  /* ------------------ CONTROLLER::BIAS_LAYER -------------------- */
  biasLayer[0].state         = 1.0;
  for(int i=0; i< num_output; i++){
    biasLayer[0].weightsOut[i] = get_value(genes, counter)*(upper_bound_Wts - low_bound_Wts) + low_bound_Wts;
    counter++;
  }
  /* --------------------------------------------------------------- */

  if( counter != genotype_length ){
    cerr << "'In perceptron.cpp init():: the number of genes is wrong"
	 << " genotype_length is " << genotype_length << " and counter is " << counter << endl;
    exit(0);
  }
}

/* -------------------------------------------------------------------------------------- */

void Perceptron::step ( const vector <double> &input, vector <double> &output){
  for(int out=0; out<num_output; out++){
    output[out] = biasLayer[0].weightsOut[out];
    for( int i=0; i<num_input; i++){
      output[out] += input[i] * inputLayer[i].weightsOut[out];
    }
    output[out] = f_sigmoid( output[out] );
  }
}

/* -------------------------------------------------------------------------------------- */

void Perceptron::reset ( void ){
  for(int i=0; i<num_input; i++){
    inputLayer[i].state = 0.0;
  }
}

/* -------------------------------------------------------------------------------------- */

Perceptron& Perceptron::operator=(const Perceptron &other)
{
  if(this != &other)
    {
      destroy();
      allocate(other.num_input, other.num_output);
      copy(other);
    }
  return *this;
}

/* -------------------------------------------------------------------------------------- */
/* -------------------------------------------------------------------------------------- */
