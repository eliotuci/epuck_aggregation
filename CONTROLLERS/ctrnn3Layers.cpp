#include "ctrnn3Layers.h"

/* -------------------------------------------------------------------------------------- */

void  Ctrnn3Layers::read_from_file ( void )
{
    ifstream I ("../CONTROLLERS/ctrnn3Layers");
    if(!I) {
        cerr << "File with network structure not found" <<endl;
        exit(0);
    }

    delta_t                 = getDouble   ('=', I);
    num_input               = getInt      ('=', I);
    num_hidden              = getInt      ('=', I);
    num_output              = getInt      ('=', I);

    low_bound_inputWts      = getDouble   ('=', I);
    upper_bound_inputWts    = getDouble   ('=', I);
    low_bound_inputBias     = getDouble   ('=', I);
    upper_bound_inputBias   = getDouble   ('=', I);

    low_bound_hiddenWts     = getDouble   ('=', I);
    upper_bound_hiddenWts   = getDouble   ('=', I);
    low_bound_hiddenTau     = getDouble   ('=', I);
    upper_bound_hiddenTau   = getDouble   ('=', I);
    low_bound_hiddenBias    = getDouble   ('=', I);
    upper_bound_hiddenBias  = getDouble   ('=', I);

    low_bound_outputBias    = getDouble   ('=', I);
    upper_bound_outputBias  = getDouble   ('=', I);

    low_bound_sensorsGain   = getDouble   ('=', I);
    upper_bound_sensorsGain = getDouble   ('=', I);

    int check = getInt('=',I);
    I.close();
    if( check != 999 ){
        cerr << " In ../CONTROLLERS/ctrnn3Layers specifications' file --- End check was not 999" << endl;
        exit(0);
    }
}

/* -------------------------------------------------------------------------------------- */

void Ctrnn3Layers::compute_genotype_length ( void ){
    genotype_length = 0;
    
    //weights input-hidden
    genotype_length += num_input * num_hidden;

    //For INPUT-OUTPUT CONNECTIONS
    //genotype_length += num_input * num_output;
    
    //weights hidden-hidden
    genotype_length += num_hidden * num_hidden;
    
    //weights hidden-output
    genotype_length += num_hidden * num_output;
    
    //tau - it only applies to hidden nodes
    for(int h = 0; h < num_hidden; h++) genotype_length++;

    //bias - it applies to hidden nodes plus a single bias for all
    //input nodes, and a single bias for all output nodes
    for(int h = 0; h < num_hidden+2; h++) genotype_length++;

    //sensor gain - there is only one single gain for all input nodes
    genotype_length++;
}

/* -------------------------------------------------------------------------------------- */

Ctrnn3Layers::Ctrnn3Layers( )
    : Controller()
{
    read_from_file ( );
    compute_genotype_length ( );

    allocate(num_input, num_hidden, num_output);
}

Ctrnn3Layers::Ctrnn3Layers(const Ctrnn3Layers& other)
{
    allocate(other.num_input, other.num_hidden, other.num_output);
    copy(other);
}

void Ctrnn3Layers::copy(const Ctrnn3Layers &other)
{
    num_hidden = other.num_hidden;

    inputLayer = other.inputLayer;
    hiddenLayer = other.hiddenLayer;
    outputLayer = other.outputLayer;

    low_bound_inputWts = other.low_bound_inputWts;
    upper_bound_inputWts = other.upper_bound_inputWts;
    low_bound_inputBias = other.low_bound_inputBias;
    upper_bound_inputBias = other.upper_bound_inputBias;

    low_bound_hiddenWts = other.low_bound_hiddenWts;
    upper_bound_hiddenWts = other.upper_bound_hiddenWts;
    low_bound_hiddenTau = other.low_bound_hiddenTau;
    upper_bound_hiddenTau = other.upper_bound_hiddenTau;
    low_bound_hiddenBias = other.low_bound_hiddenBias;
    upper_bound_hiddenBias = other.upper_bound_hiddenBias;

    low_bound_outputBias = other.low_bound_outputBias;
    upper_bound_outputBias = other.upper_bound_outputBias;

    low_bound_sensorsGain = other.low_bound_sensorsGain;
    upper_bound_sensorsGain = other.upper_bound_sensorsGain;
    sensorsGain = other.sensorsGain;
}

void Ctrnn3Layers::allocate(int numInput, int numHidden, int numOutput)
{

  inputLayer.resize(numInput);
  for(int i = 0 ; i < numInput ; i++){
    //For INPUT-OUTPUT CONNECTIONS
    inputLayer[i].weightsOut.assign(numHidden /*+numOutput*/, 0.0);
  }
  
  hiddenLayer.resize(numHidden);
  for(int i = 0 ; i < numHidden ; i++) {
    hiddenLayer[i].weightsOut.assign(numOutput, 0.0);
    hiddenLayer[i].weightsSelf.assign(numHidden, 0.0);
  }
  
  outputLayer.resize(numOutput);
  
  sensorsGain.assign(numInput, 0.0);
  
}

void Ctrnn3Layers::destroy()
{
    for(int i = 0; i < inputLayer.size() ; i++)
    {
        inputLayer[i].weightsOut.clear();
        inputLayer[i].weightsSelf.clear();
    }
    inputLayer.clear();

    for(int i = 0; i < hiddenLayer.size() ; i++)
    {
        hiddenLayer[i].weightsOut.clear();
        hiddenLayer[i].weightsSelf.clear();
    }
    hiddenLayer.clear();

    for(int i = 0; i < outputLayer.size() ; i++)
    {
        outputLayer[i].weightsOut.clear();
        outputLayer[i].weightsSelf.clear();
    }
    outputLayer.clear();

    sensorsGain.clear();
}

/* -------------------------------------------------------------------------------------- */

Ctrnn3Layers::~Ctrnn3Layers()
{
    destroy();
}

/* -------------------------------------------------------------------------------------- */

void Ctrnn3Layers::init ( const vector <chromosome_type> &genes ){

  int    counter     = 0;
  double single_tau  = 0.0;
  double single_gain = 0.0;
  double single_bias = 0.0;
  
  /* ------------------ INPUT_LAYER -------------------- */
  //SINGLE TAU EQUAL TO DELTA_T
  single_tau = delta_t;
  
  // SINGLE BIAS FOR ALL INPUT
  single_bias = get_value(genes, counter)*(upper_bound_inputBias - low_bound_inputBias) + low_bound_inputBias;
  counter++;
  
  //SINGLE GAIN FOR ALL INPUT
  single_gain = get_value(genes, counter)*(upper_bound_sensorsGain - low_bound_sensorsGain) + low_bound_sensorsGain;
  counter++;
  
  for( int i=0; i<inputLayer.size(); i++){
    for(int j=0; j< inputLayer[i].weightsOut.size(); j++){
      inputLayer[i].weightsOut[j] =  get_value(genes, counter)*(upper_bound_inputWts - low_bound_inputWts) + low_bound_inputWts;
      counter++;
    }
    inputLayer[i].tau  = single_tau;
    inputLayer[i].bias = single_bias;
    sensorsGain[i]     = single_gain;
  }
  /* --------------------------------------------------------------- */
  
  /* ------------------ CONTROLLER::HIDDEN_LAYER -------------------- */
  for( int i=0; i<hiddenLayer.size() ; i++){
    for(int j=0; j<hiddenLayer[i].weightsOut.size() ; j++){
      hiddenLayer[i].weightsOut[j] = get_value(genes, counter)*(upper_bound_hiddenWts - low_bound_hiddenWts) + low_bound_hiddenWts;
      counter++;
    }
    hiddenLayer[i].tau         = pow(10, (low_bound_hiddenTau + (upper_bound_hiddenTau * get_value(genes, counter) )));
    counter++;
    hiddenLayer[i].bias        = get_value(genes, counter)*(upper_bound_hiddenBias - low_bound_hiddenBias) + low_bound_hiddenBias;
    counter++;
    for( int j=0; j<hiddenLayer.size() ; j++){
      hiddenLayer[i].weightsSelf[j] = get_value(genes, counter)*(upper_bound_hiddenWts - low_bound_hiddenWts) + low_bound_hiddenWts;
      counter++;
    }
  }
  /* ---------------------------------------------------------------- */
  
  /* ------------------ CONTROLLER::OUTPUT_LAYER -------------------- */
  //SINGLE TAU EQUAL TO DELTA_T
  single_tau           = delta_t;
  
  // SINGLE BIAS FOR ALL OUTPUT
  single_bias = get_value(genes, counter)*(upper_bound_outputBias - low_bound_outputBias) + low_bound_outputBias;
  counter++;
  
  for(int i=0; i<outputLayer.size() ; i++){
    outputLayer[i].tau  =  single_tau;
    outputLayer[i].bias =  single_bias;
  }
  /* --------------------------------------------------------------- */
  
  if( counter != genotype_length ){
    cerr << "'In ctrnn3Layers.cpp init():: the number of genes is wrong"
	 << " " << genotype_length << " " << counter << endl;
    exit(0);
  }
}

/* -------------------------------------------------------------------------------------- */

void Ctrnn3Layers::step ( const vector <double> &inputs, vector <double> &outputs ){
  for( int i=0; i < num_input ; i++) {
    inputLayer[i].state = (inputs[i] * sensorsGain[i] );
  }
  
  update_hidden_layer();
  update_output_layer( );
  
  /* here we set the actuators state */
  for( int i = 0; i < num_output; i++ ){
    outputs[i] = f_sigmoid( (outputLayer[i].state + outputLayer[i].bias) );
  }
}

/* -------------------------------------------------------------------------------------- */

/* This function compute the activation of each hidden neurons */
void Ctrnn3Layers::update_hidden_layer( ){
  for(int i=0; i < num_hidden; i++) {
    hiddenLayer[i].s = -hiddenLayer[i].state;
    update_StimesW  ( i, inputLayer, &(hiddenLayer[i].s) );
    update_StimesWself  ( i, hiddenLayer, &(hiddenLayer[i].s) );
  }
  for(int i=0; i < num_hidden; i++)
    hiddenLayer[i].state += (hiddenLayer[i].s * (delta_t/hiddenLayer[i].tau));
}

/* -------------------------------------------------------------------------------------- */

/* This function compute the activation of each output neurons taking
   into account the hidden layer */
void Ctrnn3Layers::update_output_layer ( ){
  
  for(int i=0; i < num_output; i++) {
    outputLayer[i].s = -outputLayer[i].state;
    //FOR INPUT-OUTPUT CONNECTIONS
    //if( i > or < XXX )
    //outputLayer[i].s += update_StimesW  ( num_hidden + i, inputLayer );
    update_StimesW  ( i, hiddenLayer, &(outputLayer[i].s) );
  }
  for(int i=0; i < num_output; i++)
    outputLayer[i].state += (outputLayer[i].s * (delta_t/outputLayer[i].tau));
}

/* -------------------------------------------------------------------------------------- */

/* This function is used to multiply the firig rate 
   of the neurons (layer) with a connection to neuron i */
void Ctrnn3Layers::update_StimesW ( int i, vector<Node> &layer, double *s ){
  for(  int j = 0; j < layer.size(); j++ ) {
    *s +=  layer[j].weightsOut[i] * f_sigmoid ( ( layer[j].state + layer[j].bias ) );
  }
}

/* -------------------------------------------------------------------------------------- */

/* This function is used to multiply the firig rate 
   of the neurons (layer) with the self-connections to neuron i */
void Ctrnn3Layers::update_StimesWself ( int i, vector<Node> &layer, double *s ){
  for(  int j = 0; j < layer.size(); j++ ) {
    *s +=  layer[j].weightsSelf[i] * f_sigmoid ( ( layer[j].state + layer[j].bias ) );
  }
}

/* -------------------------------------------------------------------------------------- */

void Ctrnn3Layers::reset ( void ){
  for(int i=0; i<num_input; i++){
    inputLayer[i].state = 0.0;
    inputLayer[i].s = 0.0;
  }
  for(int i=0; i<num_hidden; i++){
    hiddenLayer[i].state = 0.0;
    hiddenLayer[i].s = 0.0;
  }
  for(int i=0; i<num_output; i++){
    outputLayer[i].state = 0.0;
    outputLayer[i].s = 0.0;
  }
}

/* -------------------------------------------------------------------------------------- */

Ctrnn3Layers& Ctrnn3Layers::operator=(const Ctrnn3Layers &other)
{
  if(this != &other)
    {
      destroy();
      allocate(other.num_input, other.num_hidden, other.num_output);
      copy(other);
    }
  return *this;
}

/* -------------------------------------------------------------------------------------- */
