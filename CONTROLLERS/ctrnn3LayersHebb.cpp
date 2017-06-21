#include "ctrnn3LayersHebb.h"

/* -------------------------------------------------------------------------------------- */

void  Ctrnn3LayersHebb::read_from_file ( void )
{
    ifstream I ("../CONTROLLERS/ctrnn3LayersHebb");
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
        cerr << " In ../CONTROLLERS/ctrnn3LayersHebb specifications' file --- End check was not 999" << endl;
        exit(0);
    }
}

/* -------------------------------------------------------------------------------------- */

void Ctrnn3LayersHebb::compute_genotype_length ( void )
{
    genotype_length = 0;
    //weights input-hidden
    for(int i = 0; i < num_input; i++)
        for(int h = 0; h < num_hidden; h++)
            genotype_length++;

    //Hebbian parameters for hidden neurons
    for(int h = 0; h < num_hidden; h++)
        for(int p = 0; p < HEBB_PARAMETERS_PER_NEURON; p++)
            genotype_length++;

    //weights hidden-output
    for(int h = 0; h < num_hidden; h++)
        for(int o = 0; o < num_output; o++)
            genotype_length++;

    //tau - it only applies to hidden nodes
    for(int h = 0; h < num_hidden; h++) genotype_length++;

    //bias - it applies to hidden nodes plus a single bias for all input
    //nodes, and a single bias for all output nodes
    for(int h = 0; h < num_hidden+2; h++) genotype_length++;

    //sensor gain - there is only one single gain for all input nodes
    genotype_length++;
}

/* -------------------------------------------------------------------------------------- */

Ctrnn3LayersHebb::Ctrnn3LayersHebb( )
    : Controller()
{
    read_from_file ( );
    compute_genotype_length ( );

    allocate(num_input, num_hidden, num_output);
}

/* -------------------------------------------------------------------------------------- */

Ctrnn3LayersHebb::Ctrnn3LayersHebb(const Ctrnn3LayersHebb& other)
{
    allocate(other.num_input, other.num_hidden, other.num_output);
    copy(other);
}

/* -------------------------------------------------------------------------------------- */

void Ctrnn3LayersHebb::copy(const Ctrnn3LayersHebb &other)
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

    learning_rate = other.learning_rate;
    learning_rule = other.learning_rule;
    synapsis_sign = other.synapsis_sign;
    hebbianWeights = other.hebbianWeights;
}

/* -------------------------------------------------------------------------------------- */

void Ctrnn3LayersHebb::allocate(int numInput, int numHidden, int numOutput)
{
    inputLayer.resize(numInput);
    for(int i = 0 ; i < numInput ; i++)
        inputLayer[i].weightsOut.assign(numHidden, 0.0);

    hiddenLayer.resize(numHidden);
    for(int i = 0 ; i < numHidden ; i++)
    {
        hiddenLayer[i].weightsOut.assign(numOutput, 0.0);
        hiddenLayer[i].weightsSelf.assign(numHidden, 0.0);
    }

    outputLayer.resize(numOutput);

    sensorsGain.assign(numInput, 0.0);

    learning_rate.assign(numHidden, 0.0);
    learning_rule.assign(numHidden, 0);
    synapsis_sign.assign(numHidden, 0.0);
    hebbianWeights.resize(numHidden);
}

/* -------------------------------------------------------------------------------------- */

void Ctrnn3LayersHebb::destroy()
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

    learning_rate.clear();
    learning_rule.clear();
    synapsis_sign.clear();
    hebbianWeights.clear();
}

/* -------------------------------------------------------------------------------------- */

Ctrnn3LayersHebb::~Ctrnn3LayersHebb()
{
    destroy();
}

/* -------------------------------------------------------------------------------------- */

void Ctrnn3LayersHebb::init ( const vector <chromosome_type> &genes ){
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

    for( int i=0; i<num_input; i++){
        inputLayer[i].state         = 0.0;
        inputLayer[i].fire_rate     = 0.0;
        inputLayer[i].pre_fire_rate = 0.0;
        for(int j=0; j< num_hidden; j++){
            inputLayer[i].weightsOut[j] =  get_value(genes, counter)*(upper_bound_inputWts - low_bound_inputWts) + low_bound_inputWts;
            //cerr << " inputLayer["<<i<<"].weightsOut["<<j<<"] = " << get_value(genes, counter) /*inputLayer[i].weightsOut[j]*/ << endl;
            counter++;
        }
        inputLayer[i].tau  = single_tau;
        inputLayer[i].bias = single_bias;
        sensorsGain[i]     = single_gain;
    }
    /* --------------------------------------------------------------- */

    /* ------------------ CONTROLLER::HIDDEN_LAYER -------------------- */
    for( int i=0; i<num_hidden; i++){
        hiddenLayer[i].state         = 0.0;
        hiddenLayer[i].fire_rate     = 0.0;
        hiddenLayer[i].pre_fire_rate = 0.0;
        for(int j=0; j<num_output; j++){
            hiddenLayer[i].weightsOut[j] = get_value(genes, counter)*(upper_bound_hiddenWts - low_bound_hiddenWts) + low_bound_hiddenWts;
            counter++;
        }
        hiddenLayer[i].tau         = pow(10, (low_bound_hiddenTau + (upper_bound_hiddenTau * get_value(genes, counter) )));
        counter++;
        hiddenLayer[i].bias        = get_value(genes, counter)*(upper_bound_hiddenBias - low_bound_hiddenBias) + low_bound_hiddenBias;
        counter++;
        for( int j=0; j<num_hidden; j++){
            hebbianWeights[i][j] = gsl_rng_uniform_pos( GSL_randon_generator::r_rand ) * INIT_HEBB_SYNAPSIS_MAX_VAL;
            hiddenLayer[i].weightsSelf[j] = hebbianWeights[i][j]*upper_bound_hiddenWts;
        }
    }
    /* ---------------------------------------------------------------- */

    /* ------------------ CONTROLLER::OUTPUT_LAYER -------------------- */
    //SINGLE TAU EQUAL TO DELTA_T
    single_tau           = delta_t;

    // SINGLE BIAS FOR ALL OUTPUT
    single_bias = get_value(genes, counter)*(upper_bound_outputBias - low_bound_outputBias) + low_bound_outputBias;
    counter++;

    for(int i=0; i<num_output; i++){
        outputLayer[i].state      = 0.0;
        outputLayer[i].fire_rate  = 0.0;
        outputLayer[i].pre_fire_rate  = 0.0;
        outputLayer[i].tau        =  single_tau;
        outputLayer[i].bias       =  single_bias;
    }
    /* --------------------------------------------------------------- */

    /* ------------------ CONTROLLER::HEBBIAN PARM ------------------- */
    for( int h=0; h<num_hidden; h++){
        if( get_value(genes, counter) ) synapsis_sign[h] = 1.0;
        else {
            synapsis_sign[h] = -1.0;
            for( int j=0; j<num_hidden; j++) hiddenLayer[h].weightsSelf[j] *= synapsis_sign[h];
        }
        counter++;
        //cerr << " synapsis_sign[" << h << " ] = " <<  synapsis_sign[h]  << endl;

        if( get_value(genes, counter) && get_value(genes, counter+1) ) learning_rate[h] = 0.0;
        else if( get_value(genes, counter) && !get_value(genes, counter+1) ) learning_rate[h] = 0.3;
        else if( !get_value(genes, counter) && get_value(genes, counter+1) ) learning_rate[h] = 0.6;
        else learning_rate[h] = 1.0;
        counter += 2;
        //cerr << " learn rate[" << h << " ] = " <<  learning_rate[h]  << endl;

        if( get_value(genes, counter) && get_value(genes, counter+1) )
            hiddenLayer[h].delta_hebbian_rule = &Ctrnn3LayersHebb::plain_delta_hebbian_weights;//learning_rule[h] = PLAIN;
        else if( get_value(genes, counter) && !get_value(genes, counter+1) )
            hiddenLayer[h].delta_hebbian_rule = &Ctrnn3LayersHebb::post_delta_hebbian_weights;//learning_rule[h] = POST;
        else if( !get_value(genes, counter) && get_value(genes, counter+1) )
            hiddenLayer[h].delta_hebbian_rule = &Ctrnn3LayersHebb::pre_delta_hebbian_weights;//learning_rule[h] = PRE;
        else
            hiddenLayer[h].delta_hebbian_rule = &Ctrnn3LayersHebb::cov_delta_hebbian_weights;//learning_rule[h] = COV;
        counter += 2;
        //cerr << " learn rule[" << h << " ] = " <<  learning_rule[h]  << endl;
    }

    if( counter != genotype_length ){
        cerr << "'In ctrnn3LayersHebb.cpp init():: the number of genes is wrong"
             << " " << genotype_length << " " << counter << endl;
        exit(0);
    }
}

/* -------------------------------------------------------------------------------------- */

void Ctrnn3LayersHebb::step ( const vector <double> &inputs, vector <double> &outputs ){

    for( int i=0; i < num_input; i++) {
        inputLayer[i].state = (inputs[i] * sensorsGain[i] );
    }

    update_hidden_layer();
    update_output_layer_from_hidden( );

    /* here we set the actuators state */
    for( int i = 0; i < num_output; i++ ){
        outputs[i] = f_sigmoid( (outputLayer[i].state + outputLayer[i].bias) );
    }
}

/* -------------------------------------------------------------------------------------- */

/* This function compute the activation of each hidden neurons */
void Ctrnn3LayersHebb::update_hidden_layer( ){
    for(int i=0; i < num_hidden; i++) {
        hiddenLayer[i].pre_fire_rate = hiddenLayer[i].fire_rate;
        hiddenLayer[i].s             = -hiddenLayer[i].state;
        hiddenLayer[i].s            += update_StimesW  ( i, inputLayer );
        hiddenLayer[i].s            += update_StimesWself  ( i, hiddenLayer );
    }
    double delta_w = 0.0;
    for(int j=0; j < num_hidden; j++){ //to neuron j
        hiddenLayer[j].state += (hiddenLayer[j].s * (delta_t/hiddenLayer[j].tau));
        hiddenLayer[j].fire_rate = f_sigmoid(hiddenLayer[j].state + hiddenLayer[j].bias);

        for( int i=0; i<num_hidden; i++) {//from neuron i
            (this->*hiddenLayer[j].delta_hebbian_rule)( i, j, &delta_w );
            hebbianWeights[i][j] += (learning_rate[j] * delta_w );
            hiddenLayer[i].weightsSelf[j] = hebbianWeights[i][j] * upper_bound_hiddenWts * synapsis_sign[i];
        }
    }
}

/* -------------------------------------------------------------------------------------- */

/* This function compute the activation of each output neurons taking
   into account the hidden layer */
void Ctrnn3LayersHebb::update_output_layer_from_hidden ( ){
    for(int i=0; i < num_output; i++) {
        outputLayer[i].s = -outputLayer[i].state;
        outputLayer[i].s += update_StimesW  ( i, hiddenLayer );
    }
    for(int i=0; i < num_output; i++)
        outputLayer[i].state += (outputLayer[i].s * (delta_t/outputLayer[i].tau));
}

/* -------------------------------------------------------------------------------------- */

/* This function is used to multiply the firig rate 
   of the neurons (layer) with a connection to neuron j */
double Ctrnn3LayersHebb::update_StimesW ( int j, vector<Node> &layer ){
    double sum = 0.0;
    for(  int i = 0; i < layer.size(); i++ ) {
        double z = f_sigmoid ( ( layer[i].state + layer[i].bias ) );
        sum  +=  layer[i].weightsOut[j] * z;
    }
    return sum;
}

/* -------------------------------------------------------------------------------------- */

/* This function is used to multiply the firig rate 
   of the neurons (layer) with the self-connections to neuron j */
double Ctrnn3LayersHebb::update_StimesWself ( int j, vector<Node> &layer ){
    double sum = 0.0;
    for( int i = 0; i < layer.size(); i++ ) {
        double z = f_sigmoid ( ( layer[i].state + layer[i].bias ) );
        sum  +=  layer[i].weightsSelf[j] * z;
    }
    return sum;
}

/* -------------------------------------------------------------------------------------- */

void Ctrnn3LayersHebb::reset ( void ){
    for(int i=0; i<num_input; i++){
        inputLayer[i].state = 0.0;
        inputLayer[i].s = 0.0;
    }
    for(int i=0; i<num_hidden; i++){
        hiddenLayer[i].state = 0.0;
        hiddenLayer[i].s = 0.0;
        hiddenLayer[i].fire_rate = 0.0;
        hiddenLayer[i].pre_fire_rate = 0.0;
        for(int j = 0; j<num_hidden; j++){
            hebbianWeights[i][j] = gsl_rng_uniform_pos( GSL_randon_generator::r_rand ) * INIT_HEBB_SYNAPSIS_MAX_VAL;
            hiddenLayer[i].weightsSelf[j] = hebbianWeights[i][j]*upper_bound_hiddenWts;
        }
    }
    for(int i=0; i<num_output; i++){
        outputLayer[i].state = 0.0;
        outputLayer[i].s = 0.0;
    }
}

/* -------------------------------------------------------------------------------------- */

void Ctrnn3LayersHebb::plain_delta_hebbian_weights( int i, int j, double *val ){
    *val = hiddenLayer[j].fire_rate * hiddenLayer[i].pre_fire_rate * (1.0 - hebbianWeights[i][j]);
}

/* -------------------------------------------------------------------------------------- */

void Ctrnn3LayersHebb::post_delta_hebbian_weights( int i, int j, double *val ) {
    *val = ( (hebbianWeights[i][j] * hiddenLayer[j].fire_rate * (-1.0 + hiddenLayer[i].pre_fire_rate)) +
             (hiddenLayer[j].fire_rate * hiddenLayer[i].pre_fire_rate * (1.0 - hebbianWeights[i][j]) ) );
}

/* -------------------------------------------------------------------------------------- */

void Ctrnn3LayersHebb::pre_delta_hebbian_weights( int i, int j, double *val ){
    *val = ( (hebbianWeights[i][j] * /*hiddenLayer[i].pre_fire_rate **/ (-1.0 + hiddenLayer[j].fire_rate)) +
             (hiddenLayer[j].fire_rate * hiddenLayer[i].pre_fire_rate * (1.0 - hebbianWeights[i][j]) ) );
}

/* -------------------------------------------------------------------------------------- */

void Ctrnn3LayersHebb::cov_delta_hebbian_weights( int i, int j, double *val ){
    double res=tanh(4.0*(1.0-fabs(hiddenLayer[i].pre_fire_rate - hiddenLayer[j].fire_rate) - 2.0) );
    if( res > 0.0 ) *val = (1.0-hebbianWeights[i][j])*res;
    else *val = (hebbianWeights[i][j]*res);
}

/* -------------------------------------------------------------------------------------- */

Ctrnn3LayersHebb& Ctrnn3LayersHebb::operator=(const Ctrnn3LayersHebb &other)
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
/* -------------------------------------------------------------------------------------- */
