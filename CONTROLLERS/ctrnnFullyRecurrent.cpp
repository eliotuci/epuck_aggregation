#include "ctrnnFullyRecurrent.h"

/* ---------------------------------------------------------- */

void CtrnnFullyRecurrent::read_from_file ( void )
{
    ifstream I ("../CONTROLLERS/ctrnnFullyRecurrent");
    if(!I) {
        cerr << "File with network structure not found" <<endl;
        exit(0);
    }

    delta_t    = getDouble   ('=', I);
    num_input  = getInt   ('=', I);
    num_output = getInt   ('=', I);
    num_node   = getInt   ('=', I);

    low_bound_wts           = getDouble   ('=', I);
    upper_bound_wts         = getDouble   ('=', I);
    low_bound_tau           = getDouble   ('=', I);
    upper_bound_tau         = getDouble   ('=', I);
    low_bound_bias          = getDouble   ('=', I);
    upper_bound_bias        = getDouble   ('=', I);
    low_bound_sensorsGain   = getDouble   ('=', I);
    upper_bound_sensorsGain = getDouble   ('=', I);

    int check = getInt('=',I);
    I.close();
    if( check != 999 ){
        cerr << " In ../CONTROLLERS/ctrnn_fully_recurrent specifications file -- End check was not 999 " << endl;
    }
}

/* ---------------------------------------------------------- */

void CtrnnFullyRecurrent::compute_genotype_length()
{
    genotype_length = 0;
    genotype_length = ( num_node * 2.0 ) + 1; //one Tau and one Bias for
    //each node plus one
    //single Gain;
    for( int i=0; i<num_node; i++){
        for( int j=0; j<num_node; j++){
            genotype_length++;
        }
    }
}

/* ---------------------------------------------------------- */

CtrnnFullyRecurrent::CtrnnFullyRecurrent( )
    : Controller()
{
    read_from_file (  );

    compute_genotype_length();

    allocate(num_node);
}

/* ---------------------------------------------------------- */

CtrnnFullyRecurrent::CtrnnFullyRecurrent (const CtrnnFullyRecurrent& other)
{
    allocate(other.num_node);
    copy(other);
}

/* ---------------------------------------------------------- */

void CtrnnFullyRecurrent::copy(const CtrnnFullyRecurrent &other)
{
    Controller::copy(other);

    num_node = other.num_node;
    low_bound_wts = other.low_bound_wts;
    upper_bound_wts = other.upper_bound_wts;
    low_bound_tau = other.low_bound_tau;
    upper_bound_tau = other.upper_bound_tau;
    low_bound_bias = other.low_bound_bias;
    upper_bound_bias = other.upper_bound_bias;
    low_bound_sensorsGain = other.low_bound_sensorsGain;
    upper_bound_sensorsGain = other.upper_bound_sensorsGain;

    neurons = other.neurons;
}

/* ---------------------------------------------------------- */

void CtrnnFullyRecurrent::allocate(int numNode)
{
    neurons.resize(numNode);
    for(int i = 0; i < numNode ; i++)
        neurons[i].weights.assign(numNode, 0.0);
}

/* ---------------------------------------------------------- */

void CtrnnFullyRecurrent::destroy()
{
    Controller::destroy();

    for(int i = 0; i < num_node ; i++)
        neurons[i].weights.clear();
    neurons.clear();
}

/* ---------------------------------------------------------- */

CtrnnFullyRecurrent::~CtrnnFullyRecurrent()
{
    destroy();
}

/* ---------------------------------------------------------- */

/* This is the function which transforms 
   a string of genes (real number between [0, 1])
   in a network */
void CtrnnFullyRecurrent::init ( const vector <chromosome_type> &genes ){
    int  counter = 0;

    double Gain = get_value(genes, counter) * ( upper_bound_sensorsGain - low_bound_sensorsGain ) + low_bound_sensorsGain;
    counter++;
    for( int i=0; i<num_node; i++){
        neurons[i].state     = 0.0;
        neurons[i].z         = 0.0;
        neurons[i].s         = 0.0;
        for( int j=0; j<num_node; j++){
            neurons[i].weights[j] = get_value(genes, counter) * (upper_bound_wts - low_bound_wts ) + low_bound_wts;
            counter++;
        }
        neurons[i].tau        = pow(10, (low_bound_tau + (upper_bound_tau * get_value(genes, counter))));
        counter++;
        neurons[i].bias       = get_value(genes, counter) * (upper_bound_bias - low_bound_bias) + low_bound_bias;
        counter++;
        neurons[i].gain       = Gain;
    }

    if( counter != genotype_length ){
        cerr << "In ctrnnFullyRecurrent.cpp init():: the number of genes is wrong"
             << " genotype_length = " << genotype_length << " counter = " << counter << endl;
        exit(0);
    }
}

/* ---------------------------------------------------------- */

/* This function makes the update of the net step-by-step
   you should call this fucntion every time-step*/
void CtrnnFullyRecurrent::step ( const vector <double> &input, vector <double> &output){
    //Here we compute the firing rate "z"
    for( int i=0; i < num_node; i++){
        neurons[i].z = f_sigmoid(neurons[i].state + neurons[i].bias);
    }

    /* Set i-th neuron state update (Euler forward) */
    for( int i = 0; i < num_node; i++ ) {
        neurons[i].s = -neurons[i].state;
        for( int j=0; j < num_node; j++) {
            neurons[i].s +=  neurons[i].weights[j]*neurons[j].z;
        }

        /* add input here */
        if( i < num_input ) {
            neurons[i].s  += (input[i] * neurons[i].gain);
        }

        /* Euler numerical integration */
        neurons[i].state += (neurons[i].s * (delta_t/neurons[i].tau));
    }

    /* here we set the actuators state */
    for( int out = 0; out < num_output; out++ ) {
        output[out] = f_sigmoid( neurons[(num_node-out-1)].state + neurons[(num_node-out-1)].bias );
    }
}


/* ---------------------------------------------------------- */

void CtrnnFullyRecurrent::reset( void ){
    for( int i=0; i< num_node; i++){
        neurons[i].state = 0.0;
    }
}

/* ---------------------------------------------------------- */

CtrnnFullyRecurrent& CtrnnFullyRecurrent::operator=(const CtrnnFullyRecurrent &other)
{
    if(this != &other)
    {
        destroy();
        allocate(other.num_node);
        copy(other);
    }
    return *this;
}

/* ---------------------------------------------------------- */
/* ---------------------------------------------------------- */
