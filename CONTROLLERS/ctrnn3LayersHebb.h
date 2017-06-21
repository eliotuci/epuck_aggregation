#ifndef CTRNN_3_LAYERS_HEBB_H
#define CTRNN_3_LAYERS_HEBB_H

#include "controller.h"

#define HEBB_PARAMETERS_PER_NEURON 5
#define INIT_HEBB_SYNAPSIS_MAX_VAL 0.1

enum {PLAIN=0, POST=1, PRE=2, COV=3};

class Ctrnn3LayersHebb : public Controller {

private:
    int num_hidden;

    typedef void (Ctrnn3LayersHebb::*FuncPtr_1)( int i, int j, double *val);

    typedef struct {
        double state;
        vector<double> weightsOut;
        vector<double> weightsSelf;
        double pre_fire_rate;
        double fire_rate;
        double s;
        double tau;
        double bias;
        double gain;
        FuncPtr_1 delta_hebbian_rule;
    } Node;
    vector <Node> inputLayer;
    vector <Node> hiddenLayer;
    vector <Node> outputLayer;
    
    double low_bound_inputWts;
    double upper_bound_inputWts;
    double low_bound_inputBias;
    double upper_bound_inputBias;

    double low_bound_hiddenWts;
    double upper_bound_hiddenWts;
    double low_bound_hiddenTau;
    double upper_bound_hiddenTau;
    double low_bound_hiddenBias;
    double upper_bound_hiddenBias;

    double low_bound_outputBias;
    double upper_bound_outputBias;

    double low_bound_sensorsGain;
    double upper_bound_sensorsGain;
    vector<double> sensorsGain;

    void   update_hidden_layer                 ( void );
    void   update_output_layer_from_hidden     ( void );
    double update_StimesW                      ( int, vector<Node>& );
    double update_StimesWself                  ( int, vector<Node>& );


    /* -------------------------------------------------------------------------------------- */
    /*                                     FOR HEBBIAN LEARNING                               */
    /* -------------------------------------------------------------------------------------- */
    vector <double> learning_rate;
    vector <int>    learning_rule;
    vector <double> synapsis_sign;
    vector < vector <double> > hebbianWeights;

    void plain_delta_hebbian_weights ( int i, int j, double *val );
    void post_delta_hebbian_weights  ( int i, int j, double *val );
    void pre_delta_hebbian_weights   ( int i, int j, double *val );
    void cov_delta_hebbian_weights   ( int i, int j, double *val );
    /* -------------------------------------------------------------------------------------- */
    /* -------------------------------------------------------------------------------------- */

    void copy(const Ctrnn3LayersHebb &other);
    void allocate(int numInput, int numHidden, int numOutput);
    void destroy();

public:
    Ctrnn3LayersHebb(  );
    Ctrnn3LayersHebb(const Ctrnn3LayersHebb& other);
    virtual ~Ctrnn3LayersHebb();

    /* -------------------------------------------------------------------------------------------------- */
    /*                                             VIRTUAL FUNCTIONS                                      */
    /* -------------------------------------------------------------------------------------------------- */
    void init                          ( const vector <chromosome_type> &genes );
    void step                          ( const vector <double> &input_array, vector <double> &output_array);
    void reset                         ( void );
    void read_from_file                ( void );
    void compute_genotype_length       ( void );
    /* -------------------------------------------------------------------------------------------------- */
    /* -------------------------------------------------------------------------------------------------- */

    Ctrnn3LayersHebb& operator=(const Ctrnn3LayersHebb &other);

};

#endif
