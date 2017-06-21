#ifndef _CTRNN_FULLY_RECURRENT_
#define _CTRNN_FULLY_RECURRENT_

#include "controller.h"

class CtrnnFullyRecurrent : public Controller {

private:
    int    num_node;
    double low_bound_wts;
    double upper_bound_wts;
    double low_bound_tau;
    double upper_bound_tau;
    double low_bound_bias;
    double upper_bound_bias;
    double low_bound_sensorsGain;
    double upper_bound_sensorsGain;

    typedef struct {
        double state;
        vector<double> weights;
        double tau;
        double bias;
        double gain;
        double z;
        double s;
    } Node;
    vector <Node> neurons;

    void copy(const CtrnnFullyRecurrent &other);
    void allocate(int numNode);
    void destroy();

public:

    CtrnnFullyRecurrent();
    CtrnnFullyRecurrent(const CtrnnFullyRecurrent& other);
    virtual ~CtrnnFullyRecurrent();

    /* -------------------------------------------------------------------------------------------------- */
    /*                                             VIRTUAL FUNCTIONS                                      */
    /* -------------------------------------------------------------------------------------------------- */
    void init                          ( const vector <chromosome_type> &genes );
    void step                          ( const vector <double> &input, vector <double> &output);
    void reset                         ( void );
    void read_from_file                ( void );
    void compute_genotype_length       ( void );
    /* -------------------------------------------------------------------------------------------------- */
    /* -------------------------------------------------------------------------------------------------- */

    inline vector <Node> getNeurons  ( void )  { return neurons;  }

    CtrnnFullyRecurrent& operator=(const CtrnnFullyRecurrent &other);
};

#endif
