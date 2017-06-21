#include "./MISC/parameters.h"
#include "./EXP/experiment.h"
#include "./EXP/first_exp.h"

#ifdef _GRAPHICS_
#include "./RENDERING/viewer.h"
#include "./RENDERING/interface.h"
#endif

int main(int argc, char** argv)
{

  MPI_Init(&argc, &argv);
  MPI_Comm_rank(MPI_COMM_WORLD, &NbRank);
  MPI_Comm_size(MPI_COMM_WORLD, &NbTotProcesses);
  
  //parse the command line  
  Parameters* parameters = new Parameters( argc, argv );
  
  //Init the Experiment
  std::vector< Experiment* > exp;
  exp.push_back(new First_experiment ( parameters ) );

  //Evolutionary mode
  if( parameters->isModeEvolution() ){
    while(parameters->getCurrentGeneration() < parameters->getNbMaxGenerations() )
      {
	//if(NbRank == 0)
	//cerr << " Generation = " << parameters->getCurrentGeneration() << endl;
	exp[0]->runOneGeneration( );
	parameters->increment_current_generation();
      }
  }
  //Viewing mode
  else if( parameters->isModeViewing() ){
#ifdef _GRAPHICS_
    QApplication application(argc, argv);
    Viewer    *viewer = new Viewer( 0, exp[0], argc, argv );
    Interface *render = new Interface( 0, exp[0], viewer );
    render->show();
    return application.exec();
    delete viewer;
    delete render;
#endif
  }
  //Evaluation mode
  else if( parameters->isModeEvaluation() ){
    //exp[0]->from_genome_to_controllers( vector < vector <chromosome_type> > &genes, const int which_group );
    exp[0]->evaluate_solutions( );
  }

  for(int i = 0; i < exp.size(); i++)
    delete exp[i];
  exp.clear();

  delete parameters;
  MPI_Finalize();
  
  std::cout << " Rank = " << NbRank << " Programm End..." << std::endl; 
  return 0;
}
