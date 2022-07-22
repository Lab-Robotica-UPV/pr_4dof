/*
  Copyright (c) 2008 Florent D'halluin , Sylvain Calinon, 
  LASA Lab, EPFL, CH-1015 Lausanne, Switzerland, 
  http://www.calinon.ch, http://lasa.epfl.ch

  The program is free for non-commercial academic use. 
  Please acknowledge the authors in any academic publications that have 
  made use of this code or part of it. Please use this BibTex reference: 
 
  @article{Calinon07SMC,
  title="On Learning, Representing and Generalizing a Task in a Humanoid 
  Robot",
  author="S. Calinon and F. Guenter and A. Billard",
  journal="IEEE Transactions on Systems, Man and Cybernetics, Part B. 
  Special issue on robot learning by observation, demonstration and 
  imitation",
  year="2007",
  volume="37",
  number="2",
  pages="286--298"
  }
*/

#include "MathLib.h"
#include "gmr.h"
#include <iostream>
#include <string>

int main(int argc, char *argv[]) 
{ 

  // Directorio con los datos
  const std::string directory = "Rafa_der_joint";
  // Number of files
  const int n_files = 5;
  // If offset_removal is true, the data will be subtracted the first row
  const bool offset_removal = true;
  // Number of temporal samples to use for regression (the data will be interpolated)
  const unsigned int nbData = 620;
  // Number of states of the GMM
  const int n_states = 10;
  // Include norm of the signal?
  const bool include_norm = true;

  GaussianMixture g;

  ///////////////////////////////////////////////////////////////////////
  std::cout << "Load raw data from './indata/'..." << std::flush;
  Matrix rawData[n_files];
  // unsigned int nbData=0;
  char filename[256];
  for (unsigned int i = 0; i < n_files; i++){
    sprintf(filename,"./%s/indata/data%.2d.txt",directory.c_str(),i+1);
    rawData[i] = g.loadDataFile(filename); 
    // nbData += rawData[i].RowSize(); 
    if (offset_removal){
        Vector first_row = rawData[i].GetRow(0);
        for (int j=0; j<rawData[i].RowSize(); j++){
            float timestamp = rawData[i](j,0);
            rawData[i].SetRow(rawData[i].GetRow(j)-first_row, j);
            rawData[i](j,0) = timestamp;
        }
    }
    if (include_norm){
        rawData[i].Resize(rawData[i].RowSize(), rawData[i].ColumnSize()+1, true);
            for (int j=0; j<rawData[i].RowSize(); j++){
                Vector row = rawData[i].GetRow(j);
                // Timestamp does not participate in norm
                row[0] = 0.0;
                rawData[i](j,rawData[i].ColumnSize()-1) = row.Norm();
            }  
    }
  }
  // nbData = (int)(nbData/NBSAMPLES);
  unsigned int nbVar = rawData[0].ColumnSize();
  std::cout << "ok" << std::endl;

  ///////////////////////////////////////////////////////////////////////
  std::cout << "Rescale the raw data and save the result to './outdata'..." 
	    << std::flush; 
  Matrix interpol, dataset;
  interpol.Resize(nbData,nbVar);
  // Initial sample time
  double ts_init = rawData[0](1,0) - rawData[0](0,0);
  for (unsigned int i = 0; i < n_files; i++){
    g.HermitteSplineFit(rawData[i],nbData,interpol);
    // HermitteSplineFit does not perform a linear time interpolation, we do it now.
    // Final sample time
    double ts_fin = ts_init*(rawData[i].RowSize())/(nbData);
    for (int j=0; j<interpol.RowSize(); j++){
        interpol(j,0) *= ts_fin;
    } 
    dataset = interpol.VCat(dataset);
    sprintf(filename,"./%s/outdata/data%.2d_rescaled.txt",directory.c_str(),i+1);
    g.saveDataFile(filename,interpol);
  }
  std::cout << "ok" << std::endl;

  /////////////////////////////////////////////////////////////////////// 
  std::cout << "Learn the GMM model and save the result to './outdata'..." 
	    << std::flush; 
  g.initEM_TimeSplit(n_states,dataset); // initialize the model

  //g.debug();
  g.doEM(dataset); // performs EM
  sprintf(filename,"./%s/outdata/gmm.txt",directory.c_str());
  g.saveParams(filename);
  std::cout << "ok" << std::endl;

  ///////////////////////////////////////////////////////////////////////
  std::cout << "Apply the GMR regression and save the result to './outdata'..." 
	    << std::flush; 
  Vector inC(1), outC(nbVar-1);
  inC(0)=0; // Columns of the input data for regression (here, time)
  for(unsigned int i=0;i<nbVar-1;i++) 
    outC(i)=(float)(i+1); // Columns for output : remainings
  //Matrix inData = rawData[0].GetColumnSpace(0,1);
  Matrix inData = interpol.GetColumnSpace(0,1);

  Matrix *outSigma;
  outSigma = new Matrix[nbData];
  Matrix outData = g.doRegression(inData,outSigma,inC,outC);
  std::string filename_mu = directory + "/outdata/gmr_Mu.txt";
  std::string filename_sigma = directory + "/outdata/gmr_Sigma.txt";
  g.saveRegressionResult(filename_mu.c_str(),filename_sigma.c_str(), 
			inData, outData, outSigma);
  std::cout << "ok" << std::endl;
  std::cout << "You can now run 'plotall' in matlab (or type 'make plot' here) to display the GMM/GMR results." << std::endl;
  
  return 0;
}