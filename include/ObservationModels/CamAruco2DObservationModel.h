/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2013, Texas A&M University
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Rice University nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Authors: Saurav Agarwal, Ali-akbar Agha-mohammadi */

#ifndef LANDMARK_RANGEBEARING_H_
#define LANDMARK_RANGEBEARING_H_


#include "ObservationModelMethod.h"

class CamAruco2DObservationModel : public ObservationModelMethod
{

  static const int stateDim = 3;
  //static const int singleObservationDim = 3;
  static const int singleObservationDim = 4; /*[ ID, X, Y, Orientation of Landmark in Environment ]*/
  static const int landmarkInfoDim = 2; /*[ X, Y]*/
  //static const int obsNoiseDim = 3;
  
  public:
    typedef typename MPTraits::CfgType CfgType;

    typedef typename ObservationModelMethod<MPTraits>::ObservationType ObservationType;
    typedef typename ObservationModelMethod<MPTraits>::NoiseType ObsNoiseType;
    typedef arma::mat JacobianType;
    // z = h(x,v)
    // get the observation for a given configuration,
    // corrupted by noise from a given distribution

    CamAruco2DObservationModel() : ObservationModelMethod<MPTraits>() {
      this->SetName("CamAruco2DObservationModel");  
    }

    CamAruco2DObservationModel(typename MPTraits::MPProblemType* _problem,
    XMLNodeReader& _node);

      
    void PrintOptions(ostream& _out);
    void ParseXML(XMLNodeReader& _node);

    ObservationType GetObservation(const CfgType& _x, bool _isSimulation);
    ObservationType GetObservationPrediction(const CfgType& _x, const ObservationType& _Zg);

    // Jx = dh/dx
    JacobianType GetObservationJacobian(const CfgType& _x, const ObsNoiseType& _v, const ObservationType& _z);
    // Jv = dh/dv
    JacobianType GetNoiseJacobian(const CfgType& _x, const ObsNoiseType& _v, const ObservationType& _z);  

    ObservationType ComputeInnovation(CfgType& _xPrd, const ObservationType& _Zg);
    ObservationType RemoveSpuriousObservations(const ObservationType& _Zg);
    
    arma::mat GetObservationNoiseCovariance(const CfgType& _x, const ObservationType& _z);

    //TODO: this is a specialization; it could do well if it were generalized and moved somewhere else
    bool ArePointsCovisible(CfgType const& _x, arma::colvec const& _l);
    bool IsLandmarkVisible(const CfgType& _x, const arma::colvec& _l, double& _range, double& _bearing, double& _viewingAngle);
  //void WriteLandmarks();

  private:

    vector<arma::colvec> m_landmarks;
    //vector<int> m_seenLandmarks; // contains the list of Landmarks that are currently in the observation
    //vector<int> m_predictedLandmarks;
};



#endif
