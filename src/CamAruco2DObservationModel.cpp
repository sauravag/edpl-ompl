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

template <class MPTraits>
CamAruco2DObservationModel<MPTraits>::
CamAruco2DObservationModel(typename MPTraits::MPProblemType* _problem,
XMLNodeReader& _node) :
  ObservationModelMethod<MPTraits>(_problem, _node) {
  using namespace arma;
  this->SetName("CamAruco2DObservationModel");
  ParseXML(_node);
  if(this->m_debug)
    PrintOptions(cout);
  
}

template <class MPTraits>
void
CamAruco2DObservationModel<MPTraits>::
ParseXML(XMLNodeReader& _node) {
  using namespace arma;

  double sigmaRange = _node.numberXMLParameter("sigma_range", true, 0.5, 0.0, std::numeric_limits<double>::max(),"???");
  double sigmaAngle = _node.numberXMLParameter("sigma_angle", true, 0.5, 0.0, std::numeric_limits<double>::max(),"???");
  double etaRD = _node.numberXMLParameter("eta_rd", true, 0.5, 0.0, std::numeric_limits<double>::max(),"???");
  double etaRPhi = _node.numberXMLParameter("eta_rphi", true, 0.5, 0.0, std::numeric_limits<double>::max(),"???");
  double etaThetaD = _node.numberXMLParameter("eta_thetad", true, 0.5, 0.0, std::numeric_limits<double>::max(),"???");
  double etaThetaPhi = _node.numberXMLParameter("eta_thetaphi", true, 0.5, 0.0, std::numeric_limits<double>::max(),"???");

  this->m_sigma << sigmaRange << sigmaAngle*PI/180 << endr; //sigmaAngle input in degrees; converted to radians here
  this->m_etaD << etaRD << etaThetaD << endr;
  this->m_etaPhi << etaRPhi << etaThetaPhi << endr;

  for(XMLNodeReader::childiterator witr = _node.children_begin();
    witr != _node.children_end(); ++witr){
    if(witr->getName() == "landmark") {
        colvec landmark(singleObservationDim);
        landmark[0] = witr->numberXMLParameter("id", true, 0.0, 0.0, 1023.0, "id of landmark");  // ID will also be read in from XML, so update XML
        landmark[1] = witr->numberXMLParameter("x", true, 0.0, DBL_MIN, DBL_MAX, "X position of landmark");   
        landmark[2] = witr->numberXMLParameter("y", true, 0.0, DBL_MIN, DBL_MAX, "Y position of landmark");
        landmark[3] = witr->numberXMLParameter("theta", false, 0.0, -1.0, 1.0, "Global Orientation of landmark");
        
        m_landmarks.push_back(landmark);  
    }
  }

  OGLDisplay<MPTraits>::AddLandmarks(m_landmarks);

  //WriteLandmarks();

  _node.warnUnrequestedAttributes();
}

//template <class MPTraits>
//void 
//CamAruco2DObservationModel<MPTraits>::
//WriteLandmarks() {
//
//  stringstream ss;
//  for(int i = 0; i < m_landmarks.size(); ++i) {
//    ss << "AddLandmark ";
//    ss << m_landmarks[i][0] << " ";
//    ss << m_landmarks[i][1] << " ";
//    ss << " ; " ;
//    
//  }
//
//  shared_ptr<Writer> writer = GetMPProblem()->GetFileWriter("");
//  writer << ss;
//}



template <class MPTraits>
void
CamAruco2DObservationModel<MPTraits>::
PrintOptions(ostream& _out) {

}

template <class MPTraits>
typename CamAruco2DObservationModel<MPTraits>::ObservationType
CamAruco2DObservationModel<MPTraits>::
GetObservation
(const CfgType& _x,
bool _isSimulation) {
  using namespace arma;


  //For each landmark, produces an observation that is the range and bearing
  //of the given Cfg from that landmark
  //Result is the concatenation of all such observations
  //Obs dim is 2x the size of the landmarks

  colvec xVec = _x.GetArmaData();
  
  //cout<<"CamArucoObservationModel2D, in GetObservation, xVec(2) :"<<xVec(2)<<endl;
  assert(abs(xVec(2)) <= PI+0.01);

  ObservationType z;
  
  int counter = 0;
  //generate observation from state, and corrupt with the given noise
  for(unsigned int i = 0; i < m_landmarks.size(); ++i) {

    double landmarkRange =0, landmarkBearing = 0, relativeAngle = 0;
    
    if(IsLandmarkVisible(_x, m_landmarks[i], landmarkRange , landmarkBearing, relativeAngle)){
       
        //cout<<"Trying to resize"<<endl;
        z.resize((counter+1)*singleObservationDim ,  1);
        
        colvec noise(2);
        
        if(_isSimulation){
        
          // generate gaussian noise
          //get standard deviations of noise (sqrt of covariance matrix)
          //extract state from Cfg and normalize

          //generate noise scaling/shifting factor
          
          colvec noise_std = this->m_etaD*landmarkRange + this->m_etaPhi*relativeAngle + this->m_sigma;
          
          //generate raw noise
          colvec randNoiseVec = randn<colvec>(2);
          
          //generate noise from a distribution scaled and shifted from
          //normal distribution N(0,1) to N(0,eta*range + sigma)
          //(shifting was done in GetNoiseCovariance)
          noise = noise_std%randNoiseVec;
        }
        else{
          
          // generate zero noise
          colvec zeronoise =  zeros<colvec>(landmarkInfoDim); 
          noise =  zeronoise;
        
        }
        
        z[singleObservationDim*counter] = m_landmarks[i](0) ; // id of the landmark
        z[singleObservationDim*counter + 1 ] = landmarkRange + noise[0]; // distance to landmark
        z[singleObservationDim*counter+2] = landmarkBearing + noise[1];
        z[singleObservationDim*counter+3] = m_landmarks[i](3); 
          
        //cout<<"Observation set for i = "<<i<<endl;
        counter++;
    }
  }

  return z;

}

template <class MPTraits>
typename CamAruco2DObservationModel<MPTraits>::ObservationType
CamAruco2DObservationModel<MPTraits>::GetObservationPrediction(const CfgType& _x, const ObservationType& _Zg) {

  using namespace arma;


  //For each landmark that is seen by the camera (or sensor), produces a the prediction based on the previous state estimate
  //observation that is the range and bearing 
  //of the given Cfg from that landmark
  //Result is the concatenation of all such observations
  //Obs dim is 2n x 1 , where "n" is the number of landarks and "2" is the size of each landmark

  colvec xVec = _x.GetArmaData();

  assert(abs(xVec(2)) <= PI+0.01); // making sure that the angles in Cfg are normalised between -PI and PI

  ObservationType z;
  
  //int counter = 0;

  //generate observation from state, and corrupt with the given noise
  for(int k = 0; k< _Zg.n_rows / singleObservationDim ;k++){
  
    for(unsigned int i = 0; i < m_landmarks.size() ; i++) {

      double landmarkRange =0, landmarkBearing = 0, viewingAngle = 0;
      
      if(m_landmarks[i](0) == _Zg[singleObservationDim*k]){
         
          //cout<<"Trying to resize"<<endl;
          z.resize((k+1)*singleObservationDim ,  1);
          
          IsLandmarkVisible(_x, m_landmarks[i], landmarkRange , landmarkBearing, viewingAngle) ; // calling isLandmarkVisible to calculate the range and bearing 
          
          z[singleObservationDim*k] = m_landmarks[i](0)  ; // id of the landmark
          z[singleObservationDim*k + 1] = landmarkRange ; // distance to landmark
          z[singleObservationDim*k+2] = landmarkBearing  ; 
          z[singleObservationDim*k + 3] = m_landmarks[i](3)  ; // orientation of the landmark
          //cout<<"Observation set for i = "<<i<<endl;
          break;
      }
    }   
  }
  return z;

}

template <class MPTraits>
bool
CamAruco2DObservationModel<MPTraits>::
ArePointsCovisible(CfgType const& _x, arma::colvec const& _l) {
  Vector3D viewer = _x.GetRobotCenterPosition();
  Vector3D v1 = viewer + Vector3D(0,0,-0.0001);
  Vector3D v2 = viewer + Vector3D(0,0,0.0001);
  Vector3D v3(_l[1], _l[2], 0);
  Vector3D center = (v1+v2+v3)/3.0;

  //create a triangle in BYU format
  std::stringstream byuStream;
  byuStream << setprecision(16) << "1 3 1 3\n" << "1 3\n"
      << v1[0] << "  " << v1[1] << "  " << v1[2] << "\n"
      << v2[0] << "  " << v2[1] << "  " << v2[2] << "\n"
      << v3[0] << "  " << v3[1] << "  " << v3[2] << "\n"
      << "1 2 -3";

  shared_ptr<MultiBody> lineSegment(new MultiBody());
  FreeBody fb(lineSegment.get());
  fb.ReadBYU(byuStream);
  Transformation trans(Orientation(IdentityMatrix), center);
  fb.Configure(trans);
  lineSegment->AddBody(fb);
  lineSegment->buildCDstructure(RAPID);

  Rapid rapid;
  CDInfo cdInfo;
  string caller = "CamAruco2DObservationModel::ArePointsCovisible";

  Environment *env = this->GetMPProblem()->GetEnvironment();
  size_t usableCount = env->GetUsableMultiBodyCount();

  for(size_t i=0; i<usableCount; ++i) {
    shared_ptr<MultiBody> envBody = env->GetMultiBody(i);
    if(envBody->GetBodyType() == ACTIVE)
      continue;

    //TODO: this seems not to collide when it should, but only sometimes
    if(rapid.IsInCollision(lineSegment, envBody, 
          *this->GetMPProblem()->GetStatClass(), cdInfo, 
          &caller, 0)) {
      return false;
    }
  }

  return true;
}

template <class MPTraits>
bool
CamAruco2DObservationModel<MPTraits>::
IsLandmarkVisible
(const CfgType& _x, const arma::colvec& _l, double& _range, double& _bearing, double& _viewingAngle) {
  
    using namespace arma;
  
    //cout<<"Checking visibility for ID: "<<_l[0]<<endl;
    
    double fov = 30* PI/180; // radians
    double maxrange = 2.5; // meters // NOTE: if you change this value, you must make a corresponding change in the "draw" function of the Cfg.cpp file.
  
    colvec xVec = _x.GetArmaData(); // GetArmaData 

    //cout<<"in isLandmarkVisible Robot Orientation (degrees)= "<< xVec[2]*180/PI<<endl; 
    
    colvec diff =  _l.subvec(1,2) - xVec.subvec(0,1);
    
    //norm is the 2nd Holder norm, i.e. the Euclidean norm
    double d = norm(diff,2); 
    
    double landmark_bearing =0;
    double robot_landmark_ray =  atan2(diff[1],diff[0]) ;
  
    //cout<<"The robot_landmark_ray angle is in Global frame (degrees):"<< robot_landmark_ray*180/PI <<endl;
     
    double delta_theta = robot_landmark_ray - xVec[2];

    //cout<<"The delta_theta calculated is: "<<delta_theta<<endl;

    landmark_bearing = delta_theta ;

    if(delta_theta > PI ) {

        landmark_bearing =  (delta_theta - 2*PI) ;
    }
    if( delta_theta < -PI ){
  
        landmark_bearing =  delta_theta + 2*PI ;
    }
    
    double thetaLandmark =  _l[3]*PI ; 

    _viewingAngle = abs(acos(cos(thetaLandmark )*cos(xVec[2]) + sin(thetaLandmark )*sin(xVec[2])));
    
    //cout<<"The range to landmark is :"<<d<<endl;
    //cout<<"The bearing to landmark is :"<<landmark_bearing<<endl;
     if( _viewingAngle > PI/2 ) _viewingAngle = abs(_viewingAngle-PI );

    _range = d;
    _bearing = landmark_bearing; 
    if( abs(landmark_bearing) < fov && d < maxrange  ){
      //cout<<"Theta Landmark is :" <<thetaLandmark*180/PI<<endl;
      //cout<<"The robot heading is :"<<xVec[2]*180/PI<<endl;
      //cout<<" viewing angle of landmark is :"<<_viewingAngle*180/PI <<"  degrees"<<endl;
      assert(abs(_viewingAngle) <= PI / 2 );
        //cout<<"----Landmark #"<<_l[0]<<" is visible-----"<<endl;
      return true;//ArePointsCovisible(_x, _l);
    }
    
    //cout<<"Predicted to not see marker ID# "<<_l[0]<<"  based on estimate, but returning true" <<endl;
    return false;

}

template <class MPTraits>
typename CamAruco2DObservationModel<MPTraits>::JacobianType
CamAruco2DObservationModel<MPTraits>::
GetObservationJacobian
(const CfgType& _x,
const ObsNoiseType& _v,
const ObservationType& _z) {
  using namespace arma;
  
  int number_of_landmarks = _z.n_rows / singleObservationDim ;
  
  colvec xVec = _x.GetArmaData();
  
  mat H( (landmarkInfoDim)* number_of_landmarks, stateDim); // Since we are passing the common id list

  for(unsigned int i = 0; i < number_of_landmarks ; ++i) {

    int Indx=-999;

    for(int j=0;j< m_landmarks.size() ; j++){
    
      if( _z(i*singleObservationDim) == m_landmarks[j](0)){
      
        Indx = j;
        break;
      }
    }

    assert(Indx>=0);

    colvec diff =  m_landmarks[Indx].subvec(1,2) - xVec.subvec(0,1);
    
    double phi = atan2(diff[1], diff[0]);
    
    double r = norm(diff,2);
    
    mat H_i((landmarkInfoDim),stateDim);
    
    H_i <<  -cos(phi)    <<  -sin(phi)    <<   0 << endr
        <<   sin(phi)/r  <<  -cos(phi)/r  <<  -1 << endr;
        
    H.submat((landmarkInfoDim)*i, 0, (landmarkInfoDim)*i+1, 2) = H_i; 

  }

  return H;
}


template <class MPTraits>
typename CamAruco2DObservationModel<MPTraits>::JacobianType
CamAruco2DObservationModel<MPTraits>::
GetNoiseJacobian(const CfgType& _x, const ObsNoiseType& _v, const ObservationType& _z) {
  using namespace arma;

  //noise jacobian is just an identity matrix
  int number_of_landmarks = _z.n_rows / singleObservationDim ;

  mat M = eye(number_of_landmarks*2, number_of_landmarks*2); 

  return M;

}

template <class MPTraits>
arma::mat
CamAruco2DObservationModel<MPTraits>::
GetObservationNoiseCovariance
(const CfgType& _x,
const ObservationType& _z) {
  using namespace arma;

  //extract state from Cfg and normalize
  colvec xVec = _x.GetArmaData();

  int number_of_landmarks = _z.n_rows/singleObservationDim ;

  //generate noise scaling/shifting factors
  colvec noise( number_of_landmarks*(landmarkInfoDim)); 
 
  for(int i =0; i< number_of_landmarks ; i++){
  
    int indx=0; // is the index of the landmark in the vector m_landmarks, whose id matches the id contained in _z[i]
    
    // we find the landmark whose id matches the one in the list of common Ids
    for(int j=0;j< m_landmarks.size() ; j++){
    
      if( _z(i*singleObservationDim) == m_landmarks[j](0)){
      
        indx = j;
        break;
      }
    }
    
    double range = norm( m_landmarks[indx].subvec(1,2) - xVec.subvec(0,1) , 2);

    double thetaLandmark =  m_landmarks[indx][3]*PI ; 

    double viewingAngle = abs(acos(cos(thetaLandmark )*cos(xVec[2]) + sin(thetaLandmark )*sin(xVec[2])));
    
    //cout<<"The range to landmark is :"<<d<<endl;
    //cout<<"The bearing to landmark is :"<<landmark_bearing<<endl;
    if( viewingAngle > PI/2 ) viewingAngle = abs(viewingAngle-PI );
    
    noise.subvec(2*i, 2*i+1) = this->m_etaD*range + this->m_etaPhi*viewingAngle + this->m_sigma;

  }
  
  //square the factors to get the covariances
  noise = pow(noise,2);

  //return covariance matrix generated from vector of covariances
  return diagmat(noise);

}

/*
template <class MPTraits>
const typename CamAruco2DObservationModel<MPTraits>::ObsNoiseType
CamAruco2DObservationModel<MPTraits>::
GetZeroNoise() {
  using namespace arma;

  colvec zeroNoise = zeros<colvec>(singleLandmarkDim-1); // Have a look
  return zeroNoise;

}
*/

template <class MPTraits>
typename CamAruco2DObservationModel<MPTraits>::ObservationType
CamAruco2DObservationModel<MPTraits>::
ComputeInnovation
(CfgType& _xPrd, const ObservationType& _Zg) {
  using namespace arma;

  //return the discrepancy between the expected observation
  //for a predicted state and the actual observation generated

  //ObsNoiseType V = GetZeroNoise();
  //zeros<ObsNoiseType>(m_landmarks.size()*2); // have a look

  ObservationType Zprd = GetObservationPrediction(_xPrd, _Zg);
  
  if(Zprd.n_rows == 0){
    ObservationType innov;   
    return innov;
  }

  ObservationType innov( (landmarkInfoDim)* _Zg.n_rows /singleObservationDim ) ;
  
  assert( _Zg.n_rows == Zprd.n_rows); 

  for(int i =0; i< _Zg.n_rows/singleObservationDim ; i++){
   
    //cout<<" The current iteration in innovation is: "<<i<<endl;
    assert(_Zg(i*singleObservationDim) == Zprd(i*singleObservationDim)) ;
    
    innov( i*(landmarkInfoDim) ) = _Zg(i*singleObservationDim + 1) - Zprd(i*singleObservationDim + 1) ; 

    double delta_theta = _Zg(i*singleObservationDim + 2) - Zprd(i*singleObservationDim + 2) ; 

    if(delta_theta > PI ) {
      delta_theta = delta_theta - 2*PI ;
    }
    
    if( delta_theta < -PI ){
      delta_theta =  delta_theta + 2*PI ;
    }

    innov( i*(landmarkInfoDim) + 1 ) =  delta_theta;
    
  }
  return innov;
}

template <class MPTraits>
typename CamAruco2DObservationModel<MPTraits>::ObservationType
CamAruco2DObservationModel<MPTraits>::
RemoveSpuriousObservations(const ObservationType& _Zg){

  ObservationType Zcorrected;
  
  int counter  = 0;
  
  for(int j=0; j < m_landmarks.size() ; j++){
  
    for(int i=0; i < _Zg.n_rows / singleObservationDim ; i++) {
          
      if(_Zg(i*singleObservationDim) == m_landmarks[j](0)) {
        
        Zcorrected.resize(singleObservationDim*(counter +1));
      
        Zcorrected(singleObservationDim*counter) =    _Zg(i*singleObservationDim);
      
        Zcorrected(singleObservationDim*counter+1) =  _Zg(i*singleObservationDim+1);
        
        Zcorrected(singleObservationDim*counter+2) =  _Zg(i*singleObservationDim+2);
      
        counter++;
        break;
      }
    
    }
  
  }

  return Zcorrected;

}
/*
template <class MPTraits>
typename CamAruco2DObservationModel<MPTraits>::ObsNoiseType
CamAruco2DObservationModel<MPTraits>::
GenerateObservationNoise
(const CfgType& _x) {
  using namespace arma;
  //generate random noise for each element of the observation vector,
  //scaled and shifted according to eta and sigma, which determine the
  //weights of the range-proportional and independent components of noise

  //get standard deviations of noise (sqrt of covariance matrix)
  mat noiseStdDev = sqrt(GetObservationNoiseCovariance(_x));

  //generate raw noise
  colvec randNoiseVec = randn<colvec>(m_landmarks.size()*2);
  
  //generate noise from a distribution scaled and shifted from
  //normal distribution N(0,1) to N(0,eta*range + sigma)
  //(shifting was done in GetNoiseCovariance)
  colvec noise = noiseStdDev*randNoiseVec;
  return noise;

}
*/