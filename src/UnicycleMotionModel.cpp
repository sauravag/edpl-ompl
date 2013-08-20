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
#include "../include/Spaces/SE2BeliefSpace.h"
#include "../include/MotionModels/UnicycleMotionModel.h"
#include "../tinyxml/tinyxml.h"

inline void pirange(double& d)
{
  while(d > boost::math::constants::pi<double>() )
    d -= 2*boost::math::constants::pi<double>() ;
  while(d < -boost::math::constants::pi<double>() )
    d += 2*boost::math::constants::pi<double>() ;
}

inline int signum(double& d)
{
  if(d>0) return 1;
  if(d<0) return -1;
}

//Produce the next state, given the current state, a control and a noise
ompl::base::State* UnicycleMotionModel::Evolve(const ompl::base::State *state, const ControlType& u, const NoiseType& w)
{

  using namespace arma;
  typedef typename MotionModelMethod::StateType StateType;
  //cout << "Evolve" << endl;
  typedef std::vector<double> stdvec;
  //Assume the cfg type is compatible with this motion model (this should have been checked in the constructor)

  //generate a column vector (arma::colvec) corresponding to the next state
  //and return a Cfg initialized with it
  //cout << "noise vector: " << endl << w << endl;
  //cout << "getting control noise " << endl;
  //cout << "controlDim_ = " << this->controlDim_ << endl;
  const colvec& Un = w.subvec(0, this->controlDim_-1);
  //cout << "getting process noise " << endl;
  //cout << "noiseDim_ = " << this->noiseDim_ << endl;
  const colvec& Wg = w.subvec(this->controlDim_, this->noiseDim_-1);

  colvec x = state->as<StateType>()->getArmaData();

  const double c = cos(x[2]);
  const double s = sin(x[2]);

  colvec u2(3);
  u2 << u[0]*c << u[0]*s << u[1] << endr;
  colvec Un2(3);
  Un2 << Un[0]*c << Un[0]*s << Un[1] << endr;

  x += (u2*this->dt_) + (Un2*sqrt(this->dt_)) + (Wg*sqrt(this->dt_));

  SpaceType *space;
  space =  new SpaceType();
  ompl::base::State *nextState = space->allocState();

  nextState->as<StateType>()->setXYYaw(x[0],x[1],x[2]);

  return nextState;
}



//Generate open loop control between two specified Cfgs/states
std::vector<typename UnicycleMotionModel::ControlType>
UnicycleMotionModel::generateOpenLoopControls(const ompl::base::State *startState,
                                                  const ompl::base::State *endState)
{

  using namespace arma;
  typedef typename MotionModelMethod::StateType StateType;
  //cout << "starting generate open-loop control" << endl;
  colvec start = startState->as<StateType>()->getArmaData(); // turn into colvec (in radian)
  colvec end = endState->as<StateType>()->getArmaData(); // turn into colvec (in radian)

  colvec x_c, y_c;
  x_c << start[0] << endr
      << end[0] << endr;
  y_c << start[1] << endr
      << end[1] << endr;

  //TODO: init all these to zero
  double th_p = 0;
  double delta_th_p_start = 0;
  double delta_th_p_end = 0;
  double delta_disp = 0;
  double translation_steps = 0;
  double rotation_steps_start = 0;
  double rotation_steps_end = 0;
  int kf = 0;

  double theta_start = start[2];
  double th_end = end[2];

    // connceting line slope
    th_p = atan2(y_c[1]-y_c[0], x_c[1]-x_c[0]);

    // turining anlge at start
    delta_th_p_start = th_p - theta_start;
    pirange(delta_th_p_start); // bringing the delta_th_p_start to the -PI to PI range
   // turining anlge at end
    delta_th_p_end = th_end - th_p;
    pirange(delta_th_p_end); // bringing the delta_th_p_end to the -PI to PI range

    //cout << "theta_start: " << theta_start*180/PI << endl;
    //cout << "th_line: " << th_p*180/PI << endl;
    //cout << "delta_th_p_start: " << delta_th_p_start*180/PI << endl;

    //count rotational steps
    rotation_steps_start = fabs(delta_th_p_start/(maxAngularVelocity_*this->dt_));
    rotation_steps_end = fabs(delta_th_p_end/(maxAngularVelocity_*this->dt_));

    //count translation steps
    delta_disp = sqrt( pow(y_c[1]-y_c[0], 2) + pow(x_c[1]-x_c[0], 2) );
    translation_steps = fabs(delta_disp/(maxLinearVelocity_*this->dt_));

    // total number of steps
    kf += ceil(rotation_steps_start) + ceil(translation_steps) + ceil(rotation_steps_end);

   //cout << "kf: " << kf << endl;

    std::vector<ControlType> controlSeq(kf);

    /////////////////////////Generating control signal for start rotation
    const double& rsi = rotation_steps_start;
    const double frsi = floor(rsi);
    colvec u_const_rot;
    u_const_rot << 0 << endr
                << maxAngularVelocity_*signum(delta_th_p_start) << endr;
    int ix = 0;
    for(int j=0; j<frsi; ++j, ++ix)
      controlSeq[ix] = u_const_rot;
    if(frsi < rsi)
      controlSeq[ix++] = u_const_rot*(rsi-frsi);

    /////////////////////////Generating control signal for translation
    const double& tsi = translation_steps;
    const double ftsi = floor(tsi);
    static colvec u_const_trans;
    if(u_const_trans.n_rows == 0) {
      u_const_trans << maxLinearVelocity_ << endr
                    << 0        << endr;
    }
    for(int j=0; j<ftsi; ++j, ++ix)
      controlSeq[ix] = u_const_trans;
    if(ftsi < tsi)
      controlSeq[ix++] = u_const_trans*(tsi-ftsi);

    /////////////////////////Generating control signal for end rotation

    const double& rsi_end = rotation_steps_end;
    const double frsi_end = floor(rsi_end);
    colvec u_const_rot_end;
    u_const_rot_end << 0 << endr
                << maxAngularVelocity_*signum(delta_th_p_end) << endr;

    for(int j=0; j<frsi_end; ++j, ++ix)
      controlSeq[ix] = u_const_rot_end;
    if(frsi_end < rsi_end)
      controlSeq[ix++] = u_const_rot_end*(rsi_end-frsi_end);


  assert(ix == kf);

  return controlSeq;
}

//Generate noise according to specified state and control input
typename UnicycleMotionModel::NoiseType
UnicycleMotionModel::generateNoise(const ompl::base::State *state, const ControlType& u)
{

  using namespace arma;
  //TODO: confirm that this is the correct noise generation method
  NoiseType noise(this->noiseDim_);
  colvec indepUn = randn(this->controlDim_,1);
  mat P_Un = controlNoiseCovariance(u);
  colvec Un = indepUn % sqrt((P_Un.diag()));

  colvec Wg = sqrt(P_Wg_) * randn(this->stateDim_,1);
  noise = join_cols(Un, Wg);
  //cout << "noise" << noise << endl;
  return noise;
}

// df/dx
typename UnicycleMotionModel::JacobianType
UnicycleMotionModel::getStateJacobian(const ompl::base::State *state, const ControlType& u, const NoiseType& w)
{

  using namespace arma;

  typedef typename MotionModelMethod::StateType StateType;

  //assert(!"UnicycleMotionModel<MPTraits>::GetStateJacobian not yet implemented!");

  colvec xData = state->as<StateType>()->getArmaData();

  assert (xData.n_rows == (size_t)stateDim);

  const colvec& Un = w.subvec(0,this->controlDim_-1);
  double c = cos(xData[2]);
  double s = sin(xData[2]);

  mat uMat(3,3), UnMat(3,3);
  uMat  <<  0   << 0 <<   -u[0]*s << endr
        <<  0   << 0 <<    u[0]*c << endr
        <<  0   << 0 <<       0    << endr;

  UnMat <<  0   << 0 <<   -Un[0]*s << endr
        <<  0   << 0 <<    Un[0]*c << endr
        <<  0   << 0 <<       0    << endr;

  JacobianType A = eye(this->stateDim_,this->stateDim_) + uMat*this->dt_ + UnMat*sqrt(this->dt_);
  return A;
}

// df/du
typename UnicycleMotionModel::JacobianType
UnicycleMotionModel::getControlJacobian(const ompl::base::State *state, const ControlType& u, const NoiseType& w)
{

  using namespace arma;
  typedef typename MotionModelMethod::StateType StateType;

  colvec xData = state->as<StateType>()->getArmaData();
  assert (xData.n_rows == (size_t)this->stateDim_);

  const double& theta = xData[2];

  mat B(3,2);

  B   <<  cos(theta)  <<  0   <<  endr
      <<  sin(theta)  <<  0   <<  endr
      <<          0   <<  1   <<  endr;

  B *= this->dt_;
  return B;

}

// df/dw
typename UnicycleMotionModel::JacobianType
UnicycleMotionModel::getNoiseJacobian(const ompl::base::State *state, const ControlType& u, const NoiseType& w)
{

  using namespace arma;
  typedef typename MotionModelMethod::StateType StateType;
  //assert(!"UnicycleMotionModel<MPTraits>::GetNoiseJacobian not yet implemented!");
  colvec xData = state->as<StateType>()->getArmaData();
  assert (xData.n_rows == (size_t)this->stateDim_);

  const double& theta = xData[2];

  /*
  th=x(3);
  G = [cos(th) , 0 , 1 , 0 , 0 ;  sin(th) , 0 , 0 ,1,0 ;  0 , 1 , 0 ,0,1] * sqrt(Unicycle_robot.dt);
  */

  mat G(3,5);

  G   <<  cos(theta) << 0 << 1 << 0 << 0 << endr
      <<  sin(theta) << 0 << 0 << 1 << 0 << endr
      <<          0  << 1 << 0 << 0 << 1 << endr;

  G *= sqrt(this->dt_);
  return G;

}

// Q matrix
arma::mat UnicycleMotionModel::processNoiseCovariance(const ompl::base::State *state, const ControlType& u)
{

  using namespace arma;

  //assert(!"UnicycleMotionModel<MPTraits>::ProcessNoiseCovariance not yet implemented!");

  /*
  P_Un = control_noise_covariance(u);
  Q_process_noise = blkdiag(P_Un,Unicycle_robot.P_Wg);
  ?? What is P_Wg?
  */

  mat P_Un = controlNoiseCovariance(u);
  mat Q_processNoise = zeros<mat>(P_Un.n_rows + P_Wg_.n_rows, P_Un.n_cols +
  P_Wg_.n_cols);

  Q_processNoise.submat(0, 0, P_Un.n_rows-1, P_Un.n_cols-1) = P_Un;
  Q_processNoise.submat(P_Un.n_rows, P_Un.n_cols,
                  P_Un.n_rows + P_Wg_.n_rows -1,
                  P_Un.n_cols + P_Wg_.n_cols -1) = P_Wg_;

  return Q_processNoise;

}


arma::mat
UnicycleMotionModel::controlNoiseCovariance(const ControlType& u)
{

  using namespace arma;

  /*
    u_std=(Unicycle_robot.eta_u).*U+(Unicycle_robot.sigma_b_u);
    P_Un=diag(u_std.^2);
  */
  colvec uStd = eta_ % u + sigma_;
  mat P_Un = diagmat(square(uStd));
  return P_Un;
}


void UnicycleMotionModel::loadParameters(const char *pathToSetupFile)
{
  using namespace arma;
  // Load XML containing landmarks
  TiXmlDocument doc(pathToSetupFile);
  bool loadOkay = doc.LoadFile();

  if ( !loadOkay )
  {
    printf( "Could not load setup file . Error='%s'. Exiting.\n", doc.ErrorDesc() );

    exit( 1 );
  }

  TiXmlNode* node = 0;
  TiXmlElement* obsModelElement = 0;
  TiXmlElement* itemElement = 0;

  // Get the landmarklist node
  node = doc.FirstChild( "MotionModels" );
  assert( node );
  //obsModelElement  = node->ToElement(); //convert node to element
  //assert( obsModelElement   );

  TiXmlNode* child = 0;

  child = node->FirstChild("UnicycleMotionModel");
  //Iterate through all the landmarks and put them into the "landmarks_" list
  assert( child );
  itemElement = child->ToElement();
  assert( itemElement );

  /*
  rma::colvec sigma_; // Bias standard deviation of the motion noise
  arma::colvec eta_; // Proportional standard deviation of the motion noise
  arma::mat    P_Wg_; // Covariance of state additive noise

  double maxAngularVelocity_; //max rotational velocity
  double maxLinearVelocity_; //max translational velocity
  double orbitRadius_;
  double minLinearVelocity_;
  */

  double sigmaV=0;
  double etaV = 0;
  double sigmaOmega=0;
  double etaOmega=0;
  double windNoisePos=0;
  double windNoiseAng = 0;
  double minLinearVelocity=0;
  double maxLinearVelocity=0;
  double maxAngularVelocity =0;
  double dt = 0;

  itemElement->QueryDoubleAttribute("sigmaV", &sigmaV) ;
  itemElement->QueryDoubleAttribute("etaV", &etaV) ;
  itemElement->QueryDoubleAttribute("sigmaOmega", &sigmaOmega) ;
  itemElement->QueryDoubleAttribute("etaOmega", &etaOmega) ;
  itemElement->QueryDoubleAttribute("wind_noise_pos", &windNoisePos) ;
  itemElement->QueryDoubleAttribute("wind_noise_ang", &windNoiseAng) ;
  itemElement->QueryDoubleAttribute("min_linear_velocity", &minLinearVelocity) ;
  itemElement->QueryDoubleAttribute("max_linear_velocity", &maxLinearVelocity) ;
  itemElement->QueryDoubleAttribute("max_angular_velocity", &maxAngularVelocity) ;
  itemElement->QueryDoubleAttribute("dt", &dt) ;

  this->sigma_ << sigmaV << sigmaOmega <<endr;
  this->eta_  << etaV << etaOmega << endr;

  rowvec Wg_root_vec(3);
  Wg_root_vec << windNoisePos << windNoisePos << windNoiseAng*boost::math::constants::pi<double>() / 180.0 << endr;
  P_Wg_ = diagmat(square(Wg_root_vec));

  minLinearVelocity_  = minLinearVelocity;
  maxLinearVelocity_  = maxLinearVelocity;
  maxAngularVelocity_ = maxAngularVelocity;
  dt_                 = dt;

  std::cout<<"Motion Model parameters loaded"<<std::endl;
  std::cout<<"sigma_  : "<<sigma_<<std::endl;
  std::cout<<"eta_   :"<<eta_<<std::endl;
  std::cout<<"P_Wg_ :"<<P_Wg_<<std::endl;
  std::cout<<"minLinearVelocity_ :"<<minLinearVelocity_<<std::endl;
  std::cout<<"maxLinearVelocity_ :"<<maxLinearVelocity_<<std::endl;
  std::cout<<"maxAngularVelocity_ :"<<maxAngularVelocity_<<std::endl;
  std::cout<<"dt_ :"<<dt_<<std::endl;
  std::cout<<"Motion Model parameters finished"<<std::endl;

}
/*
bool UnicycleMotionModel::GenerateOrbit(const CfgType& _start,
                        vector<CfgType>& _intermediates,
                        vector<ControlType>& _controls)
{

  using namespace arma;

  typedef vector<double>::iterator DIT;

  //make sure the vectors being passed in to store controls and
  //intermediate configurations do not already contain data
  assert(_intermediates.size() == 0);
  assert(_controls.size() == 0);

  //calculate the number of steps T in the orbit based on the orbit
  //circumference, the agent's linear/angular velocities, and the
  //timestep resolution
  double orbitLength = 2*PI*m_orbitRadius;
  double orbitTime = orbitLength / m_minLinearVelocity;
  double T_rational = orbitTime / this->dt_;
  size_t T = ceil(T_rational);

  //generate the linear and angular controls at each step
  //want linear control to be minimum and angular control to be
  //maximum in order to produce the tightest possible orbit

  //generate linear controls
  vector<double> vLin(T,1.0);
  vLin.back() = T_rational - floor(T_rational);
  for(DIT d = vLin.begin(); d != vLin.end(); ++d) {
    *d *= m_minLinearVelocity;
  }

  //generate angular controls
  vector<double> omegas (T, 1.0);
  omegas.back() = T_rational - floor(T_rational);
  for(DIT d = omegas.begin(); d != omegas.end(); ++d) {
    *d = maxAngularVelocity_;
  }

  //TODO: wtf is going on here??? sizes are set to T... and vLin/omegas.back() are being overwritten...
  assert(T == vLin.size());
  assert(T == omegas.size());

  //concatenate linear and angular controls
  for(unsigned i = 0; i < T; ++i) {
    UnicycleMotionModel<MPTraits>::ControlType u(2);
    u(0) = vLin[i];
    u(1) = omegas[i];
    _controls.push_back(u);
  }

        typedef typename MPTraits::MPProblemType MPProblemType;
  typedef typename MPProblemType::ValidityCheckerPointer ValidityCheckerPointer;

        //generate configurations along orbit to check if it is valid

        Environment* env = this->GetMPProblem()->GetEnvironment();
        ValidityCheckerPointer vc = this->GetMPProblem()->GetValidityChecker(m_vcMethod);

        StatClass* stats = this->GetMPProblem()->GetStatClass();
        string callee(this->GetName() + "::SampleImpl()");
  CDInfo cdInfo;

  NoiseType wZero = this->GetZeroNoise();
  _intermediates.push_back(_start);
  for(unsigned i = 0; i < T; ++i) {
    CfgType& prev = _intermediates.back();
    CfgType next = Evolve(prev, _controls[i], wZero);
    //if any configuration along the orbit is not valid, discard this orbit
    //by discarding the intermediate configurations and controls
    //and return false

    if(!next.InBoundary(env) || !vc->IsValid(next, env, *stats, cdInfo, &callee)) {
      _controls.clear();
      _intermediates.clear();
      return false;
    }
    _intermediates.push_back(next);
  }
  _intermediates.pop_back();  // We do not want to add the last point to the orbit. So, we pop it out.
  return true;
}
*/
