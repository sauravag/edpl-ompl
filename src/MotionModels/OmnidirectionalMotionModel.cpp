/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, Texas A&M University
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
*   * Neither the name of the Texas A&M University nor the names of its
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

/* Authors:  Saurav Agarwal, Ali-akbar Agha-mohammadi */


#include <tinyxml.h>
#include "Spaces/SE2BeliefSpace.h"
#include "MotionModels/OmnidirectionalMotionModel.h"
#include "Utils/FIRMUtils.h"
#include<cassert>

//Produce the next state, given the current state, a control and a noise
void OmnidirectionalMotionModel::Evolve(const ompl::base::State *state, const ompl::control::Control *control, const NoiseType& w, ompl::base::State *result)
{

    using namespace arma;

    typedef typename MotionModelMethod::StateType StateType;

    arma::colvec u = OMPL2ARMA(control);

    const colvec& Un = w.subvec(0, this->controlDim_-1);

    const colvec& Wg = w.subvec(this->controlDim_, this->noiseDim_-1);

    colvec x = state->as<StateType>()->getArmaData();

    colvec Un2(3);
    Un2 << Un[0] << Un[1] << Un[2] << endr;

    x += (u*this->dt_) + (Un2*sqrt(this->dt_)) + (Wg*sqrt(this->dt_));

    FIRMUtils::normalizeAngleToPiRange(x[2]);

    result->as<StateType>()->setXYYaw(x[0],x[1],x[2]);
}


void OmnidirectionalMotionModel::generateOpenLoopControls(const ompl::base::State *startState,
                                                  const ompl::base::State *endState,
                                                  std::vector<ompl::control::Control*> &openLoopControls)
{

    using namespace arma;
    typedef typename MotionModelMethod::StateType StateType;

    colvec start = startState->as<StateType>()->getArmaData(); // turn into colvec (in radian)
    colvec target = endState->as<StateType>()->getArmaData(); // turn into colvec (in radian)

    colvec x_c, y_c;

    x_c << start[0] << endr
      << target[0] << endr;

    y_c << start[1] << endr
      << target[1] << endr;

    double theta_p = 0;

    double delta_theta = 0;

    double delta_disp = 0;

    double translation_steps = 0;

    double theta_start = start[2];

    double theta_target = target[2];

    // connecting line slope
    theta_p = atan2(y_c[1]-y_c[0], x_c[1]-x_c[0]);

    // turining anlge from start to target
    delta_theta = theta_target - theta_start;

    // bringing the delta_th_p_start to the -PI to PI range
    FIRMUtils::normalizeAngleToPiRange(delta_theta);

    //count translation steps
    delta_disp = sqrt( pow(y_c[1]-y_c[0], 2) + pow(x_c[1]-x_c[0], 2) );

    translation_steps = ceil(fabs(delta_disp/(maxLinearVelocity_*this->dt_)));

    // If finite number of translation steps required
    if(translation_steps > 0)
    {

        // required angular velocity to orient to target state in calculated number of steps
        double omega = delta_theta / (translation_steps*this->dt_)  ;

        // checking to see if required angular velocity is within limits
        //assert(abs(omega) < maxAngularVelocity_);

        colvec u_const_rot;
        u_const_rot << maxLinearVelocity_*cos(theta_p) << endr
                    << maxLinearVelocity_*sin(theta_p) << endr
                    << omega << endr;

        for(int i=0; i<translation_steps; i++)
        {
          ompl::control::Control *tempControl = si_->allocControl();
          ARMA2OMPL(u_const_rot, tempControl);
          openLoopControls.push_back(tempControl);
        }
    }
    else // If only change in heading required
    {
        // required angular velocity to orient to target state in calculated number of steps
        int rotation_steps = ceil(abs(delta_theta / maxAngularVelocity_* this->dt_));

        if(rotation_steps> 0)
        {
            double omega =  maxAngularVelocity_*(delta_theta/abs(delta_theta));

            // checking to see if required angular velocity is within limits
            //assert(abs(omega) < maxAngularVelocity_);

            colvec u_const_rot;
            u_const_rot << 0 << endr
                        << 0 << endr
                        << omega << endr;

            for(int i=0; i<rotation_steps; i++)
            {
              ompl::control::Control *tempControl = si_->allocControl();
              ARMA2OMPL(u_const_rot, tempControl);
              openLoopControls.push_back(tempControl);
            }
        }
        else
        {
            openLoopControls.push_back(this->getZeroControl());
        }

    }

}

void OmnidirectionalMotionModel::generateOpenLoopControlsForPath(const ompl::geometric::PathGeometric path, std::vector<ompl::control::Control*> &openLoopControls)
{
    for(int i=0;i<path.getStateCount()-1;i++)
    {
        std::vector<ompl::control::Control*> olc;

        this->generateOpenLoopControls(path.getState(i),path.getState(i+1),olc) ;

        openLoopControls.insert(openLoopControls.end(),olc.begin(),olc.end());
    }
}


typename OmnidirectionalMotionModel::NoiseType
OmnidirectionalMotionModel::generateNoise(const ompl::base::State *state, const ompl::control::Control* control)
{

    using namespace arma;

    NoiseType noise(this->noiseDim_);
    colvec indepUn = randn(this->controlDim_,1);
    mat P_Un = controlNoiseCovariance(control);
    colvec Un = indepUn % sqrt((P_Un.diag()));

    colvec Wg = sqrt(P_Wg_) * randn(this->stateDim_,1);
    noise = join_cols(Un, Wg);

    return noise;
}

typename OmnidirectionalMotionModel::JacobianType
OmnidirectionalMotionModel::getStateJacobian(const ompl::base::State *state, const ompl::control::Control* control, const NoiseType& w)
{

    using namespace arma;

    typedef typename MotionModelMethod::StateType StateType;

    arma::colvec u = OMPL2ARMA(control);

    colvec xData = state->as<StateType>()->getArmaData();

    assert (xData.n_rows == (size_t)stateDim);

    const colvec& Un = w.subvec(0,this->controlDim_-1);

    JacobianType A = eye(this->stateDim_,this->stateDim_) ;

    return A;
}

typename OmnidirectionalMotionModel::JacobianType
OmnidirectionalMotionModel::getControlJacobian(const ompl::base::State *state, const ompl::control::Control* control, const NoiseType& w)
{

    using namespace arma;
    typedef typename MotionModelMethod::StateType StateType;

    colvec xData = state->as<StateType>()->getArmaData();
    assert (xData.n_rows == (size_t)this->stateDim_);

    const double& theta = xData[2];

    mat B(3,3);

    B <<  1 <<   0  <<  0   <<  endr
      <<  0  <<  1  <<  0   << endr
      <<  0  <<  0  <<  1   <<  endr;

    B *= this->dt_;

    return B;

}


typename OmnidirectionalMotionModel::JacobianType
OmnidirectionalMotionModel::getNoiseJacobian(const ompl::base::State *state, const ompl::control::Control* control, const NoiseType& w)
{

    using namespace arma;
    typedef typename MotionModelMethod::StateType StateType;

    colvec xData = state->as<StateType>()->getArmaData();

    assert (xData.n_rows == (size_t)this->stateDim_);

    const double& theta = xData[2];

    mat G(3,6);


    G   << 1 << 0 << 0 << 1 << 0 << 0 << endr
        << 0 << 1 << 0 << 0 << 1 << 0 << endr
        << 0  << 0 << 1 << 0 << 0 << 1 << endr;


    G *= sqrt(this->dt_);
    return G;

}

arma::mat OmnidirectionalMotionModel::processNoiseCovariance(const ompl::base::State *state, const ompl::control::Control* control)
{

    using namespace arma;

    mat P_Un = controlNoiseCovariance(control);
    mat Q_processNoise = zeros<mat>(P_Un.n_rows + P_Wg_.n_rows, P_Un.n_cols +
    P_Wg_.n_cols);

    Q_processNoise.submat(0, 0, P_Un.n_rows-1, P_Un.n_cols-1) = P_Un;
    Q_processNoise.submat(P_Un.n_rows, P_Un.n_cols,
                  P_Un.n_rows + P_Wg_.n_rows -1,
                  P_Un.n_cols + P_Wg_.n_cols -1) = P_Wg_;

    return Q_processNoise;

}


arma::mat OmnidirectionalMotionModel::controlNoiseCovariance(const ompl::control::Control* control)
{

    using namespace arma;

    arma::colvec u = OMPL2ARMA(control);

    colvec uStd = eta_ % u + sigma_;

    mat P_Un = diagmat(square(uStd));

    return P_Un;
}

void OmnidirectionalMotionModel::loadParameters(const char *pathToSetupFile)
{
    using namespace arma;

    TiXmlDocument doc(pathToSetupFile);
    bool loadOkay = doc.LoadFile();

    if ( !loadOkay )
    {
        printf( "Could not load setup file in motion model. Error='%s'. Exiting.\n", doc.ErrorDesc() );

        exit( 1 );
    }

    TiXmlNode* node = 0;
    TiXmlElement* itemElement = 0;

    node = doc.FirstChild( "MotionModels" );
    assert( node );

    TiXmlNode* child = 0;

    child = node->FirstChild("OmnidirectionalMotionModel");

    assert( child );
    itemElement = child->ToElement();
    assert( itemElement );

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

    this->sigma_ << sigmaV << sigmaV << sigmaOmega <<endr;
    this->eta_  << etaV << etaV << etaOmega << endr;

    rowvec Wg_root_vec(3);
    Wg_root_vec << windNoisePos << windNoisePos << windNoiseAng*boost::math::constants::pi<double>() / 180.0 << endr;
    P_Wg_ = diagmat(square(Wg_root_vec));

    minLinearVelocity_  = minLinearVelocity;
    maxLinearVelocity_  = maxLinearVelocity;
    maxAngularVelocity_ = maxAngularVelocity;
    dt_                 = dt;

    OMPL_INFORM("OmnidirectionalMotionModel: sigma_ = ");
    std::cout<<sigma_<<std::endl;

    OMPL_INFORM("OmnidirectionalMotionModel: eta_ = ");
    std::cout<<eta_<<std::endl;

    OMPL_INFORM("OmnidirectionalMotionModel: P_Wg_ = ");
    std::cout<<P_Wg_<<std::endl;

    OMPL_INFORM("OmnidirectionalMotionModel: min Linear Velocity (m/s)    = %f", minLinearVelocity_ );

    OMPL_INFORM("OmnidirectionalMotionModel: max Linear Velocity (m/s)    = %f", maxLinearVelocity_);

    OMPL_INFORM("OmnidirectionalMotionModel: max Angular Velocity (rad/s) = %f",maxAngularVelocity_);

    OMPL_INFORM("OmnidirectionalMotionModel: Timestep (seconds) = %f", dt_);

}
