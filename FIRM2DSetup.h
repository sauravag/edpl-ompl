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

/* Author: Saurav Agarwal */

#ifndef FIRM_2D_SETUP_H
#define FIRM_2D_SETUP_H

#include <omplapp/geometry/RigidBodyGeometry.h>
#include "include/Planner/FIRM.h"
//#include "include/Spaces/SE2BeliefSpace.h"
#include "FIRMOMPL.h"


/** \brief Wrapper for ompl::app::RigidBodyPlanning that plans for rigid bodies in SE2BeliefSpace using FIRM */
class FIRM2DSetup : public ompl::app::RigidBodyGeometry
{

    typedef SE2BeliefSpace::StateType StateType;

public:

    FIRM2DSetup() : ompl::app::RigidBodyGeometry(ompl::app::Motion_2D),
    ss_(ompl::base::StateSpacePtr(new SE2BeliefSpace()))
    {
        // setting the mean and norm weights (used in reachability check)
        StateType::covNormWeight_  =  1.0;
        StateType::meanNormWeight_ =  2.0;
        StateType::reachDist_ =  0.1;

        // set the state component norm weights
        arma::colvec normWeights(3);
        normWeights(0) = 2.0/3.0;
        normWeights(1) = 2.0/3.0;
        normWeights(2) = 1.0/3.0;
        StateType::normWeights_ = normWeights;

        // The bounds should be inferred from the geometry files,
        // there is a function in Apputils to do this, so use that.
        // set the bounds for the R^3 part of SE(3)
        ompl::base::RealVectorBounds bounds(2);
        // set X bound
        bounds.setLow(0,0.0);
        bounds.setHigh(0,20.0);
        //set Y bound
        bounds.setLow(1,0.0);
        bounds.setHigh(1,20.0);
        ss_->as<SE2BeliefSpace>()->setBounds(bounds);

        //Construct the control space
        ompl::control::ControlSpacePtr controlspace( new ompl::control::RealVectorControlSpace(ss_,2) ) ;
        cs_ = controlspace;

        // construct an instance of space information from this state space
        firm::SpaceInformation::SpaceInformationPtr si(new firm::SpaceInformation(ss_, cs_));
        siF_ = si;

        ompl::base::ProblemDefinitionPtr prblm(new ompl::base::ProblemDefinition(siF_));

        pdef_ = prblm;

        ompl::base::PlannerPtr plnr(new FIRM(siF_, false));

        planner_ = plnr;

        start_ = siF_->allocState();
        goal_  = siF_->allocState();

        setup_ = false;
    }

    virtual ~FIRM2DSetup(void)
    {
    }

    const ompl::base::SpaceInformationPtr& getSpaceInformation() const
    {
        return siF_;
    }

    void setPathToSetupFile(const std::string &path)
    {
        pathToSetupFile_  = path;
    }

    void setStartState(const double X, const double Y, const double Yaw)
    {
        ompl::base::State *temp = siF_->allocState();

        temp->as<StateType>()->setXYYaw(X,Y,Yaw);

        siF_->copyState(start_, temp);

        siF_->freeState(temp);

    }

    void setGoalState(const double X, const double Y, const double Yaw)
    {
        ompl::base::State *temp = siF_->allocState();

        temp->as<StateType>()->setXYYaw(X,Y,Yaw);

        siF_->copyState(goal_,temp);

        siF_->freeState(temp);

    }


    void setStartAndGoalStates()
    {
        pdef_->setStartAndGoalStates(start_, goal_, 1.0);
    }

    void setup()
    {
        if(pathToSetupFile_.length() == 0)
        {
            throw ompl::Exception("Path to setup file not set!");
        }

        // Create an FCL state validity checker and assign to space information
        //const ompl::base::StateValidityCheckerPtr &fclSVC = allocStateValidityChecker(siF_, getGeometricStateExtractor(), false);
        //siF_->setStateValidityChecker(fclSVC);

        siF_->setStateValidityChecker(ompl::base::StateValidityCheckerPtr(new FIRMValidityChecker(siF_)));

        ompl::control::StatePropagatorPtr prop(ompl::control::StatePropagatorPtr(new UnicycleStatePropagator(siF_)));

        statePropagator_ = prop;

        siF_->setStatePropagator(prop);
        siF_->setPropagationStepSize(0.1); // this is the duration that a control is applied
        siF_->setMinMaxControlDuration(1,1000);

        // provide the observation model to the space
        ObservationModelMethod::ObservationModelPointer om(new CamAruco2DObservationModel(pathToSetupFile_.c_str()));
        siF_->setObservationModel(om);

        // Provide the motion model to the space
        MotionModelMethod::MotionModelPointer mm(new UnicycleMotionModel(siF_, pathToSetupFile_.c_str()));
        siF_->setMotionModel(mm);

        if(!start_ || !goal_)
        {
            throw ompl::Exception("Start/Goal not set");
        }

        this->setStartAndGoalStates();

        planner_->setProblemDefinition(pdef_);

        planner_->setup();

        setup_ = true;

    }

    ompl::base::PlannerStatus solve(const double solveTime)
    {
        if(!setup_)
        {
            this->setup();
        }

        return planner_->solve(solveTime);
    }

    void executeSolution()
    {
        planner_->as<FIRM>()->executeFeedback();
    }


    ompl::app::GeometricStateExtractor getGeometricStateExtractor(void) const
    {
        return boost::bind(&FIRM2DSetup::getGeometricComponentStateInternal, this, _1, _2);
    }

protected:

    const ompl::base::State* getGeometricComponentStateInternal(const ompl::base::State *state, unsigned int /*index*/) const
    {
        return state;
    }

private:

    ompl::base::State *start_;

    ompl::base::State *goal_;

    firm::SpaceInformation::SpaceInformationPtr siF_;

    ompl::control::StatePropagatorPtr statePropagator_;

    ompl::control::ControlSpacePtr cs_;

    ompl::base::StateSpacePtr ss_;

    ompl::base::PlannerPtr planner_;

    ompl::base::ProblemDefinitionPtr pdef_;

    std::string pathToSetupFile_;

    bool setup_;
};
#endif
