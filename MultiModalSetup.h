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

/* Author: Saurav Agarwal, Ali-akbar Agha-mohammadi */

#ifndef MULTI_MODAL_SETUP_H
#define MULTI_MODAL_SETUP_H

#include <omplapp/geometry/RigidBodyGeometry.h>
#include "include/Planner/FIRM.h"
#include "FIRMOMPL.h"

#include <tinyxml.h>

namespace ompl
{
    namespace magic
    {
        static const unsigned int MAX_MM_POLICY_LENGTH   = 1000;

        static const float MIN_ROBOT_CLEARANCE = 0.10;

        static const unsigned int MIN_STEPS_AFTER_CLEARANCE_VIOLATION_REPLANNING = 100;

    }
}


/** \brief Wrapper for ompl::app::RigidBodyPlanning that plans for rigid bodies in SE2BeliefSpace using FIRM */
class MultiModalSetup : public ompl::app::RigidBodyGeometry
{

    typedef SE2BeliefSpace::StateType StateType;

public:

    MultiModalSetup() : ompl::app::RigidBodyGeometry(ompl::app::Motion_2D),
    ss_(ompl::base::StateSpacePtr(new SE2BeliefSpace()))
    {
        // set static variables
        RHCICreate::setControlQueueSize(10);
        RHCICreate::setTurnOnlyDistance(0.05);
        Controller<RHCICreate, ExtendedKF>::setNodeReachedAngle(30); // degrees
        Controller<RHCICreate, ExtendedKF>::setNodeReachedDistance(0.20);// meters
        Controller<RHCICreate, ExtendedKF>::setMaxTries(200);
        Controller<RHCICreate, ExtendedKF>::setMaxTrajectoryDeviation(4.0); // meters

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

        ompl::base::RealVectorBounds bounds(2);
        // set X bound
        bounds.setLow(0,10);
        bounds.setHigh(0,10);
        //set Y bound
        bounds.setLow(1,10);
        bounds.setHigh(1,10);
        ss_->as<SE2BeliefSpace>()->setBounds(bounds);

        //Construct the control space
        ompl::control::ControlSpacePtr controlspace( new ompl::control::RealVectorControlSpace(ss_,2) ) ;
        cs_ = controlspace;

        // construct an instance of space information from this state space
        firm::SpaceInformation::SpaceInformationPtr si(new firm::SpaceInformation(ss_, cs_));
        siF_ = si;

        ompl::base::ProblemDefinitionPtr prblm(new ompl::base::ProblemDefinition(siF_));

        pdef_ = prblm;

        start_ = siF_->allocState();
        goal_  = siF_->allocState();

        setup_ = false;
    }

    virtual ~MultiModalSetup(void)
    {
        delete policyGenerator_;
    }

    const firm::SpaceInformation::SpaceInformationPtr& getSpaceInformation() const
    {
        return siF_;
    }

    void setPathToSetupFile(const std::string &path)
    {
        pathToSetupFile_  = path;

        this->loadParameters();
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
        if(!setup_)
        {
            if(pathToSetupFile_.length() == 0)
            {
                throw ompl::Exception("Path to setup file not set!");
            }

            if(!hasEnvironment() || !hasRobot())
            {
                throw ompl::Exception("Robot/Environment mesh files not setup!");
            }

            ss_->as<SE2BeliefSpace>()->setBounds(inferEnvironmentBounds());

            // Create an FCL state validity checker and assign to space information
            const ompl::base::StateValidityCheckerPtr &fclSVC = this->allocStateValidityChecker(siF_, getGeometricStateExtractor(), false);
            siF_->setStateValidityChecker(fclSVC);

            siF_->setStateValidityCheckingResolution(0.005);

            // provide the observation model to the space
            ObservationModelMethod::ObservationModelPointer om(new CamAruco2DObservationModel(siF_, pathToSetupFile_.c_str()));
            siF_->setObservationModel(om);

            // Provide the motion model to the space
            MotionModelMethod::MotionModelPointer mm(new UnicycleMotionModel(siF_, pathToSetupFile_.c_str()));
            siF_->setMotionModel(mm);

            ompl::control::StatePropagatorPtr prop(ompl::control::StatePropagatorPtr(new UnicycleStatePropagator(siF_)));
            statePropagator_ = prop;
            siF_->setStatePropagator(statePropagator_);
            siF_->setPropagationStepSize(0.01); // this is the duration that a control is applied
            siF_->setMinMaxControlDuration(1,100);

            if(!start_ || !goal_)
            {
                throw ompl::Exception("Start/Goal not set");
            }

            this->setStartAndGoalStates();

            ompl::base::PlannerPtr plnr(new FIRM(siF_, false));

            planner_ = plnr;

            planner_->setProblemDefinition(pdef_);

            planner_->setup();

            policyGenerator_ = new NBM3P(siF_);

            //policyGenerator_->sampleNewBeliefStates();

            //policyGenerator_->setCurrentBeliefStates(beliefStates_);

            policyGenerator_->setBeliefTargetStates(targetStates_);

            siF_->setTrueState(start_);

            setup_ = true;
        }

    }

    bool solve()
    {
        //std::vector<ompl::base::State*> tempbStates;
        policyGenerator_->sampleNewBeliefStates();

        if(!setup_)
        {
            this->setup();
        }

        std::vector<ompl::base::State*> bstates;
        ompl::base::State *currentTrueState = siF_->allocState();
        siF_->getTrueState(currentTrueState);

        bstates = beliefStates_;

        int counter = 0;
         while(!policyGenerator_->isConverged())
        {
            std::vector<ompl::control::Control*> policy;

            policyGenerator_->generatePolicy(policy);

            int rndnum = FIRMUtils::generateRandomIntegerInRange(0, ompl::magic::MAX_MM_POLICY_LENGTH/*policy.size()-1*/);

            int hzn = rndnum > policy.size()? policy.size() : rndnum;

            for(int i=0; i < hzn ; i++)
            {
                siF_->applyControl(policy[i],true);

                //policyGenerator_->getCurrentBeliefStates(tempbStates);

                policyGenerator_->propagateBeliefs(policy[i]);

                siF_->getTrueState(currentTrueState);

                policyGenerator_->getCurrentBeliefStates(bstates);

                // If the robot's clearance gets below the threshold, break loop & replan
                if(!policyGenerator_->areCurrentBeliefsValid() || siF_->getStateValidityChecker()->clearance(currentTrueState) < ompl::magic::MIN_ROBOT_CLEARANCE)
                {
                    if(counter == 0)
                    {
                        counter++;
                        break;
                    }

                }
                if(counter > ompl::magic::MIN_STEPS_AFTER_CLEARANCE_VIOLATION_REPLANNING)
                    counter = 0;

                //std::cout<<"Clearance :"<<siF_->getStateValidityChecker()->clearance(currentTrueState)<<std::endl;

                boost::this_thread::sleep(boost::posix_time::milliseconds(20));
            }
        }

        return true;

    }

    void executeSolution()
    {
        planner_->as<FIRM>()->executeFeedback();
    }


    ompl::app::GeometricStateExtractor getGeometricStateExtractor(void) const
    {
        return boost::bind(&MultiModalSetup::getGeometricComponentStateInternal, this, _1, _2);
    }


protected:

    const ompl::base::State* getGeometricComponentStateInternal(const ompl::base::State *state, unsigned int /*index*/) const
    {
        return state;
    }

    void loadParameters()
    {
        using namespace arma;

        TiXmlDocument doc(pathToSetupFile_);

        bool loadOkay = doc.LoadFile();

        if ( !loadOkay )
        {
            printf( "Could not load setup file in planning problem. Error='%s'. Exiting.\n", doc.ErrorDesc() );

            exit( 1 );
        }

        TiXmlNode* node = 0;

        TiXmlElement* itemElement = 0;

        node = doc.FirstChild( "PlanningProblem" );
        assert( node );

        TiXmlNode* child = 0;

        // Read the env mesh file
        child = node->FirstChild("Environment");
        assert( child );

        itemElement = child->ToElement();
        assert( itemElement );

        std::string environmentFilePath;
        itemElement->QueryStringAttribute("environmentFile", &environmentFilePath);

        this->setEnvironmentMesh(environmentFilePath);

        // Read the robot mesh file
        child  = node->FirstChild("Robot");
        assert( child );

        itemElement = child->ToElement();
        assert( itemElement );

        std::string robotFilePath;
        itemElement->QueryStringAttribute("robotFile", &robotFilePath);

        this->setRobotMesh(robotFilePath);

        // Read the start Pose
        child  = node->FirstChild("StartPose");
        assert( child );

        itemElement = child->ToElement();
        assert( itemElement );

        double startX = 0,startY = 0, startTheta = 0;

        itemElement->QueryDoubleAttribute("x", &startX);
        itemElement->QueryDoubleAttribute("y", &startY);
        itemElement->QueryDoubleAttribute("theta", &startTheta);

        setStartState(startX, startY, startTheta);

        // Read the Goal Pose

        child  = node->FirstChild("GoalPose");
        assert( child );

        itemElement = child->ToElement();
        assert( itemElement );

        double goalX = 0 , goalY = 0, goalTheta = 0;

        itemElement->QueryDoubleAttribute("x", &goalX);
        itemElement->QueryDoubleAttribute("y", &goalY);
        itemElement->QueryDoubleAttribute("theta", &goalTheta);

        setGoalState(goalX, goalY, goalTheta);

        // read planning time
        child  = node->FirstChild("PlanningTime");
        assert( child );

        itemElement = child->ToElement();
        assert( itemElement );

        double time = 0;

        itemElement->QueryDoubleAttribute("maxTime", &time) ;

        planningTime_ = time;

        this->loadStartBeliefs();

        this->loadTargets();

        OMPL_INFORM("Problem configuration is");

        std::cout<<"Path to environment mesh"<<environmentFilePath<<std::endl;

        std::cout<<"Path to robot mesh"<<robotFilePath<<std::endl;

        std::cout<<"Start Pose X: "<<startX<<" Y: "<<startY<<" Theta: "<<startTheta<<std::endl;

        std::cout<<"Planning Time: "<<planningTime_<<std::endl;

    }

    void loadStartBeliefs()
    {
        TiXmlDocument doc(pathToSetupFile_);
        bool loadOkay = doc.LoadFile();

        if ( !loadOkay )
        {
            printf( "Could not load beliefs . Error='%s'. Exiting.\n", doc.ErrorDesc() );
            exit( 1 );
        }

        TiXmlNode* node = 0;
        TiXmlElement* landmarkElement = 0;
        TiXmlElement* itemElement = 0;

        node = doc.FirstChild( "BeliefList" );
        assert( node );
        landmarkElement = node->ToElement(); //convert node to element
        assert( landmarkElement  );

        TiXmlNode* child = 0;

        arma::mat cov = arma::eye(3,3);
        cov(0,0) = 0.03;
        cov(1,1) = 0.03;
        cov(2,2) = 0.004;

        while( (child = landmarkElement ->IterateChildren(child)))
        {
            assert( child );
            itemElement = child->ToElement();
            assert( itemElement );

            ompl::base::State *tempState = siF_->allocState();

            double x = 0, y = 0, theta = 0;

            itemElement->QueryDoubleAttribute("x", &x) ;
            itemElement->QueryDoubleAttribute("y", &y) ;
            itemElement->QueryDoubleAttribute("theta", &theta) ;

            std::cout<<"Read the state :"<<x<<" "<<y<<" "<<theta<<std::endl;

            tempState->as<SE2BeliefSpace::StateType>()->setXYYaw(x,y,theta);
            tempState->as<SE2BeliefSpace::StateType>()->setCovariance(cov);

            this->beliefStates_.push_back(tempState);

        }
    }

    void loadTargets()
    {
         TiXmlDocument doc(pathToSetupFile_);
        bool loadOkay = doc.LoadFile();

        if ( !loadOkay )
        {
            printf( "Could not load beliefs . Error='%s'. Exiting.\n", doc.ErrorDesc() );
            exit( 1 );
        }

        TiXmlNode* node = 0;
        TiXmlElement* landmarkElement = 0;
        TiXmlElement* itemElement = 0;

        node = doc.FirstChild( "TargetList" );
        assert( node );
        landmarkElement = node->ToElement(); //convert node to element
        assert( landmarkElement  );

        TiXmlNode* child = 0;

        while( (child = landmarkElement ->IterateChildren(child)))
        {
            assert( child );
            itemElement = child->ToElement();
            assert( itemElement );

            ompl::base::State *tempState = siF_->allocState();

            double x = 0, y = 0, theta = 0;

            itemElement->QueryDoubleAttribute("x", &x) ;
            itemElement->QueryDoubleAttribute("y", &y) ;
            itemElement->QueryDoubleAttribute("theta", &theta) ;

            std::cout<<"Read the state :"<<x<<" "<<y<<" "<<theta<<std::endl;

            tempState->as<SE2BeliefSpace::StateType>()->setXYYaw(x,y,theta);

            this->targetStates_.push_back(tempState);

        }

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

    ompl::base::StateValidityCheckerPtr vc_;

    std::string pathToSetupFile_;

    double planningTime_;

    bool setup_;

    std::vector<ompl::base::State*> beliefStates_;

    std::vector<ompl::base::State*> targetStates_;

    NBM3P *policyGenerator_;

};
#endif

