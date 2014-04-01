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
#ifndef FIRM_WEIGHT_
#define FIRM_WEIGHT_

class FIRMWeight {

  public:

    // Constructors and Destructor
    FIRMWeight(double cost=0, double successProbability = 0, int controllerID = -1):
    cost_(cost), controllerID_(controllerID), successProbability_(successProbability) {}

    ~FIRMWeight(){}

    bool operator==(const FIRMWeight& w) const
    {
      // Check with Ali if equality needs to be checked for attributes, FIRMApplication doesn't
      //return ( cost_== w.cost_ && controllerID_ == w.controllerID_ && successProbability_ == w.successProbability_);
      return cost_== w.cost_;
    }

    // Check with Ali
    const FIRMWeight& operator=(const FIRMWeight& w)
    {
      cost_ = w.cost_;
      //Note: should successprob and controllerid also be assigned? original pmpl FIRMApplication doesn't
      //successProbability_ = w.successProbability_;
      //controllerID_ = w.controllerID_;
      return *this;
    }


    FIRMWeight operator+(const FIRMWeight& w) const
    {
      return FIRMWeight(cost_+w.cost_,successProbability_,controllerID_);
    }

    bool operator<(const FIRMWeight& w) const
    {
      return cost_ < w.cost_;
    }

    // Read/Write values of datamember to given input/output stream.
    //friend ostream& operator<< (ostream& _os, const FIRMWeight& _w);
    //friend istream& operator>> (istream& _is, FIRMWeight& _w);


    double getCost(){return cost_;}
    void setCost(double c){ cost_ = c;}

    int getControllerID() { return controllerID_ ;}
    void   setControllerID(int id) { controllerID_ = id ;}

    double getSuccessProbability(){return successProbability_ ;}
    void setSuccessProbability(double p){ successProbability_ = p ;}

    // Data
  protected:
    double  cost_; // the cost of traversing the edge

    size_t controllerID_; // the identifier of the controller associated with the edge

    double successProbability_; //  the transition probability of the edge


};

#endif
