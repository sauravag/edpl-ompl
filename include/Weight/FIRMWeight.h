#ifndef FIRM_WEIGHT_
#define FIRM_WEIGHT_

class FIRMWeight {

  public:

    // Constructors and Destructor
    FIRMWeight(double cost=1, double successProbability = 0, size_t controllerID = -1):
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
      //Note: should successprob and controllerid also be assigned? FIRMApplication doesn't
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

    size_t getControllerID() { return controllerID_ ;}
    void   setControllerID(size_t id) { controllerID_ = id ;}

    double getSuccessProbability(){return successProbability_ ;}
    void setSuccessProbability(double p){ successProbability_ = p ;}

    // Data
  protected:
    double  cost_; // the cost of traversing the edge

    double successProbability_; //  the transition probability of the edge

    size_t controllerID_; // the identifier of the controller associated with the edge
};

#endif
