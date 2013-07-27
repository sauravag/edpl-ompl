#ifndef BELIEF_SPACE_H_
#define BELIEF_SPACE_H_

// OMPL includes
#include "ompl/base/spaces/SE2StateSpace.h"
#include "ompl/base/spaces/SE3StateSpace.h"
//other includes
#include <boost/math/constants/constants.hpp>
#include <armadillo>

using namespace ompl::base;
class SE2BeliefSpace : public ompl::base::CompoundStateSpace
{

  public:

    /** \brief A belief in SE(2): (x, y, yaw, covariance) */
    class StateType : public CompoundStateSpace::StateType
    {
    public:
        StateType(void) : CompoundStateSpace::StateType()
        {
          covariance_ = arma::zeros<arma::mat>(2,2);
          reachDist_ = 0;
          controllerID_ = -1;

        }

        /** \brief Get the X component of the state */
        double getX(void) const
        {
            return as<RealVectorStateSpace::StateType>(0)->values[0];
        }

        /** \brief Get the Y component of the state */
        double getY(void) const
        {
            return as<RealVectorStateSpace::StateType>(0)->values[1];
        }

        /** \brief Get the yaw component of the state. This is
            the rotation in plane, with respect to the Z
            axis. */
        double getYaw(void) const
        {
            return as<SO2StateSpace::StateType>(1)->value;
        }

        arma::mat getCovariance(void) const
        {
            return covariance_;
        }

        /** \brief Set the X component of the state */
        void setX(double x)
        {
            as<RealVectorStateSpace::StateType>(0)->values[0] = x;
        }

        /** \brief Set the Y component of the state */
        void setY(double y)
        {
            as<RealVectorStateSpace::StateType>(0)->values[1] = y;
        }

        /** \brief Set the X and Y components of the state */
        void setXY(double x, double y)
        {
            setX(x);
            setY(y);
        }

        /** \brief Set the yaw component of the state. This is
            the rotation in plane, with respect to the Z
            axis. */
        void setYaw(double yaw)
        {
            as<SO2StateSpace::StateType>(1)->value = yaw;
        }

        void setXYYaw(double x, double y, double yaw)
        {
            setX(x);
            setY(y);
            setYaw(yaw);
        }

        void setCovariance(arma::mat cov){
            covariance_ = cov;
        }

        arma::colvec getArmaData(void) const
        {
            arma::colvec stateVec(3);

            stateVec[0] = getX();
            stateVec[1] = getY();
            stateVec[2] = getYaw();
            return stateVec;
        }

        static double meanNormWeight_, covNormWeight_;

        private:
          arma::mat covariance_;
          double reachDist_;
          size_t controllerID_;

    };


    SE2BeliefSpace(void) : CompoundStateSpace()
    {
        setName("SE2_BELIEF" + getName());
        type_ = 2;
        addSubspace(StateSpacePtr(new RealVectorStateSpace(2)), 1.0);
        addSubspace(StateSpacePtr(new SO2StateSpace()), 0.5);
        lock();
    }

    virtual ~SE2BeliefSpace(void)
    {
    }

    /** \copydoc RealVectorStateSpace::setBounds() */
    void setBounds(const RealVectorBounds &bounds)
    {
        as<RealVectorStateSpace>(0)->setBounds(bounds);
    }

    /** \copydoc RealVectorStateSpace::getBounds() */
    const RealVectorBounds& getBounds(void) const
    {
        return as<RealVectorStateSpace>(0)->getBounds();
    }

    virtual State* allocState(void) const;
    virtual void freeState(State *state) const;

    //virtual void registerProjections(void);

    // gets the relative vector between "from" and "to"
    // equivalent to result = vectorA-vectorB
    void getRelativeState(const State *from, const State *to, State *state);

    void printState(const State *state);

};
#endif
/*
  BeliefSpace(const SE2StateSpace& state = SE2StateSpace(),
                const arma::mat& covariance = arma::zeros<arma::mat>(0,0),
                double reachDist = 0.0,
                size_t controllerID = -1);

  GaussianBelief& operator-=(const GaussianBelief& b);
  GaussianBelief operator-(const GaussianBelief& b) const;
  GaussianBelief operator-() const;

  bool operator==(const GaussianBelief& b) const;

  virtual bool equalStates

  //virtual const string GetName() const {return "GaussianBelief";}

  //template<class DistanceMetricPointer>
  //void GetRandomRay(double _incr, Environment* _env,  DistanceMetricPointer _dm, bool _norm=true);

  size_t GetControllerID() const {return controllerID_;}
  void SetControllerID(size_t id){controllerID_ = id;}

  void SetCovariance(const arma::mat& covariance) {covariance_ = covariance;}

  const arma::mat& GetCovariance() const {return covariance_;}

  double Norm();

  bool IsReached(const GaussianBelief& b) const;

  //void Draw();

  static double meanNormWeight_, covNormWeight_;

  //virtual ostream& Write(ostream& _os) {return (_os << *this);}

  //I/O
  //friend ostream& operator<< (ostream&, const GaussianBelief& _gb);
  //friend istream& operator>> (istream&, GaussianBelief& _gb);

private:
  arma::mat covariance_;
  double reachDist_;
  size_t controllerID_;
*/
