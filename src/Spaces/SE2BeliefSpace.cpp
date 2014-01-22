#include "../../include/Spaces/SE2BeliefSpace.h"

ompl::base::State* SE2BeliefSpace::allocState(void) const
{
    StateType *state = new StateType();
    allocStateComponents(state);
    return state;
}

void SE2BeliefSpace::copyState(State *destination, const State *source) const
{
    destination->as<StateType>()->setX(source->as<StateType>()->getX());
    destination->as<StateType>()->setY(source->as<StateType>()->getY());
    destination->as<StateType>()->setYaw(source->as<StateType>()->getYaw());
    destination->as<StateType>()->setCovariance(source->as<StateType>()->getCovariance());
}

void SE2BeliefSpace::freeState(State *state) const
{
    CompoundStateSpace::freeState(state);
}

double SE2BeliefSpace::distance(const State* state1, const State *state2)
{
    double dx = state1->as<StateType>()->getX() - state2->as<StateType>()->getX();
    double dy = state1->as<StateType>()->getY() - state2->as<StateType>()->getY();

    std::cout<<"Getting distance :"<<std::endl;
    std::cin.get();

    return pow(dx*dx+dy*dy, 0.5);
}
void SE2BeliefSpace::getRelativeState(const State *from, const State *to, State *state)
{
	state->as<StateType>()->setX(to->as<StateType>()->getX() - from->as<StateType>()->getX());
	state->as<StateType>()->setY(to->as<StateType>()->getY() - from->as<StateType>()->getY());

	/*
    	Calculating relative angle is a bit tricky.
    	Refer to "interpolate" function of SO2StateSpace at line 122 of SO2StateSpace.h  in OMPL lib
        to see the original implementation in OMPL
	*/

	double diff = to->as<StateType>()->getYaw() - from->as<StateType>()->getYaw();
	if (fabs(diff) <= boost::math::constants::pi<double>())
        state->as<StateType>()->setYaw(diff);
    else
    {
        double v;
        if (diff > 0.0)
            diff = 2.0 * boost::math::constants::pi<double>() - diff;
        else
            diff = -2.0 * boost::math::constants::pi<double>() - diff;

        v = - diff ;
        // input states are within bounds, so the following check is sufficient
        if (v > boost::math::constants::pi<double>())
            v -= 2.0 * boost::math::constants::pi<double>();
        else
            if (v < -boost::math::constants::pi<double>())
                v += 2.0 * boost::math::constants::pi<double>();
    	state->as<StateType>()->setYaw(v);
    }

    arma::mat fcov = from->as<StateType>()->getCovariance();
    arma::mat tocov = to->as<StateType>()->getCovariance();

    if (fcov.n_rows != 0 && fcov.n_cols != 0 && tocov.n_rows != 0 && tocov.n_cols != 0 )
    {
   		state->as<StateType>()->setCovariance(tocov - fcov);
    }
}

void SE2BeliefSpace::printBeliefState(const State *state)
{
    std::cout<<"----Printing BeliefState----"<<std::endl;
    std::cout<<"State [X, Y, Yaw]: ";
    std::cout<<"["<<state->as<SE2BeliefSpace::StateType>()->getX()<<", "<<state->as<SE2BeliefSpace::StateType>()->getY()
        <<", "<<state->as<SE2BeliefSpace::StateType>()->getYaw()<<"]"<<std::endl;
    std::cout<<"Covariance  is" <<std::endl;
    std::cout<<state->as<SE2BeliefSpace::StateType>()->getCovariance()<<std::endl;
    std::cout<<"------End BeliefState-------"<<std::endl;
}

/*
void SE2BeliefSpace::registerProjections(void)
{
    class SE2DefaultProjection : public ProjectionEvaluator
    {
    public:

        SE2DefaultProjection(const StateSpace *space) : ProjectionEvaluator(space)
        {
        }

        virtual unsigned int getDimension(void) const
        {
            return 2;
        }

        virtual void defaultCellSizes(void)
        {
            cellSizes_.resize(2);
            const RealVectorBounds &b = space_->as<SE2StateSpace>()->getBounds();
            cellSizes_[0] = (b.high[0] - b.low[0]) / magic::PROJECTION_DIMENSION_SPLITS;
            cellSizes_[1] = (b.high[1] - b.low[1]) / magic::PROJECTION_DIMENSION_SPLITS;
        }

        virtual void project(const State *state, EuclideanProjection &projection) const
        {
            memcpy(&projection(0), state->as<SE2StateSpace::StateType>()->as<RealVectorStateSpace::StateType>(0)->values, 2 * sizeof(double));
        }
    };

    registerDefaultProjection(ProjectionEvaluatorPtr(dynamic_cast<ProjectionEvaluator*>(new SE2DefaultProjection(this))));
}
*/
