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

#ifndef FStrategy_H_
#define FStrategy_H_


template <class Milestone>
class FStrategy
{
public:

    /** \brief Constructor takes the maximum number of nearest neighbors to return (\e k) and the
        nearest neighbors datastruture to use (\e nn) */
    FStrategy(double r, const boost::shared_ptr< ompl::NearestNeighbors<Milestone> > &nn) :
        radius_(r), nn_(nn){}

    virtual ~FStrategy(void)
    {
    }

    /** \brief Set the nearest neighbors datastructure to use */
    void setNearestNeighbors(const boost::shared_ptr< ompl::NearestNeighbors<Milestone> > &nn)
    {
        nn_ = nn;
    }

    /** \brief Given a milestone \e m, find the number of nearest
        neighbors connection attempts that should be made from it,
        according to the connection strategy */
    std::vector<Milestone>& operator()(const Milestone& m)
    {
        nn_->nearestR(m, radius_, neighbors_);
        return neighbors_;
    }

protected:

    /** \brief Maximum distance to nearest neighbors to attempt to connect new milestones to */
    double                                     radius_;

    /** \brief Nearest neighbors data structure */
    boost::shared_ptr< ompl::NearestNeighbors<Milestone> > nn_;

    /** \brief Scratch space for storing k-nearest neighbors */
    std::vector<Milestone>                           neighbors_;
};

#endif
