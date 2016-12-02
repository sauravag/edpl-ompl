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

#ifndef FIRM_UTILS_H
#define FIRM_UTILS_H

#include "Weight/FIRMWeight.h"
#include "Spaces/SE2BeliefSpace.h"

/** \brief A class containing utility functions used commonly*/
class FIRMUtils
{
    public:

        /** \brief Normalizes angle to between -pi and pi */
        static void normalizeAngleToPiRange(double &theta);

        /** \brief Returns the sign of the value 'd' */
        static int signum(const double d);

        /** \brief Generates a random number within the give range */
        static int generateRandomIntegerInRange(const int floor, const int ceiling);

        /** \brief Save the FIRM graph to an XML file */
        static void writeFIRMGraphToXML(const std::vector<std::pair<int,std::pair<arma::colvec,arma::mat> > > nodes, const std::vector<std::pair<std::pair<int,int>,FIRMWeight> > edgeWeights);

        /** \brief Reads the Graph properties from an XML file */
        static bool readFIRMGraphFromXML(const std::string &pathToXML,std::vector<std::pair<int, arma::colvec> > &FIRMNodePosList, std::vector<std::pair<int, arma::mat> > &FIRMNodeCovarianceList, std::vector<std::pair<std::pair<int,int>,FIRMWeight> > &edgeWeights);

        /** \brief Convert degree to radian */
        static double degree2Radian(double deg);

        /** \brief radians to degree */
        static double radian2Degree(double rads);
};

#endif
