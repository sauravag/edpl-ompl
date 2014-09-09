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


#include "../../include/Utils/FIRMUtils.h"
#include <boost/math/constants/constants.hpp>
#include <random>
#include <tinyxml.h>


void FIRMUtils::normalizeAngleToPiRange(double &theta)
{

    while(theta > boost::math::constants::pi<double>())
    {
        theta -= 2*boost::math::constants::pi<double>();
    }

    while(theta < -boost::math::constants::pi<double>())
    {
        theta += 2*boost::math::constants::pi<double>();
    }

}

int FIRMUtils::signum(const double d)
{
        if(d>0)
            return 1;

        if(d<0)
            return -1;
}

int FIRMUtils::generateRandomIntegerInRange(const int floor, const int ceiling)
{
    std::random_device rd; // obtain a random number from hardware

    std::mt19937 eng(rd()); // seed the generator

    std::uniform_int_distribution<> distr(floor, ceiling); // define the range

    return distr(eng);
}

void FIRMUtils::writeFIRMGraphToXML(const std::vector<std::pair<int,std::pair<arma::colvec,arma::mat> > > nodes, const std::vector<std::pair<std::pair<int,int>,FIRMWeight> > edgeWeights)
{
    TiXmlDocument doc;

 	TiXmlDeclaration* decl = new TiXmlDeclaration( "1.0", "", "" );
	doc.LinkEndChild( decl );

	TiXmlElement * Nodes = new TiXmlElement( "Nodes" );
	doc.LinkEndChild( Nodes );


	for(int i = 0; i < nodes.size(); i++)
	{
        TiXmlElement * node;
        node = new TiXmlElement( "node" );
        Nodes->LinkEndChild( node );

        int nodeID = nodes[i].first; // id of the node in the graph

        arma::colvec xVec = nodes[i].second.first; // x,y,yaw

        arma::mat cov = nodes[i].second.second; // covariance matrix

        node->SetAttribute("id", nodeID);
        node->SetDoubleAttribute("x", xVec(0));
        node->SetDoubleAttribute("y", xVec(1));
        node->SetDoubleAttribute("theta",xVec(2));
        node->SetDoubleAttribute("c11", cov(0,0));
        node->SetDoubleAttribute("c12", cov(0,1));
        node->SetDoubleAttribute("c13", cov(0,2));
        node->SetDoubleAttribute("c21", cov(1,0));
        node->SetDoubleAttribute("c22", cov(1,1));
        node->SetDoubleAttribute("c23", cov(1,2));
        node->SetDoubleAttribute("c31", cov(2,0));
        node->SetDoubleAttribute("c32", cov(2,1));
        node->SetDoubleAttribute("c33", cov(2,2));

        std::cout<<"Writing Node: "<<xVec(0)<<" "<<xVec(1)<<" "<<xVec(2)<<" \n"<<cov<<std::endl;

   }

    TiXmlElement * Edges = new TiXmlElement( "Edges" );
	doc.LinkEndChild( Edges );

	for(int i = 0; i < edgeWeights.size(); i++)
	{
        TiXmlElement * edge;
        edge = new TiXmlElement( "edge" );
        Edges->LinkEndChild( edge );

        FIRMWeight w = edgeWeights[i].second;


        edge->SetAttribute("startVertexID", edgeWeights[i].first.first);
        edge->SetAttribute("endVertexID", edgeWeights[i].first.second);
        edge->SetDoubleAttribute("successProb", w.getSuccessProbability());
        edge->SetDoubleAttribute("cost", w.getCost());


        std::cout<<"Writing edge : "<<w.getCost()<<" "<< w.getSuccessProbability()<<std::endl;

   }

	doc.SaveFile( "FIRMRoadMap.xml" );
}


























