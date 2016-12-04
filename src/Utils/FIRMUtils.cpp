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


#include "Utils/FIRMUtils.h"
#include <boost/math/constants/constants.hpp>
#include <boost/date_time.hpp>
#include <utility>
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
    //std::random_device rd; // obtain a random number from hardware

    //std::mt19937 eng(rd()); // seed the generator

    //std::uniform_int_distribution<> distr(floor, ceiling); // define the range

    //return distr(eng);

    int r = rand()%(ceiling - floor + 1) + floor;

    return r;
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


   }

   // Generate time stamp for saving roadmap
    namespace pt = boost::posix_time;

    pt::ptime now = pt::second_clock::local_time();

    std::string timeStamp(to_iso_string(now)) ;

    std::string roadmapFileName =  "FIRMRoadMap.xml" + timeStamp ;

	doc.SaveFile(roadmapFileName);
}

bool FIRMUtils::readFIRMGraphFromXML(const std::string &pathToXML, std::vector<std::pair<int, arma::colvec> > &FIRMNodePosList, std::vector<std::pair<int, arma::mat> > &FIRMNodeCovarianceList, std::vector<std::pair<std::pair<int,int>,FIRMWeight> > &edgeWeights)
{

    TiXmlDocument doc(pathToXML);

    bool loadOkay = doc.LoadFile();

    if ( !loadOkay )
    {
        OMPL_INFORM("FIRMUtils: Could not load Graph from XML . Need to construct graph.");
        return false;
    }

    TiXmlNode* NodeList = 0;

    TiXmlElement* nodeElement = 0;

    TiXmlElement* itemElement = 0;

    NodeList = doc.FirstChild( "Nodes" );

    assert( NodeList );

    nodeElement = NodeList->ToElement(); //convert NodeList to element

    assert( nodeElement  );

    TiXmlNode* child = 0;

    while( (child = nodeElement->IterateChildren(child)))
    {
        assert( child );

        itemElement = child->ToElement();

        assert( itemElement );

        double x = 0, y = 0, theta = 0, c11 = 0, c12 = 0, c13 = 0, c21 = 0, c22 = 0, c23 = 0, c31 = 0, c32 = 0, c33 = 0;
        int id = 0;

        itemElement->QueryIntAttribute("id", &id) ;
        itemElement->QueryDoubleAttribute("x", &x) ;
        itemElement->QueryDoubleAttribute("y", &y) ;
        itemElement->QueryDoubleAttribute("theta", &theta) ;
        itemElement->QueryDoubleAttribute("c11", &c11) ;
        itemElement->QueryDoubleAttribute("c12", &c12) ;
        itemElement->QueryDoubleAttribute("c13", &c13) ;
        itemElement->QueryDoubleAttribute("c21", &c21) ;
        itemElement->QueryDoubleAttribute("c22", &c22) ;
        itemElement->QueryDoubleAttribute("c23", &c23) ;
        itemElement->QueryDoubleAttribute("c31", &c31) ;
        itemElement->QueryDoubleAttribute("c32", &c32) ;
        itemElement->QueryDoubleAttribute("c33", &c33) ;

        arma::colvec xVec(3);
        arma::mat cov(3,3);

        xVec(0) = x;
        xVec(1) = y;
        xVec(2) = theta;

        cov(0,0) = c11;
        cov(0,1) = c12;
        cov(0,2) = c13;
        cov(1,0) = c21;
        cov(1,1) = c22;
        cov(1,2) = c23;
        cov(2,0) = c31;
        cov(2,1) = c32;
        cov(2,2) = c33;

        FIRMNodePosList.push_back(std::make_pair(id,xVec));
        FIRMNodeCovarianceList.push_back(std::make_pair(id,cov));

    }


    //////////////////////
    TiXmlNode* edgeList = 0;

    TiXmlElement* edgeElement = 0;

    TiXmlElement* itemElement2 = 0;

    edgeList = doc.FirstChild( "Edges" );

    assert( edgeList );

    edgeElement = edgeList->ToElement(); //convert NodeList to element

    assert( edgeElement  );

    TiXmlNode* child2 = 0;

    while( (child2 = edgeElement->IterateChildren(child2)))
    {
        assert( child2 );

        itemElement2 = child2->ToElement();

        assert( itemElement2 );

        int startVertexID = 0, endVertexID = 0;
        double successProb = 0, cost = 0;

        itemElement2->QueryIntAttribute("startVertexID", &startVertexID) ;
        itemElement2->QueryIntAttribute("endVertexID", &endVertexID) ;
        itemElement2->QueryDoubleAttribute("successProb", &successProb) ;
        itemElement2->QueryDoubleAttribute("cost", &cost) ;

        FIRMWeight w(cost, successProb);

        edgeWeights.push_back(std::make_pair(std::make_pair(startVertexID, endVertexID),w));

    }

    return true;
}

double FIRMUtils::degree2Radian(double deg)
{
    return boost::math::constants::pi<double>()*deg/180.0;
}

double FIRMUtils::radian2Degree(double rads)
{
    return rads*180.0/boost::math::constants::pi<double>();
}

