/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2020 Yuchen and Yubing
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include "obstacle.h"
#include <ns3/log.h>
#include <random>
#include <cmath>
#include <vector>
#include <ns3/random-variable-stream.h>
#include "ns3/simulator.h"
#include "ns3/core-module.h"
#include <fstream>      // std::ofstream
#include "yans-wifi-channel.h"
#include <algorithm>
// #include "directional-60-ghz-antenna.h"

#define PI 3.14159265

#define min(a,b) ((a) < (b) ? (a) : (b))
#define max(a,b) ((a) > (b) ? (a) : (b))


namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("Obstacle");

NS_OBJECT_ENSURE_REGISTERED (Obstacle);

Obstacle::Obstacle ()
{
  NS_LOG_FUNCTION (this);  
  // Obstacle configuration based on practical scenario
  m_obstalceNumber = 22; // 43;
  m_obstalceNumber_human = (uint16_t)(floor(m_obstalceNumber*1.0*0.3));
  m_obstaclePenetrationLoss_low = {1130, 1470, 1700, 9, 60, 9, 860, 430, 130, 112};  
  m_obstaclePenetrationLoss_medium = {1130, 430, 1700, 430, 60, 130, 860, 430, 130, 112};
  m_obstaclePenetrationLoss_high = {1130, 1470, 1700, 430, 860, 1700, 860, 430, 860, 1130};
  m_xObsTNDistConf = {0.5, 1.75, 1.08, 0.18};
  m_yObsTNDistConf = {0.25, 1.25, 0.56, 0.08};
  // m_zObsTNDistConf = {0.15, 1.75, 0.61, 0.21};
  // m_zObsTNDistConf = {0.5, 2.0, 0.81, 0.41};
  // m_zObsTNDistConf = {0.5, 2.0, 0.94, 0.56};  // modified by Yuchen
  m_zObsTNDistConf = {1.65, 2.4, 1.85, 0.2}; // {1.6, 2.3, 1.8, 0.3}; // {0.5, 2.0, 0.85, 0.46}; org  // modified by Yuchen 2021
  // human obstacle
  m_xObsTNDistConf_human = {0.2, 0.9, 0.5, 0.1};
  m_yObsTNDistConf_human = {0.1, 0.5, 0.25, 0.05};
  m_zObsTNDistConf_human = {1.4, 1.9, 1.65, 0.20};
  // set by default
  m_SV_channel = false;
  m_TGad_channel = false;
  m_reflectionMode = 2; // medium reflective
  m_OnObsConflictCheck = false; // no conflict cheking
  m_allowClientInHall_BL = true;
  m_numFloors_BL = 1;
  // m_itfFlagforCal = LINE_OF_SIGHT; // by default -- LoS for interference calculation
  m_multiRoomFlag = true;
  
  m_apDimension = Vector (0.23, 0.23, 0.12);

  // Create TN distribution object
  m_tNDist = CreateObject<TruncatedNormalDistribution> ();
}

Obstacle::~Obstacle ()
{
  NS_LOG_FUNCTION (this);
}

void 
Obstacle::SetObstacleNumber (uint16_t obsNumber)
{
  m_obstalceNumber = obsNumber;
}

void 
Obstacle::SetMultiRoomFlag (bool multiRoomFlag)
{
  m_multiRoomFlag = multiRoomFlag;
}


void 
Obstacle::SetHumanObstacleNumber (uint16_t obsNumber_human)
{
  m_obstalceNumber_human = obsNumber_human;
}

void 
Obstacle::SetRoomConf (Vector roomSize)
{
  m_roomSize = roomSize;
}

void 
Obstacle::SetNumFloorsInbuilding (uint16_t numFloors)
{
  m_numFloors_BL = numFloors;
}


void 
Obstacle::SetAllowClientInHall (bool allowInHall)
{
  m_allowClientInHall_BL = allowInHall;
}


void 
Obstacle::EnableSVChannel (bool SV_channel)
{
  m_SV_channel = SV_channel;
}

void 
Obstacle::EnableTGadChannel (bool TGad_channel)
{
  m_TGad_channel = TGad_channel;
}


void 
Obstacle::SetReflectedMode (bool reflectedMode)
{
  m_reflectionMode = reflectedMode;
}

void 
Obstacle::SetPenetrationLossMode (std::vector<double> obstaclePenetrationLoss)
{
  m_obstaclePenetrationLoss = obstaclePenetrationLoss;
}

void 
Obstacle::SetObsConflictCheck (bool setObsConflictCheck)
{
  m_OnObsConflictCheck = setObsConflictCheck;
}

void 
Obstacle::SetLoSFlag(bool LoSFlag, uint16_t clientID)
{
  m_channelInfo.at(clientID).losFlag = LoSFlag;
}

void 
Obstacle::SetLoSFlag_ext(int16_t LoSFlag_ext, uint16_t clientID)
{
  m_channelInfo.at(clientID).losFlag_ext = LoSFlag_ext;
}

void 
Obstacle::SetPenLoss(double penLoss, uint16_t clientID)
{
  m_channelInfo.at(clientID).penLoss = penLoss;
}

void 
Obstacle::SetDistToOpen(double distToOpen, uint16_t clientID)
{
  m_channelInfo.at(clientID).distToOpen = distToOpen;
}

void 
Obstacle::SetItfComp(uint16_t beItfed, uint16_t clientID)
{
  m_channelInfo.at(clientID).beInfed = beItfed;
}

void 
Obstacle::SetDistToSrc(double dist, uint16_t clientID)
{
  m_channelInfo.at(clientID).iftDist = dist;
}

void 
Obstacle::SetItfSrcPos(Vector itfSrcPos, uint16_t clientID)
{
  m_channelInfo.at(clientID).iftSrcPos = itfSrcPos;
}




void 
Obstacle::SetInterferFlag(bool InterferFlag, uint16_t clientID)
{
  m_channelInfo.at(clientID).losInfFlag = InterferFlag;
}


void
Obstacle::SetFadingLoss(double fadingLoss, uint16_t clientID)
{
  m_channelInfo.at(clientID).fadingLoss = fadingLoss;
}

void 
Obstacle::SetAPPos(Vector apPos, uint16_t clientID)
{
  m_channelInfo.at(clientID).apPos = apPos;
}

void 
Obstacle::SetClientPos(Vector clientPos, uint16_t clientID)
{
  m_channelInfo.at(clientID).clientPos = clientPos;
}

void 
Obstacle::CreatChannelInfo()
{
  m_channelInfo.push_back(ChannelInfo());
}

void 
Obstacle::SetServedAPID(uint16_t APid, uint16_t clientID)
{
  m_channelInfo.push_back(ChannelInfo());
  m_channelInfo.at(clientID).servedAPId.push_back(APid);
}



std::vector<Box> 
Obstacle::GetObsDim ()
{
  return m_obstacleDimension;
}

std::vector<Vector> 
Obstacle::GetClientPos_BL ()
{
  return m_clientPos_BL;
}

std::vector<int16_t> 
Obstacle::GetClientRoomID_BL ()
{
  return m_roomNum;
}

uint16_t 
Obstacle::GetTotalAPNum_BL (std::vector<uint16_t> numAPs)
{
  uint16_t totalAPnum = 0;
  if (numAPs.size() > 0)
  	{
      for (uint16_t ia = 0; ia < numAPs.size(); ++ia)
       {
	     totalAPnum+=numAPs.at(ia);
       }
  	}
  return totalAPnum;
}


int16_t 
Obstacle::GetRoomIDforAP(uint16_t APID, std::vector<uint16_t> numAPs)
{
  // uint16_t totalAPNum = GetTotalAPNum_BL(numAPs);
  // uint16_t numRooms = numAPs.size();
  uint16_t cur = 0;
  uint16_t ia = 0;
  for (ia = 0; ia < numAPs.size(); ++ia)
   {
     cur+=numAPs.at(ia);
	 if ((APID) <= cur)
	  {
	    return ia;
	  }
   }
  return -1; // outside all rooms
}


std::vector<double> 
Obstacle::GetPenetrationLossMode ()
{
  return m_obstaclePenetrationLoss;
}

uint32_t 
Obstacle::GetFixedObsNum()
{
  return m_obstalceNumber;
}

uint32_t 
Obstacle::GetHumanObsNum()
{
  return m_obstalceNumber_human;
}

bool 
Obstacle::GetLoSFlag(uint16_t clientID)
{
  return m_channelInfo.at(clientID).losFlag;
}

bool
Obstacle::GetMultiRoomFlag()
{
  return m_multiRoomFlag;
}



int16_t
Obstacle::GetLoSFlag_ext(uint16_t clientID)
{
  return m_channelInfo.at(clientID).losFlag_ext;
}

double 
Obstacle::GetPenLoss(uint16_t clientID)
{
  return m_channelInfo.at(clientID).penLoss;
}


double 
Obstacle::GetDistToOpen(uint16_t clientID)
{
  return m_channelInfo.at(clientID).distToOpen;
}

bool 
Obstacle::GetInterferFlag(uint16_t clientID)
{
  return m_channelInfo.at(clientID).losInfFlag;
}


double 
Obstacle::GetFadingLoss(uint16_t clientID)
{
  return m_channelInfo.at(clientID).fadingLoss;
}

Vector 
Obstacle::GetAPPos(uint16_t clientID)
{
  return m_channelInfo.at(clientID).apPos;
}

Vector 
Obstacle::GetVirtualAPPos(uint16_t clientID)
{
  return m_channelInfo.at(clientID).virtualAPPos;
}


Vector 
Obstacle::GetClientPos(uint16_t clientID)
{
  return m_channelInfo.at(clientID).clientPos;
}

uint16_t 
Obstacle::GetNumClient ()
{
  return m_channelInfo.size();
}



std::pair<double, double> 
Obstacle::GetFadingInfo (Vector txPos, Vector rxPos)
{
  std::pair<double, double> fadingInfo;
  if (txPos.x == rxPos.x && txPos.y == rxPos.y && txPos.z == rxPos.z)
    {
      fadingInfo.first = 0;
      fadingInfo.second = 0;
      return fadingInfo;
    }

  for (uint16_t i=0; i<m_channelInfo.size(); i++)
    {
      if ((txPos.x == m_channelInfo.at(i).apPos.x || txPos.x == m_channelInfo.at(i).clientPos.x) && (rxPos.x == m_channelInfo.at(i).apPos.x || rxPos.x == m_channelInfo.at(i).clientPos.x) && (txPos.y == m_channelInfo.at(i).apPos.y || txPos.y == m_channelInfo.at(i).clientPos.y) && (rxPos.y == m_channelInfo.at(i).apPos.y || rxPos.y == m_channelInfo.at(i).clientPos.y) && (txPos.z == m_channelInfo.at(i).apPos.z || txPos.z == m_channelInfo.at(i).clientPos.z) && (rxPos.z == m_channelInfo.at(i).apPos.z || rxPos.z == m_channelInfo.at(i).clientPos.z))
        {
          fadingInfo.first = m_channelInfo.at(i).losFlag;
          fadingInfo.second = m_channelInfo.at(i).fadingLoss; 
          break;         
        }
    }  
  return fadingInfo;
}

std::pair<int16_t, double> 
Obstacle::GetInterferenceInfo (Vector txPos, Vector rxPos)
{
  std::pair<int16_t, double> inteferenceInfo;
  if (txPos.x == rxPos.x && txPos.y == rxPos.y && txPos.z == rxPos.z)
    {
      inteferenceInfo.first = 0;
      inteferenceInfo.second = 0;
      return inteferenceInfo;
    }

  for (uint16_t i=0; i<m_channelInfo.size(); i++)
    {
      if ((txPos.x == m_channelInfo.at(i).apPos.x || txPos.x == m_channelInfo.at(i).clientPos.x) && (rxPos.x == m_channelInfo.at(i).apPos.x || rxPos.x == m_channelInfo.at(i).clientPos.x) && (txPos.y == m_channelInfo.at(i).apPos.y || txPos.y == m_channelInfo.at(i).clientPos.y) && (rxPos.y == m_channelInfo.at(i).apPos.y || rxPos.y == m_channelInfo.at(i).clientPos.y) && (txPos.z == m_channelInfo.at(i).apPos.z || txPos.z == m_channelInfo.at(i).clientPos.z) && (rxPos.z == m_channelInfo.at(i).apPos.z || rxPos.z == m_channelInfo.at(i).clientPos.z))
        {
          inteferenceInfo.first = m_channelInfo.at(i).beInfed;
          inteferenceInfo.second = m_channelInfo.at(i).iftDist; 
          break;         
        }
    }  
  return inteferenceInfo;
}


std::pair<double, double> 
Obstacle::GetFadingInfo_ext (Vector txPos, Vector rxPos)
{
  std::pair<double, double> fadingInfo;
  if (txPos.x == rxPos.x && txPos.y == rxPos.y && txPos.z == rxPos.z)
    {
      fadingInfo.first = 0;
      fadingInfo.second = 0;
      return fadingInfo;
    }

  for (uint16_t i=0; i<m_channelInfo.size(); i++)
    {
      if ((txPos.x == m_channelInfo.at(i).apPos.x || txPos.x == m_channelInfo.at(i).clientPos.x) && (rxPos.x == m_channelInfo.at(i).apPos.x || rxPos.x == m_channelInfo.at(i).clientPos.x) && (txPos.y == m_channelInfo.at(i).apPos.y || txPos.y == m_channelInfo.at(i).clientPos.y) && (rxPos.y == m_channelInfo.at(i).apPos.y || rxPos.y == m_channelInfo.at(i).clientPos.y) && (txPos.z == m_channelInfo.at(i).apPos.z || txPos.z == m_channelInfo.at(i).clientPos.z) && (rxPos.z == m_channelInfo.at(i).apPos.z || rxPos.z == m_channelInfo.at(i).clientPos.z))
        {
          fadingInfo.first = (double)m_channelInfo.at(i).losFlag_ext;
          fadingInfo.second = m_channelInfo.at(i).fadingLoss; 
          break;         
        }
    }  
  return fadingInfo;
}



double 
Obstacle::multiPathFadingVarianceGen()
{
  double multipathFading;
  std::default_random_engine generator (m_clientRS);
  std::normal_distribution<double> multipathDistribution(0,2.24);
  multipathFading = multipathDistribution(generator);
  
  return multipathFading;
}

uint16_t 
Obstacle::checkItfAngleWithinBW (Vector apLocation, Vector client1, Vector client2, double BW)
{
  uint16_t itf = 0;
  Vector vec1 = Vector(client1.x - apLocation.x, client1.y - apLocation.y, client1.z - apLocation.z);
  Vector vec2 = Vector(client2.x - apLocation.x, client2.y - apLocation.y, client2.z - apLocation.z);
  double mod1 = std::sqrt(vec1.x*vec1.x + vec1.y*vec1.y + vec1.z*vec1.z);
  double mod2 = std::sqrt(vec2.x*vec2.x + vec2.y*vec2.y + vec2.z*vec2.z);
  double is_angle = std::acos((vec1.x*vec2.x + vec1.y*vec2.y + vec1.z*vec2.z)/(mod1*mod2));
  if (is_angle <= BW)
  	{
  	  itf = 1;
  	}
  return itf;
}

void 
Obstacle::GetBeInterferedFlag (std::vector<Ptr<Obstacle> > otheRoomInfo_vec, Vector apLocation_local, std::vector<Vector> clientLocation_local, std::vector<Box> obstacleDimension_local, Vector apDimension)
{
  for (uint16_t is = 0; is < otheRoomInfo_vec.size(); ++is)
  	{
  	  Ptr<Obstacle> otherRoom = otheRoomInfo_vec.at(is);
	  // first, check the LoS_ext signal, if there exists signal will propogate to the local room
	  uint16_t clientNum_oR = otherRoom->GetNumClient(); // in other room
	  for (uint16_t ic = 0; ic < clientLocation_local.size(); ++ic) // local client
	  	{
	  	   Vector clientLocation_this = clientLocation_local.at(ic);
	  	   for (uint16_t i = 0; i < clientNum_oR; ++i) // non-local client
	  	   	{
	  	   	  Vector clientLocation_oR = otherRoom->GetClientPos(i);
			  Vector apLocation_oR = otherRoom->GetAPPos(i);
              // get LoS_ext Flag
			  std::pair<double, double> fadingInfo = otherRoom->GetFadingInfo_ext(apLocation_oR, clientLocation_oR);
              int16_t LoS_ext_Flag = (int16_t) fadingInfo.first;
			  if (LoS_ext_Flag == LINE_OF_SIGHT_PassWin)
			  	{
			  	  // condition 1 -- opening propagation checking: if there exists signal propagate to the local room
			  	  // then check if the source AP, virtual AP, and local client are on the same line
			  	  Vector VirtualapLocation_oR = otherRoom->GetVirtualAPPos(i);
				  double yo = ((apLocation_oR.y - VirtualapLocation_oR.y)/(apLocation_oR.x - VirtualapLocation_oR.x))*(clientLocation_this.x - apLocation_oR.x) + apLocation_oR.y;
			  	  double zo = ((apLocation_oR.z - VirtualapLocation_oR.z)/(apLocation_oR.x - VirtualapLocation_oR.x))*(clientLocation_this.x - apLocation_oR.x) + apLocation_oR.z;
                  if (yo == clientLocation_this.y && zo == clientLocation_this.z)
                  	{
                  	  // condition 2 -- signal directivity checking: if three points are on the same line, which means the interference signal directs to the local client
                  	  // then check LoS status btw the virtual AP and this local client
                      std::vector<Vector> apLocation_vec;
					  apLocation_vec.push_back(VirtualapLocation_oR);
					  std::vector<Vector> LoSAP = checkLoS (apLocation_vec, clientLocation_this, obstacleDimension_local, apDimension);
					  if (LoSAP.empty() == false)
					  	{
					  	   // condition 3 -- local LoS checking: if LoS locally
					  	   // then the interference effect should be counted
					  	   m_channelInfo.at(ic).beInfed = 1;
	                       m_channelInfo.at(ic).virtualAPPos_self.push_back(VirtualapLocation_oR);
						   m_channelInfo.at(ic).iftSrcPosVec.push_back(apLocation_oR);
					  	}
                  	}
			  }
	  	   	}
	  	}
	  
  	}
}



bool
Obstacle::RecCollision(std::vector<Box> preObs, double cx, double cy, double length, double width)
{
  bool re;
  uint16_t numPreObs = preObs.size();
  double theta = 0;
  double granularity = 0.1;
  for (uint16_t i = 0; i < numPreObs; ++i)
  	{
  	  double cx1=cx-width/2*sin(theta)-length/2*cos(theta);	  
      double cy1=cy+width/2*cos(theta)-length/2*sin(theta);
    
      double cx2=cx-width/2*sin(theta)+length/2*cos(theta);
      double cy2=cy+width/2*cos(theta)+length/2*sin(theta);
    
      double cx3=cx+width/2*sin(theta)-length/2*cos(theta);
      double cy3=cy-width/2*cos(theta)-length/2*sin(theta);

      double cx4=cx+width/2*sin(theta)+length/2*cos(theta);
      double cy4=cy-width/2*cos(theta)+length/2*sin(theta);
    
      double min_cx=min(cx4,min(cx3,min(cx1,cx2)));
      double max_cx=max(cx4,max(cx3,max(cx1,cx2)));
      double min_cy=min(cy4,min(cy3,min(cy1,cy2)));
      double max_cy=max(cy4,max(cy3,max(cy1,cy2)));
    
      double cx_k=(preObs.at(i).xMax + preObs.at(i).xMin)/2;   // BK(ri,1);
      double cy_k=(preObs.at(i).yMax + preObs.at(i).yMin)/2;   // BK(ri,2);
      double length_k=(preObs.at(i).xMax - preObs.at(i).xMin); // BK(ri,3);
      double width_k=(preObs.at(i).yMax - preObs.at(i).yMin);  // BK(ri,4);
      double theta_k=0; // BK(ri,5);
    
      double BKx1=cx_k-width_k/2*sin(theta_k)-length_k/2*cos(theta_k);
      double BKy1=cy_k+width_k/2*cos(theta_k)-length_k/2*sin(theta_k);
    
      double BKx2=cx_k-width_k/2*sin(theta_k)+length_k/2*cos(theta_k);
      double BKy2=cy_k+width_k/2*cos(theta_k)+length_k/2*sin(theta_k);
    
      double BKx3=cx_k+width_k/2*sin(theta_k)-length_k/2*cos(theta_k);
      double BKy3=cy_k-width_k/2*cos(theta_k)-length_k/2*sin(theta_k);
    
      double BKx4=cx_k+width_k/2*sin(theta_k)+length_k/2*cos(theta_k);
      double BKy4=cy_k-width_k/2*cos(theta_k)+length_k/2*sin(theta_k);
    
      double min_BKx=min(BKx4,min(BKx3,min(BKx1,BKx2)));
      double max_BKx=max(BKx4,max(BKx3,max(BKx1,BKx2)));
      double min_BKy=min(BKy4,min(BKy3,min(BKy1,BKy2)));
      double max_BKy=max(BKy4,max(BKy3,max(BKy1,BKy2)));

      if(min_cx>(max_BKx+granularity) || min_BKx>(max_cx+granularity) || min_cy>(max_BKy+granularity) || min_BKy>(max_cy+granularity))
      	{
          re = false;
      	}
      else
      	{
          re = true; // has conflict
		  break;
      	}
  	}
  return re;
}


void
Obstacle::AllocateObstacle(Box railLocation, Vector roomSize, uint16_t clientRS)
{
  NS_LOG_FUNCTION (this);
  m_roomSize = roomSize;
  std::vector<Box> m_obstacleDimensionComparison;  
  uint16_t granularity = 10;

  bool obsConflictCheckFlag = m_OnObsConflictCheck;

  //--- GSL random init ---
  gsl_rng_env_setup();                          // Read variable environnement
  const gsl_rng_type* type = gsl_rng_default;   // Default algorithm 'twister'
  gsl_rng *gen = gsl_rng_alloc (type);          // Rand generator allocation
  gsl_rng_set(gen, 2);

  // each run with different scenario case
  RngSeedManager::SetSeed (7);

  // Poisson point process obstacle number identification
  double lambda = m_obstalceNumber/(m_roomSize.x*m_roomSize.y*granularity*granularity);
  double poissonProb = exp (-lambda)*lambda;
  uint16_t obstalceNoPlacement = 0;
  for (uint16_t t=0; t<m_roomSize.x*m_roomSize.y*granularity*granularity; t++)
     {
       bool ifConflict = true; // init
       // RngSeedManager::SetSeed (2);
       Ptr<UniformRandomVariable> obstacleNoUV = CreateObject<UniformRandomVariable> ();
       obstacleNoUV->SetAttribute ("Min", DoubleValue (0.0));
       obstacleNoUV->SetAttribute ("Max", DoubleValue (1.0));
       double probability = obstacleNoUV->GetValue ();
	   bool outsideroomFlag = false;

       // Place obstacle based on Poisson point process 
       if (probability < poissonProb)
         {
           double x, y;
		   std::pair<double, double> obstacleXRange, obstacleYRange, obstacleZRange;
           while (ifConflict == true)
           	{
           		x = (double) (t % (uint16_t)(m_roomSize.x * granularity))/granularity + 0.05;
           		y = floor (t/(m_roomSize.x * granularity))/granularity + 0.05; 
				outsideroomFlag = false;

           		// Define size of obstacle
           		// std::pair<double, double> obstacleXRange, obstacleYRange, obstacleZRange;
           		std::vector<double> tempXDistConf = m_xObsTNDistConf;
           		std::vector<double> tempYDistConf = m_yObsTNDistConf;
           		if (m_roomSize.x - x < tempXDistConf.at(1) * 0.5 || x < tempXDistConf.at(1) * 0.5)
             		tempXDistConf.at(1) = (m_roomSize.x - x > x)?x*2:2*(m_roomSize.x-x);
           		if (m_roomSize.y - y < tempYDistConf.at(1) * 0.5 || y < tempYDistConf.at(1) * 0.5)
             		tempYDistConf.at(1) = (m_roomSize.y - y > y)?y*2:2*(m_roomSize.y-y);
		//           NS_LOG_FUNCTION (this << tempXDistConf.at(1) << tempXDistConf.at(0) << tempYDistConf.at(1));
           		if (tempXDistConf.at(1) <= tempXDistConf.at(0) || tempYDistConf.at(1) <= tempYDistConf.at(0))
           		{
             		outsideroomFlag = true; // continue;
             		break;
           		}

           		obstacleXRange = m_tNDist->rtnorm(gen,tempXDistConf.at(0),tempXDistConf.at(1),tempXDistConf.at(2),tempXDistConf.at(3));
           		obstacleYRange = m_tNDist->rtnorm(gen,tempYDistConf.at(0),tempYDistConf.at(1),tempYDistConf.at(2),tempYDistConf.at(3));
           		obstacleZRange = m_tNDist->rtnorm(gen,m_zObsTNDistConf.at(0),m_zObsTNDistConf.at(1),m_zObsTNDistConf.at(2),m_zObsTNDistConf.at(3));

		   		// collision detection
		   		if ((m_obstacleDimensionComparison.empty() == false)&& (obsConflictCheckFlag == true))
		   		{
		   	   		ifConflict = RecCollision(m_obstacleDimensionComparison, x, y, obstacleXRange.first, obstacleYRange.first);
				}
				else
				{
				    ifConflict = false;
				}
           	}

		   if (outsideroomFlag == true)
		   	{ 
		   	  outsideroomFlag = false;
		   	  continue;
		   	}
		   

		   NS_LOG_FUNCTION (this << obstacleXRange.first << obstacleYRange.first << m_obstalceNumber);
           m_obstacleDimension.push_back(Box (x-obstacleXRange.first*0.5, x+obstacleXRange.first*0.5, y-obstacleYRange.first*0.5, y+obstacleYRange.first*0.5, 0, obstacleZRange.first));

           // collision detection
           // m_obstacleDimensionComparison.push_back(railLocation);
           // m_obstacleDimensionComparison.push_back(m_obstacleDimension.at(obstalceNoPlacement));

           m_obstacleDimensionComparison.push_back(m_obstacleDimension.at(obstalceNoPlacement));
           obstalceNoPlacement++;
         } 
     }
  m_obstalceNumber = obstalceNoPlacement;

  // Change obstacle location based on Poisson distribution
  for (uint16_t i=0; i<m_obstalceNumber*0.1; i++)
    {
      Ptr<UniformRandomVariable> obstacleIdUV = CreateObject<UniformRandomVariable> ();
      obstacleIdUV->SetAttribute ("Min", DoubleValue (0.0));
      obstacleIdUV->SetAttribute ("Max", DoubleValue (m_obstalceNumber-0.01));
      uint16_t obstacleId = floor(obstacleIdUV->GetValue ());  

      std::default_random_engine generator;
      std::poisson_distribution<int> distribution(0.1);
      for (uint16_t j=0; j<clientRS/10; j++)
        {
          uint16_t movingFlag = distribution(generator);
          //std::cout << movingFlag << std::endl;
          Ptr<UniformRandomVariable> movingAxisUV = CreateObject<UniformRandomVariable> ();
          movingAxisUV->SetAttribute ("Min", DoubleValue (0.0));
          movingAxisUV->SetAttribute ("Max", DoubleValue (PI*2));
          uint16_t movingAngle = movingAxisUV->GetValue ();

          if (movingFlag > 0)
            {
              if (m_obstacleDimension.at(obstacleId).xMax+0.1*cos(movingAngle)<m_roomSize.x && m_obstacleDimension.at(obstacleId).xMin+0.1*cos(movingAngle)>0)
                {
                  m_obstacleDimension.at(obstacleId).xMin += 0.1*cos(movingAngle);
                  m_obstacleDimension.at(obstacleId).xMax += 0.1*cos(movingAngle);
                }
              if (m_obstacleDimension.at(obstacleId).yMax+0.1*sin(movingAngle)<m_roomSize.y && m_obstacleDimension.at(obstacleId).yMin+0.1*cos(movingAngle)>0)
                {
                  m_obstacleDimension.at(obstacleId).yMin += 0.1*sin(movingAngle);
                  m_obstacleDimension.at(obstacleId).yMax += 0.1*sin(movingAngle);               
                } 
            }
        }
    }

  std::ofstream ofs;
  ofs.open ("obstacleList.txt", std::ofstream::out);
  ofs << m_obstalceNumber << std::endl;

  // Allocate obstacle type for penetration analysis
  for(uint16_t j=0; j<m_obstalceNumber; j++)
    { 
       ofs << m_obstacleDimension.at(j) << std::endl;
       NS_LOG_FUNCTION (this << m_obstalceNumber << m_obstacleDimension.at(j));
       Ptr<UniformRandomVariable> obstacleIndex = CreateObject<UniformRandomVariable> ();
       obstacleIndex->SetAttribute ("Min", DoubleValue (0.0));
       obstacleIndex->SetAttribute ("Max", DoubleValue (9.99));
       int index = floor(obstacleIndex->GetValue ());
       m_obstaclePenetrationLoss.push_back (m_obstaclePenetrationLoss.at(index));
    }
  ofs.close();

  // Allocate Human blockage
  if (m_obstalceNumber_human > 0)
  	{
  	   // Poisson point process obstacle number identification
  		lambda = m_obstalceNumber_human/(m_roomSize.x*m_roomSize.y*granularity*granularity);
  		poissonProb = exp (-lambda)*lambda;
  		uint16_t obstalceNoPlacement_hm = 0;
  		for (uint16_t t=0; t<m_roomSize.x*m_roomSize.y*granularity*granularity; t++)
     	{
       		RngSeedManager::SetSeed (2);
       		Ptr<UniformRandomVariable> obstacleNoUV_hm = CreateObject<UniformRandomVariable> ();
       		obstacleNoUV_hm->SetAttribute ("Min", DoubleValue (0.0));
       		obstacleNoUV_hm->SetAttribute ("Max", DoubleValue (1.0));
       		double probability = obstacleNoUV_hm->GetValue ();

       		// Place obstacle based on Poisson point process 
       		if (probability < poissonProb)
         	{
           		double x = (double) (t % (uint16_t)(m_roomSize.x * granularity))/granularity + 0.05;
           		double y = floor (t/(m_roomSize.x * granularity))/granularity + 0.05; 

           		// Define size of obstacle
           		std::pair<double, double> obstacleXRange, obstacleYRange, obstacleZRange;
           		std::vector<double> tempXDistConf = m_xObsTNDistConf_human;
           		std::vector<double> tempYDistConf = m_yObsTNDistConf_human;
           		if (m_roomSize.x - x < tempXDistConf.at(1) * 0.5 || x < tempXDistConf.at(1) * 0.5)
            	 	tempXDistConf.at(1) = (m_roomSize.x - x > x)?x*2:2*(m_roomSize.x-x);
           		if (m_roomSize.y - y < tempYDistConf.at(1) * 0.5 || y < tempYDistConf.at(1) * 0.5)
             		tempYDistConf.at(1) = (m_roomSize.y - y > y)?y*2:2*(m_roomSize.y-y);
		//           NS_LOG_FUNCTION (this << tempXDistConf.at(1) << tempXDistConf.at(0) << tempYDistConf.at(1));
           		if (tempXDistConf.at(1) <= tempXDistConf.at(0) || tempYDistConf.at(1) <= tempYDistConf.at(0))
             		continue;

           		obstacleXRange = m_tNDist->rtnorm(gen,tempXDistConf.at(0),tempXDistConf.at(1),tempXDistConf.at(2),tempXDistConf.at(3));
           		obstacleYRange = m_tNDist->rtnorm(gen,tempYDistConf.at(0),tempYDistConf.at(1),tempYDistConf.at(2),tempYDistConf.at(3));
           		obstacleZRange = m_tNDist->rtnorm(gen,m_zObsTNDistConf_human.at(0),m_zObsTNDistConf_human.at(1),m_zObsTNDistConf_human.at(2),m_zObsTNDistConf_human.at(3));

           		NS_LOG_FUNCTION (this << obstacleXRange.first << obstacleYRange.first << m_obstalceNumber_human);
           		m_obstacleDimension.push_back(Box (x-obstacleXRange.first*0.5, x+obstacleXRange.first*0.5, y-obstacleYRange.first*0.5, y+obstacleYRange.first*0.5, 0, obstacleZRange.first));
				/*
           		// collision detection
           		m_obstacleDimensionComparison.push_back(railLocation);
           		m_obstacleDimensionComparison.push_back(m_obstacleDimension.at(obstalceNoPlacement));

           		m_obstacleDimensionComparison.push_back(m_obstacleDimension.at(obstalceNoPlacement));
				*/
           		obstalceNoPlacement_hm++;
         	} 
     	}
  		m_obstalceNumber_human = obstalceNoPlacement_hm;

  		// Change human obstacle location based on Poisson distribution (orientation)
  		for (uint16_t i=0; i<m_obstalceNumber_human*0.1; i++)
    	{
      		Ptr<UniformRandomVariable> obstacleIdUV = CreateObject<UniformRandomVariable> ();
      		obstacleIdUV->SetAttribute ("Min", DoubleValue (m_obstalceNumber*1.0));
      		obstacleIdUV->SetAttribute ("Max", DoubleValue (m_obstalceNumber_human-0.01));
      		uint16_t obstacleId = floor(obstacleIdUV->GetValue ());  

      		std::default_random_engine generator;
      		std::poisson_distribution<int> distribution(0.1);
      		for (uint16_t j=0; j<clientRS/10; j++)
        	{
         		uint16_t movingFlag = distribution(generator);
          		//std::cout << movingFlag << std::endl;
          		Ptr<UniformRandomVariable> movingAxisUV = CreateObject<UniformRandomVariable> ();
          		movingAxisUV->SetAttribute ("Min", DoubleValue (0.0));
          		movingAxisUV->SetAttribute ("Max", DoubleValue (PI*2));
          		uint16_t movingAngle = movingAxisUV->GetValue ();

          		if (movingFlag > 0)
            	{
              		if (m_obstacleDimension.at(obstacleId).xMax+0.1*cos(movingAngle)<m_roomSize.x && m_obstacleDimension.at(obstacleId).xMin+0.1*cos(movingAngle)>0)
                	{
                  		m_obstacleDimension.at(obstacleId).xMin += 0.1*cos(movingAngle);
                  		m_obstacleDimension.at(obstacleId).xMax += 0.1*cos(movingAngle);
                	}
              		if (m_obstacleDimension.at(obstacleId).yMax+0.1*sin(movingAngle)<m_roomSize.y && m_obstacleDimension.at(obstacleId).yMin+0.1*cos(movingAngle)>0)
                	{
                  		m_obstacleDimension.at(obstacleId).yMin += 0.1*sin(movingAngle);
                  		m_obstacleDimension.at(obstacleId).yMax += 0.1*sin(movingAngle);               
                	} 
            	}
        	}
    	}

  		// Allocate obstacle type for penetration analysis
 		 for(uint16_t j=0; j<m_obstalceNumber_human; j++)
    	  { 
    	    /*
       		ofs << m_obstacleDimension.at(j) << std::endl;
       		NS_LOG_FUNCTION (this << m_obstalceNumber << m_obstacleDimension.at(j));
       		Ptr<UniformRandomVariable> obstacleIndex = CreateObject<UniformRandomVariable> ();
       		obstacleIndex->SetAttribute ("Min", DoubleValue (0.0));
       		obstacleIndex->SetAttribute ("Max", DoubleValue (9.99));
       		int index = floor(obstacleIndex->GetValue ());
       		*/
       		Ptr<UniformRandomVariable> humanPenloss = CreateObject<UniformRandomVariable> ();
       		humanPenloss->SetAttribute ("Min", DoubleValue (25.0));
       		humanPenloss->SetAttribute ("Max", DoubleValue (30.0));
       		double penlossVal = humanPenloss->GetValue ();
       		m_obstaclePenetrationLoss.push_back (penlossVal); // human penetration loss between 25~30
    	  }
  	}
  
}


void
Obstacle::AllocateObstacle_human(Box railLocation, Vector roomSize, uint16_t clientRS)
{
  NS_LOG_FUNCTION (this);
  m_roomSize = roomSize;
  std::vector<Box> m_obstacleDimensionComparison;  
  uint16_t granularity = 10;

  bool obsConflictCheckFlag = m_OnObsConflictCheck;

  //--- GSL random init ---
  gsl_rng_env_setup();                          // Read variable environnement
  const gsl_rng_type* type = gsl_rng_default;   // Default algorithm 'twister'
  gsl_rng *gen = gsl_rng_alloc (type);          // Rand generator allocation
  gsl_rng_set(gen, 2);

  // each run with different scenario case
  RngSeedManager::SetSeed (7);

  // Allocate Human blockage
  if (m_obstalceNumber_human > 0)
  	{
  	   // Poisson point process obstacle number identification
  		double lambda = m_obstalceNumber_human/(m_roomSize.x*m_roomSize.y*granularity*granularity);
  		double poissonProb = exp (-lambda)*lambda;
  		uint16_t obstalceNoPlacement_hm = 0;
  		for (uint16_t t=0; t<m_roomSize.x*m_roomSize.y*granularity*granularity; t++)
     	{
     	    bool ifConflict = true; // init
       		RngSeedManager::SetSeed (2);
       		Ptr<UniformRandomVariable> obstacleNoUV_hm = CreateObject<UniformRandomVariable> ();
       		obstacleNoUV_hm->SetAttribute ("Min", DoubleValue (0.0));
       		obstacleNoUV_hm->SetAttribute ("Max", DoubleValue (1.0));
       		double probability = obstacleNoUV_hm->GetValue ();
			bool outsideroomFlag = false;

       		// Place obstacle based on Poisson point process 
       		if (probability < poissonProb)
         	{
         	  double x, y;
		      std::pair<double, double> obstacleXRange, obstacleYRange, obstacleZRange;
         	  while (ifConflict == true)
           	  {
           		x = (double) (t % (uint16_t)(m_roomSize.x * granularity))/granularity + 0.05;
           		y = floor (t/(m_roomSize.x * granularity))/granularity + 0.05; 

           		// Define size of obstacle
           		// std::pair<double, double> obstacleXRange, obstacleYRange, obstacleZRange;
           		std::vector<double> tempXDistConf = m_xObsTNDistConf_human;
           		std::vector<double> tempYDistConf = m_yObsTNDistConf_human;
           		if (m_roomSize.x - x < tempXDistConf.at(1) * 0.5 || x < tempXDistConf.at(1) * 0.5)
            	 	tempXDistConf.at(1) = (m_roomSize.x - x > x)?x*2:2*(m_roomSize.x-x);
           		if (m_roomSize.y - y < tempYDistConf.at(1) * 0.5 || y < tempYDistConf.at(1) * 0.5)
             		tempYDistConf.at(1) = (m_roomSize.y - y > y)?y*2:2*(m_roomSize.y-y);
		//           NS_LOG_FUNCTION (this << tempXDistConf.at(1) << tempXDistConf.at(0) << tempYDistConf.at(1));
           		// if (tempXDistConf.at(1) <= tempXDistConf.at(0) || tempYDistConf.at(1) <= tempYDistConf.at(0))
             		// continue;
				if (tempXDistConf.at(1) <= tempXDistConf.at(0) || tempYDistConf.at(1) <= tempYDistConf.at(0))
           		{
             		outsideroomFlag = true; // continue;
             		break;
           		}

           		obstacleXRange = m_tNDist->rtnorm(gen,tempXDistConf.at(0),tempXDistConf.at(1),tempXDistConf.at(2),tempXDistConf.at(3));
           		obstacleYRange = m_tNDist->rtnorm(gen,tempYDistConf.at(0),tempYDistConf.at(1),tempYDistConf.at(2),tempYDistConf.at(3));
           		obstacleZRange = m_tNDist->rtnorm(gen,m_zObsTNDistConf_human.at(0),m_zObsTNDistConf_human.at(1),m_zObsTNDistConf_human.at(2),m_zObsTNDistConf_human.at(3));

                // collision detection
		   		if ((m_obstacleDimensionComparison.empty() == false)&& (obsConflictCheckFlag == true))
		   		{
		   	   		ifConflict = RecCollision(m_obstacleDimensionComparison, x, y, obstacleXRange.first, obstacleYRange.first);
				}
				else
				{
				    ifConflict = false;
				}

         	  }

			  if (outsideroomFlag == true)
		   	  { 
		   	    outsideroomFlag = false;
		   	    continue;
		   	  }
				
           	  NS_LOG_FUNCTION (this << obstacleXRange.first << obstacleYRange.first << m_obstalceNumber_human);
           	  m_obstacleDimension.push_back(Box (x-obstacleXRange.first*0.5, x+obstacleXRange.first*0.5, y-obstacleYRange.first*0.5, y+obstacleYRange.first*0.5, 0, obstacleZRange.first));
				/*
           		// collision detection
           		m_obstacleDimensionComparison.push_back(railLocation);
           		m_obstacleDimensionComparison.push_back(m_obstacleDimension.at(obstalceNoPlacement));

           		m_obstacleDimensionComparison.push_back(m_obstacleDimension.at(obstalceNoPlacement));
				*/
			  m_obstacleDimensionComparison.push_back(m_obstacleDimension.at(m_obstalceNumber + obstalceNoPlacement_hm));
           	  obstalceNoPlacement_hm++;
         	  
         	} 
     	}
  		m_obstalceNumber_human = obstalceNoPlacement_hm;

  		// Change human obstacle location based on Poisson distribution (orientation)
  		for (uint16_t i=0; i<m_obstalceNumber_human*0.1; i++)
    	{
      		Ptr<UniformRandomVariable> obstacleIdUV = CreateObject<UniformRandomVariable> ();
      		obstacleIdUV->SetAttribute ("Min", DoubleValue (m_obstalceNumber*1.0));
      		obstacleIdUV->SetAttribute ("Max", DoubleValue (m_obstalceNumber_human-0.01));
      		uint16_t obstacleId = floor(obstacleIdUV->GetValue ());  

      		std::default_random_engine generator;
      		std::poisson_distribution<int> distribution(0.1);
      		for (uint16_t j=0; j<clientRS/10; j++)
        	{
         		uint16_t movingFlag = distribution(generator);
          		//std::cout << movingFlag << std::endl;
          		Ptr<UniformRandomVariable> movingAxisUV = CreateObject<UniformRandomVariable> ();
          		movingAxisUV->SetAttribute ("Min", DoubleValue (0.0));
          		movingAxisUV->SetAttribute ("Max", DoubleValue (PI*2));
          		uint16_t movingAngle = movingAxisUV->GetValue ();

          		if (movingFlag > 0)
            	{
              		if (m_obstacleDimension.at(obstacleId).xMax+0.1*cos(movingAngle)<m_roomSize.x && m_obstacleDimension.at(obstacleId).xMin+0.1*cos(movingAngle)>0)
                	{
                  		m_obstacleDimension.at(obstacleId).xMin += 0.1*cos(movingAngle);
                  		m_obstacleDimension.at(obstacleId).xMax += 0.1*cos(movingAngle);
                	}
              		if (m_obstacleDimension.at(obstacleId).yMax+0.1*sin(movingAngle)<m_roomSize.y && m_obstacleDimension.at(obstacleId).yMin+0.1*cos(movingAngle)>0)
                	{
                  		m_obstacleDimension.at(obstacleId).yMin += 0.1*sin(movingAngle);
                  		m_obstacleDimension.at(obstacleId).yMax += 0.1*sin(movingAngle);               
                	} 
            	}
        	}
    	}

  		// Allocate obstacle type for penetration analysis
 		 for(uint16_t j=0; j<m_obstalceNumber_human; j++)
    	  { 
    	    /*
       		ofs << m_obstacleDimension.at(j) << std::endl;
       		NS_LOG_FUNCTION (this << m_obstalceNumber << m_obstacleDimension.at(j));
       		Ptr<UniformRandomVariable> obstacleIndex = CreateObject<UniformRandomVariable> ();
       		obstacleIndex->SetAttribute ("Min", DoubleValue (0.0));
       		obstacleIndex->SetAttribute ("Max", DoubleValue (9.99));
       		int index = floor(obstacleIndex->GetValue ());
       		*/
       		Ptr<UniformRandomVariable> humanPenloss = CreateObject<UniformRandomVariable> ();
       		humanPenloss->SetAttribute ("Min", DoubleValue (25.0));
       		humanPenloss->SetAttribute ("Max", DoubleValue (30.0));
       		double penlossVal = humanPenloss->GetValue ();
       		m_obstaclePenetrationLoss.push_back (penlossVal); // human penetration loss between 25~30
    	  }
  	}
  
}



void
Obstacle::AllocateObstacle_Fixed(Box railLocation, Vector roomSize, uint16_t clientRS, std::vector<double> xPos, std::vector<double> yPos, std::vector<double> wObs, std::vector<double> lObs, std::vector<double> hObs, std::vector<double> dirObs)
{
  NS_LOG_FUNCTION (this);
  // m_roomSize = roomSize;
  // std::vector<Box> m_obstacleDimensionComparison;  
  // uint16_t granularity = 10;

  //--- GSL random init ---
  // gsl_rng_env_setup();                          // Read variable environnement
  // const gsl_rng_type* type = gsl_rng_default;   // Default algorithm 'twister'
  // gsl_rng *gen = gsl_rng_alloc (type);          // Rand generator allocation
  // gsl_rng_set(gen, 2);

  // allocate dimensions and locations
  for (uint16_t i = 0; i < xPos.size(); ++i)
  	{
  		double x = xPos[i]; // obstacle's x coordinate
		double y = yPos[i]; // obstacle's y coordinate
		double width = wObs[i]; // obstacle's width
		double length = lObs[i]; // obstacle's length
		double h = hObs[i]; // obstacle's height
		// double h_min = hObs_min[i]; // obstacle's height base
		double theta = dirObs[i]; // obstacle's orientation, rad

		double cx = x;
        double cy = y;
        double cx1 = cx-width/2.0*std::sin(theta)-length/2.0*std::cos(theta);
    	double cy1 = cy+width/2.0*std::cos(theta)-length/2.0*std::sin(theta); 
	    double cx2 = cx-width/2.0*std::sin(theta)+length/2.0*std::cos(theta);
    	double cy2 = cy+width/2.0*std::cos(theta)+length/2.0*std::sin(theta);
   		double cx3 = cx+width/2.0*std::sin(theta)-length/2.0*std::cos(theta);
    	double cy3 = cy-width/2.0*std::cos(theta)-length/2.0*std::sin(theta);    
    	double cx4 = cx+width/2.0*std::sin(theta)+length/2.0*std::cos(theta);
    	double cy4 = cy-width/2.0*std::cos(theta)+length/2.0*std::sin(theta);
    
        double min_x_o = min(cx4,min(cx3,min(cx1,cx2)));
    	double max_x_o = max(cx4,max(cx3,max(cx1,cx2)));
    	double min_y_o = min(cy4,min(cy3,min(cy1,cy2)));
        double max_y_o = max(cy4,max(cy3,max(cy1,cy2)));
		
		m_obstacleDimension.push_back(Box (min_x_o, max_x_o, min_y_o, max_y_o, 0, h));
    }
  

  std::ofstream ofs;
  ofs.open ("obstacleList.txt", std::ofstream::out);
  ofs << m_obstalceNumber << std::endl;

  // Allocate obstacle type for penetration analysis
  for(uint16_t j=0; j<m_obstacleDimension.size(); j++)
    { 
       ofs << m_obstacleDimension.at(j) << std::endl;
       NS_LOG_FUNCTION (this << m_obstalceNumber << m_obstacleDimension.at(j));
       Ptr<UniformRandomVariable> obstacleIndex = CreateObject<UniformRandomVariable> ();
       obstacleIndex->SetAttribute ("Min", DoubleValue (0.0));
       obstacleIndex->SetAttribute ("Max", DoubleValue (9.99));
       int index = floor(obstacleIndex->GetValue ());
       m_obstaclePenetrationLoss.push_back (m_obstaclePenetrationLoss.at(index));
    }
  ofs.close();
}


void
Obstacle::AllocateObstacle_Fixed_withHB(Box railLocation, Vector roomSize, uint16_t clientRS, std::vector<double> xPos, std::vector<double> yPos, std::vector<double> wObs, std::vector<double> lObs, std::vector<double> hObs, std::vector<double> hObs_min, std::vector<double> dirObs)
{
  NS_LOG_FUNCTION (this);
  // m_roomSize = roomSize;
  // std::vector<Box> m_obstacleDimensionComparison;  
  // uint16_t granularity = 10;

  //--- GSL random init ---
  // gsl_rng_env_setup();                          // Read variable environnement
  // const gsl_rng_type* type = gsl_rng_default;   // Default algorithm 'twister'
  // gsl_rng *gen = gsl_rng_alloc (type);          // Rand generator allocation
  // gsl_rng_set(gen, 2);

  // allocate dimensions and locations
  for (uint16_t i = 0; i < xPos.size(); ++i)
  	{
  		double x = xPos[i]; // obstacle's x coordinate
		double y = yPos[i]; // obstacle's y coordinate
		double width = wObs[i]; // obstacle's width
		double length = lObs[i]; // obstacle's length
		double h = hObs[i]; // obstacle's height
		double h_min = hObs_min[i]; // obstacle's height base
		double theta = dirObs[i]; // obstacle's orientation, rad

		double cx = x;
        double cy = y;
        double cx1 = cx-width/2.0*std::sin(theta)-length/2.0*std::cos(theta);
    	double cy1 = cy+width/2.0*std::cos(theta)-length/2.0*std::sin(theta); 
	    double cx2 = cx-width/2.0*std::sin(theta)+length/2.0*std::cos(theta);
    	double cy2 = cy+width/2.0*std::cos(theta)+length/2.0*std::sin(theta);
   		double cx3 = cx+width/2.0*std::sin(theta)-length/2.0*std::cos(theta);
    	double cy3 = cy-width/2.0*std::cos(theta)-length/2.0*std::sin(theta);    
    	double cx4 = cx+width/2.0*std::sin(theta)+length/2.0*std::cos(theta);
    	double cy4 = cy-width/2.0*std::cos(theta)+length/2.0*std::sin(theta);
    
        double min_x_o = min(cx4,min(cx3,min(cx1,cx2)));
    	double max_x_o = max(cx4,max(cx3,max(cx1,cx2)));
    	double min_y_o = min(cy4,min(cy3,min(cy1,cy2)));
        double max_y_o = max(cy4,max(cy3,max(cy1,cy2)));
		
		m_obstacleDimension.push_back(Box (min_x_o, max_x_o, min_y_o, max_y_o, h_min, h));
    }
  

  std::ofstream ofs;
  ofs.open ("obstacleList.txt", std::ofstream::out);
  ofs << m_obstalceNumber << std::endl;

  // Allocate obstacle type for penetration analysis
  for(uint16_t j=0; j<m_obstacleDimension.size(); j++)
    { 
       ofs << m_obstacleDimension.at(j) << std::endl;
       NS_LOG_FUNCTION (this << m_obstalceNumber << m_obstacleDimension.at(j));
       Ptr<UniformRandomVariable> obstacleIndex = CreateObject<UniformRandomVariable> ();
       obstacleIndex->SetAttribute ("Min", DoubleValue (0.0));
       obstacleIndex->SetAttribute ("Max", DoubleValue (9.99));
       int index = floor(obstacleIndex->GetValue ());
       m_obstaclePenetrationLoss.push_back (m_obstaclePenetrationLoss.at(index));
    }
  ofs.close();
}



void 
Obstacle::AllocateObstacle_KnownBox(std::vector<Box> obsDim)
{
  NS_LOG_FUNCTION (this);
  // m_roomSize = roomSize;
  // std::vector<Box> m_obstacleDimensionComparison;  
  // uint16_t granularity = 10;

  //--- GSL random init ---
  // gsl_rng_env_setup();                          // Read variable environnement
  // const gsl_rng_type* type = gsl_rng_default;   // Default algorithm 'twister'
  // gsl_rng *gen = gsl_rng_alloc (type);          // Rand generator allocation
  // gsl_rng_set(gen, 2);

  // allocate dimensions and locations
  for (uint16_t i = 0; i < obsDim.size(); ++i)
  	{
  	   m_obstacleDimension.push_back(obsDim.at(i));
    }
  

  std::ofstream ofs;
  ofs.open ("obstacleList.txt", std::ofstream::out);
  ofs << m_obstacleDimension.size() << std::endl;

  // Allocate obstacle type for penetration analysis
  for(uint16_t j=0; j<m_obstacleDimension.size(); j++)
    { 
       ofs << m_obstacleDimension.at(j) << std::endl;
       // NS_LOG_FUNCTION (this << m_obstalceNumber << m_obstacleDimension.at(j));
       Ptr<UniformRandomVariable> obstacleIndex = CreateObject<UniformRandomVariable> ();
       obstacleIndex->SetAttribute ("Min", DoubleValue (0.0));
       obstacleIndex->SetAttribute ("Max", DoubleValue (9.99));
       int index = floor(obstacleIndex->GetValue ());
       m_obstaclePenetrationLoss.push_back (m_obstaclePenetrationLoss.at(index));
    }
  ofs.close();

}



void
Obstacle::AddWallwithWindow(Vector wallSize, Vector wallCenter, Vector windowCenter, double windowLength, double windowWidth)
{
  // surpport the wall with a small opening, and the wall is paralleled with room's width
  // divide the wall with window as for big obstacles
  double wallThickness = wallSize.x;
  Box obs1 = Box (wallCenter.x - wallThickness/2.0, wallCenter.x + wallThickness/2.0, windowCenter.y + windowLength/2.0, wallSize.y, 0, wallSize.z);
  Box obs2 = Box (wallCenter.x - wallThickness/2.0, wallCenter.x + wallThickness/2.0, 0, windowCenter.y - windowLength/2.0, 0, wallSize.z);
  Box obs3 = Box (wallCenter.x - wallThickness/2.0, wallCenter.x + wallThickness/2.0, windowCenter.y - windowLength/2.0, windowCenter.y + windowLength/2.0, 0, windowCenter.z - windowWidth/2.0);
  Box obs4 = Box (wallCenter.x - wallThickness/2.0, wallCenter.x + wallThickness/2.0, windowCenter.y - windowLength/2.0, windowCenter.y + windowLength/2.0, windowCenter.z + windowWidth/2.0, wallSize.z);
  
  m_obstacleDimension.insert(m_obstacleDimension.begin() + m_obstalceNumber, obs1);
  m_obstacleDimension.insert(m_obstacleDimension.begin() + (m_obstalceNumber + 1), obs2);
  m_obstacleDimension.insert(m_obstacleDimension.begin() + (m_obstalceNumber + 2), obs3);
  m_obstacleDimension.insert(m_obstacleDimension.begin() + (m_obstalceNumber + 3), obs4);
  // m_obstacleDimension.push_back(obs1);
  // m_obstacleDimension.push_back(obs2);
  // m_obstacleDimension.push_back(obs3);
  // m_obstacleDimension.push_back(obs4);


  // Allocate penetration loss for the wall
  double penLossWall = 1470; // 1470 dB/m for brick wall
  for(uint16_t j=m_obstalceNumber; j<(m_obstalceNumber+4); j++)
    { 
       m_obstaclePenetrationLoss.insert(m_obstaclePenetrationLoss.begin() + m_obstalceNumber, penLossWall);
	   m_obstaclePenetrationLoss.insert(m_obstaclePenetrationLoss.begin() + (m_obstalceNumber + 1), penLossWall);
	   m_obstaclePenetrationLoss.insert(m_obstaclePenetrationLoss.begin() + (m_obstalceNumber + 2), penLossWall);
	   m_obstaclePenetrationLoss.insert(m_obstaclePenetrationLoss.begin() + (m_obstalceNumber + 3), penLossWall);
    }

  // adjust the (fixed) obstacle number
  m_obstalceNumber += 4;
    
}



// add by Yuchen 2019
void
Obstacle::AllocateObstacle_MultiAP(std::vector<Box> railLocation, Vector roomSize, uint16_t clientRS)
{
  NS_LOG_FUNCTION (this);
  m_roomSize = roomSize;
  std::vector<Box> m_obstacleDimensionComparison;  
  uint16_t granularity = 10;

  //--- GSL random init ---
  gsl_rng_env_setup();                          // Read variable environnement
  const gsl_rng_type* type = gsl_rng_default;   // Default algorithm 'twister'
  gsl_rng *gen = gsl_rng_alloc (type);          // Rand generator allocation
  gsl_rng_set(gen, 2);

  // Poisson point process obstacle number identification
  double lambda = m_obstalceNumber/(m_roomSize.x*m_roomSize.y*granularity*granularity);
  double poissonProb = exp (-lambda)*lambda;
  uint16_t obstalceNoPlacement = 0;
  for (uint16_t t=0; t<m_roomSize.x*m_roomSize.y*granularity*granularity; t++)
     {
       RngSeedManager::SetSeed (2);
       Ptr<UniformRandomVariable> obstacleNoUV = CreateObject<UniformRandomVariable> ();
       obstacleNoUV->SetAttribute ("Min", DoubleValue (0.0));
       obstacleNoUV->SetAttribute ("Max", DoubleValue (1.0));
       double probability = obstacleNoUV->GetValue ();

       // Place obstacle based on Poisson point process 
       if (probability < poissonProb)
         {
           double x = (double) (t % (uint16_t)(m_roomSize.x * granularity))/granularity + 0.05;
           double y = floor (t/(m_roomSize.x * granularity))/granularity + 0.05; 

           // Define size of obstacle
           std::pair<double, double> obstacleXRange, obstacleYRange, obstacleZRange;
           std::vector<double> tempXDistConf = m_xObsTNDistConf;
           std::vector<double> tempYDistConf = m_yObsTNDistConf;
           if (m_roomSize.x - x < tempXDistConf.at(1) * 0.5 || x < tempXDistConf.at(1) * 0.5)
             tempXDistConf.at(1) = (m_roomSize.x - x > x)?x*2:2*(m_roomSize.x-x);
           if (m_roomSize.y - y < tempYDistConf.at(1) * 0.5 || y < tempYDistConf.at(1) * 0.5)
             tempYDistConf.at(1) = (m_roomSize.y - y > y)?y*2:2*(m_roomSize.y-y);
//           NS_LOG_FUNCTION (this << tempXDistConf.at(1) << tempXDistConf.at(0) << tempYDistConf.at(1));
           if (tempXDistConf.at(1) <= tempXDistConf.at(0) || tempYDistConf.at(1) <= tempYDistConf.at(0))
             continue;

           obstacleXRange = m_tNDist->rtnorm(gen,tempXDistConf.at(0),tempXDistConf.at(1),tempXDistConf.at(2),tempXDistConf.at(3));
           obstacleYRange = m_tNDist->rtnorm(gen,tempYDistConf.at(0),tempYDistConf.at(1),tempYDistConf.at(2),tempYDistConf.at(3));
           obstacleZRange = m_tNDist->rtnorm(gen,m_zObsTNDistConf.at(0),m_zObsTNDistConf.at(1),m_zObsTNDistConf.at(2),m_zObsTNDistConf.at(3));

           NS_LOG_FUNCTION (this << obstacleXRange.first << obstacleYRange.first << m_obstalceNumber);
           m_obstacleDimension.push_back(Box (x-obstacleXRange.first*0.5, x+obstacleXRange.first*0.5, y-obstacleYRange.first*0.5, y+obstacleYRange.first*0.5, 0, obstacleZRange.first));

           // collision detection
           for (uint16_t na=0; na<railLocation.size(); na++)
           	{
               m_obstacleDimensionComparison.push_back(railLocation.at(na));
       	    }
		   
           m_obstacleDimensionComparison.push_back(m_obstacleDimension.at(obstalceNoPlacement));

           m_obstacleDimensionComparison.push_back(m_obstacleDimension.at(obstalceNoPlacement));
           obstalceNoPlacement++;
         } 
     }
  m_obstalceNumber = obstalceNoPlacement;

  // Change obstacle location based on Poisson distribution
  for (uint16_t i=0; i<m_obstalceNumber*0.1; i++)
    {
      Ptr<UniformRandomVariable> obstacleIdUV = CreateObject<UniformRandomVariable> ();
      obstacleIdUV->SetAttribute ("Min", DoubleValue (0.0));
      obstacleIdUV->SetAttribute ("Max", DoubleValue (m_obstalceNumber-0.01));
      uint16_t obstacleId = floor(obstacleIdUV->GetValue ());  

      std::default_random_engine generator;
      std::poisson_distribution<int> distribution(0.1);
      for (uint16_t j=0; j<clientRS/10; j++)
        {
          uint16_t movingFlag = distribution(generator);
          //std::cout << movingFlag << std::endl;
          Ptr<UniformRandomVariable> movingAxisUV = CreateObject<UniformRandomVariable> ();
          movingAxisUV->SetAttribute ("Min", DoubleValue (0.0));
          movingAxisUV->SetAttribute ("Max", DoubleValue (PI*2));
          uint16_t movingAngle = movingAxisUV->GetValue ();

          if (movingFlag > 0)
            {
              if (m_obstacleDimension.at(obstacleId).xMax+0.1*cos(movingAngle)<m_roomSize.x && m_obstacleDimension.at(obstacleId).xMin+0.1*cos(movingAngle)>0)
                {
                  m_obstacleDimension.at(obstacleId).xMin += 0.1*cos(movingAngle);
                  m_obstacleDimension.at(obstacleId).xMax += 0.1*cos(movingAngle);
                }
              if (m_obstacleDimension.at(obstacleId).yMax+0.1*sin(movingAngle)<m_roomSize.y && m_obstacleDimension.at(obstacleId).yMin+0.1*cos(movingAngle)>0)
                {
                  m_obstacleDimension.at(obstacleId).yMin += 0.1*sin(movingAngle);
                  m_obstacleDimension.at(obstacleId).yMax += 0.1*sin(movingAngle);               
                } 
            }
        }
    }

  std::ofstream ofs;
  ofs.open ("obstacleList.txt", std::ofstream::out);
  ofs << m_obstalceNumber << std::endl;

  // Allocate obstacle type for penetration analysis
  for(uint16_t j=0; j<m_obstalceNumber; j++)
    { 
       ofs << m_obstacleDimension.at(j) << std::endl;
       NS_LOG_FUNCTION (this << m_obstalceNumber << m_obstacleDimension.at(j));
       Ptr<UniformRandomVariable> obstacleIndex = CreateObject<UniformRandomVariable> ();
       obstacleIndex->SetAttribute ("Min", DoubleValue (0.0));
       obstacleIndex->SetAttribute ("Max", DoubleValue (9.99));
       int index = floor(obstacleIndex->GetValue ());
       m_obstaclePenetrationLoss.push_back (m_obstaclePenetrationLoss.at(index));
    }
  ofs.close();
}


/*
// add by Yuchen
std::vector<Vector>
Obstacle::FindServingAP(std::vector<Vector> apLocation, std::vector<Vector> clientLocation, Vector apDimension)
{
  std::vector<Vector> servedAP;
  
  // do LOS analysis for each STA with all APs
  for (uint16_t clientId = 0; clientId < clientLocation.size(); clientId++)
    {
      bool channelStatus = LINE_OF_SIGHT;
      double fadingLoss = 1e7;

	  std::vector<uint16_t> LOSap;
	  std::vector<double> fadingLossAP;

	  // each AP
	  for (uint16_t apId = 0; apId < apLocation.size(); apId++)
	  {

      // Identify antenna model dimension
      Box antennaSize = Box {apLocation.at(apId).x-apDimension.x*0.5, apLocation.at(apId).x+apDimension.x*0.5, apLocation.at(apId).y-apDimension.y*0.5, apLocation.at(apId).y+apDimension.y*0.5, apLocation.at(apId).z-apDimension.z*0.5, apLocation.at(apId).z+apDimension.z*0.5};

      std::vector<Vector> apAntennaEdge = {{antennaSize.xMin, antennaSize.yMin, antennaSize.zMin}, {antennaSize.xMin, antennaSize.yMin, antennaSize.zMax}, {antennaSize.xMin, antennaSize.yMax, antennaSize.zMin}, {antennaSize.xMin, antennaSize.yMax, antennaSize.zMax}, {antennaSize.xMax, antennaSize.yMin, antennaSize.zMin}, {antennaSize.xMax, antennaSize.yMin, antennaSize.zMax}, {antennaSize.xMax, antennaSize.yMax, antennaSize.zMin}, {antennaSize.xMax, antennaSize.yMax, antennaSize.zMax}};

      Vector clientAntenna = clientLocation.at(clientId);

      // LoS analysis
      for (uint16_t i=0; i<apAntennaEdge.size(); i++)
        {
          double tempFadingLoss = 0;
          bool tempChannelStatus = LINE_OF_SIGHT;
          for (uint16_t j=0; j<m_obstalceNumber; j++) 
            {
              bool hitFlag = false;
              m_hitList.clear();
              Vector Hit = Vector (0, 0, 0);
              Vector B1 = Vector (m_obstacleDimension.at(j).xMin, m_obstacleDimension.at(j).yMin, m_obstacleDimension.at(j).zMin);
              Vector B2 = Vector (m_obstacleDimension.at(j).xMax, m_obstacleDimension.at(j).yMax, m_obstacleDimension.at(j).zMax);
              if (clientAntenna.x < m_obstacleDimension.at(j).xMin && apAntennaEdge.at(i).x < m_obstacleDimension.at(j).xMin) continue;
              if (clientAntenna.x > m_obstacleDimension.at(j).xMax && apAntennaEdge.at(i).x > m_obstacleDimension.at(j).xMax) continue;
              if (clientAntenna.y < m_obstacleDimension.at(j).yMin && apAntennaEdge.at(i).y < m_obstacleDimension.at(j).yMin) continue;
              if (clientAntenna.y > m_obstacleDimension.at(j).yMax && apAntennaEdge.at(i).y > m_obstacleDimension.at(j).yMax) continue;
              if (clientAntenna.z < m_obstacleDimension.at(j).zMin && apAntennaEdge.at(i).z < m_obstacleDimension.at(j).zMin) continue;
              if (clientAntenna.z > m_obstacleDimension.at(j).zMax && apAntennaEdge.at(i).z > m_obstacleDimension.at(j).zMax) continue;
              bool insideFlag = 0;
              if (clientAntenna.x >= m_obstacleDimension.at(j).xMin && clientAntenna.x <= m_obstacleDimension.at(j).xMax && clientAntenna.y >= m_obstacleDimension.at(j).yMin && clientAntenna.y <= m_obstacleDimension.at(j).yMax && clientAntenna.z >= m_obstacleDimension.at(j).zMin && clientAntenna.z <= m_obstacleDimension.at(j).zMax) 
                {
                  insideFlag = true;
                  m_hitList.push_back (clientAntenna);
                }              

              bool x1 = GetIntersection(apAntennaEdge.at(i).x-B1.x, clientAntenna.x-B1.x, apAntennaEdge.at(i), clientAntenna, Hit) && InBox(Hit, B1, B2, 1);
              bool x2 = GetIntersection(apAntennaEdge.at(i).x-B2.x, clientAntenna.x-B2.x, apAntennaEdge.at(i), clientAntenna, Hit) && InBox(Hit, B1, B2, 1);
              bool y1 = GetIntersection(apAntennaEdge.at(i).y-B1.y, clientAntenna.y-B1.y, apAntennaEdge.at(i), clientAntenna, Hit) && InBox(Hit, B1, B2, 2);
              bool y2 = GetIntersection(apAntennaEdge.at(i).y-B2.y, clientAntenna.y-B2.y, apAntennaEdge.at(i), clientAntenna, Hit) && InBox(Hit, B1, B2, 2);
              bool z1 = GetIntersection(apAntennaEdge.at(i).z-B1.z, clientAntenna.z-B1.z, apAntennaEdge.at(i), clientAntenna, Hit) && InBox(Hit, B1, B2, 3);
              bool z2 = GetIntersection(apAntennaEdge.at(i).z-B2.z, clientAntenna.z-B2.z, apAntennaEdge.at(i), clientAntenna, Hit) && InBox(Hit, B1, B2, 3);
              NS_LOG_FUNCTION (clientAntenna << j << x1 << x2 << y1 << y2 << z1 << z2);

              if (x1 + y1 + z1 + x2 + y2 + z2 + insideFlag > 1)
                {
                  hitFlag = true;
	              tempChannelStatus = NON_LINE_OF_SIGHT;
                }

              if (hitFlag == true)
                {
                  tempFadingLoss += CalculateDistance(m_hitList.at(0), m_hitList.at(1))*m_obstaclePenetrationLoss.at(j);
                } 
            }
          fadingLoss = (fadingLoss <= tempFadingLoss)?fadingLoss:tempFadingLoss;
          if (tempChannelStatus == LINE_OF_SIGHT) 
            channelStatus = LINE_OF_SIGHT;
		  
        }

      if (fadingLoss > 0)
        channelStatus = NON_LINE_OF_SIGHT;

	  if (channelStatus == LINE_OF_SIGHT)
	  {
        LOSap.push_back(apId);
	  }

	  // record fadingloss between all APs and this client
	  fadingLossAP.push_back(fadingLoss);
	  
	  }
	  

      // AP selection strategy for this client
	  if (LOSap.empty()== 0) // LOS
	  {
	    // First, record as LOS
	    channelStatus == LINE_OF_SIGHT;

		std::vector<double> distance3D;
		double dis;
		
		// Then, start to find min-distance AP as the serving AP
        for (uint16_t cap=0; cap<LOSap.size(); cap++)
        {
		   dis = CalculateDistance(apLocation.at(LOSap.at(cap)), clientLocation.at(clientId));

		   distance3D.push_back(dis);
        }

		std::vector<double>::iterator smallest = std::min_element(std::begin(distance3D), std::end(distance3D));
        uint16_t selectedAPId = std::distance(std::begin(distance3D), smallest);

		servedAP.push_back(apLocation.at(LOSap.at(selectedAPId)));

		distance3D.clear();
		
	  }
	  else
	  {
	    channelStatus = NON_LINE_OF_SIGHT;

		std::vector<double> distance3D;
		double dis;
		
		// No LOS APs, start to find min-distance AP as the "virtual serving" AP
        for (uint16_t cap=0; cap<apLocation.size(); cap++)
        {
		   dis = CalculateDistance(apLocation.at(cap), clientLocation.at(clientId));

		   distance3D.push_back(dis);
        }

		std::vector<double>::iterator smallest = std::min_element(std::begin(distance3D), std::end(distance3D));
        uint16_t selectedAPId = std::distance(std::begin(distance3D), smallest);

		servedAP.push_back(apLocation.at(selectedAPId));

		distance3D.clear();
		
	  }
	  
	  LOSap.clear();

	  // add multi-path effect for this client
      bool multipathFlag = true;
      double multipathFading;
	  
      // redirect fadingloss between served ap and this client
      fadingLoss=fadingLossAP.at(servedAP.at(clientId));
	  
      if (multipathFlag)
        {
          std::default_random_engine generator (m_clientRS);
          std::normal_distribution<double> multipathDistribution(0,2.24);
          multipathFading = multipathDistribution(generator);
          fadingLoss = fadingLoss - multipathFading;
        } 

	  // record the LOS status and fadingloss for this client
      m_channelInfo.push_back(ChannelInfo());
      m_channelInfo.at(clientId).losFlag = channelStatus;
      m_channelInfo.at(clientId).fadingLoss = -fadingLoss;
      m_channelInfo.at(clientId).apPos = servedAP.at(clientId);
      m_channelInfo.at(clientId).clientPos = clientLocation.at(clientId);
	  m_channelInfo.at(clientId).servedAPId = clientLocation.at(clientId);

	  fadingLossAP.clear();
        
      //channelStats.first.push_back (channelStatus);
      //channelStats.second.push_back(-fadingLoss);

      //std::cout<< channelStatus << " " << fadingLoss << " " << apLocation << " " << m_channelInfo.at(clientId).clientPos << " " << clientId << std::endl; 
      NS_LOG_FUNCTION ("los nlos status" << channelStatus << fadingLoss);

    }

  return servedAP;
}
*/


// determine the optimal locations of APs
std::vector<Vector> 
Obstacle::AllocateOptAP (Vector roomSize,  uint16_t numAPs)
{
  	  std::vector<Vector> apPosVec;

      double rl;
      double rw;
      if (roomSize.x >= roomSize.y)
      	{
      	   rl = roomSize.x;
      	   rw = roomSize.y;
      	}
	  else
	  	{
	  	   rw = roomSize.x;
      	   rl = roomSize.y;
	  	}
	  double z = roomSize.z;

	  uint16_t i = numAPs;

	  // i is the number of AP
	  if (i<=1)
	  	{
	  	  apPosVec.push_back(Vector (rl/2, rw/2, z)); 
	  	}
	  else if (i==2)
	  	{
		  apPosVec.push_back(Vector (rl/4, rw/2, z)); // 1st AP
		  apPosVec.push_back(Vector (3*rl/4, rw/2, z)); // 2nd AP
	  	}
	  else if (i==3)
	  	{
	  	  if (rl/rw <= 1.5)
	  	  	{
		  	  apPosVec.push_back(Vector (rl/6, rw/2, z));
		      apPosVec.push_back(Vector (2*rl/3, rw/4, z));
		      apPosVec.push_back(Vector (2*rl/3, 3*rw/4, z));
	  	  	}
		  else
		  	{
		  	  apPosVec.push_back(Vector (rl/6, rw/2, z));
		      apPosVec.push_back(Vector (rl/2, rw/2, z));
		      apPosVec.push_back(Vector (5*rl/6, rw/2, z));
		  	}
	  	}
	  else if (i==4)
	  	{
		  if (rl/rw <= 1.925209)
		  	{
		  	  apPosVec.push_back(Vector (rl/4, rw/4, z));
		      apPosVec.push_back(Vector (rl/4, 3*rw/4, z));
		      apPosVec.push_back(Vector (3*rl/4, rw/4, z));
		      apPosVec.push_back(Vector (3*rl/4, 3*rw/4, z));
		  	}
		  else if ((rl/rw > 1.925209)&& (rl/rw < 2.309401))
		  	{
		  	  double Kx1 = std::sqrt(std::pow(rw,2)/36*std::pow((2*std::sqrt(std::pow(rl/rw,2)+3)-rl/rw),2)-std::pow(rw,2)/4);
              apPosVec.push_back(Vector (Kx1, rw/2, z));
		      apPosVec.push_back(Vector (rl/2, 0, z));
		      apPosVec.push_back(Vector (rl/2, rw, z));
		      apPosVec.push_back(Vector (rl-Kx1, rw/2, z));
		    }
		  else
		  	{
		  	  apPosVec.push_back(Vector (rl/8, rw/2, z));
		      apPosVec.push_back(Vector (3*rl/8, rw/2, z));
		      apPosVec.push_back(Vector (5*rl/8, rw/2, z));
		      apPosVec.push_back(Vector (7*rl/8, rw/2, z));
		  	}
	  	}
	  else // here only supports linear arrangement if i >= 5
	  	{
	  	  for (int k = 1; k <= i; ++k)
	  	  	{
		      apPosVec.push_back(Vector ((2*k-1)*rl/(2*i), rw/2, z));
	  	  	}
	  	}

	  // for the case where roomSize.x < roomSize.y, exchange AP's x and y coordinates
	  if (roomSize.x < roomSize.y)
	  	{
	  	  for (int k = 0; k < numAPs; ++k)
	  	  	{
	  	  	  double temp = apPosVec.at(k).x;
			  apPosVec.at(k).x = apPosVec.at(k).y;
			  apPosVec.at(k).y = temp;
	  	  	}
	  	}
	  

	  return apPosVec;      
}

std::vector<Vector> 
Obstacle::multiRoomConfig(std::vector<Box> multiRooms, Vector buildingSize, uint16_t numRooms)
{
  std::vector<Vector> multiRoomSize;
  for (uint16_t i = 0; i < numRooms; ++i)
  	{
  	  Vector roomSize (multiRooms.at(i).xMax-multiRooms.at(i).xMin, multiRooms.at(i).yMax-multiRooms.at(i).yMin, multiRooms.at(i).zMax-multiRooms.at(i).zMin);
	  multiRoomSize.push_back(roomSize);
  	}
  return multiRoomSize;
}


void
Obstacle::LoSAnalysis (Vector apLocation, std::vector<Vector> clientLocation, Vector apDimension)
{
  NS_LOG_FUNCTION (this << m_obstalceNumber);
  //std::pair<std::vector<bool>, std::vector<double>> channelStats;
  uint16_t size_mChannel = m_channelInfo.size();
  for (uint16_t clientId = 0; clientId < clientLocation.size(); clientId++)
    {
      bool channelStatus = LINE_OF_SIGHT;
      double fadingLoss = 1e7;

	  if (m_obstalceNumber > 0)
	  {

      // Identify antenna model dimension
      Box antennaSize = Box {apLocation.x-apDimension.x*0.5, apLocation.x+apDimension.x*0.5, apLocation.y-apDimension.y*0.5, apLocation.y+apDimension.y*0.5, apLocation.z-apDimension.z*0.5, apLocation.z+apDimension.z*0.5};

      std::vector<Vector> apAntennaEdge = {{antennaSize.xMin, antennaSize.yMin, antennaSize.zMin}, {antennaSize.xMin, antennaSize.yMin, antennaSize.zMax}, {antennaSize.xMin, antennaSize.yMax, antennaSize.zMin}, {antennaSize.xMin, antennaSize.yMax, antennaSize.zMax}, {antennaSize.xMax, antennaSize.yMin, antennaSize.zMin}, {antennaSize.xMax, antennaSize.yMin, antennaSize.zMax}, {antennaSize.xMax, antennaSize.yMax, antennaSize.zMin}, {antennaSize.xMax, antennaSize.yMax, antennaSize.zMax}};

      Vector clientAntenna = clientLocation.at(clientId);

      // LoS analysis
      for (uint16_t i=0; i<apAntennaEdge.size(); i++)
        {
          double tempFadingLoss = 0;
          bool tempChannelStatus = LINE_OF_SIGHT;
          for (uint16_t j=0; j<(m_obstalceNumber + m_obstalceNumber_human); j++) 
            {
              bool hitFlag = false;
              m_hitList.clear();
              Vector Hit = Vector (0, 0, 0);
              Vector B1 = Vector (m_obstacleDimension.at(j).xMin, m_obstacleDimension.at(j).yMin, m_obstacleDimension.at(j).zMin);
              Vector B2 = Vector (m_obstacleDimension.at(j).xMax, m_obstacleDimension.at(j).yMax, m_obstacleDimension.at(j).zMax);
              if (clientAntenna.x < m_obstacleDimension.at(j).xMin && apAntennaEdge.at(i).x < m_obstacleDimension.at(j).xMin) continue;
              if (clientAntenna.x > m_obstacleDimension.at(j).xMax && apAntennaEdge.at(i).x > m_obstacleDimension.at(j).xMax) continue;
              if (clientAntenna.y < m_obstacleDimension.at(j).yMin && apAntennaEdge.at(i).y < m_obstacleDimension.at(j).yMin) continue;
              if (clientAntenna.y > m_obstacleDimension.at(j).yMax && apAntennaEdge.at(i).y > m_obstacleDimension.at(j).yMax) continue;
              if (clientAntenna.z < m_obstacleDimension.at(j).zMin && apAntennaEdge.at(i).z < m_obstacleDimension.at(j).zMin) continue;
              if (clientAntenna.z > m_obstacleDimension.at(j).zMax && apAntennaEdge.at(i).z > m_obstacleDimension.at(j).zMax) continue;
              bool insideFlag = 0;
              if (clientAntenna.x >= m_obstacleDimension.at(j).xMin && clientAntenna.x <= m_obstacleDimension.at(j).xMax && clientAntenna.y >= m_obstacleDimension.at(j).yMin && clientAntenna.y <= m_obstacleDimension.at(j).yMax && clientAntenna.z >= m_obstacleDimension.at(j).zMin && clientAntenna.z <= m_obstacleDimension.at(j).zMax) 
                {
                  insideFlag = true;
                  m_hitList.push_back (clientAntenna);
                }              

              bool x1 = GetIntersection(apAntennaEdge.at(i).x-B1.x, clientAntenna.x-B1.x, apAntennaEdge.at(i), clientAntenna, Hit) && InBox(Hit, B1, B2, 1);
              bool x2 = GetIntersection(apAntennaEdge.at(i).x-B2.x, clientAntenna.x-B2.x, apAntennaEdge.at(i), clientAntenna, Hit) && InBox(Hit, B1, B2, 1);
              bool y1 = GetIntersection(apAntennaEdge.at(i).y-B1.y, clientAntenna.y-B1.y, apAntennaEdge.at(i), clientAntenna, Hit) && InBox(Hit, B1, B2, 2);
              bool y2 = GetIntersection(apAntennaEdge.at(i).y-B2.y, clientAntenna.y-B2.y, apAntennaEdge.at(i), clientAntenna, Hit) && InBox(Hit, B1, B2, 2);
              bool z1 = GetIntersection(apAntennaEdge.at(i).z-B1.z, clientAntenna.z-B1.z, apAntennaEdge.at(i), clientAntenna, Hit) && InBox(Hit, B1, B2, 3);
              bool z2 = GetIntersection(apAntennaEdge.at(i).z-B2.z, clientAntenna.z-B2.z, apAntennaEdge.at(i), clientAntenna, Hit) && InBox(Hit, B1, B2, 3);
              NS_LOG_FUNCTION (clientAntenna << j << x1 << x2 << y1 << y2 << z1 << z2);

              if (x1 + y1 + z1 + x2 + y2 + z2 + insideFlag > 1)
                {
                  hitFlag = true;
	              tempChannelStatus = NON_LINE_OF_SIGHT;
                }

              if (hitFlag == true)
                {
                  // furniture obstacles
                  if (j < m_obstalceNumber)
                  	{
                  	  tempFadingLoss += CalculateDistance(m_hitList.at(0), m_hitList.at(1))*m_obstaclePenetrationLoss.at(j+10); // fix this bug by Yuchen 7/2020
                  	}
				  else
				  	{
				  	  tempFadingLoss += m_obstaclePenetrationLoss.at(j+10);
				  	}
                  
                } 
            }
          fadingLoss = (fadingLoss <= tempFadingLoss)?fadingLoss:tempFadingLoss;
          if (tempChannelStatus == LINE_OF_SIGHT) 
            channelStatus = LINE_OF_SIGHT;
        }

      if (fadingLoss > 0)
        channelStatus = NON_LINE_OF_SIGHT;

      bool multipathFlag = true;
      double multipathFading;
      if (multipathFlag)
        {
          std::default_random_engine generator (m_clientRS);
          std::normal_distribution<double> multipathDistribution(0,2.24);
          multipathFading = multipathDistribution(generator);
          fadingLoss = fadingLoss - multipathFading;
        } 

	  }

	  // for no-obstacle cases
	  if (m_obstalceNumber == 0)
	  {
	    	fadingLoss = 0;
	  }
	  
      m_channelInfo.push_back(ChannelInfo());
      m_channelInfo.at(clientId + size_mChannel).losFlag = channelStatus;
      m_channelInfo.at(clientId + size_mChannel).fadingLoss = -fadingLoss;
      m_channelInfo.at(clientId + size_mChannel).apPos = apLocation;
      m_channelInfo.at(clientId + size_mChannel).clientPos = clientLocation.at(clientId);
	  m_channelInfo.at(clientId + size_mChannel).beInfed = 0;
        
      //channelStats.first.push_back (channelStatus);
      //channelStats.second.push_back(-fadingLoss);

      //std::cout<< channelStatus << " " << fadingLoss << " " << apLocation << " " << m_channelInfo.at(clientId).clientPos << " " << clientId << std::endl; 
      NS_LOG_FUNCTION ("los nlos status" << channelStatus << fadingLoss);
    }
}


void 
Obstacle::LoSAnalysisMultiAPItf (std::vector<Vector> apLocationAll, std::vector<Vector> clientLocation, Vector apDimension, double HPBF)
{
    for (uint16_t clientId = 0; clientId < clientLocation.size(); clientId++)
    {
       // for served AP
       std::vector<uint16_t> sVAPid = m_channelInfo.at(clientId).servedAPId;
       for (uint16_t sId = 0; sId < sVAPid.size(); sId++)
       	{
       	   Vector apLocation = apLocationAll.at(sVAPid.at(sId));
		   bool channelStatus = LINE_OF_SIGHT;
           double fadingLoss = 1e7;

	       if (m_obstalceNumber > 0)
	       {

             // Identify antenna model dimension
      		 Box antennaSize = Box {apLocation.x-apDimension.x*0.5, apLocation.x+apDimension.x*0.5, apLocation.y-apDimension.y*0.5, apLocation.y+apDimension.y*0.5, apLocation.z-apDimension.z*0.5, apLocation.z+apDimension.z*0.5};

       		 std::vector<Vector> apAntennaEdge = {{antennaSize.xMin, antennaSize.yMin, antennaSize.zMin}, {antennaSize.xMin, antennaSize.yMin, antennaSize.zMax}, {antennaSize.xMin, antennaSize.yMax, antennaSize.zMin}, {antennaSize.xMin, antennaSize.yMax, antennaSize.zMax}, {antennaSize.xMax, antennaSize.yMin, antennaSize.zMin}, {antennaSize.xMax, antennaSize.yMin, antennaSize.zMax}, {antennaSize.xMax, antennaSize.yMax, antennaSize.zMin}, {antennaSize.xMax, antennaSize.yMax, antennaSize.zMax}};
	
      		 Vector clientAntenna = clientLocation.at(clientId);

      		 // LoS analysis
      	     for (uint16_t i=0; i<apAntennaEdge.size(); i++)
              {
          		double tempFadingLoss = 0;
          		bool tempChannelStatus = LINE_OF_SIGHT;
          		for (uint16_t j=0; j<(m_obstalceNumber + m_obstalceNumber_human); j++) 
            	{
              		bool hitFlag = false;
              		m_hitList.clear();
              		Vector Hit = Vector (0, 0, 0);
              		Vector B1 = Vector (m_obstacleDimension.at(j).xMin, m_obstacleDimension.at(j).yMin, m_obstacleDimension.at(j).zMin);
              		Vector B2 = Vector (m_obstacleDimension.at(j).xMax, m_obstacleDimension.at(j).yMax, m_obstacleDimension.at(j).zMax);
              		if (clientAntenna.x < m_obstacleDimension.at(j).xMin && apAntennaEdge.at(i).x < m_obstacleDimension.at(j).xMin) continue;
              		if (clientAntenna.x > m_obstacleDimension.at(j).xMax && apAntennaEdge.at(i).x > m_obstacleDimension.at(j).xMax) continue;
              		if (clientAntenna.y < m_obstacleDimension.at(j).yMin && apAntennaEdge.at(i).y < m_obstacleDimension.at(j).yMin) continue;
              		if (clientAntenna.y > m_obstacleDimension.at(j).yMax && apAntennaEdge.at(i).y > m_obstacleDimension.at(j).yMax) continue;
              		if (clientAntenna.z < m_obstacleDimension.at(j).zMin && apAntennaEdge.at(i).z < m_obstacleDimension.at(j).zMin) continue;
              		if (clientAntenna.z > m_obstacleDimension.at(j).zMax && apAntennaEdge.at(i).z > m_obstacleDimension.at(j).zMax) continue;
              		bool insideFlag = 0;
              		if (clientAntenna.x >= m_obstacleDimension.at(j).xMin && clientAntenna.x <= m_obstacleDimension.at(j).xMax && clientAntenna.y >= m_obstacleDimension.at(j).yMin && clientAntenna.y <= m_obstacleDimension.at(j).yMax && clientAntenna.z >= m_obstacleDimension.at(j).zMin && clientAntenna.z <= m_obstacleDimension.at(j).zMax) 
                	{
                  		insideFlag = true;
                  		m_hitList.push_back (clientAntenna);
                	}              

              		bool x1 = GetIntersection(apAntennaEdge.at(i).x-B1.x, clientAntenna.x-B1.x, apAntennaEdge.at(i), clientAntenna, Hit) && InBox(Hit, B1, B2, 1);
              		bool x2 = GetIntersection(apAntennaEdge.at(i).x-B2.x, clientAntenna.x-B2.x, apAntennaEdge.at(i), clientAntenna, Hit) && InBox(Hit, B1, B2, 1);
              		bool y1 = GetIntersection(apAntennaEdge.at(i).y-B1.y, clientAntenna.y-B1.y, apAntennaEdge.at(i), clientAntenna, Hit) && InBox(Hit, B1, B2, 2);
              		bool y2 = GetIntersection(apAntennaEdge.at(i).y-B2.y, clientAntenna.y-B2.y, apAntennaEdge.at(i), clientAntenna, Hit) && InBox(Hit, B1, B2, 2);
              		bool z1 = GetIntersection(apAntennaEdge.at(i).z-B1.z, clientAntenna.z-B1.z, apAntennaEdge.at(i), clientAntenna, Hit) && InBox(Hit, B1, B2, 3);
              		bool z2 = GetIntersection(apAntennaEdge.at(i).z-B2.z, clientAntenna.z-B2.z, apAntennaEdge.at(i), clientAntenna, Hit) && InBox(Hit, B1, B2, 3);
              		NS_LOG_FUNCTION (clientAntenna << j << x1 << x2 << y1 << y2 << z1 << z2);

              		if (x1 + y1 + z1 + x2 + y2 + z2 + insideFlag > 1)
                	{
                  		hitFlag = true;
	              		tempChannelStatus = NON_LINE_OF_SIGHT;
                	}

              		if (hitFlag == true)
                	{
                  		// furniture obstacles
                  		if (j < m_obstalceNumber)
                  		{
                  	  		tempFadingLoss += CalculateDistance(m_hitList.at(0), m_hitList.at(1))*m_obstaclePenetrationLoss.at(j+10); // fix this bug by Yuchen 7/2020
                  		}
				  		else
				  		{
				  	  		tempFadingLoss += m_obstaclePenetrationLoss.at(j+10);
				  		}
                  
                	} 
            	}
          		fadingLoss = (fadingLoss <= tempFadingLoss)?fadingLoss:tempFadingLoss;
          		if (tempChannelStatus == LINE_OF_SIGHT) 
            		channelStatus = LINE_OF_SIGHT;
        	}

      		if (fadingLoss > 0)
        		channelStatus = NON_LINE_OF_SIGHT;

      		bool multipathFlag = true;
      		double multipathFading;
      		if (multipathFlag)
        	{
          		std::default_random_engine generator (m_clientRS);
          		std::normal_distribution<double> multipathDistribution(0,2.24);
          		multipathFading = multipathDistribution(generator);
          		fadingLoss = fadingLoss - multipathFading;
        	} 

	  	  }

	  	  // for no-obstacle cases
	      if (m_obstalceNumber == 0)
	      {
	    	 fadingLoss = 0;
	      }
	  
      	  // m_channelInfo.push_back(ChannelInfo());
          m_channelInfo.at(clientId).losFlag = channelStatus;
          m_channelInfo.at(clientId).fadingLoss = -fadingLoss;
          m_channelInfo.at(clientId).apPos = apLocation;
          m_channelInfo.at(clientId).clientPos = clientLocation.at(clientId);
	      m_channelInfo.at(clientId).beInfed = 0;
       	}
	   

	   // for non-served AP -- checking if the interference effect should be counted
	   for (uint16_t apId = 0; apId < apLocationAll.size(); apId++)
	   	{
       	    // uint16_t selfAPid = sVAPid.at(sId);
		    std::vector<uint16_t>::iterator it = std::find(sVAPid.begin(), sVAPid.end(), apId);
			if (it != sVAPid.end()) // is served AP
			{
			  continue;
			}
			
			// otherwise, this is a potentially interfered AP			
			for (uint16_t cId = 0; cId < clientLocation.size(); cId++)
			{
			  if (cId == clientId) // self
			  {
			  	continue;
			  }
			  // get its served AP
			  std::vector<uint16_t> sid = m_channelInfo.at(cId).servedAPId;
			  // check if this AP is the served AP of this client
			  std::vector<uint16_t>::iterator it2 = std::find(sid.begin(), sid.end(), apId);
			  if (it2 == sid.end()) // is not served AP
			  {
			    continue;
			  }
			  // otherwise, this is a potentially interfered AP <-> client link
			  // 1) check if it is LoS to its served client
			  // bool lf = m_channelInfo.at(cId).losFlag;
			  if (m_channelInfo.at(cId).losFlag == NON_LINE_OF_SIGHT) // non-LoS
			  {
			    continue;
			  }
			  // 2) check if it is LoS to current client
			  std::vector<Vector> apLocation_vec;
			  apLocation_vec.push_back(apLocationAll.at(apId));
			  std::vector<Vector> LoSAP = checkLoS (apLocation_vec, clientLocation.at(clientId), m_obstacleDimension, apDimension);
			  // also consider the uplink, i.e., source node is the client
			  std::vector<Vector> clientLocation_vecItf;
			  clientLocation_vecItf.push_back(clientLocation.at(cId));
			  std::vector<Vector> LoSclient = checkLoS (clientLocation_vecItf, clientLocation.at(clientId), m_obstacleDimension, apDimension);
			  if ((LoSAP.empty() == true) && (LoSclient.empty() == true))// non-LoS to current client
			  {
			    continue;
			  }
			  // 3) check if the intersection angle between two links is within the HPBW of the AP or that client
			  // assume client's using directional antenna as well
			  uint16_t apItfFlag = checkItfAngleWithinBW (apLocationAll.at(apId), clientLocation.at(clientId), clientLocation.at(cId), HPBF);
			  uint16_t clientItfFlag = checkItfAngleWithinBW (clientLocation.at(cId), clientLocation.at(clientId), apLocationAll.at(apId), HPBF);
              if ((apItfFlag == 1) || (clientItfFlag == 1)) // interfered
              {
                 m_channelInfo.at(clientId).beInfed += 1;
              }              			  
			}
		  
	   	}
    }
}



void 
Obstacle::LoSAnalysis_ext (Vector apLocation, std::vector<Vector> clientLocation, Vector apDimension, std::vector<Vector> windowCenterVec, std::vector<double> windowLengthVec, std::vector<double> windowWidthVec)
{
  NS_LOG_FUNCTION (this << m_obstalceNumber);
  //std::pair<std::vector<bool>, std::vector<double>> channelStats;
  
  uint16_t size_mChannel = m_channelInfo.size();
  for (uint16_t clientId = 0; clientId < clientLocation.size(); clientId++)
    {
      bool channelStatus = LINE_OF_SIGHT;
	  int16_t channelStatus_ext = LINE_OF_SIGHT;
      double fadingLoss = 1e7;
	  bool interfstatus = LINE_OF_SIGHT_noInf;
	  double penLoss = 0;
	  double distToOpen = -1;
	  Vector virtualAPPos = Vector(0, 0, 0);
	  uint16_t openingId = 0;

	  if (m_obstalceNumber > 0)
	  {

      // Identify antenna model dimension
      Box antennaSize = Box {apLocation.x-apDimension.x*0.5, apLocation.x+apDimension.x*0.5, apLocation.y-apDimension.y*0.5, apLocation.y+apDimension.y*0.5, apLocation.z-apDimension.z*0.5, apLocation.z+apDimension.z*0.5};

      std::vector<Vector> apAntennaEdge = {{antennaSize.xMin, antennaSize.yMin, antennaSize.zMin}, {antennaSize.xMin, antennaSize.yMin, antennaSize.zMax}, {antennaSize.xMin, antennaSize.yMax, antennaSize.zMin}, {antennaSize.xMin, antennaSize.yMax, antennaSize.zMax}, {antennaSize.xMax, antennaSize.yMin, antennaSize.zMin}, {antennaSize.xMax, antennaSize.yMin, antennaSize.zMax}, {antennaSize.xMax, antennaSize.yMax, antennaSize.zMin}, {antennaSize.xMax, antennaSize.yMax, antennaSize.zMax}};

      Vector clientAntenna = clientLocation.at(clientId);

      // LoS analysis
      for (uint16_t i=0; i<apAntennaEdge.size(); i++)
        {
          double tempFadingLoss = 0;
          bool tempChannelStatus = LINE_OF_SIGHT;
          for (uint16_t j=0; j<(m_obstalceNumber + m_obstalceNumber_human); j++) 
            {
              bool hitFlag = false;
              m_hitList.clear();
              Vector Hit = Vector (0, 0, 0);
              Vector B1 = Vector (m_obstacleDimension.at(j).xMin, m_obstacleDimension.at(j).yMin, m_obstacleDimension.at(j).zMin);
              Vector B2 = Vector (m_obstacleDimension.at(j).xMax, m_obstacleDimension.at(j).yMax, m_obstacleDimension.at(j).zMax);
              if (clientAntenna.x < m_obstacleDimension.at(j).xMin && apAntennaEdge.at(i).x < m_obstacleDimension.at(j).xMin) continue;
              if (clientAntenna.x > m_obstacleDimension.at(j).xMax && apAntennaEdge.at(i).x > m_obstacleDimension.at(j).xMax) continue;
              if (clientAntenna.y < m_obstacleDimension.at(j).yMin && apAntennaEdge.at(i).y < m_obstacleDimension.at(j).yMin) continue;
              if (clientAntenna.y > m_obstacleDimension.at(j).yMax && apAntennaEdge.at(i).y > m_obstacleDimension.at(j).yMax) continue;
              if (clientAntenna.z < m_obstacleDimension.at(j).zMin && apAntennaEdge.at(i).z < m_obstacleDimension.at(j).zMin) continue;
              if (clientAntenna.z > m_obstacleDimension.at(j).zMax && apAntennaEdge.at(i).z > m_obstacleDimension.at(j).zMax) continue;
              bool insideFlag = 0;
              if (clientAntenna.x >= m_obstacleDimension.at(j).xMin && clientAntenna.x <= m_obstacleDimension.at(j).xMax && clientAntenna.y >= m_obstacleDimension.at(j).yMin && clientAntenna.y <= m_obstacleDimension.at(j).yMax && clientAntenna.z >= m_obstacleDimension.at(j).zMin && clientAntenna.z <= m_obstacleDimension.at(j).zMax) 
                {
                  insideFlag = true;
                  m_hitList.push_back (clientAntenna);
                }              

              bool x1 = GetIntersection(apAntennaEdge.at(i).x-B1.x, clientAntenna.x-B1.x, apAntennaEdge.at(i), clientAntenna, Hit) && InBox(Hit, B1, B2, 1);
              bool x2 = GetIntersection(apAntennaEdge.at(i).x-B2.x, clientAntenna.x-B2.x, apAntennaEdge.at(i), clientAntenna, Hit) && InBox(Hit, B1, B2, 1);
              bool y1 = GetIntersection(apAntennaEdge.at(i).y-B1.y, clientAntenna.y-B1.y, apAntennaEdge.at(i), clientAntenna, Hit) && InBox(Hit, B1, B2, 2);
              bool y2 = GetIntersection(apAntennaEdge.at(i).y-B2.y, clientAntenna.y-B2.y, apAntennaEdge.at(i), clientAntenna, Hit) && InBox(Hit, B1, B2, 2);
              bool z1 = GetIntersection(apAntennaEdge.at(i).z-B1.z, clientAntenna.z-B1.z, apAntennaEdge.at(i), clientAntenna, Hit) && InBox(Hit, B1, B2, 3);
              bool z2 = GetIntersection(apAntennaEdge.at(i).z-B2.z, clientAntenna.z-B2.z, apAntennaEdge.at(i), clientAntenna, Hit) && InBox(Hit, B1, B2, 3);
              NS_LOG_FUNCTION (clientAntenna << j << x1 << x2 << y1 << y2 << z1 << z2);

              if (x1 + y1 + z1 + x2 + y2 + z2 + insideFlag > 1)
                {
                  hitFlag = true;
	              tempChannelStatus = NON_LINE_OF_SIGHT;
                }

              if (hitFlag == true)
                {
                  // furniture obstacles
                  if (j < m_obstalceNumber)
                  	{
                  	  tempFadingLoss += CalculateDistance(m_hitList.at(0), m_hitList.at(1))*m_obstaclePenetrationLoss.at(j+10); // fix this bug by Yuchen 7/2020
                  	}
				  else
				  	{
				  	  tempFadingLoss += m_obstaclePenetrationLoss.at(j+10);
				  	}
                  
                } 
            }
          fadingLoss = (fadingLoss <= tempFadingLoss)?fadingLoss:tempFadingLoss;
          if (tempChannelStatus == LINE_OF_SIGHT) 
          	{
              channelStatus = LINE_OF_SIGHT;
			  channelStatus_ext = LINE_OF_SIGHT;
          	}
		  
        }

      if (fadingLoss > 0)
      	{
          channelStatus = NON_LINE_OF_SIGHT;
		  channelStatus_ext = NON_LINE_OF_SIGHT;
      	}

      bool multipathFlag = true;
      double multipathFading;
      if (multipathFlag)
        {
          std::default_random_engine generator (m_clientRS);
          std::normal_distribution<double> multipathDistribution(0,2.24);
          multipathFading = multipathDistribution(generator);
          fadingLoss = fadingLoss - multipathFading;
        } 

	  }

	  // for no-obstacle cases
	  if (m_obstalceNumber == 0)
	  {
	    	fadingLoss = 0;
	  }

	  // Consider double directions (AP-user and user-AP)
	  // check if potentially interfer the user through the window (AP to user)
	  if (channelStatus == LINE_OF_SIGHT)
	  {
	    for (uint16_t ip = 0; ip < windowCenterVec.size(); ++ip)
	    {
	      Vector windowCenter = windowCenterVec.at(ip);
		  double windowLength = windowLengthVec.at(ip);
		  double windowWidth = windowWidthVec.at(ip);
		  
	      double window_xy_x1 = windowCenter.x;
          double window_xy_y1 = windowCenter.y + windowLength/2.0;
          double window_xy_x2 = windowCenter.x;
          double window_xy_y2 = windowCenter.y - windowLength/2.0;

          double window_xz_z1 = windowCenter.z + windowWidth/2.0;
          double window_xz_x1 = windowCenter.x;
          double window_xz_z2 = windowCenter.z - windowWidth/2.0;
          double window_xz_x2 = windowCenter.x;
  
	      Vector ap_client = Vector (clientLocation.at(clientId).x - apLocation.x, clientLocation.at(clientId).y - apLocation.y, clientLocation.at(clientId).z - apLocation.z);
	  	  // x-y plane
	  	  Vector xy_ap_w1 = Vector (window_xy_x1 - apLocation.x, window_xy_y1 - apLocation.y, 0);
		  Vector xy_ap_w2 = Vector (window_xy_x2 - apLocation.x, window_xy_y2 - apLocation.y, 0);

		  // std::cerr << ap_client.x << " " << ap_client.y << " " << ap_client.z << std::endl;
		  // std::cerr << xy_ap_w1.x << " " << xy_ap_w1.y << " " << xy_ap_w1.z << std::endl;
		  // std::cerr << xy_ap_w2.x << " " << xy_ap_w2.y << " " << xy_ap_w2.z << std::endl;

		  double w1_w2_dot = xy_ap_w1.x * xy_ap_w2.x + xy_ap_w1.y * xy_ap_w2.y;
		  double w1_mode = std::sqrt(xy_ap_w1.x * xy_ap_w1.x + xy_ap_w1.y * xy_ap_w1.y);
		  double w2_mode = std::sqrt(xy_ap_w2.x * xy_ap_w2.x + xy_ap_w2.y * xy_ap_w2.y);
	  	  double xy_angle_w1_w2 = std::acos(w1_w2_dot/(w1_mode*w2_mode));

		  double w1_client_dot = xy_ap_w1.x * ap_client.x + xy_ap_w1.y * ap_client.y;
		  double client_mode = std::sqrt(ap_client.x * ap_client.x + ap_client.y * ap_client.y);
	  	  double xy_angle_w1_client = std::acos(w1_client_dot/(w1_mode*client_mode));
		  
		  double w2_client_dot = xy_ap_w2.x * ap_client.x + xy_ap_w2.y * ap_client.y;
	  	  double xy_angle_w2_client = std::acos(w2_client_dot/(w2_mode*client_mode));

		  /*
		  // y-z plane
	  	  Vector yz_ap_w1 = Vector (0, window_yz_y1 - apLocation.y, window_yz_z1 - apLocation.z);
		  Vector yz_ap_w2 = Vector (0, window_yz_y2 - apLocation.y, window_yz_z2 - apLocation.z);

		  w1_w2_dot = yz_ap_w1.y * yz_ap_w2.y + yz_ap_w1.z * yz_ap_w2.z;
		  w1_mode = std::sqrt(yz_ap_w1.y * yz_ap_w1.y + yz_ap_w1.z + yz_ap_w1.z);
		  w2_mode = std::sqrt(yz_ap_w2.y * yz_ap_w2.y + yz_ap_w2.z + yz_ap_w2.z);
	  	  double yz_angle_w1_w2 = std::acos(w1_w2_dot/(w1_mode*w2_mode));

		  w1_client_dot = yz_ap_w1.y * ap_client.y + yz_ap_w1.z * ap_client.z;
		  client_mode = std::sqrt(ap_client.y * ap_client.y + ap_client.z + ap_client.z);
	  	  double yz_angle_w1_client = std::acos(w1_client_dot/(w1_mode*client_mode));
		  
		  w2_client_dot = yz_ap_w2.y * ap_client.y + yz_ap_w2.z * ap_client.z;
	  	  double yz_angle_w2_client = std::acos(w2_client_dot/(w2_mode*client_mode));

		  std::cerr << xy_angle_w1_client << " " << xy_angle_w2_client << " " << xy_angle_w1_w2 << std::endl;
		  std::cerr << yz_angle_w1_client << " " << yz_angle_w2_client << " " << yz_angle_w1_w2 << std::endl;
          
		  if ((xy_angle_w1_client <= xy_angle_w1_w2) && (xy_angle_w2_client <= xy_angle_w1_w2) && (yz_angle_w1_client <= yz_angle_w1_w2) && (yz_angle_w2_client <= yz_angle_w1_w2))
		  	{
		  	  interfstatus = LINE_OF_SIGHT_Inf;
			  channelStatus_ext = LINE_OF_SIGHT_PassWin;
			  std::cerr << "hit here" << std::endl;
		  	}
		  */
		  // x-z plane
	  	  Vector xz_ap_w1 = Vector (window_xz_x1 - apLocation.x, 0, window_xz_z1 - apLocation.z);
		  Vector xz_ap_w2 = Vector (window_xz_x2 - apLocation.x, 0, window_xz_z2 - apLocation.z);

		  w1_w2_dot = xz_ap_w1.x * xz_ap_w2.x + xz_ap_w1.z * xz_ap_w2.z;
		  w1_mode = std::sqrt(xz_ap_w1.x * xz_ap_w1.x + xz_ap_w1.z * xz_ap_w1.z);
		  w2_mode = std::sqrt(xz_ap_w2.x * xz_ap_w2.x + xz_ap_w2.z * xz_ap_w2.z);
	  	  double xz_angle_w1_w2 = std::acos(w1_w2_dot/(w1_mode*w2_mode));

		  w1_client_dot = xz_ap_w1.x * ap_client.x + xz_ap_w1.z * ap_client.z;
		  client_mode = std::sqrt(ap_client.x * ap_client.x + ap_client.z * ap_client.z);
	  	  double xz_angle_w1_client = std::acos(w1_client_dot/(w1_mode*client_mode));
		  
		  w2_client_dot = xz_ap_w2.x * ap_client.x + xz_ap_w2.z * ap_client.z;
	  	  double xz_angle_w2_client = std::acos(w2_client_dot/(w2_mode*client_mode));

		  // std::cerr << xy_angle_w1_client << " " << xy_angle_w2_client << " " << xy_angle_w1_w2 << std::endl;
		  // std::cerr << xz_angle_w1_client << " " << xz_angle_w2_client << " " << xz_angle_w1_w2 << std::endl;
          
		  if ((xy_angle_w1_client <= xy_angle_w1_w2) && (xy_angle_w2_client <= xy_angle_w1_w2) && (xz_angle_w1_client <= xz_angle_w1_w2) && (xz_angle_w2_client <= xz_angle_w1_w2))
		  	{
		  	  interfstatus = LINE_OF_SIGHT_Inf; // possible
			  channelStatus_ext = LINE_OF_SIGHT_PassWin;
			  // std::cerr << "hit here" << std::endl;

			  // if this is opening (assuming it is the opening case here ...)
			  penLoss = 0;

			  // calculate the 3D distance from AP to the opening/window
			  double ox = windowCenter.x;
			  // x-y plane
			  double oy = (apLocation.y - clientLocation.at(clientId).y)/(apLocation.x - clientLocation.at(clientId).x)*(ox - apLocation.x) + apLocation.y;
              // x-z plane
			  double oz = (apLocation.z - clientLocation.at(clientId).z)/(apLocation.x - clientLocation.at(clientId).x)*(ox - apLocation.x) + apLocation.z;
              Vector ap_open = Vector (apLocation.x - ox, apLocation.y - oy, apLocation.z - oz);
			  distToOpen = std::sqrt(ap_open.x * ap_open.x + ap_open.y * ap_open.y + ap_open.z * ap_open.z);		  
              virtualAPPos = Vector (ox, oy, oz);
			  openingId = ip + 1;

			  break; // break the for loop for all openings since signal will only pass through one of the openings
		    }
	    }
	  }

	  // check if potentially interfer the user through the window (user to AP)
	  if ((channelStatus == LINE_OF_SIGHT) && (channelStatus_ext != LINE_OF_SIGHT_PassWin))
	  {
	      // switch the loactions of AP and client for checking with the same logic
          Vector Location_temp = apLocation;
		  apLocation = clientLocation.at(clientId);
		  clientLocation.at(clientId) = Location_temp;

		for (uint16_t ip = 0; ip < windowCenterVec.size(); ++ip)
	    {
	      Vector windowCenter = windowCenterVec.at(ip);
		  double windowLength = windowLengthVec.at(ip);
		  double windowWidth = windowWidthVec.at(ip);
		  
	      double window_xy_x1 = windowCenter.x;
          double window_xy_y1 = windowCenter.y + windowLength/2.0;
          double window_xy_x2 = windowCenter.x;
          double window_xy_y2 = windowCenter.y - windowLength/2.0;

          double window_xz_z1 = windowCenter.z + windowWidth/2.0;
          double window_xz_x1 = windowCenter.x;
          double window_xz_z2 = windowCenter.z - windowWidth/2.0;
          double window_xz_x2 = windowCenter.x;
		  
	      Vector ap_client = Vector (clientLocation.at(clientId).x - apLocation.x, clientLocation.at(clientId).y - apLocation.y, clientLocation.at(clientId).z - apLocation.z);
	  	  // x-y plane
	  	  Vector xy_ap_w1 = Vector (window_xy_x1 - apLocation.x, window_xy_y1 - apLocation.y, 0);
		  Vector xy_ap_w2 = Vector (window_xy_x2 - apLocation.x, window_xy_y2 - apLocation.y, 0);

		  // std::cerr << ap_client.x << " " << ap_client.y << " " << ap_client.z << std::endl;
		  // std::cerr << xy_ap_w1.x << " " << xy_ap_w1.y << " " << xy_ap_w1.z << std::endl;
		  // std::cerr << xy_ap_w2.x << " " << xy_ap_w2.y << " " << xy_ap_w2.z << std::endl;

		  double w1_w2_dot = xy_ap_w1.x * xy_ap_w2.x + xy_ap_w1.y * xy_ap_w2.y;
		  double w1_mode = std::sqrt(xy_ap_w1.x * xy_ap_w1.x + xy_ap_w1.y * xy_ap_w1.y);
		  double w2_mode = std::sqrt(xy_ap_w2.x * xy_ap_w2.x + xy_ap_w2.y * xy_ap_w2.y);
	  	  double xy_angle_w1_w2 = std::acos(w1_w2_dot/(w1_mode*w2_mode));

		  double w1_client_dot = xy_ap_w1.x * ap_client.x + xy_ap_w1.y * ap_client.y;
		  double client_mode = std::sqrt(ap_client.x * ap_client.x + ap_client.y * ap_client.y);
	  	  double xy_angle_w1_client = std::acos(w1_client_dot/(w1_mode*client_mode));
		  
		  double w2_client_dot = xy_ap_w2.x * ap_client.x + xy_ap_w2.y * ap_client.y;
	  	  double xy_angle_w2_client = std::acos(w2_client_dot/(w2_mode*client_mode));

		  /*
		  // y-z plane
	  	  Vector yz_ap_w1 = Vector (0, window_yz_y1 - apLocation.y, window_yz_z1 - apLocation.z);
		  Vector yz_ap_w2 = Vector (0, window_yz_y2 - apLocation.y, window_yz_z2 - apLocation.z);

		  w1_w2_dot = yz_ap_w1.y * yz_ap_w2.y + yz_ap_w1.z * yz_ap_w2.z;
		  w1_mode = std::sqrt(yz_ap_w1.y * yz_ap_w1.y + yz_ap_w1.z + yz_ap_w1.z);
		  w2_mode = std::sqrt(yz_ap_w2.y * yz_ap_w2.y + yz_ap_w2.z + yz_ap_w2.z);
	  	  double yz_angle_w1_w2 = std::acos(w1_w2_dot/(w1_mode*w2_mode));

		  w1_client_dot = yz_ap_w1.y * ap_client.y + yz_ap_w1.z * ap_client.z;
		  client_mode = std::sqrt(ap_client.y * ap_client.y + ap_client.z + ap_client.z);
	  	  double yz_angle_w1_client = std::acos(w1_client_dot/(w1_mode*client_mode));
		  
		  w2_client_dot = yz_ap_w2.y * ap_client.y + yz_ap_w2.z * ap_client.z;
	  	  double yz_angle_w2_client = std::acos(w2_client_dot/(w2_mode*client_mode));

		  std::cerr << xy_angle_w1_client << " " << xy_angle_w2_client << " " << xy_angle_w1_w2 << std::endl;
		  std::cerr << yz_angle_w1_client << " " << yz_angle_w2_client << " " << yz_angle_w1_w2 << std::endl;
          
		  if ((xy_angle_w1_client <= xy_angle_w1_w2) && (xy_angle_w2_client <= xy_angle_w1_w2) && (yz_angle_w1_client <= yz_angle_w1_w2) && (yz_angle_w2_client <= yz_angle_w1_w2))
		  	{
		  	  interfstatus = LINE_OF_SIGHT_Inf;
			  channelStatus_ext = LINE_OF_SIGHT_PassWin;
			  std::cerr << "hit here" << std::endl;
		  	}
		  */
		  // x-z plane
	  	  Vector xz_ap_w1 = Vector (window_xz_x1 - apLocation.x, 0, window_xz_z1 - apLocation.z);
		  Vector xz_ap_w2 = Vector (window_xz_x2 - apLocation.x, 0, window_xz_z2 - apLocation.z);

		  w1_w2_dot = xz_ap_w1.x * xz_ap_w2.x + xz_ap_w1.z * xz_ap_w2.z;
		  w1_mode = std::sqrt(xz_ap_w1.x * xz_ap_w1.x + xz_ap_w1.z * xz_ap_w1.z);
		  w2_mode = std::sqrt(xz_ap_w2.x * xz_ap_w2.x + xz_ap_w2.z * xz_ap_w2.z);
	  	  double xz_angle_w1_w2 = std::acos(w1_w2_dot/(w1_mode*w2_mode));

		  w1_client_dot = xz_ap_w1.x * ap_client.x + xz_ap_w1.z * ap_client.z;
		  client_mode = std::sqrt(ap_client.x * ap_client.x + ap_client.z * ap_client.z);
	  	  double xz_angle_w1_client = std::acos(w1_client_dot/(w1_mode*client_mode));
		  
		  w2_client_dot = xz_ap_w2.x * ap_client.x + xz_ap_w2.z * ap_client.z;
	  	  double xz_angle_w2_client = std::acos(w2_client_dot/(w2_mode*client_mode));

		  // std::cerr << xy_angle_w1_client << " " << xy_angle_w2_client << " " << xy_angle_w1_w2 << std::endl;
		  // std::cerr << xz_angle_w1_client << " " << xz_angle_w2_client << " " << xz_angle_w1_w2 << std::endl;
          
		  if ((xy_angle_w1_client <= xy_angle_w1_w2) && (xy_angle_w2_client <= xy_angle_w1_w2) && (xz_angle_w1_client <= xz_angle_w1_w2) && (xz_angle_w2_client <= xz_angle_w1_w2))
		  	{
		  	  interfstatus = LINE_OF_SIGHT_Inf; // possible
			  channelStatus_ext = LINE_OF_SIGHT_PassWin;
			  // std::cerr << "hit here" << std::endl;

			  // if this is opening (assuming it is the opening case here ...)
			  penLoss = 0;

			  // calculate the 3D distance from AP to the opening/window
			  double ox = windowCenter.x;
			  // x-y plane
			  double oy = (apLocation.y - clientLocation.at(clientId).y)/(apLocation.x - clientLocation.at(clientId).x)*(ox - apLocation.x) + apLocation.y;
              // x-z plane
			  double oz = (apLocation.z - clientLocation.at(clientId).z)/(apLocation.x - clientLocation.at(clientId).x)*(ox - apLocation.x) + apLocation.z;
              Vector ap_open = Vector (apLocation.x - ox, apLocation.y - oy, apLocation.z - oz);
			  distToOpen = std::sqrt(ap_open.x * ap_open.x + ap_open.y * ap_open.y + ap_open.z * ap_open.z);
			  virtualAPPos = Vector (ox, oy, oz);
			  openingId = ip + 1;
			  
			  break; // break the for loop for all openings since signal will only pass through one of the openings
		    }
		}
		
		  // Get back: switch the loactions of AP and client
          Location_temp = apLocation;
		  apLocation = clientLocation.at(clientId);
		  clientLocation.at(clientId) = Location_temp;
	  }
	  
	  
      m_channelInfo.push_back(ChannelInfo());
      m_channelInfo.at(clientId + size_mChannel).losFlag = channelStatus;
      m_channelInfo.at(clientId + size_mChannel).fadingLoss = -fadingLoss;
      m_channelInfo.at(clientId + size_mChannel).apPos = apLocation;
      m_channelInfo.at(clientId + size_mChannel).clientPos = clientLocation.at(clientId);
	  m_channelInfo.at(clientId + size_mChannel).losInfFlag = interfstatus;
	  m_channelInfo.at(clientId + size_mChannel).losFlag_ext = channelStatus_ext;
	  m_channelInfo.at(clientId + size_mChannel).penLoss = penLoss;
	  m_channelInfo.at(clientId + size_mChannel).distToOpen = distToOpen;
	  m_channelInfo.at(clientId + size_mChannel).beInfed = 0;
	  m_channelInfo.at(clientId + size_mChannel).virtualAPPos = virtualAPPos;
	  m_channelInfo.at(clientId + size_mChannel).openingId = openingId;
        
      //channelStats.first.push_back (channelStatus);
      //channelStats.second.push_back(-fadingLoss);

      //std::cout<< channelStatus << " " << fadingLoss << " " << apLocation << " " << m_channelInfo.at(clientId).clientPos << " " << clientId << std::endl; 
      NS_LOG_FUNCTION ("los nlos status" << channelStatus << fadingLoss);
    }
}



void 
Obstacle::LoSAnalysis_BL (Vector apLocation, std::vector<Vector> clientLocation, Vector apDimension, 
                              std::vector<int16_t> areaID, std::vector<uint16_t> numAPs, uint16_t APId, 
                              std::vector<std::vector<Box> > obsDimRoom, std::vector<uint16_t> numObstacles_fixed,
                              std::vector<Ptr<Obstacle> > roomScenarios)
{
  // NS_LOG_FUNCTION (this << m_obstalceNumber);
  //std::pair<std::vector<bool>, std::vector<double>> channelStats;

  // check which roomID the current AP belongs to
  int16_t roomID_AP = GetRoomIDforAP(APId, numAPs);
  // this rooms config
  std::vector<Box> obstacleDimension;
  uint32_t obstalceNumber = 0;
  uint32_t obstalceNumber_fixed = 0;
  if (roomID_AP >= 0)
  	{
      // Vector roomSize = multiRoomSize.at(roomID_AP);
      obstacleDimension = obsDimRoom.at(roomID_AP);
      obstalceNumber = obstacleDimension.size(); // total obstacle number: furniture + human
      obstalceNumber_fixed = numObstacles_fixed.at(roomID_AP); // total obstacle number: furniture
  	}
  
  for (uint16_t clientId = 0; clientId < clientLocation.size(); clientId++)
    {
      bool channelStatus = LINE_OF_SIGHT;
      double fadingLoss = 1e7;
	
      int16_t roomID_client = areaID.at(clientId);  // this client in roomID's place	  

	  // if the client in the hall or other rooms
	  if ((roomID_client < 0) || (roomID_client != roomID_AP))
	    {
	  	  m_channelInfo.push_back(ChannelInfo());
          m_channelInfo.at(clientId).losFlag = NON_LINE_OF_SIGHT; // NLoS
          m_channelInfo.at(clientId).fadingLoss = -fadingLoss; // inf
          m_channelInfo.at(clientId).apPos = apLocation; // room-level position
          m_channelInfo.at(clientId).clientPos = clientLocation.at(clientId); // m_clientPos_BL.at(clientId);
          m_channelInfo.at(clientId).beInfed = 0;

		  //channelStats.first.push_back (channelStatus);
          //channelStats.second.push_back(-fadingLoss);

          //std::cout<< channelStatus << " " << fadingLoss << " " << apLocation << " " << m_channelInfo.at(clientId).clientPos << " " << clientId << std::endl; 
          NS_LOG_FUNCTION ("los nlos status" << channelStatus << fadingLoss);
	  	  continue;
	  	}


	  // if the client is inside this room where AP is located
	  std::vector<double> obstaclePenetrationLoss = roomScenarios.at(roomID_AP)->GetPenetrationLossMode();
	  if (obstalceNumber > 0)
	  {
        // Identify antenna model dimension
        Box antennaSize = Box {apLocation.x-apDimension.x*0.5, apLocation.x+apDimension.x*0.5, apLocation.y-apDimension.y*0.5, apLocation.y+apDimension.y*0.5, apLocation.z-apDimension.z*0.5, apLocation.z+apDimension.z*0.5};

        std::vector<Vector> apAntennaEdge = {{antennaSize.xMin, antennaSize.yMin, antennaSize.zMin}, {antennaSize.xMin, antennaSize.yMin, antennaSize.zMax}, {antennaSize.xMin, antennaSize.yMax, antennaSize.zMin}, {antennaSize.xMin, antennaSize.yMax, antennaSize.zMax}, {antennaSize.xMax, antennaSize.yMin, antennaSize.zMin}, {antennaSize.xMax, antennaSize.yMin, antennaSize.zMax}, {antennaSize.xMax, antennaSize.yMax, antennaSize.zMin}, {antennaSize.xMax, antennaSize.yMax, antennaSize.zMax}};

        Vector clientAntenna = clientLocation.at(clientId);

        // LoS analysis
        for (uint16_t i=0; i<apAntennaEdge.size(); i++)
        {
          double tempFadingLoss = 0;
          bool tempChannelStatus = LINE_OF_SIGHT;
          for (uint16_t j=0; j<obstalceNumber; j++) 
            {
              bool hitFlag = false;
              m_hitList.clear();
              Vector Hit = Vector (0, 0, 0);
              Vector B1 = Vector (obstacleDimension.at(j).xMin, obstacleDimension.at(j).yMin, obstacleDimension.at(j).zMin);
              Vector B2 = Vector (obstacleDimension.at(j).xMax, obstacleDimension.at(j).yMax, obstacleDimension.at(j).zMax);
              if (clientAntenna.x < obstacleDimension.at(j).xMin && apAntennaEdge.at(i).x < obstacleDimension.at(j).xMin) continue;
              if (clientAntenna.x > obstacleDimension.at(j).xMax && apAntennaEdge.at(i).x > obstacleDimension.at(j).xMax) continue;
              if (clientAntenna.y < obstacleDimension.at(j).yMin && apAntennaEdge.at(i).y < obstacleDimension.at(j).yMin) continue;
              if (clientAntenna.y > obstacleDimension.at(j).yMax && apAntennaEdge.at(i).y > obstacleDimension.at(j).yMax) continue;
              if (clientAntenna.z < obstacleDimension.at(j).zMin && apAntennaEdge.at(i).z < obstacleDimension.at(j).zMin) continue;
              if (clientAntenna.z > obstacleDimension.at(j).zMax && apAntennaEdge.at(i).z > obstacleDimension.at(j).zMax) continue;
              bool insideFlag = 0;
              if (clientAntenna.x >= obstacleDimension.at(j).xMin && clientAntenna.x <= obstacleDimension.at(j).xMax && clientAntenna.y >= obstacleDimension.at(j).yMin && clientAntenna.y <= obstacleDimension.at(j).yMax && clientAntenna.z >= obstacleDimension.at(j).zMin && clientAntenna.z <= obstacleDimension.at(j).zMax) 
                {
                  insideFlag = true;
                  m_hitList.push_back (clientAntenna);
                }              

              bool x1 = GetIntersection(apAntennaEdge.at(i).x-B1.x, clientAntenna.x-B1.x, apAntennaEdge.at(i), clientAntenna, Hit) && InBox(Hit, B1, B2, 1);
              bool x2 = GetIntersection(apAntennaEdge.at(i).x-B2.x, clientAntenna.x-B2.x, apAntennaEdge.at(i), clientAntenna, Hit) && InBox(Hit, B1, B2, 1);
              bool y1 = GetIntersection(apAntennaEdge.at(i).y-B1.y, clientAntenna.y-B1.y, apAntennaEdge.at(i), clientAntenna, Hit) && InBox(Hit, B1, B2, 2);
              bool y2 = GetIntersection(apAntennaEdge.at(i).y-B2.y, clientAntenna.y-B2.y, apAntennaEdge.at(i), clientAntenna, Hit) && InBox(Hit, B1, B2, 2);
              bool z1 = GetIntersection(apAntennaEdge.at(i).z-B1.z, clientAntenna.z-B1.z, apAntennaEdge.at(i), clientAntenna, Hit) && InBox(Hit, B1, B2, 3);
              bool z2 = GetIntersection(apAntennaEdge.at(i).z-B2.z, clientAntenna.z-B2.z, apAntennaEdge.at(i), clientAntenna, Hit) && InBox(Hit, B1, B2, 3);
              NS_LOG_FUNCTION (clientAntenna << j << x1 << x2 << y1 << y2 << z1 << z2);

              if (x1 + y1 + z1 + x2 + y2 + z2 + insideFlag > 1)
                {
                  hitFlag = true;
	              tempChannelStatus = NON_LINE_OF_SIGHT;
                }

              if (hitFlag == true)
                {
                  // furniture obstacles
                  if (j < obstalceNumber_fixed)
                  	{
                  	  tempFadingLoss += CalculateDistance(m_hitList.at(0), m_hitList.at(1))*obstaclePenetrationLoss.at(j+10); // fix this bug by Yuchen 7/2020
                  	}
				  else
				  	{
				  	  // tempFadingLoss += m_obstaclePenetrationLoss.at(j+10);
				  	  tempFadingLoss +=30; // human causes 30 dB fading loss
				  	}
                  
                } 
            }
          fadingLoss = (fadingLoss <= tempFadingLoss)?fadingLoss:tempFadingLoss;
          if (tempChannelStatus == LINE_OF_SIGHT) 
            channelStatus = LINE_OF_SIGHT;
        }

      if (fadingLoss > 0)
        channelStatus = NON_LINE_OF_SIGHT;

      bool multipathFlag = true;
      double multipathFading;
      if (multipathFlag)
        {
          std::default_random_engine generator (m_clientRS);
          std::normal_distribution<double> multipathDistribution(0,2.24);
          multipathFading = multipathDistribution(generator);
          fadingLoss = fadingLoss - multipathFading;
        } 

	  }

	  // for no-obstacle cases
	  if (obstalceNumber == 0)
	  {
	    	fadingLoss = 0;
	  }
	  
      m_channelInfo.push_back(ChannelInfo());
      m_channelInfo.at(clientId).losFlag = channelStatus;
      m_channelInfo.at(clientId).fadingLoss = -fadingLoss;
      m_channelInfo.at(clientId).apPos = apLocation; // room-level position
      m_channelInfo.at(clientId).clientPos = clientLocation.at(clientId); // m_clientPos_BL.at(clientId);
      m_channelInfo.at(clientId).beInfed = 0;
	  
      //channelStats.first.push_back (channelStatus);
      //channelStats.second.push_back(-fadingLoss);

      //std::cout<< channelStatus << " " << fadingLoss << " " << apLocation << " " << m_channelInfo.at(clientId).clientPos << " " << clientId << std::endl; 
      NS_LOG_FUNCTION ("los nlos status" << channelStatus << fadingLoss);
    }
}




// Yuchen 7/2020
std::vector<bool>
Obstacle::LoSAnalysis_MultiAP (Vector apLocation, std::vector<Vector> clientLocation, Vector apDimension)
{
  NS_LOG_FUNCTION (this << m_obstalceNumber);
  std::vector<bool> LoSstatus;
  
  //std::pair<std::vector<bool>, std::vector<double>> channelStats;
  for (uint16_t clientId = 0; clientId < clientLocation.size(); clientId++)
    {
      bool channelStatus = LINE_OF_SIGHT;
      double fadingLoss = 1e7;

	  if (m_obstalceNumber > 0)
	  {

      // Identify antenna model dimension
      Box antennaSize = Box {apLocation.x-apDimension.x*0.5, apLocation.x+apDimension.x*0.5, apLocation.y-apDimension.y*0.5, apLocation.y+apDimension.y*0.5, apLocation.z-apDimension.z*0.5, apLocation.z+apDimension.z*0.5};

      std::vector<Vector> apAntennaEdge = {{antennaSize.xMin, antennaSize.yMin, antennaSize.zMin}, {antennaSize.xMin, antennaSize.yMin, antennaSize.zMax}, {antennaSize.xMin, antennaSize.yMax, antennaSize.zMin}, {antennaSize.xMin, antennaSize.yMax, antennaSize.zMax}, {antennaSize.xMax, antennaSize.yMin, antennaSize.zMin}, {antennaSize.xMax, antennaSize.yMin, antennaSize.zMax}, {antennaSize.xMax, antennaSize.yMax, antennaSize.zMin}, {antennaSize.xMax, antennaSize.yMax, antennaSize.zMax}};

      Vector clientAntenna = clientLocation.at(clientId);

      // LoS analysis
      for (uint16_t i=0; i<apAntennaEdge.size(); i++)
        {
          double tempFadingLoss = 0;
          bool tempChannelStatus = LINE_OF_SIGHT;
          for (uint16_t j=0; j<(m_obstalceNumber + m_obstalceNumber_human); j++) 
            {
              bool hitFlag = false;
              m_hitList.clear();
              Vector Hit = Vector (0, 0, 0);
              Vector B1 = Vector (m_obstacleDimension.at(j).xMin, m_obstacleDimension.at(j).yMin, m_obstacleDimension.at(j).zMin);
              Vector B2 = Vector (m_obstacleDimension.at(j).xMax, m_obstacleDimension.at(j).yMax, m_obstacleDimension.at(j).zMax);
              if (clientAntenna.x < m_obstacleDimension.at(j).xMin && apAntennaEdge.at(i).x < m_obstacleDimension.at(j).xMin) continue;
              if (clientAntenna.x > m_obstacleDimension.at(j).xMax && apAntennaEdge.at(i).x > m_obstacleDimension.at(j).xMax) continue;
              if (clientAntenna.y < m_obstacleDimension.at(j).yMin && apAntennaEdge.at(i).y < m_obstacleDimension.at(j).yMin) continue;
              if (clientAntenna.y > m_obstacleDimension.at(j).yMax && apAntennaEdge.at(i).y > m_obstacleDimension.at(j).yMax) continue;
              if (clientAntenna.z < m_obstacleDimension.at(j).zMin && apAntennaEdge.at(i).z < m_obstacleDimension.at(j).zMin) continue;
              if (clientAntenna.z > m_obstacleDimension.at(j).zMax && apAntennaEdge.at(i).z > m_obstacleDimension.at(j).zMax) continue;
              bool insideFlag = 0;
              if (clientAntenna.x >= m_obstacleDimension.at(j).xMin && clientAntenna.x <= m_obstacleDimension.at(j).xMax && clientAntenna.y >= m_obstacleDimension.at(j).yMin && clientAntenna.y <= m_obstacleDimension.at(j).yMax && clientAntenna.z >= m_obstacleDimension.at(j).zMin && clientAntenna.z <= m_obstacleDimension.at(j).zMax) 
                {
                  insideFlag = true;
                  m_hitList.push_back (clientAntenna);
                }              

              bool x1 = GetIntersection(apAntennaEdge.at(i).x-B1.x, clientAntenna.x-B1.x, apAntennaEdge.at(i), clientAntenna, Hit) && InBox(Hit, B1, B2, 1);
              bool x2 = GetIntersection(apAntennaEdge.at(i).x-B2.x, clientAntenna.x-B2.x, apAntennaEdge.at(i), clientAntenna, Hit) && InBox(Hit, B1, B2, 1);
              bool y1 = GetIntersection(apAntennaEdge.at(i).y-B1.y, clientAntenna.y-B1.y, apAntennaEdge.at(i), clientAntenna, Hit) && InBox(Hit, B1, B2, 2);
              bool y2 = GetIntersection(apAntennaEdge.at(i).y-B2.y, clientAntenna.y-B2.y, apAntennaEdge.at(i), clientAntenna, Hit) && InBox(Hit, B1, B2, 2);
              bool z1 = GetIntersection(apAntennaEdge.at(i).z-B1.z, clientAntenna.z-B1.z, apAntennaEdge.at(i), clientAntenna, Hit) && InBox(Hit, B1, B2, 3);
              bool z2 = GetIntersection(apAntennaEdge.at(i).z-B2.z, clientAntenna.z-B2.z, apAntennaEdge.at(i), clientAntenna, Hit) && InBox(Hit, B1, B2, 3);
              NS_LOG_FUNCTION (clientAntenna << j << x1 << x2 << y1 << y2 << z1 << z2);

              if (x1 + y1 + z1 + x2 + y2 + z2 + insideFlag > 1)
                {
                  hitFlag = true;
	              tempChannelStatus = NON_LINE_OF_SIGHT;
                }

              if (hitFlag == true)
                {
                  // furniture obstacles
                  if (j < m_obstalceNumber)
                  	{
                  	  tempFadingLoss += CalculateDistance(m_hitList.at(0), m_hitList.at(1))*m_obstaclePenetrationLoss.at(j+10); // fix this bug by Yuchen 7/2020
                  	}
				  else
				  	{
				  	  tempFadingLoss += m_obstaclePenetrationLoss.at(j+10);
				  	}
                  
                }
             
            }
          // fadingLoss = (fadingLoss <= tempFadingLoss)?fadingLoss:tempFadingLoss;
          if (tempChannelStatus == LINE_OF_SIGHT) 
            {
         	  channelStatus = LINE_OF_SIGHT;
          	}
        }

      if (fadingLoss > 0)
        channelStatus = NON_LINE_OF_SIGHT;

      bool multipathFlag = true;
      double multipathFading;
      if (multipathFlag)
        {
          std::default_random_engine generator (m_clientRS);
          std::normal_distribution<double> multipathDistribution(0,2.24);
          multipathFading = multipathDistribution(generator);
          fadingLoss = fadingLoss - multipathFading;
        } 

	  }

	  // for no-obstacle cases
	  if (m_obstalceNumber == 0)
	  {
	    	fadingLoss = 0;
	  }

      LoSstatus.push_back(channelStatus);

	  /*
      m_channelInfo.push_back(ChannelInfo());
      m_channelInfo.at(clientId).losFlag = channelStatus;
      m_channelInfo.at(clientId).fadingLoss = -fadingLoss;
      m_channelInfo.at(clientId).apPos = apLocation;
      m_channelInfo.at(clientId).clientPos = clientLocation.at(clientId);
        
      //channelStats.first.push_back (channelStatus);
      //channelStats.second.push_back(-fadingLoss);

      //std::cout<< channelStatus << " " << fadingLoss << " " << apLocation << " " << m_channelInfo.at(clientId).clientPos << " " << clientId << std::endl; 
      NS_LOG_FUNCTION ("los nlos status" << channelStatus << fadingLoss);
      */
    }

 return LoSstatus;
}


// Yuchen 2/2021
std::vector<Vector>
Obstacle::checkLoS (std::vector<Vector> apLocation_all, Vector clientLocation, std::vector<Box> obstacleDimension, Vector apDimension)
{

  // ------ check LoS for a specific client with multiple APs, return locations of LoS APs ------ //
  
  
  // NS_LOG_FUNCTION (this << m_obstalceNumber);
  // std::vector<bool> LoSstatus;
  std::vector<Vector> LoSAPLocation;
  
  //std::pair<std::vector<bool>, std::vector<double>> channelStats;
  for (uint16_t APId = 0; APId < apLocation_all.size(); APId++)
    {
      bool channelStatus = LINE_OF_SIGHT;
      double fadingLoss = 1e7;

	  uint16_t obstalceNumber = obstacleDimension.size();
	  Vector apLocation = apLocation_all.at(APId);

	  if (obstalceNumber > 0)
	  {

      // Identify antenna model dimension
      Box antennaSize = Box {apLocation.x-apDimension.x*0.5, apLocation.x+apDimension.x*0.5, apLocation.y-apDimension.y*0.5, apLocation.y+apDimension.y*0.5, apLocation.z-apDimension.z*0.5, apLocation.z+apDimension.z*0.5};

      std::vector<Vector> apAntennaEdge = {{antennaSize.xMin, antennaSize.yMin, antennaSize.zMin}, {antennaSize.xMin, antennaSize.yMin, antennaSize.zMax}, {antennaSize.xMin, antennaSize.yMax, antennaSize.zMin}, {antennaSize.xMin, antennaSize.yMax, antennaSize.zMax}, {antennaSize.xMax, antennaSize.yMin, antennaSize.zMin}, {antennaSize.xMax, antennaSize.yMin, antennaSize.zMax}, {antennaSize.xMax, antennaSize.yMax, antennaSize.zMin}, {antennaSize.xMax, antennaSize.yMax, antennaSize.zMax}};

      Vector clientAntenna = clientLocation;

      // LoS analysis
      for (uint16_t i=0; i<apAntennaEdge.size(); i++)
        {
          double tempFadingLoss = 0;
          bool tempChannelStatus = LINE_OF_SIGHT;
          for (uint16_t j=0; j<(obstalceNumber); j++) 
            {
              bool hitFlag = false;
              m_hitList.clear();
              Vector Hit = Vector (0, 0, 0);
              Vector B1 = Vector (obstacleDimension.at(j).xMin, obstacleDimension.at(j).yMin, obstacleDimension.at(j).zMin);
              Vector B2 = Vector (obstacleDimension.at(j).xMax, obstacleDimension.at(j).yMax, obstacleDimension.at(j).zMax);
              if (clientAntenna.x < obstacleDimension.at(j).xMin && apAntennaEdge.at(i).x < obstacleDimension.at(j).xMin) continue;
              if (clientAntenna.x > obstacleDimension.at(j).xMax && apAntennaEdge.at(i).x > obstacleDimension.at(j).xMax) continue;
              if (clientAntenna.y < obstacleDimension.at(j).yMin && apAntennaEdge.at(i).y < obstacleDimension.at(j).yMin) continue;
              if (clientAntenna.y > obstacleDimension.at(j).yMax && apAntennaEdge.at(i).y > obstacleDimension.at(j).yMax) continue;
              if (clientAntenna.z < obstacleDimension.at(j).zMin && apAntennaEdge.at(i).z < obstacleDimension.at(j).zMin) continue;
              if (clientAntenna.z > obstacleDimension.at(j).zMax && apAntennaEdge.at(i).z > obstacleDimension.at(j).zMax) continue;
              bool insideFlag = 0;
              if (clientAntenna.x >= obstacleDimension.at(j).xMin && clientAntenna.x <= obstacleDimension.at(j).xMax && clientAntenna.y >= obstacleDimension.at(j).yMin && clientAntenna.y <= obstacleDimension.at(j).yMax && clientAntenna.z >= obstacleDimension.at(j).zMin && clientAntenna.z <= obstacleDimension.at(j).zMax) 
                {
                  insideFlag = true;
                  m_hitList.push_back (clientAntenna);
                }              

              bool x1 = GetIntersection(apAntennaEdge.at(i).x-B1.x, clientAntenna.x-B1.x, apAntennaEdge.at(i), clientAntenna, Hit) && InBox(Hit, B1, B2, 1);
              bool x2 = GetIntersection(apAntennaEdge.at(i).x-B2.x, clientAntenna.x-B2.x, apAntennaEdge.at(i), clientAntenna, Hit) && InBox(Hit, B1, B2, 1);
              bool y1 = GetIntersection(apAntennaEdge.at(i).y-B1.y, clientAntenna.y-B1.y, apAntennaEdge.at(i), clientAntenna, Hit) && InBox(Hit, B1, B2, 2);
              bool y2 = GetIntersection(apAntennaEdge.at(i).y-B2.y, clientAntenna.y-B2.y, apAntennaEdge.at(i), clientAntenna, Hit) && InBox(Hit, B1, B2, 2);
              bool z1 = GetIntersection(apAntennaEdge.at(i).z-B1.z, clientAntenna.z-B1.z, apAntennaEdge.at(i), clientAntenna, Hit) && InBox(Hit, B1, B2, 3);
              bool z2 = GetIntersection(apAntennaEdge.at(i).z-B2.z, clientAntenna.z-B2.z, apAntennaEdge.at(i), clientAntenna, Hit) && InBox(Hit, B1, B2, 3);
              NS_LOG_FUNCTION (clientAntenna << j << x1 << x2 << y1 << y2 << z1 << z2);

              if (x1 + y1 + z1 + x2 + y2 + z2 + insideFlag > 1)
                {
                  hitFlag = true;
	              tempChannelStatus = NON_LINE_OF_SIGHT;
                }
              
              if (hitFlag == true)
                {
                  // furniture obstacles
                  // if (j < m_obstalceNumber)
                  	// {
                  	  tempFadingLoss += CalculateDistance(m_hitList.at(0), m_hitList.at(1))*m_obstaclePenetrationLoss.at(j+10); // fix this bug by Yuchen 7/2020
                  	// }
				  // else
				  	// {
				  	  // tempFadingLoss += m_obstaclePenetrationLoss.at(j+10);
				  	// }
                  
                }
             
            }
		  
          fadingLoss = (fadingLoss <= tempFadingLoss)?fadingLoss:tempFadingLoss;
          if (tempChannelStatus == LINE_OF_SIGHT) 
            {
         	  channelStatus = LINE_OF_SIGHT;
          	}
        }

      
      if (fadingLoss > 0)
        channelStatus = NON_LINE_OF_SIGHT;

	  /*
      bool multipathFlag = true;
      double multipathFading;
      if (multipathFlag)
        {
          std::default_random_engine generator (m_clientRS);
          std::normal_distribution<double> multipathDistribution(0,2.24);
          multipathFading = multipathDistribution(generator);
          fadingLoss = fadingLoss - multipathFading;
        } 
      */
      
	  }

	  // for no-obstacle cases
	  /*
	  if (m_obstalceNumber == 0)
	  {
	    	fadingLoss = 0;
	  }
	  */

	  if (channelStatus == LINE_OF_SIGHT) // LoS to this AP
	  	{
	  	   LoSAPLocation.push_back(apLocation);
	  	}

      // LoSstatus.push_back(channelStatus);

	  /*
      m_channelInfo.push_back(ChannelInfo());
      m_channelInfo.at(clientId).losFlag = channelStatus;
      m_channelInfo.at(clientId).fadingLoss = -fadingLoss;
      m_channelInfo.at(clientId).apPos = apLocation;
      m_channelInfo.at(clientId).clientPos = clientLocation.at(clientId);
        
      //channelStats.first.push_back (channelStatus);
      //channelStats.second.push_back(-fadingLoss);

      //std::cout<< channelStatus << " " << fadingLoss << " " << apLocation << " " << m_channelInfo.at(clientId).clientPos << " " << clientId << std::endl; 
      NS_LOG_FUNCTION ("los nlos status" << channelStatus << fadingLoss);
      */
    }

 	return LoSAPLocation;
}

std::pair<bool, double> 
Obstacle::checkLoS (Vector transmitter, Vector receiver)
{
  std::pair<bool, double> re;
  bool channelStatus = LINE_OF_SIGHT;
  double fadingLoss = 1e7;
  uint16_t obstalceNumber = m_obstacleDimension.size();
  std::vector<Box> obstacleDimension = m_obstacleDimension;
  Vector apLocation = transmitter;

  if (obstalceNumber > 0)
  {

      // Identify antenna model dimension
      Box antennaSize = Box {apLocation.x-m_apDimension.x*0.5, apLocation.x+m_apDimension.x*0.5, apLocation.y-m_apDimension.y*0.5, apLocation.y+m_apDimension.y*0.5, apLocation.z-m_apDimension.z*0.5, apLocation.z+m_apDimension.z*0.5};

      std::vector<Vector> apAntennaEdge = {{antennaSize.xMin, antennaSize.yMin, antennaSize.zMin}, {antennaSize.xMin, antennaSize.yMin, antennaSize.zMax}, {antennaSize.xMin, antennaSize.yMax, antennaSize.zMin}, {antennaSize.xMin, antennaSize.yMax, antennaSize.zMax}, {antennaSize.xMax, antennaSize.yMin, antennaSize.zMin}, {antennaSize.xMax, antennaSize.yMin, antennaSize.zMax}, {antennaSize.xMax, antennaSize.yMax, antennaSize.zMin}, {antennaSize.xMax, antennaSize.yMax, antennaSize.zMax}};

      Vector clientAntenna = receiver;

      // LoS analysis
      for (uint16_t i=0; i<apAntennaEdge.size(); i++)
        {
          double tempFadingLoss = 0;
          bool tempChannelStatus = LINE_OF_SIGHT;
          for (uint16_t j=0; j<(obstalceNumber); j++) 
            {
              bool hitFlag = false;
              m_hitList.clear();
              Vector Hit = Vector (0, 0, 0);
              Vector B1 = Vector (obstacleDimension.at(j).xMin, obstacleDimension.at(j).yMin, obstacleDimension.at(j).zMin);
              Vector B2 = Vector (obstacleDimension.at(j).xMax, obstacleDimension.at(j).yMax, obstacleDimension.at(j).zMax);
              if (clientAntenna.x < obstacleDimension.at(j).xMin && apAntennaEdge.at(i).x < obstacleDimension.at(j).xMin) continue;
              if (clientAntenna.x > obstacleDimension.at(j).xMax && apAntennaEdge.at(i).x > obstacleDimension.at(j).xMax) continue;
              if (clientAntenna.y < obstacleDimension.at(j).yMin && apAntennaEdge.at(i).y < obstacleDimension.at(j).yMin) continue;
              if (clientAntenna.y > obstacleDimension.at(j).yMax && apAntennaEdge.at(i).y > obstacleDimension.at(j).yMax) continue;
              if (clientAntenna.z < obstacleDimension.at(j).zMin && apAntennaEdge.at(i).z < obstacleDimension.at(j).zMin) continue;
              if (clientAntenna.z > obstacleDimension.at(j).zMax && apAntennaEdge.at(i).z > obstacleDimension.at(j).zMax) continue;
              bool insideFlag = 0;
              if (clientAntenna.x >= obstacleDimension.at(j).xMin && clientAntenna.x <= obstacleDimension.at(j).xMax && clientAntenna.y >= obstacleDimension.at(j).yMin && clientAntenna.y <= obstacleDimension.at(j).yMax && clientAntenna.z >= obstacleDimension.at(j).zMin && clientAntenna.z <= obstacleDimension.at(j).zMax) 
                {
                  insideFlag = true;
                  m_hitList.push_back (clientAntenna);
                }              

              bool x1 = GetIntersection(apAntennaEdge.at(i).x-B1.x, clientAntenna.x-B1.x, apAntennaEdge.at(i), clientAntenna, Hit) && InBox(Hit, B1, B2, 1);
              bool x2 = GetIntersection(apAntennaEdge.at(i).x-B2.x, clientAntenna.x-B2.x, apAntennaEdge.at(i), clientAntenna, Hit) && InBox(Hit, B1, B2, 1);
              bool y1 = GetIntersection(apAntennaEdge.at(i).y-B1.y, clientAntenna.y-B1.y, apAntennaEdge.at(i), clientAntenna, Hit) && InBox(Hit, B1, B2, 2);
              bool y2 = GetIntersection(apAntennaEdge.at(i).y-B2.y, clientAntenna.y-B2.y, apAntennaEdge.at(i), clientAntenna, Hit) && InBox(Hit, B1, B2, 2);
              bool z1 = GetIntersection(apAntennaEdge.at(i).z-B1.z, clientAntenna.z-B1.z, apAntennaEdge.at(i), clientAntenna, Hit) && InBox(Hit, B1, B2, 3);
              bool z2 = GetIntersection(apAntennaEdge.at(i).z-B2.z, clientAntenna.z-B2.z, apAntennaEdge.at(i), clientAntenna, Hit) && InBox(Hit, B1, B2, 3);
              NS_LOG_FUNCTION (clientAntenna << j << x1 << x2 << y1 << y2 << z1 << z2);

              if (x1 + y1 + z1 + x2 + y2 + z2 + insideFlag > 1)
                {
                  hitFlag = true;
	              tempChannelStatus = NON_LINE_OF_SIGHT;
                }
              
              if (hitFlag == true)
                {
                  // furniture obstacles
                  // if (j < m_obstalceNumber)
                  	// {
                  	  tempFadingLoss += CalculateDistance(m_hitList.at(0), m_hitList.at(1))*m_obstaclePenetrationLoss.at(j+10); // fix this bug by Yuchen 7/2020
                  	// }
				  // else
				  	// {
				  	  // tempFadingLoss += m_obstaclePenetrationLoss.at(j+10);
				  	// }
                  
                }
             
            }
		  
          fadingLoss = (fadingLoss <= tempFadingLoss)?fadingLoss:tempFadingLoss;
          if (tempChannelStatus == LINE_OF_SIGHT) 
            {
         	  channelStatus = LINE_OF_SIGHT;
          	}
        }

      
     if (fadingLoss > 0)
       channelStatus = NON_LINE_OF_SIGHT;
	  
     bool multipathFlag = true; // set as true by default
     double multipathFading;
     if (multipathFlag)
        {
          std::default_random_engine generator (m_clientRS);
          std::normal_distribution<double> multipathDistribution(0,2.24);
          multipathFading = multipathDistribution(generator);
          fadingLoss = fadingLoss - multipathFading;
        } 
      
  }
  else
  	{
  	  bool multipathFlag = true; // set as true by default
      double multipathFading;
      if (multipathFlag)
        {
          std::default_random_engine generator (m_clientRS);
          std::normal_distribution<double> multipathDistribution(0,2.24);
          multipathFading = multipathDistribution(generator);
          fadingLoss = multipathFading;
        } 
  	}

  re.first = channelStatus;
  re.second = -fadingLoss;
  
  return re;
}


// consider the wall with huge penetration loss
// 0 - LoS, 1 - NLoS, 2 Wall_NLoS
std::pair<uint16_t, double> 
Obstacle::checkLoS_withWall (Vector transmitter, Vector receiver)
{
  std::pair<uint16_t, double> re;
  uint16_t channelStatus = 0;
  double fadingLoss = 1e7;
  uint16_t obstalceNumber = m_obstacleDimension.size();
  std::vector<Box> obstacleDimension = m_obstacleDimension;
  Vector apLocation = transmitter;

  if (obstalceNumber > 0)
  {

      // Identify antenna model dimension
      Box antennaSize = Box {apLocation.x-m_apDimension.x*0.5, apLocation.x+m_apDimension.x*0.5, apLocation.y-m_apDimension.y*0.5, apLocation.y+m_apDimension.y*0.5, apLocation.z-m_apDimension.z*0.5, apLocation.z+m_apDimension.z*0.5};

      std::vector<Vector> apAntennaEdge = {{antennaSize.xMin, antennaSize.yMin, antennaSize.zMin}, {antennaSize.xMin, antennaSize.yMin, antennaSize.zMax}, {antennaSize.xMin, antennaSize.yMax, antennaSize.zMin}, {antennaSize.xMin, antennaSize.yMax, antennaSize.zMax}, {antennaSize.xMax, antennaSize.yMin, antennaSize.zMin}, {antennaSize.xMax, antennaSize.yMin, antennaSize.zMax}, {antennaSize.xMax, antennaSize.yMax, antennaSize.zMin}, {antennaSize.xMax, antennaSize.yMax, antennaSize.zMax}};

      Vector clientAntenna = receiver;

	  bool blockByWall = false;

      // LoS analysis
      for (uint16_t i=0; i<apAntennaEdge.size(); i++)
        {
          double tempFadingLoss = 0;
          uint16_t tempChannelStatus = 0;
          for (uint16_t j=0; j<(obstalceNumber); j++) 
            {
              bool hitFlag = false;
              m_hitList.clear();
              Vector Hit = Vector (0, 0, 0);
              Vector B1 = Vector (obstacleDimension.at(j).xMin, obstacleDimension.at(j).yMin, obstacleDimension.at(j).zMin);
              Vector B2 = Vector (obstacleDimension.at(j).xMax, obstacleDimension.at(j).yMax, obstacleDimension.at(j).zMax);
              if (clientAntenna.x < obstacleDimension.at(j).xMin && apAntennaEdge.at(i).x < obstacleDimension.at(j).xMin) continue;
              if (clientAntenna.x > obstacleDimension.at(j).xMax && apAntennaEdge.at(i).x > obstacleDimension.at(j).xMax) continue;
              if (clientAntenna.y < obstacleDimension.at(j).yMin && apAntennaEdge.at(i).y < obstacleDimension.at(j).yMin) continue;
              if (clientAntenna.y > obstacleDimension.at(j).yMax && apAntennaEdge.at(i).y > obstacleDimension.at(j).yMax) continue;
              if (clientAntenna.z < obstacleDimension.at(j).zMin && apAntennaEdge.at(i).z < obstacleDimension.at(j).zMin) continue;
              if (clientAntenna.z > obstacleDimension.at(j).zMax && apAntennaEdge.at(i).z > obstacleDimension.at(j).zMax) continue;
              bool insideFlag = 0;
              if (clientAntenna.x >= obstacleDimension.at(j).xMin && clientAntenna.x <= obstacleDimension.at(j).xMax && clientAntenna.y >= obstacleDimension.at(j).yMin && clientAntenna.y <= obstacleDimension.at(j).yMax && clientAntenna.z >= obstacleDimension.at(j).zMin && clientAntenna.z <= obstacleDimension.at(j).zMax) 
                {
                  insideFlag = true;
                  m_hitList.push_back (clientAntenna);
                }              

              bool x1 = GetIntersection(apAntennaEdge.at(i).x-B1.x, clientAntenna.x-B1.x, apAntennaEdge.at(i), clientAntenna, Hit) && InBox(Hit, B1, B2, 1);
              bool x2 = GetIntersection(apAntennaEdge.at(i).x-B2.x, clientAntenna.x-B2.x, apAntennaEdge.at(i), clientAntenna, Hit) && InBox(Hit, B1, B2, 1);
              bool y1 = GetIntersection(apAntennaEdge.at(i).y-B1.y, clientAntenna.y-B1.y, apAntennaEdge.at(i), clientAntenna, Hit) && InBox(Hit, B1, B2, 2);
              bool y2 = GetIntersection(apAntennaEdge.at(i).y-B2.y, clientAntenna.y-B2.y, apAntennaEdge.at(i), clientAntenna, Hit) && InBox(Hit, B1, B2, 2);
              bool z1 = GetIntersection(apAntennaEdge.at(i).z-B1.z, clientAntenna.z-B1.z, apAntennaEdge.at(i), clientAntenna, Hit) && InBox(Hit, B1, B2, 3);
              bool z2 = GetIntersection(apAntennaEdge.at(i).z-B2.z, clientAntenna.z-B2.z, apAntennaEdge.at(i), clientAntenna, Hit) && InBox(Hit, B1, B2, 3);
              NS_LOG_FUNCTION (clientAntenna << j << x1 << x2 << y1 << y2 << z1 << z2);

              if (x1 + y1 + z1 + x2 + y2 + z2 + insideFlag > 1)
                {
                  hitFlag = true;
	              tempChannelStatus = 1; // NLoS
                }
              
              if (hitFlag == true)
                {
                  // furniture obstacles
                  // if (j < m_obstalceNumber)
                  	// {
                  	  tempFadingLoss += CalculateDistance(m_hitList.at(0), m_hitList.at(1))*m_obstaclePenetrationLoss.at(j+10); // fix this bug by Yuchen 7/2020
                  	// }
				  // else
				  	// {
				  	  // tempFadingLoss += m_obstaclePenetrationLoss.at(j+10);
				  	// }
				  	if (j >= obstalceNumber - 4) // wall obstacles
				  		{
				  		   blockByWall = true;
						   tempFadingLoss = 1e7;
				  		}
                  
                }
             
            }
		  
          fadingLoss = (fadingLoss <= tempFadingLoss)?fadingLoss:tempFadingLoss;
          if (tempChannelStatus == 0) 
            {
         	  channelStatus = 0; // LoS
          	}
        }

      
     if (fadingLoss > 0)
       channelStatus = 1; // nLoS
       
	 if ((fadingLoss > 0) && (blockByWall == true))
       channelStatus = 2; // Wall_NLoS
	  
     bool multipathFlag = true; // set as true by default
     double multipathFading;
     if (multipathFlag)
        {
          std::default_random_engine generator (m_clientRS);
          std::normal_distribution<double> multipathDistribution(0,2.24);
          multipathFading = multipathDistribution(generator);
          fadingLoss = fadingLoss - multipathFading;
        } 
      
  }
  else
  	{
  	  bool multipathFlag = true; // set as true by default
      double multipathFading;
      if (multipathFlag)
        {
          std::default_random_engine generator (m_clientRS);
          std::normal_distribution<double> multipathDistribution(0,2.24);
          multipathFading = multipathDistribution(generator);
          fadingLoss = multipathFading;
        } 
  	}

  re.first = channelStatus;
  re.second = -fadingLoss;
  
  return re;
}




// Yuchen 7/2020
std::vector<std::vector<double> >
Obstacle::InterferenceAnalysis (std::vector<Vector> apLocation, std::vector<Vector> clientLocation, uint16_t NumAP, uint16_t NumSTA, std::vector<std::vector<bool> > losFlag_mul, double txPowerdBm)
{ 
  std::vector<std::vector<double> > InterfVec_all;
  // antenna setting
  // Ptr<YansWifiPhy> node;
  // Ptr<Directional60GhzAntenna> nodeAnt; // = node->GetDirectionalAntenna ();
  double nodeAntennaMaxGain = 23.18; //17.625; // nodeAnt->GetMaxGainDbi();
  for (uint16_t i = 0; i < NumSTA; ++i)
  	{
  	  std::vector<double> InterfVec(NumAP,0.0);
	  for (uint16_t j = 0; j < NumAP; ++j)
	  	{
	  	   // for AP-client pair (i, j)
	  	   double intf = 0.0;
	  	   for (uint16_t ii = 0; ii < NumSTA; ++ii)
	  	   	{ 
	  	   	   if (ii == i)
	  	   	   	{
	  	   	   	  continue;
	  	   	   	}
               // random access scheme
               RngSeedManager::SetSeed (2);
               Ptr<UniformRandomVariable> serveAPIndex = CreateObject<UniformRandomVariable> ();
               serveAPIndex->SetAttribute ("Min", DoubleValue (0.0));
               serveAPIndex->SetAttribute ("Max", DoubleValue (NumAP - 0.01));
               uint16_t serveAPID = floor(serveAPIndex->GetValue ());
			   // std::cerr << "serveAPID is: " << serveAPID << std::endl;

			   if (serveAPID != j)
			   	{
                   Vector V1 = clientLocation[ii] - apLocation[serveAPID]; // primary signal
			   	   Vector V2 = clientLocation[i] - apLocation[serveAPID]; // interfered signal

				   double dot = V1.x * V2.x + V1.y * V2.y + V1.z * V2.z;
				   double len_V1 = std::sqrt (V1.x * V1.x + V1.y * V1.y + V1.z * V1.z);
				   double len_V2 = std::sqrt (V2.x * V2.x + V2.y * V2.y + V2.z * V2.z);
				   double angle_rad = std::abs(std::acos(dot/(len_V1*len_V2)));
				   double angle = 180.0*angle_rad/PI; // degree

				   double FPBW = 20.0; // nodeAnt->GetMainLobeWidth(); // 45; // degree, according to default setting with 8 sectors and 1 antenna
				   double HPBW = 12.0; // nodeAnt->GetHalfPowerBeamWidth(); // 17.3; // degree, see Directional60GhzAntenna::GetHalfPowerBeamWidth()
				   // std::cerr << "main lobe is " << FPBW << " HPBW is " << HPBW << std::endl;

				   double txAntennaGain = 0.0;
				   double rxAntennaGain = 0.0;
				   if (angle <= HPBW/2)
				   	{
				   	  txAntennaGain = nodeAntennaMaxGain; // nodeAnt->GetMaxGainDbi(); // see Directional60GhzAntenna::GetMaxGainDbi()
                      // std::cerr << "main lobe gain is " << txAntennaGain << std::endl;
				    }
				   else if ((angle > HPBW/2) && (angle <= FPBW/2))
				   	{
                      txAntennaGain = 7.256; // nodeAnt->GetMaxGainDbi() - 3;
					  // std::cerr << "HFBW lobe gain is " << txAntennaGain << std::endl;
				    }
				   else if ((angle > FPBW/2) && (angle <= 30.0))
				   	{
                      txAntennaGain = -17.256; // nodeAnt->GetMaxGainDbi() - 3;
					  // std::cerr << "HFBW lobe gain is " << txAntennaGain << std::endl;
				    }
				   else if ((angle > 30.0) && (angle <= 42.0))
				   	{
                      txAntennaGain = 1.686; // nodeAnt->GetMaxGainDbi() - 3;
					  // std::cerr << "HFBW lobe gain is " << txAntennaGain << std::endl;
				    }
				   else if ((angle > 42.0) && (angle <= 78.0))
				   	{
				   	  txAntennaGain = -10.3833; // nodeAnt->GetSideLobeGain(); // -10.3833; // see Directional60GhzAntenna::GetSideLobeGain()
					  // std::cerr << "Side lobe gain is " << txAntennaGain << std::endl;  
				    }
				   else if ((angle > 78.0) && (angle <= 90.0))
				   	{
				   	  txAntennaGain = 1.45;
				   	}
				   else
				   	{
				   	  txAntennaGain = -1000.0; // Inf
				   	}
				   double channelGain = SVChannelGain_inf(m_reflectionMode, apLocation[serveAPID], clientLocation[i], txAntennaGain, rxAntennaGain, losFlag_mul[serveAPID][i]);
                   double receiveDbmItf = txPowerdBm + channelGain;
				   intf = intf + receiveDbmItf;				   
			     }
	  	   	}
		   if (intf >= 0) // no interference signals occur
		   	{
		   	  intf = -10000.0; // set as -Inf dBm
		   	}
		   // std::cerr << "Intf dBm for one tansmission is " << intf << std::endl;
		   // transfer to packet-level estimation
		   intf = SingleCarrierPHYrSSMapping(intf)/4.62* 1000000.0/8; // byte/s, 4.62 is the scale, since we set the max rate is "1000Mbps" in the script (MultiAP_opt_2020.cc)
		   InterfVec[j] = intf;
	  	}
	  InterfVec_all.push_back(InterfVec);
  	}
  
  
  return InterfVec_all;
}

/* Saleh-Valenzuela Channel in 60 GHz indoor scenario for interference signal */
double 
Obstacle::SVChannelGain_inf(int reflectorDenseMode, Vector sender_pos, Vector receiver_pos, double txAntennaGain, double rxAntennaGain, bool LoSStatus)
{
  int lambda_K; // cluster density
  int lambda_ray; // density of rays within each cluster
  uint16_t granularity = 10;
  double G_sv = 0.0; // return value
  double obsDensity = m_obstalceNumber*1.0/(m_roomSize.x*m_roomSize.y);

  // based on the assumption of very narrow beams of the directional antennas
  lambda_K = 2;
  lambda_ray = 5;
  /*
  if (reflectorDenseMode == 1) // lower density of highly-reflective objects in the room
  	{
  	  lambda_K = 3; // 4
	  lambda_ray = 10; // 15
  	}
  else if (reflectorDenseMode == 2) // medium density of highly-reflective objects in the room
    {
  	  lambda_K = 5;
	  lambda_ray = 20;
  	}
  else // higher density of highly-reflective objects in the room
    {
  	  lambda_K = 7;
	  lambda_ray = 30;
    }
  */
  // Poisson point process cluster/ray number identification
  // 1) cluster
  double lambda = lambda_K*1.0/(granularity*granularity);
  // std::cerr << "LK " << lambda_K << " lambda " << lambda << std::endl;
  double poissonProb = exp (-lambda)*lambda;
  // std::cerr << "ProPP " << poissonProb << std::endl;
  int num_cluster = 0;
  for (uint16_t t=0; t<granularity*granularity; t++)
    {
       RngSeedManager::SetSeed (2); // random seed setting
       Ptr<UniformRandomVariable> clusterNoUV = CreateObject<UniformRandomVariable> ();
       clusterNoUV->SetAttribute ("Min", DoubleValue (0.0));
       clusterNoUV->SetAttribute ("Max", DoubleValue (1.0));
       double probability = clusterNoUV->GetValue ();
	   // std::cerr << "Pro " << probability << std::endl;
	   
       if (probability < poissonProb)
  	    {
  	  	   num_cluster++;
		   // std::cerr << "hit here " << std::endl;
  	    }
  	}
  if (num_cluster < 1)
  	{
  	   num_cluster = 1;
  	}
  // 2) ray
  lambda = lambda_ray*1.0/(granularity*granularity);
  poissonProb = exp (-lambda)*lambda;
  int num_ray = 0;
  for (uint16_t t=0; t<granularity*granularity; t++)
    {
       RngSeedManager::SetSeed (2); // random seed setting
       Ptr<UniformRandomVariable> rayNoUV = CreateObject<UniformRandomVariable> ();
       rayNoUV->SetAttribute ("Min", DoubleValue (0.0));
       rayNoUV->SetAttribute ("Max", DoubleValue (1.0));
       double probability = rayNoUV->GetValue ();
	   
       if (probability < poissonProb)
  	    {
  	  	   num_ray++;
  	    }
  	}
  if (num_ray < 1)
  	{
  	   num_ray = 1;
  	}

  // std::cerr << "num_cluster " << num_cluster << " number_ray " << num_ray << std::endl;
  
  // ------ Default setting ------
  // Ptr<MobilityModel> senderMobility = sender->GetMobility ();
  // NS_ASSERT (senderMobility != 0);
  // uint32_t j = 0; /* Phy ID */
  // Vector sender_pos = senderMobility->GetPosition ();
  // Ptr<DirectionalAntenna> senderAnt = sender->GetDirectionalAntenna ();
  // double rxPowerDbm;
  // Time delay; /* Propagation delay of the signal */
  // Ptr<MobilityModel> receiverMobility;
  // receiverMobility = receiver->GetMobility ()->GetObject<MobilityModel> ();
  // double azimuthTx = CalculateAzimuthAngle (sender_pos, receiverMobility->GetPosition ());
  // double azimuthRx = CalculateAzimuthAngle (receiverMobility->GetPosition (), sender_pos);
  // Antenna gain, get from IEEE 802.11ad direction antenna model
  double ref_dB_bias = 4.0; // beam alignment arror for multi-path reflections,refer to SIGCOMM paper "Fast mmWave Beam Alignment"
  // Ptr<UniformRandomVariable> reb = CreateObject<UniformRandomVariable> ();
  // reb->SetAttribute ("Min", DoubleValue (4.0));
  // reb->SetAttribute ("Max", DoubleValue (12.5));
  // ref_dB_bias = reb->GetValue ();
  double Gtx_dB = txAntennaGain; // very narrow beam, based on matlab simulation
  double Grx_dB = rxAntennaGain;
  double Gtx_dB_ref = txAntennaGain - ref_dB_bias;
  if (Gtx_dB_ref < 0) // in case < 0
  	{
  	   Gtx_dB_ref = 0;
   	}
  double Grx_dB_ref = Grx_dB; 
  double Gtx = std::pow(10.0,Gtx_dB/10);
  double Grx = std::pow(10.0,Grx_dB/10);
  double Gtx_ref = std::pow(10.0,Gtx_dB_ref/10);
  double Grx_ref = std::pow(10.0,Grx_dB_ref/10);

  // std::cerr << "Gtx" << Gtx_dB << " Grx" << Grx_dB << std::endl;
  
  // frequecy and wavelength
  double f_mm = 60; // GHz
  double lambda_w = 3.0e8/(f_mm*1.0e9);
  // heights of sender and receiver
  double h1 = sender_pos.z;
  double h2 = receiver_pos.z;

  // seperation distance between tranceiver
  double L = CalculateDistance(sender_pos, receiver_pos);

  // LoS status, 0 -- Line of Sight, 1 -- NLoS
  // bool LoSStatus =  m_scenario->GetFadingInfo(sender_pos, receiverMobility->GetPosition ()).first;

  // penetration loss + shadowing variable (only for NLoS case)
  // double X_pen_dB = m_scenario->GetFadingInfo(sender_pos, receiverMobility->GetPosition ()).second;

  // reflection coefficient (determined by reflection density mode)
  double mean_r0, var_r0;
  if (reflectorDenseMode == 1) // lower density of highly-reflective objects in the room
  	{
  	  mean_r0 = 0.4;
	  var_r0 = 0.1;
  	}
  else if (reflectorDenseMode == 2) // medium density of highly-reflective objects in the room
    {
  	  mean_r0 = 0.6;
	  var_r0 = 0.1;
  	}
  else // higher density of highly-reflective objects in the room
    {
  	  mean_r0 = 0.8;
	  var_r0 = 0.05;
    }
  //follow the truncated normal distribution
  Ptr<NormalRandomVariable> reff = CreateObject<NormalRandomVariable> ();
  reff->SetAttribute ("Mean", DoubleValue (mean_r0));
  reff->SetAttribute ("Variance", DoubleValue (var_r0));
  double R0 = reff->GetValue ();
  // in case beyond 0~1
  if (R0 < 0 || R0 > 1)
  	{
  	  R0 = mean_r0;
  	}
  


  // the square of two-path response
  double d1 = std::sqrt((h2-h1)*(h2-h1) + L*L);
  double d2 = std::sqrt((h2+h1)*(h2+h1) + L*L);
  double phi_r = 2*PI/lambda_w * (d2-d1);

  double rou_a = std::sqrt(Gtx*Grx);

  // determine rou_b
  double strongRefProb = obsDensity;
  RngSeedManager::SetSeed (2);
  Ptr<UniformRandomVariable> sFprob = CreateObject<UniformRandomVariable> ();
  sFprob->SetAttribute ("Min", DoubleValue (0.0));
  sFprob->SetAttribute ("Max", DoubleValue (1.0));
  double rdP = sFprob->GetValue ();
  double rou_b;
  if(rdP < strongRefProb)
  	{
  	   rou_b = std::sqrt(Gtx_ref*Grx_ref)*R0;
  	}
  else
  	{
       RngSeedManager::SetSeed (2);
       Ptr<NormalRandomVariable> reff_loss = CreateObject<NormalRandomVariable> ();
  	   reff_loss->SetAttribute ("Mean", DoubleValue (-13.0)); // include both first- and second-order reflection components, Alexander Maltsev's experiment
       reff_loss->SetAttribute ("Variance", DoubleValue (4.5)); // Alexander Maltsev's experiment
       double Rl_dB = reff_loss->GetValue ();
       double Rl_val = std::pow(10.0,Rl_dB/10);
	   rou_b = std::sqrt(Gtx_ref*Grx_ref)*Rl_val;
  	}
  
  double rou_2 = (rou_a+rou_b*std::sin(phi_r))*(rou_a+rou_b*std::sin(phi_r)) + (rou_b*std::cos(phi_r))*(rou_b*std::cos(phi_r));
  double rou_ref_2 = rou_b*rou_b;
  double rou2; // LoS or strongest reflection gain
  if (LoSStatus == LINE_OF_SIGHT)
  	{
  	  rou2 = rou_2;
  	}
  else
  	{
  	  rou2 = rou_ref_2;
  	}

  
  // The average tap weights
  std::vector<std::vector<double> > E_tap_W(num_cluster, std::vector<double>(num_ray)); // init as all zeros 

  // 1) path power gain of the cluster
  double d = L;
  double Omg_0_dB;
  if (LoSStatus == LINE_OF_SIGHT)
  	{
  	  Omg_0_dB = 3.46*d - 30.4;
  	}
  else
  	{
  	  Omg_0_dB = 4.44*d - 37.4;
  	}
  // double Omg_0 = std::pow(10,Omg_0_dB/10);

  // 2) cluster power-decay time constants
  double Gamma = 22.3;
  if (LoSStatus == LINE_OF_SIGHT)
  	{
  	  Gamma = 22.3; // ns
  	}
  else
  	{
  	  Gamma = 21.1; // ns
  	}

  double t_cluster_cur = 0; // current arrival time of the cluster
  for (int i = 0; i < num_cluster; ++i)
  	{
  	   double T_l_interval = 0.0;
	   RngSeedManager::SetSeed (2); // random seed setting
  	   // 3) the cluster arrival time
  	   if (LoSStatus == LINE_OF_SIGHT)
  	     {
  	        // exponatial decay distribution
  	        Ptr<ExponentialRandomVariable> tlv = CreateObject<ExponentialRandomVariable> ();
  			tlv->SetAttribute ("Mean", DoubleValue (0.047)); // ns
  			tlv->SetAttribute ("Bound", DoubleValue (1.0)); // ns
  			T_l_interval = tlv->GetValue (); // ns
  	     }
       else
  	     {
  	        // exponatial decay distribution
  	        Ptr<ExponentialRandomVariable> tlv = CreateObject<ExponentialRandomVariable> ();
  			tlv->SetAttribute ("Mean", DoubleValue (0.037)); // ns
  			tlv->SetAttribute ("Bound", DoubleValue (1.0)); // ns
  			T_l_interval = tlv->GetValue (); // ns
  	     }
	   double T_l = t_cluster_cur;
	   t_cluster_cur = t_cluster_cur + T_l_interval;

	   // 4) ray power-decay time constants
	   double gamma_pre = 4; // ns
  	   double gamma_post = 5.4; // ns
	   if (LoSStatus == LINE_OF_SIGHT)
  		{
  	  	   gamma_pre = 4; // ns
  	  	   gamma_post = 5.4; // ns
  		}
  	   else
  		{
  	       gamma_pre = 3.9; // ns
  	       gamma_post = 4.5; //ns
  		}

	   // 5) the Ricean factor introduced to improve the fit of the model into the experimental data
	   double K_r_dB_pre;
  	   double K_r_dB_post;
	   if (LoSStatus == LINE_OF_SIGHT)
  		{
  	  	  K_r_dB_pre = 11.5; // ns
  	  	  K_r_dB_post = 8.4; // ns
  		}
  	   else
  		{
  	      K_r_dB_pre = 3.3; // ns
  	  	  K_r_dB_post = 8.9; // ns
  		}
	   // double K_r_pre = std::pow(10,K_r_dB_pre/10);

	   // current arrival time of the ray
	   double t_ray_cur = t_cluster_cur;

	   // choose the central ray
	   Ptr<UniformRandomVariable> centralRayID = CreateObject<UniformRandomVariable> ();
       centralRayID->SetAttribute ("Min", DoubleValue (0.0));
       centralRayID->SetAttribute ("Max", DoubleValue (num_ray-0.01));
       uint16_t j_central = floor(centralRayID->GetValue ()); 

	   for (int j = 0; j < num_ray; ++j)
	   	{
	   	   // 6) the ray's arrival time
	   	   double t_l_interval_pre = 0.0;
		   double t_l_interval_post = 0.0;
		   if (LoSStatus == LINE_OF_SIGHT)
  	     	{
  	           // exponatial decay distribution
  	           Ptr<ExponentialRandomVariable> tlv_ray_pre = CreateObject<ExponentialRandomVariable> ();
  			   tlv_ray_pre->SetAttribute ("Mean", DoubleValue (0.5)); // ns
  			   tlv_ray_pre->SetAttribute ("Bound", DoubleValue (10.0)); // ns
  			   t_l_interval_pre = tlv_ray_pre->GetValue (); // ns

			   Ptr<ExponentialRandomVariable> tlv_ray_post = CreateObject<ExponentialRandomVariable> ();
  			   tlv_ray_post->SetAttribute ("Mean", DoubleValue (0.5)); // ns
  			   tlv_ray_post->SetAttribute ("Bound", DoubleValue (10.0)); // ns
  			   t_l_interval_post = tlv_ray_post->GetValue (); // ns
  	        }
       	   else
  	     	{
  	           // exponatial decay distribution
  	           Ptr<ExponentialRandomVariable> tlv_ray_pre = CreateObject<ExponentialRandomVariable> ();
  			   tlv_ray_pre->SetAttribute ("Mean", DoubleValue (0.7)); // ns
  			   tlv_ray_pre->SetAttribute ("Bound", DoubleValue (10.0)); // ns
  			   t_l_interval_pre = tlv_ray_pre->GetValue (); // ns

			   Ptr<ExponentialRandomVariable> tlv_ray_post = CreateObject<ExponentialRandomVariable> ();
  			   tlv_ray_post->SetAttribute ("Mean", DoubleValue (1.2)); // ns
  			   tlv_ray_post->SetAttribute ("Bound", DoubleValue (10.0)); // ns
  			   t_l_interval_post = tlv_ray_post->GetValue (); // ns
  	     	}

		   // pre cursor
		   double t_l;
		   if (j < j_central)
		   	{
		   	   t_ray_cur = t_ray_cur - t_l_interval_pre;
			   t_l = t_ray_cur;
			   // if it is the first cluster
			   if (i == 0)
			   	{
			   	   E_tap_W[i][j] = Omg_0_dB * exp((-1.0)*T_l/Gamma) * exp((-1.0)*t_l/gamma_pre); 
			   	}
			   else
			   	{
			   	   E_tap_W[i][j] = Omg_0_dB * exp((-1.0)*T_l/Gamma) * exp((-1.0)*t_l/gamma_pre) - K_r_dB_pre; 
			   	}		   
		   	}
		   // central ray
		   else if (j == j_central)
		   	{
		   	   t_ray_cur = t_cluster_cur;
			   E_tap_W[i][j] = Omg_0_dB * exp((-1.0)*T_l/Gamma);
		   	}
		   // post cursor
		   else
		   	{
		   	   t_ray_cur = t_ray_cur + t_l_interval_post;
			   t_l = t_ray_cur;
			   // if it is the first cluster
			   if (i == 0)
			   	{
			   	   E_tap_W[i][j] = Omg_0_dB * exp((-1.0)*T_l/Gamma) * exp((-1.0)*t_l/gamma_post); 
			   	}
			   else
			   	{
			   	   E_tap_W[i][j] = Omg_0_dB * exp((-1.0)*T_l/Gamma) * exp((-1.0)*t_l/gamma_post) - K_r_dB_post; 
			    }		   
		   	}
		   E_tap_W[i][j] = std::pow(10, E_tap_W[i][j]/10);
	   	}	   	   
  	}

  // path/channel gain
  double sum_E = 0.0;
  double G = 0.0;

  for (int i = 0; i < num_cluster; ++i)
  	{
  	  for (int j = 0; j < num_ray; ++j)
  	  	{
  	  	  sum_E = sum_E + E_tap_W[i][j];
  	  	}
  	}
  
  
  G = (lambda_w/(4*PI*L))*(lambda_w/(4*PI*L))*rou2 * sum_E;

  double G_dB = 10.0*std::log10(G);
  /*
  // add penetration loss for NLoS case, plus blocked LoS component
  if (LoSStatus != LINE_OF_SIGHT)
  	{
  	  // give the penertration loss limitation, get rid of those very high/low values
  	  if (X_pen_dB > -15)
  	  	{
  	  		X_pen_dB = -15; // 20
  	  	}
	  if (X_pen_dB < -30)
  	  	{
  	  		X_pen_dB = -30; // 35
  	  	}
  	  
  	  G_dB = G_dB + 10.0*std::log10((lambda_w/(4*PI*L))*(lambda_w/(4*PI*L))*rou2) + X_pen_dB;
	  // std::cerr << "G_dB for NLoS " << G_dB << " Pen_dB" << X_pen_dB << std::endl;
  	}
  */

  G_sv = G_dB;

  return G_sv;
}


double 
Obstacle::SingleCarrierPHYrSSMapping(double rSSdBm)
{
  double bitRate = 0.0;
  if (rSSdBm < -78.0)
  	{
  	  bitRate = 0.0;
  	}
  else if ((rSSdBm >= -78.0)&&(rSSdBm < -68.0))
  	{
  	  bitRate = 27.5;
  	}
  else if ((rSSdBm >= -68.0)&&(rSSdBm < -66.0))
  	{
  	  bitRate = 385.0;
  	}
  else if ((rSSdBm >= -66.0)&&(rSSdBm < -65.0))
  	{
  	  bitRate = 770.0;
  	}
  else if ((rSSdBm >= -65.0)&&(rSSdBm < -64.0))
  	{
  	  bitRate = 962.5;
  	}
  else if ((rSSdBm >= -64.0)&&(rSSdBm < -62.0))
  	{
  	  bitRate = 1155.0;
  	}
  else if ((rSSdBm >= -62.0)&&(rSSdBm < -61.0))
  	{
  	  bitRate = 1925.0;
  	}
  else if ((rSSdBm >= -61.0)&&(rSSdBm < -59.0))
  	{
  	  bitRate = 2310.0;
  	}
  else if ((rSSdBm >= -59.0)&&(rSSdBm < -55.0))
  	{
  	  bitRate = 2502.0;
  	}
  else if ((rSSdBm >= -55.0)&&(rSSdBm < -54.0))
  	{
  	  bitRate = 3080.0;
  	}
  else if ((rSSdBm >= -54.0)&&(rSSdBm < -53.0))
  	{
  	  bitRate = 3850.0;
  	}
  else
  	{
  	  bitRate = 4620.0;
  	}
  return bitRate;
}



int 
Obstacle::GetIntersection(float fDst1, float fDst2, Vector P1, Vector P2, Vector &Hit) 
{
  //NS_LOG_FUNCTION (this << fDst1 << fDst2);
  if ( (fDst1 * fDst2) >= 0.0f) return 0;
  if ( fDst1 == fDst2) return 0; 
  Hit = P1 + Vector (-fDst1/(fDst2-fDst1)*(P2.x-P1.x), -fDst1/(fDst2-fDst1)*(P2.y-P1.y), -fDst1/(fDst2-fDst1)*(P2.z-P1.z));
  return 1;
}

int 
Obstacle::InBox(Vector Hit, Vector B1, Vector B2, const int Axis) 
{
  //NS_LOG_FUNCTION (this << Hit << B1 << B2);
  if ( Axis==1 && Hit.z > B1.z && Hit.z < B2.z && Hit.y > B1.y && Hit.y < B2.y) 
    {
      m_hitList.push_back (Hit);
      return 1;
    }
  if ( Axis==2 && Hit.z > B1.z && Hit.z < B2.z && Hit.x > B1.x && Hit.x < B2.x)
    {
      m_hitList.push_back (Hit);
      return 1;
    }
  if ( Axis==3 && Hit.x > B1.x && Hit.x < B2.x && Hit.y > B1.y && Hit.y < B2.y)
    {
      m_hitList.push_back (Hit);
      return 1;
    }
  return 0;
}


std::vector<Vector>
Obstacle::IdentifyCLientLocation (uint16_t clientRS, uint16_t distRS, double variance, uint32_t clientNo, uint16_t clientDistType)
{
  NS_LOG_FUNCTION (this);
  // Set random seed
  m_clientRS = clientRS;
  gsl_rng_env_setup();                          // Read variable environnement
  const gsl_rng_type* type = gsl_rng_default;   // Default algorithm 'twister'
  gsl_rng *gen = gsl_rng_alloc (type);          // Rand generator allocation
  gsl_rng_set(gen, clientRS);


  std::vector<Vector> clientPos;
  bool heightDefined = false;
  for (uint16_t clientId = 0; clientId < clientNo; clientId++)
    {
     bool sfy = false;
     while (sfy == false)
     {
      sfy = true;
      double x = 0, y = 0, z = 0;
      uint8_t distributionType = clientDistType;

      // Client size configuration
      RngSeedManager::SetSeed (clientRS);
      Ptr<UniformRandomVariable> clientDimensionIndex = CreateObject<UniformRandomVariable> ();
      clientDimensionIndex->SetAttribute ("Min", DoubleValue (0.0));
      clientDimensionIndex->SetAttribute ("Max", DoubleValue (1.99));
      int clientType = floor(clientDimensionIndex->GetValue ());

      if (clientType == CELLPHONE_STA)
        m_clientDimension = Vector (0.157, 0.075, 0.008);  
      else if (clientType == LAPTOP_STA)
        m_clientDimension = Vector (0.33, 0.23, 0.25);

      if (distributionType == POISSON_DISTRIBUTION_CLIENT)
        {
          NS_LOG_FUNCTION (this);
          // Poisson distribution
          Ptr<UniformRandomVariable> xUV = CreateObject<UniformRandomVariable> ();
          xUV->SetAttribute ("Min", DoubleValue (0.0));
          xUV->SetAttribute ("Max", DoubleValue (m_roomSize.x));
          Ptr<UniformRandomVariable> yUV = CreateObject<UniformRandomVariable> ();
          yUV->SetAttribute ("Min", DoubleValue (0.0));
          yUV->SetAttribute ("Max", DoubleValue (m_roomSize.y));
          x=xUV->GetValue ();   
          y=yUV->GetValue ();  

		  // give the height, added by Yuchen 7/2020
		  Ptr<UniformRandomVariable> zUV = CreateObject<UniformRandomVariable> ();
          zUV->SetAttribute ("Min", DoubleValue (0.4)); // 0.3
          zUV->SetAttribute ("Max", DoubleValue (1.5)); // 1.5
          z=zUV->GetValue (); 
        }
      else if (distributionType == TRUNCATED_NORMAL_DISTRIBUTION_CLIENT)
        {
          RngSeedManager::SetSeed (distRS);
          Ptr<UniformRandomVariable> Mean = CreateObject<UniformRandomVariable> ();
          Mean->SetAttribute ("Min", DoubleValue (0.0));
          Mean->SetAttribute ("Max", DoubleValue (m_roomSize.x));
          m_xMean = Mean->GetValue ();
          Mean->SetAttribute ("Min", DoubleValue (0.0));
          Mean->SetAttribute ("Max", DoubleValue (m_roomSize.y));
          m_yMean = Mean->GetValue ();

          // Truncated Normal Distribution 
          std::vector<double> xTNStaDistConf = {0, m_roomSize.x, m_xMean, 1};
          std::vector<double> yTNStaDistConf = {0, m_roomSize.y, m_yMean, 1};

          std::pair<double, double> distSample;
          distSample = m_tNDist->rtnorm(gen,xTNStaDistConf.at(0),xTNStaDistConf.at(1),xTNStaDistConf.at(2),xTNStaDistConf.at(3));
          x = distSample.first;
          distSample = m_tNDist->rtnorm(gen,yTNStaDistConf.at(0),yTNStaDistConf.at(1),yTNStaDistConf.at(2),yTNStaDistConf.at(3));
          y = distSample.first;
          //std::cout<< m_xMean << " " << m_yMean << " " << x << " " << y << std::endl;
        }
      else if (distributionType == OBSTACLE_DEPENDENT_TRUNCATED_NORMAL_DISTRIBUTION_CLIENT)
        {
          // Identify which obstacle to select
          RngSeedManager::SetSeed (clientRS);
          Ptr<UniformRandomVariable> obstacleIndex = CreateObject<UniformRandomVariable> ();
          obstacleIndex->SetAttribute ("Min", DoubleValue (0.0));
          obstacleIndex->SetAttribute ("Max", DoubleValue (m_obstalceNumber-0.01));
          int obsId = floor(obstacleIndex->GetValue ());

          // Identify which side of obstacle to be distributed
          std::vector<double> xTNDistConf;
          std::vector<double> yTNDistConf; 
          obstacleIndex->SetAttribute ("Min", DoubleValue (0.0));
          obstacleIndex->SetAttribute ("Max", DoubleValue (1.99));
 
          int distSide = floor(obstacleIndex->GetValue ());
          if (distSide == 0)
              xTNDistConf = {0, m_obstacleDimension.at(obsId).xMin, 0.5*(m_obstacleDimension.at(obsId).xMax+m_obstacleDimension.at(obsId).xMin), variance}; 
          else
              xTNDistConf = {m_obstacleDimension.at(obsId).xMax, m_roomSize.x, 0.5*(m_obstacleDimension.at(obsId).xMax+m_obstacleDimension.at(obsId).xMin), variance};

          distSide = floor(obstacleIndex->GetValue ());
          if (distSide == 0)
              yTNDistConf = {0, m_obstacleDimension.at(obsId).yMin, 0.5*(m_obstacleDimension.at(obsId).yMax+m_obstacleDimension.at(obsId).yMin), variance};
          else
              yTNDistConf = {m_obstacleDimension.at(obsId).yMax, m_roomSize.y, 0.5*(m_obstacleDimension.at(obsId).yMax+m_obstacleDimension.at(obsId).yMin), variance};

          // Truncated Normal Distribution
          std::pair<double, double> distSample;
          distSample = m_tNDist->rtnorm(gen,xTNDistConf.at(0),xTNDistConf.at(1),xTNDistConf.at(2),xTNDistConf.at(3));
          x = distSample.first;
          distSample = m_tNDist->rtnorm(gen,yTNDistConf.at(0),yTNDistConf.at(1),yTNDistConf.at(2),yTNDistConf.at(3));
          y = distSample.first;
          //std::cout<< m_obstacleDimension.at(obsId) << " " << obsId << " " << x << " " << y << std::endl; 
        }   
      else if (distributionType == OBSTACLE_DEPENDENT_DISTRIBUTION_CLIENT)
        {
          heightDefined = true;
          bool STAOutobs = false;
          while (STAOutobs == false)
            {
              // Identify which obstacle to select
              RngSeedManager::SetSeed (clientRS);
              Ptr<UniformRandomVariable> obstacleIndex = CreateObject<UniformRandomVariable> ();
              obstacleIndex->SetAttribute ("Min", DoubleValue (0.0));
              obstacleIndex->SetAttribute ("Max", DoubleValue (m_obstalceNumber-0.01));
              int obsId = floor(obstacleIndex->GetValue ());

              // Identify which side of obstacle to be distributed
              std::vector<double> xTNDistConf;
              std::vector<double> yTNDistConf; 
              obstacleIndex->SetAttribute ("Min", DoubleValue (0.0));
              obstacleIndex->SetAttribute ("Max", DoubleValue (4.99));
 
              RngSeedManager::SetSeed (clientRS);
              Ptr<UniformRandomVariable> clientCoordinateX = CreateObject<UniformRandomVariable> ();
              clientCoordinateX->SetAttribute ("Min", DoubleValue (m_obstacleDimension.at(obsId).xMin)); 
              clientCoordinateX->SetAttribute ("Max", DoubleValue (m_obstacleDimension.at(obsId).xMax));
              Ptr<UniformRandomVariable> clientCoordinateY = CreateObject<UniformRandomVariable> ();
              clientCoordinateY->SetAttribute ("Min", DoubleValue (m_obstacleDimension.at(obsId).yMin)); 
              clientCoordinateY->SetAttribute ("Max", DoubleValue (m_obstacleDimension.at(obsId).yMax));
              Ptr<UniformRandomVariable> clientCoordinateZ = CreateObject<UniformRandomVariable> ();
              clientCoordinateZ->SetAttribute ("Min", DoubleValue (m_obstacleDimension.at(obsId).zMin)); 
              clientCoordinateZ->SetAttribute ("Max", DoubleValue (m_obstacleDimension.at(obsId).zMax));

              int distSide = floor(obstacleIndex->GetValue ()); // left, front, right, rear, up
              if (distSide == 0)
                {
                  x = m_obstacleDimension.at(obsId).xMin;
                  y = clientCoordinateY->GetValue (); 
                  z = clientCoordinateZ->GetValue ();   
                  x -= m_clientDimension.x*0.5;          
                }
              else if (distSide == 1)
                {
                  x = clientCoordinateX->GetValue (); 
                  y = m_obstacleDimension.at(obsId).yMin;
                  z = clientCoordinateZ->GetValue ();
                  y -= m_clientDimension.y*0.5;         
                }
              else if (distSide == 2)
                {
                  x = m_obstacleDimension.at(obsId).xMax;
                  y = clientCoordinateY->GetValue (); 
                  z = clientCoordinateZ->GetValue (); 
                  x += m_clientDimension.x*0.5;             
                }
              else if (distSide == 3)
                {
                  x = clientCoordinateX->GetValue (); 
                  y = m_obstacleDimension.at(obsId).yMax;
                  z = clientCoordinateZ->GetValue (); 
                  y += m_clientDimension.y*0.5;     
                }
              else if (distSide == 4)
                {
                  x = clientCoordinateX->GetValue ();
                  y = clientCoordinateY->GetValue (); 
                  z = m_obstacleDimension.at(obsId).zMax; 
                  z += m_clientDimension.z*0.5;    
                }
              STAOutobs = true;
              for (uint32_t i=0; i<m_obstalceNumber; i++) 
              if (x <= m_obstacleDimension.at(i).xMax && x >= m_obstacleDimension.at(i).xMin && y <= m_obstacleDimension.at(i).yMax && y >= m_obstacleDimension.at(i).yMin && z <= m_obstacleDimension.at(i).zMax && z >= m_obstacleDimension.at(i).zMin)
                {
                  STAOutobs = false;
                  break; 
                }
            }
          //std::cout<< m_obstacleDimension.at(obsId) << " " << m_clientDimension << " " << distSide << " " << x << " " << y << " " << z << std::endl; 
        } 

      // check if client is inside the obstacle (fixed + human obstacles)
      if (distributionType != OBSTACLE_DEPENDENT_DISTRIBUTION_CLIENT)
      	{
         for (uint32_t i=0; i<m_obstalceNumber + m_obstalceNumber_human; i++) 
          {
            if (x <= m_obstacleDimension.at(i).xMax && x >= m_obstacleDimension.at(i).xMin && y <= m_obstacleDimension.at(i).yMax && y >= m_obstacleDimension.at(i).yMin && z <= m_obstacleDimension.at(i).zMax && z >= m_obstacleDimension.at(i).zMin)
              {
		         sfy = false;
			     break;
              }
	        else
	  	      {
	  	         sfy = true;
	  	      }
      	  }
      	}
	  
	  if (sfy == false)
	  	{
	  	  continue; // continue to find a new client position
	  	}

      // Identigy height of STA
      if (heightDefined == false)
        {
          for (uint32_t i=0; i<m_obstalceNumber; i++) 
           {
             if (x <= m_obstacleDimension.at(i).xMax && x >= m_obstacleDimension.at(i).xMin && y <= m_obstacleDimension.at(i).yMax && y >= m_obstacleDimension.at(i).yMin)
              {
				  z = m_obstacleDimension.at(i).zMax;
                  break;
              }
           }
          z += m_clientDimension.z*0.5; 
        }
      // Identify client dimension
      m_clientSize = Box (x-m_clientDimension.x*0.5, x+m_clientDimension.x*0.5, y-m_clientDimension.y*0.5, y+m_clientDimension.y*0.5, z-m_clientDimension.z*0.5, z+m_clientDimension.z*0.5);

      NS_LOG_FUNCTION ("client location" << x << y << z << " client dimension " << m_clientDimension);
      clientPos.push_back (Vector (x, y, z));
    }
  	}
  return clientPos;  
}


std::vector<Vector> 
Obstacle::IdentifyCLientLocation_KnownLoc (uint32_t clientNo, std::vector<Vector> knownLoc)
{
  std::vector<Vector> clientPos;
  std::vector<Vector> knownLoc_temp = knownLoc;

  for (uint16_t clientId = 0; clientId < clientNo; clientId++)
  	{
  	    int numLoc = knownLoc_temp.size();
  		Ptr<UniformRandomVariable> LocID = CreateObject<UniformRandomVariable> ();
  		LocID->SetAttribute ("Min", DoubleValue (0.0));
  		LocID->SetAttribute ("Max", DoubleValue (numLoc -0.01));
  		uint16_t LocNo = floor(LocID->GetValue ());

		// assign the client locations
		clientPos.push_back(knownLoc_temp.at(LocNo));

		// avoid that multiple clients in the same location
		knownLoc_temp.erase(knownLoc_temp.begin() + LocNo);
  	}
    
  return clientPos;
}



std::vector<Vector>
Obstacle::IdentifyCLientLocation_BL (uint16_t clientRS, uint16_t distRS, double variance, uint32_t clientNo, uint16_t clientDistType, std::vector<uint16_t> numObstacles, std::vector<Vector> multiRoomSize, std::vector<std::vector<Box> > obsDimRoom, std::vector<Box> roomConf, uint16_t numRooms, std::vector<double> perfloorHeight)
{
  NS_LOG_FUNCTION (this);
  // Set random seed
  m_clientRS = clientRS;
  gsl_rng_env_setup();                          // Read variable environnement
  const gsl_rng_type* type = gsl_rng_default;   // Default algorithm 'twister'
  gsl_rng *gen = gsl_rng_alloc (type);          // Rand generator allocation
  gsl_rng_set(gen, clientRS);

  // 1) generate the location area in the building, e.g., room 1, 2 ..., or hall
  // hall --- (-1)
  std::vector<int16_t> areaID;
  std::vector<Vector> clientPos_pre;
  for (uint16_t clientId = 0; clientId < clientNo; clientId++)
    {
      bool stf = false;
      while (stf == false)
      	{
      		double x = 0, y = 0, z = 0;
	  		RngSeedManager::SetSeed (clientRS);
  	  		Ptr<UniformRandomVariable> xUV = CreateObject<UniformRandomVariable> ();
      		xUV->SetAttribute ("Min", DoubleValue (0.0));
      		xUV->SetAttribute ("Max", DoubleValue (m_roomSize.x));
      		Ptr<UniformRandomVariable> yUV = CreateObject<UniformRandomVariable> ();
      		yUV->SetAttribute ("Min", DoubleValue (0.0));
      		yUV->SetAttribute ("Max", DoubleValue (m_roomSize.y));
      		x=xUV->GetValue ();   
      		y=yUV->GetValue ();

	  		Ptr<UniformRandomVariable> zUV = CreateObject<UniformRandomVariable> ();
      		zUV->SetAttribute ("Min", DoubleValue (0.4)); // 0.3
      		zUV->SetAttribute ("Max", DoubleValue (1.5)); // 1.5
      		z=zUV->GetValue (); 

			// generate the floor ID for this client
			// choose the central ray
	        Ptr<UniformRandomVariable> floorID = CreateObject<UniformRandomVariable> ();
            floorID->SetAttribute ("Min", DoubleValue (1.0));
            floorID->SetAttribute ("Max", DoubleValue (m_numFloors_BL + 1.0 -0.01));
            uint16_t floorNo = floor(floorID->GetValue ()); 

			// client's real z coordinate
			double height_base = 0.0;
			for (uint16_t hi = 1; hi <= floorNo; ++hi)
				{
				  if (hi >= 2)
				  	{
				      height_base += perfloorHeight.at(hi-2);
				  	}
				}
			double z_real = z + height_base;
			

	  		// check in which area
	  		bool inRoom = false;
	  		for (uint16_t ia = 0; ia < numRooms; ++ia)
	  		{
	  	  		Box thisRoom = roomConf.at(ia); // real coordinates
		  		if ((x <= thisRoom.xMax) && (x >= thisRoom.xMin) && (y <= thisRoom.yMax) && (y >= thisRoom.yMin)&& (z_real <= thisRoom.zMax) && (z_real >= thisRoom.zMin))
		  	    {
		  	  		inRoom = true;
			  		areaID.push_back(ia); // get room ID
			  		stf = true;
			  		break;
		  		}
	  		}
	  		if (inRoom == false)
	  		{
	  		    if (m_allowClientInHall_BL == true)
	  		    {
	  	   		  areaID.push_back(-1); // in the hall
	  	   		  stf = true;
	  		    }
				else
				{
				  continue;
				}
	  		}
			
			if (areaID.at(areaID.size()-1) == -1) // in the hall
				{
				   clientPos_pre.push_back(Vector (x, y, z_real));
				}
			else
				{
	  			   clientPos_pre.push_back(Vector (x, y, z));
				}
  		}
  	}
  // record the areaID
  m_roomNum = areaID;
  

  // 2) generate the specific positions of the clients
  std::vector<Vector> clientPos; // virtual position
  bool heightDefined = false;
  for (uint16_t clientId = 0; clientId < clientNo; clientId++)
    {
     bool sfy = false;
     while (sfy == false)
     {
      sfy = true;
      double x = 0, y = 0, z = 0;
      uint8_t distributionType = clientDistType;

	  int16_t roomID = areaID.at(clientId);

	  if (roomID == -1) // if in the hall
	  	{
	  	  clientPos.push_back(clientPos_pre.at(clientId));
		  m_clientPos_BL.push_back(clientPos_pre.at(clientId));
		  continue;
	  	}

	  // this rooms config
	  Vector roomSize = multiRoomSize.at(roomID);
	  uint32_t obstacleNumber = numObstacles.at(roomID); // fixed obstacles
	  std::vector<Box> obstacleDimension = obsDimRoom.at(roomID);

      // Client size configuration
      RngSeedManager::SetSeed (clientRS);
      Ptr<UniformRandomVariable> clientDimensionIndex = CreateObject<UniformRandomVariable> ();
      clientDimensionIndex->SetAttribute ("Min", DoubleValue (0.0));
      clientDimensionIndex->SetAttribute ("Max", DoubleValue (1.99));
      int clientType = floor(clientDimensionIndex->GetValue ());

      if (clientType == CELLPHONE_STA)
        m_clientDimension = Vector (0.157, 0.075, 0.008);  
      else if (clientType == LAPTOP_STA)
        m_clientDimension = Vector (0.33, 0.23, 0.25);

      if (distributionType == POISSON_DISTRIBUTION_CLIENT)
        {
          NS_LOG_FUNCTION (this);
          // Poisson distribution
          Ptr<UniformRandomVariable> xUV = CreateObject<UniformRandomVariable> ();
          xUV->SetAttribute ("Min", DoubleValue (0.0));
          xUV->SetAttribute ("Max", DoubleValue (roomSize.x));
          Ptr<UniformRandomVariable> yUV = CreateObject<UniformRandomVariable> ();
          yUV->SetAttribute ("Min", DoubleValue (0.0));
          yUV->SetAttribute ("Max", DoubleValue (roomSize.y));
          x=xUV->GetValue ();   
          y=yUV->GetValue ();  

		  // give the height, added by Yuchen 7/2020
		  Ptr<UniformRandomVariable> zUV = CreateObject<UniformRandomVariable> ();
          zUV->SetAttribute ("Min", DoubleValue (0.4)); // 0.3
          zUV->SetAttribute ("Max", DoubleValue (1.5)); // 1.5
          z=zUV->GetValue (); 
        }
      else if (distributionType == TRUNCATED_NORMAL_DISTRIBUTION_CLIENT)
        {
          RngSeedManager::SetSeed (distRS);
          Ptr<UniformRandomVariable> Mean = CreateObject<UniformRandomVariable> ();
          Mean->SetAttribute ("Min", DoubleValue (0.0));
          Mean->SetAttribute ("Max", DoubleValue (roomSize.x));
          m_xMean = Mean->GetValue ();
          Mean->SetAttribute ("Min", DoubleValue (0.0));
          Mean->SetAttribute ("Max", DoubleValue (roomSize.y));
          m_yMean = Mean->GetValue ();

          // Truncated Normal Distribution 
          std::vector<double> xTNStaDistConf = {0, roomSize.x, m_xMean, 1};
          std::vector<double> yTNStaDistConf = {0, roomSize.y, m_yMean, 1};

          std::pair<double, double> distSample;
          distSample = m_tNDist->rtnorm(gen,xTNStaDistConf.at(0),xTNStaDistConf.at(1),xTNStaDistConf.at(2),xTNStaDistConf.at(3));
          x = distSample.first;
          distSample = m_tNDist->rtnorm(gen,yTNStaDistConf.at(0),yTNStaDistConf.at(1),yTNStaDistConf.at(2),yTNStaDistConf.at(3));
          y = distSample.first;
          //std::cout<< m_xMean << " " << m_yMean << " " << x << " " << y << std::endl;
        }
      else if (distributionType == OBSTACLE_DEPENDENT_TRUNCATED_NORMAL_DISTRIBUTION_CLIENT)
        {
          // Identify which obstacle to select
          RngSeedManager::SetSeed (clientRS);
          Ptr<UniformRandomVariable> obstacleIndex = CreateObject<UniformRandomVariable> ();
          obstacleIndex->SetAttribute ("Min", DoubleValue (0.0));
          obstacleIndex->SetAttribute ("Max", DoubleValue (obstacleNumber-0.01));
          int obsId = floor(obstacleIndex->GetValue ());

          // Identify which side of obstacle to be distributed
          std::vector<double> xTNDistConf;
          std::vector<double> yTNDistConf; 
          obstacleIndex->SetAttribute ("Min", DoubleValue (0.0));
          obstacleIndex->SetAttribute ("Max", DoubleValue (1.99));
 
          int distSide = floor(obstacleIndex->GetValue ());
          if (distSide == 0)
              xTNDistConf = {0, obstacleDimension.at(obsId).xMin, 0.5*(obstacleDimension.at(obsId).xMax+obstacleDimension.at(obsId).xMin), variance}; 
          else
              xTNDistConf = {obstacleDimension.at(obsId).xMax, roomSize.x, 0.5*(obstacleDimension.at(obsId).xMax+obstacleDimension.at(obsId).xMin), variance};

          distSide = floor(obstacleIndex->GetValue ());
          if (distSide == 0)
              yTNDistConf = {0, obstacleDimension.at(obsId).yMin, 0.5*(obstacleDimension.at(obsId).yMax+obstacleDimension.at(obsId).yMin), variance};
          else
              yTNDistConf = {obstacleDimension.at(obsId).yMax, roomSize.y, 0.5*(obstacleDimension.at(obsId).yMax+obstacleDimension.at(obsId).yMin), variance};

          // Truncated Normal Distribution
          std::pair<double, double> distSample;
          distSample = m_tNDist->rtnorm(gen,xTNDistConf.at(0),xTNDistConf.at(1),xTNDistConf.at(2),xTNDistConf.at(3));
          x = distSample.first;
          distSample = m_tNDist->rtnorm(gen,yTNDistConf.at(0),yTNDistConf.at(1),yTNDistConf.at(2),yTNDistConf.at(3));
          y = distSample.first;
          //std::cout<< obstacleDimension.at(obsId) << " " << obsId << " " << x << " " << y << std::endl; 
        }   
      else if (distributionType == OBSTACLE_DEPENDENT_DISTRIBUTION_CLIENT)
        {
          heightDefined = true;
          bool STAOutobs = false;
          while (STAOutobs == false)
            {
              // Identify which obstacle to select
              RngSeedManager::SetSeed (clientRS);
              Ptr<UniformRandomVariable> obstacleIndex = CreateObject<UniformRandomVariable> ();
              obstacleIndex->SetAttribute ("Min", DoubleValue (0.0));
              obstacleIndex->SetAttribute ("Max", DoubleValue (obstacleNumber-0.01));
              int obsId = floor(obstacleIndex->GetValue ());

              // Identify which side of obstacle to be distributed
              std::vector<double> xTNDistConf;
              std::vector<double> yTNDistConf; 
              obstacleIndex->SetAttribute ("Min", DoubleValue (0.0));
              obstacleIndex->SetAttribute ("Max", DoubleValue (4.99));
 
              RngSeedManager::SetSeed (clientRS);
              Ptr<UniformRandomVariable> clientCoordinateX = CreateObject<UniformRandomVariable> ();
              clientCoordinateX->SetAttribute ("Min", DoubleValue (obstacleDimension.at(obsId).xMin)); 
              clientCoordinateX->SetAttribute ("Max", DoubleValue (obstacleDimension.at(obsId).xMax));
              Ptr<UniformRandomVariable> clientCoordinateY = CreateObject<UniformRandomVariable> ();
              clientCoordinateY->SetAttribute ("Min", DoubleValue (obstacleDimension.at(obsId).yMin)); 
              clientCoordinateY->SetAttribute ("Max", DoubleValue (obstacleDimension.at(obsId).yMax));
              Ptr<UniformRandomVariable> clientCoordinateZ = CreateObject<UniformRandomVariable> ();
              clientCoordinateZ->SetAttribute ("Min", DoubleValue (obstacleDimension.at(obsId).zMin)); 
              clientCoordinateZ->SetAttribute ("Max", DoubleValue (obstacleDimension.at(obsId).zMax));

              int distSide = floor(obstacleIndex->GetValue ()); // left, front, right, rear, up
              if (distSide == 0)
                {
                  x = obstacleDimension.at(obsId).xMin;
                  y = clientCoordinateY->GetValue (); 
                  z = clientCoordinateZ->GetValue ();   
                  x -= m_clientDimension.x*0.5;          
                }
              else if (distSide == 1)
                {
                  x = clientCoordinateX->GetValue (); 
                  y = obstacleDimension.at(obsId).yMin;
                  z = clientCoordinateZ->GetValue ();
                  y -= m_clientDimension.y*0.5;         
                }
              else if (distSide == 2)
                {
                  x = obstacleDimension.at(obsId).xMax;
                  y = clientCoordinateY->GetValue (); 
                  z = clientCoordinateZ->GetValue (); 
                  x += m_clientDimension.x*0.5;             
                }
              else if (distSide == 3)
                {
                  x = clientCoordinateX->GetValue (); 
                  y = obstacleDimension.at(obsId).yMax;
                  z = clientCoordinateZ->GetValue (); 
                  y += m_clientDimension.y*0.5;     
                }
              else if (distSide == 4)
                {
                  x = clientCoordinateX->GetValue ();
                  y = clientCoordinateY->GetValue (); 
                  z = obstacleDimension.at(obsId).zMax; 
                  z += m_clientDimension.z*0.5;    
                }
              STAOutobs = true;
              for (uint32_t i=0; i<obstacleNumber; i++) 
              if (x <= obstacleDimension.at(i).xMax && x >= obstacleDimension.at(i).xMin && y <= obstacleDimension.at(i).yMax && y >= obstacleDimension.at(i).yMin && z <= obstacleDimension.at(i).zMax && z >= obstacleDimension.at(i).zMin)
                {
                  STAOutobs = false;
                  break; 
                }
            }
          //std::cout<< obstacleDimension.at(obsId) << " " << m_clientDimension << " " << distSide << " " << x << " " << y << " " << z << std::endl; 
        } 
      
      // check if client is inside the obstacle (fixed + human obstacles)
      if (distributionType != OBSTACLE_DEPENDENT_DISTRIBUTION_CLIENT)
      	{
         for (uint32_t i=0; i<obstacleDimension.size(); i++) 
          {
            if (x <= obstacleDimension.at(i).xMax && x >= obstacleDimension.at(i).xMin && y <= obstacleDimension.at(i).yMax && y >= obstacleDimension.at(i).yMin && z <= obstacleDimension.at(i).zMax && z >= obstacleDimension.at(i).zMin)
              {
		         sfy = false;
			     break;
              }
	        else
	  	      {
	  	         sfy = true;
	  	      }
      	  }
      	}
	  
	  if (sfy == false)
	  	{
	  	  continue; // continue to find a new client position
	  	}
	  
      // Identigy height of STA
      if (heightDefined == false)
        {
          for (uint32_t i=0; i<obstacleNumber; i++) 
          if (x <= obstacleDimension.at(i).xMax && x >= obstacleDimension.at(i).xMin && y <= obstacleDimension.at(i).yMax && y >= obstacleDimension.at(i).yMin)
            {
              z = obstacleDimension.at(i).zMax;
              break;
            } 
          z += m_clientDimension.z*0.5; 
        }
      // Identify client dimension
      m_clientSize = Box (x-m_clientDimension.x*0.5, x+m_clientDimension.x*0.5, y-m_clientDimension.y*0.5, y+m_clientDimension.y*0.5, z-m_clientDimension.z*0.5, z+m_clientDimension.z*0.5);

      NS_LOG_FUNCTION ("client location" << x << y << z << " client dimension " << m_clientDimension);
      clientPos.push_back (Vector (x, y, z));

	  // record the real coordinates of client positions in building level
	  double x_BL = x + roomConf.at(roomID).xMin;
	  double y_BL = y + roomConf.at(roomID).yMin;
	  double z_BL = z + roomConf.at(roomID).zMin;
	  m_clientPos_BL.push_back(Vector (x_BL, y_BL, z_BL));
     }
    }
  return clientPos;  
}


// Ang, 2/2021, for reflection functionality

Vector 
Obstacle::ptReflection(double x, double y, double z, int dir, double pos, double rl, double rw)
{
  double xx=0, yy=0;
  switch (dir) // skip if AP not in the right direction.
    {
      case 1:
          xx = 2*pos - x;
          yy = y;
          break;
      case 2:
          xx = - x + rl;
          yy = y;    
          break;
      case 3:
          yy = 2*pos - y;
          xx = x;
          break;
      case 4:
          yy = - y + rw;
          xx = x;
          break;
    } // end switch

  return Vector(xx, yy, z);
}

Vector 
Obstacle::ptReflectionOriginal(double x, double y, double z, int dir, double pos)
{
  double xx=0, yy=0;
  switch (dir) // skip if AP not in the right direction.
    {
      case 1:
          xx = 2*pos - x;
          yy = y;
          break;
      case 2:
          xx = 2*pos - x;
          yy = y;  
          break;
      case 3:
          yy = 2*pos - y;
          xx = x;
          break;
      case 4:
          yy = 2*pos - y;
          xx = x;
          break;
    } // end switch

  return Vector(xx, yy, z);
}

Vector 
Obstacle::LineFunc3Dcustom(Vector A, Vector B, double pos, bool isX)
{
  Vector slope = B - A;
  double t;

  if (isX)
    {
      t = (pos - A.x) / slope.x;
    }
  else
    {
      t = (pos - A.y) / slope.y;
    }

  return Vector(slope.x*t, slope.y*t, slope.z*t) + A;
}




} //namespace ns3
