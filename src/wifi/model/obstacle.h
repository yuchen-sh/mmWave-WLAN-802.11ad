/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2020 Yuchen and YUbing
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

#ifndef OBSTACLE_H
#define OBSTACLE_H

#include <ns3/vector.h>
#include <ns3/box.h>
#include <ns3/object.h>
#include "rtnorm.h"

namespace ns3 {

#define POISSON_DISTRIBUTION_CLIENT 0
#define TRUNCATED_NORMAL_DISTRIBUTION_CLIENT  1
#define OBSTACLE_DEPENDENT_TRUNCATED_NORMAL_DISTRIBUTION_CLIENT 2
#define OBSTACLE_DEPENDENT_DISTRIBUTION_CLIENT 3
#define LINE_OF_SIGHT 0
#define NON_LINE_OF_SIGHT 1
#define LINE_OF_SIGHT_PassWin 2
#define LINE_OF_SIGHT_noInf 0
#define LINE_OF_SIGHT_Inf 1
#define CELLPHONE_STA 0
#define LAPTOP_STA 1
#define TABLET_STA 2

class Obstacle : public Object
{
public:
  Obstacle ();
  virtual ~Obstacle ();

  std::pair<double, double> GetFadingInfo (Vector txPos, Vector rxPos);
  std::pair<double, double> GetFadingInfo_ext (Vector txPos, Vector rxPos);
  std::pair<int16_t, double> GetInterferenceInfo (Vector txPos, Vector rxPos);
  
  void SetObstacleNumber (uint16_t obsNumber);
  void SetMultiRoomFlag (bool multiRoomFlag);
  void SetHumanObstacleNumber (uint16_t obsNumber_human);
  void EnableSVChannel (bool SV_channel);
  void EnableTGadChannel (bool TGad_channel);
  void SetReflectedMode (bool reflectedMode);
  void SetRoomConf (Vector roomSize);
  void SetNumFloorsInbuilding (uint16_t numFloors);
  void SetAllowClientInHall (bool allowInHall);
  void SetPenetrationLossMode (std::vector<double> obstaclePenetrationLoss);
  void SetObsConflictCheck (bool setObsConflictCheck);
  void SetLoSFlag(bool LoSFlag, uint16_t clientID); // for one client (to one AP)
  void SetLoSFlag_ext(int16_t LoSFlag_ext, uint16_t clientID); // for one client (to one AP)
  void SetPenLoss(double penLoss, uint16_t clientID); // from associated AP to opening
  void SetDistToOpen(double distToOpen, uint16_t clientID); // from associated AP to opening
  void SetItfComp(uint16_t beItfed, uint16_t clientID); // double-room using
  void SetDistToSrc(double dist, uint16_t clientID); // double-room using
  void SetItfSrcPos(Vector itfSrcPos, uint16_t clientID); // double-room using
  void SetInterferFlag(bool InterferFlag, uint16_t clientID);
  void SetFadingLoss(double fadingLoss, uint16_t clientID); // for one client (to one AP)
  void SetAPPos(Vector apPos, uint16_t clientID); // for one client (to one AP)
  void SetClientPos(Vector clientPos, uint16_t clientID); // for one client (to one AP)
  void SetServedAPID(uint16_t APid, uint16_t clientID);
  
  bool RecCollision(std::vector<Box> preObs, double cx, double cy, double length, double width);
  void AllocateObstacle (Box railLocation, Vector roomSize,  uint16_t clientRS);
  void AllocateObstacle_human (Box railLocation, Vector roomSize,  uint16_t clientRS);
  // determine the optimal locations of APs
  std::vector<Vector> AllocateOptAP (Vector roomSize,  uint16_t numAPs);
  // for FORC scenarios
  void AllocateObstacle_Fixed(Box railLocation, Vector roomSize, uint16_t clientRS, std::vector<double> xPos, std::vector<double> yPos, std::vector<double> wObs, std::vector<double> lObs, std::vector<double> hObs, std::vector<double> dirObs);
  // with height bases of obstacles (not always 0 by default)
  void AllocateObstacle_Fixed_withHB(Box railLocation, Vector roomSize, uint16_t clientRS, std::vector<double> xPos, std::vector<double> yPos, std::vector<double> wObs, std::vector<double> lObs, std::vector<double> hObs, std::vector<double> hObs_min, std::vector<double> dirObs);
  void AllocateObstacle_KnownBox(std::vector<Box> obsDim);
  // add by Yuchen, 2019.4.16
  void AllocateObstacle_MultiAP(std::vector<Box> railLocation, Vector roomSize, uint16_t clientRS);
  void AddWallwithWindow(Vector wallSize, Vector wallCenter, Vector windowCenter, double windowLength, double windowWidth);
  std::vector<Vector> multiRoomConfig(std::vector<Box> multiRooms, Vector buildingSize, uint16_t numRooms); 
  	
  // add by Yuchen
  // std::vector<Vector> FindServingAP(std::vector<Vector> apLocation, std::vector<Vector> clientLocation, Vector apDimension);
  
  void LoSAnalysis (Vector apLocation, std::vector<Vector> clientLocation, Vector apDimension);
  void LoSAnalysisMultiAPItf (std::vector<Vector> apLocationAll, std::vector<Vector> clientLocation, Vector apDimension, double HPBF);
  // check if pass through the window
  void LoSAnalysis_ext (Vector apLocation, std::vector<Vector> clientLocation, Vector apDimension, std::vector<Vector> windowCenterVec, std::vector<double> windowLengthVec, std::vector<double> windowWidthVec);
  void LoSAnalysis_BL (Vector apLocation, std::vector<Vector> clientLocation, Vector apDimension, std::vector<int16_t> areaID, std::vector<uint16_t> numAPs, uint16_t APId, std::vector<std::vector<Box> > obsDimRoom, std::vector<uint16_t> numObstacles_fixed, std::vector<Ptr<Obstacle> > roomScenarios);
  std::vector<bool> LoSAnalysis_MultiAP (Vector apLocation, std::vector<Vector> clientLocation, Vector apDimension);
  std::vector<Vector> checkLoS (std::vector<Vector> apLocation_all, Vector clientLocation, std::vector<Box> obstacleDimension, Vector apDimension);
  std::pair<bool, double> checkLoS (Vector transmitter, Vector receiver);
  std::pair<uint16_t, double> checkLoS_withWall (Vector transmitter, Vector receiver);
  std::vector<std::vector<double> > InterferenceAnalysis (std::vector<Vector> apLocation, std::vector<Vector> clientLocation, uint16_t NumAP, uint16_t NumSTA, std::vector<std::vector<bool> > losFlag_mul, double txPowerdBm);  
  double SVChannelGain_inf(int reflectorDenseMode, Vector sender_pos, Vector receiver_pos, double txAntennaGain, double rxAntennaGain, bool LoSStatus);
  double SingleCarrierPHYrSSMapping(double rSSdBm);
  
  std::vector<Vector> IdentifyCLientLocation (uint16_t clientRS, uint16_t distRS, double variance, uint32_t clientNo, uint16_t clientDistType);
  std::vector<Vector> IdentifyCLientLocation_KnownLoc (uint32_t clientNo, std::vector<Vector> knownLoc);
  // building-level function
  std::vector<Vector> IdentifyCLientLocation_BL (uint16_t clientRS, uint16_t distRS, double variance, uint32_t clientNo, uint16_t clientDistType, std::vector<uint16_t> numObstacles, std::vector<Vector> multiRoomSize, std::vector<std::vector<Box> > obsDimRoom, std::vector<Box> roomConf, uint16_t numRooms, std::vector<double> perfloorHeight);
  int InBox(Vector Hit, Vector B1, Vector B2, const int Axis);

  int GetIntersection(float fDst1, float fDst2, Vector P1, Vector P2, Vector &Hit);
  std::vector<Box> GetObsDim ();
  std::vector<Vector> GetClientPos_BL (); // building level
  std::vector<int16_t> GetClientRoomID_BL (); // building level
  int16_t GetRoomIDforAP(uint16_t APID, std::vector<uint16_t> numAPs);
  uint16_t GetTotalAPNum_BL (std::vector<uint16_t> numAPs);
  std::vector<double> GetPenetrationLossMode ();
  uint32_t GetFixedObsNum();
  uint32_t GetHumanObsNum();
  bool GetLoSFlag(uint16_t clientID); // for one client (to one AP)
  bool GetMultiRoomFlag();
  int16_t GetLoSFlag_ext(uint16_t clientID); // for one client (to one AP)
  double GetPenLoss(uint16_t clientID); // from associated AP to the opening
  // double GetLoSFlagforIntfCal(uint16_t clientID);
  double GetDistToOpen(uint16_t clientID); // from associated AP to the opening
  bool GetInterferFlag(uint16_t clientID);
  double GetFadingLoss(uint16_t clientID); // for one client (to one AP)
  Vector GetAPPos(uint16_t clientID); // for one client (to one AP)
  Vector GetVirtualAPPos(uint16_t clientID);
  Vector GetClientPos(uint16_t clientID); // for one client (to one AP)
  // std::vector<ChannelInfo> GetChannelInfo ();
  uint16_t GetNumClient ();

  void CreatChannelInfo();
  double multiPathFadingVarianceGen();
  uint16_t checkItfAngleWithinBW (Vector apLocation, Vector client1, Vector client2, double BW);
  // This is the function for obtaining beInfed Flag for local clients (e.g., in parallel simulations with multiple rooms)
  void GetBeInterferedFlag (std::vector<Ptr<Obstacle> > otheRoomInfo_vec, Vector apLocation_local, std::vector<Vector> clientLocation_local, std::vector<Box> obstacleDimension_local, Vector apDimension);
  

  // Ang 1/2021 reflector
  // void checkIfRefExist (Vector apLocation, std::vector<Vector> clientLocation, int flagRefRange);
  Vector ptReflection(double x, double y, double z, int dir, double pos, double rl, double rw);
  Vector ptReflectionOriginal(double x, double y, double z, int dir, double pos);
  Vector LineFunc3Dcustom(Vector A, Vector B, double pos, bool isX);
  

  // Yuchen 7/2020
  std::vector<double> m_obstaclePenetrationLoss_low;
  std::vector<double> m_obstaclePenetrationLoss_medium;
  std::vector<double> m_obstaclePenetrationLoss_high;

  // Yuchen 11/2021
  // bool m_itfFlagforCal;

private:
  typedef struct {
    bool losFlag;
    double fadingLoss;
    Vector apPos;
    Vector clientPos;
	std::vector<uint16_t> servedAPId;
	bool losInfFlag;
	int16_t losFlag_ext; // 0 -- LoS, 1 -- NLoS, 2 -- LoS and will propagate to other room
	double penLoss; // penetration loss through the opening/window
	double distToOpen; // distance from AP to opening/window (-1 if NLoS to local user, > 0 for other LoS cases)
	uint16_t beInfed; // be interfered by other transmissions (number indicates if has interference component)
    double iftDist; // distance to the interfered source
    Vector iftSrcPos; // the position of the interfered source
    std::vector<Vector> iftSrcPosVec; // the position of the interfered source (record on the local client side)
    Vector virtualAPPos; // assume AP at the opening for interference calculation to the client receiver in other rooms
    std::vector<Vector> virtualAPPos_self; // assume AP at the opening for interference calculation to the client itself
    uint16_t openingId; // hit opening id (1 -- n)
  } ChannelInfo;

  std::vector<ChannelInfo> m_channelInfo;
  uint32_t m_obstalceNumber; // objects
  uint32_t m_obstalceNumber_human; // humans
  std::vector<Box> m_obstacleDimension;
  Vector m_apDimension;
  std::vector<Vector> m_hitList;
  std::vector<double> m_obstaclePenetrationLoss;
  std::vector<double> m_xObsTNDistConf, m_yObsTNDistConf, m_zObsTNDistConf;
  std::vector<double> m_xObsTNDistConf_human, m_yObsTNDistConf_human, m_zObsTNDistConf_human; // Yuchen 7/2020
  bool m_OnObsConflictCheck;

  Vector m_roomSize;
  double m_xMean;
  double m_yMean;
  Vector m_clientDimension;
  Ptr<TruncatedNormalDistribution> m_tNDist;
  Box m_clientSize;
  uint16_t m_clientRS;
  bool m_SV_channel; // Yuchen 7/2020
  bool m_TGad_channel;
  int m_reflectionMode; // Yuchen 7/2020
  std::vector<int16_t> m_roomNum; // check each client belongs to which room in a building
  std::vector<Vector> m_clientPos_BL; // building-level client positions
  bool m_allowClientInHall_BL;
  uint16_t m_numFloors_BL;
  bool m_multiRoomFlag;


  // Ang 1/2021 reflector
  std::vector<bool> m_hasRefPath;
  std::vector<double> m_refPathTotalDist;
  std::vector<Vector> m_refCenterLoc;

};


} //namespace ns3

#endif /* OBSTACLE_H */
