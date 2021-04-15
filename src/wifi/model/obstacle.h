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
#define CELLPHONE_STA 0
#define LAPTOP_STA 1
#define TABLET_STA 2

class Obstacle : public Object
{
public:
  Obstacle ();
  virtual ~Obstacle ();

  std::pair<double, double> GetFadingInfo (Vector txPos, Vector rxPos);
  
  void SetObstacleNumber (uint16_t obsNumber);
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
  void SetFadingLoss(double fadingLoss, uint16_t clientID); // for one client (to one AP)
  void SetAPPos(Vector apPos, uint16_t clientID); // for one client (to one AP)
  void SetClientPos(Vector clientPos, uint16_t clientID); // for one client (to one AP)
  
  bool RecCollision(std::vector<Box> preObs, double cx, double cy, double length, double width);
  void AllocateObstacle (Box railLocation, Vector roomSize,  uint16_t clientRS);
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
  void LoSAnalysis_BL (Vector apLocation, std::vector<Vector> clientLocation, Vector apDimension, std::vector<int16_t> areaID, std::vector<uint16_t> numAPs, uint16_t APId, std::vector<std::vector<Box> > obsDimRoom, std::vector<uint16_t> numObstacles_fixed, std::vector<Ptr<Obstacle> > roomScenarios);
  std::vector<bool> LoSAnalysis_MultiAP (Vector apLocation, std::vector<Vector> clientLocation, Vector apDimension);
  std::vector<Vector> checkLoS (std::vector<Vector> apLocation_all, Vector clientLocation, std::vector<Box> obstacleDimension, Vector apDimension);
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
  double GetFadingLoss(uint16_t clientID); // for one client (to one AP)
  Vector GetAPPos(uint16_t clientID); // for one client (to one AP)
  Vector GetClientPos(uint16_t clientID); // for one client (to one AP)

  void CreatChannelInfo();
  double multiPathFadingVarianceGen();


  // Ang 1/2021 reflector
  // void checkIfRefExist (Vector apLocation, std::vector<Vector> clientLocation, int flagRefRange);
  Vector ptReflection(double x, double y, double z, int dir, double pos, double rl, double rw);
  Vector ptReflectionOriginal(double x, double y, double z, int dir, double pos);
  Vector LineFunc3Dcustom(Vector A, Vector B, double pos, bool isX);
  

  // Yuchen 7/2020
  std::vector<double> m_obstaclePenetrationLoss_low;
  std::vector<double> m_obstaclePenetrationLoss_medium;
  std::vector<double> m_obstaclePenetrationLoss_high;

private:
  typedef struct {
    bool losFlag;
    double fadingLoss;
    Vector apPos;
    Vector clientPos;
	std::vector<uint16_t> servedAPId;
  } ChannelInfo;

  std::vector<ChannelInfo> m_channelInfo;
  uint32_t m_obstalceNumber; // objects
  uint32_t m_obstalceNumber_human; // humans
  std::vector<Box> m_obstacleDimension;
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


  // Ang 1/2021 reflector
  std::vector<bool> m_hasRefPath;
  std::vector<double> m_refPathTotalDist;
  std::vector<Vector> m_refCenterLoc;
};


} //namespace ns3

#endif /* OBSTACLE_H */
