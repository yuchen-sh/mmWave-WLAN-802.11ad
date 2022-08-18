/*
 * Copyright (c) 2021 GNAN
 * Author: Yuchen */

#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/wifi-module.h"
#include "ns3/obstacle.h"
#include "common-functions.h"
#include <string>
#include <math.h>
#include <time.h>
#include <fstream>      // std::ofstream / ifstream



#include <gsl/gsl_rng.h>

#define PI 3.14159265
#define LINE_OF_SIGHT 0
#define NON_LINE_OF_SIGHT 1


/**
 * Simulation Objective:
 * This script is used to evaluate the performance of scenario that two nodes in each room (1AP, 1 STA, no obstacles) for each simulation run 
 * To evaluate CSMA/CA channel access scheme: scheme=1, To evaluate SP channel access scheme: scheme=0
 */

NS_LOG_COMPONENT_DEFINE ("CompareAccessSchemes");



using namespace ns3;
using namespace std;

/**  Application Variables **/
uint64_t totalRx = 0;
double throughput = 0;
uint32_t allocationType = 0;               /* The type of channel access scheme during DTI (CBAP is the default) */

int
main(int argc, char *argv[])
{
  LogComponentEnable ("CompareAccessSchemes", LOG_LEVEL_ALL);
  //LogComponentEnable ("MacLow", LOG_LEVEL_ALL);
  //LogComponentEnable ("EdcaTxopN", LOG_LEVEL_ALL);
  //LogComponentEnable ("Obstacle", LOG_LEVEL_ALL);
  // LogComponentEnable ("YansWifiChannel", LOG_LEVEL_ALL);
  //LogComponentEnable ("TruncatedNormalDistribution", LOG_LEVEL_ALL);
  // LogComponentEnable ("DmgApWifiMac", LOG_LEVEL_ALL);

  uint32_t payloadSize = 1472;                  /* Application payload size in bytes. */
  string dataRate = "4500Mbps";  // 4000                 /* Application data rate. */
  uint32_t msduAggregationSize = 7935;          /* The maximum aggregation size for A-MSDU in Bytes. */
  uint32_t mpduAggregationSize = 262143;        /* The maximum aggregation size for A-MSPU in Bytes. */
  uint32_t queueSize = 1000; // 1000                    /* Wifi MAC Queue Size. */
  string phyMode = "DMG_MCS12";                 /* Type of the Physical Layer. */
  bool verbose = false;                         /* Print Logging Information. */
  double simulationTime = 2.0; // 1.5 , 1.125   /* Simulation time in seconds. */
  bool pcapTracing = false;                     /* PCAP Tracing is enabled. */
  double x = 6.0;
  double y = 4.0;
  double z = 3.0; // ceiling-mounted AP
  uint16_t clientRS = 10; // 18
  uint16_t distRS = 1;
  uint16_t ni = 2;
  uint16_t nii = 1;
  // uint16_t shapeCategary = 0;
  uint16_t centerLocation = 1;
  // double platformSize = 30;
  Vector roomSize = Vector (24.0, 8.0, 3.0); // two rooms 12*8*3.0
  // Vector trackSize = Vector(0, 0.065, 0.047); // x dimension is a parameter to be changed
  // double moveStep = 0.1;
  Vector apDimension = Vector (0.23, 0.23, 0.12);
  double depSD = 1;
  uint16_t clientNo = 2; // number of sta
  uint16_t clientNo_1 = clientNo/2; // number of sta in room1
  uint16_t clientNo_2 = clientNo/2; // number of sta in room2
  // bool hermesFlag = 0;  // 0--Multiple static AP, 1--mobile AP
  uint16_t obsNumber = 0; // furniture-type obstacles
  uint16_t obsNumber_1 = 0; // furniture-type obstacles
  uint16_t obsNumber_2 = 0; // furniture-type obstacles
  double human_obs_ratio = 0; // if no human blockage, set as 0
  bool SV_channel = false; // enable SV channel
  bool SV_channel_1 =  false; // enable SV channel
  bool SV_channel_2 = false; // enable SV channel
  int reflectorDenseMode = 2; // 1/2/3 -> lower/medium/higher density of highly-reflective objects in the room
  int reflectorDenseMode_1 = 2; // 1/2/3 -> lower/medium/higher density of highly-reflective objects in the room
  int reflectorDenseMode_2 = 2; // 1/2/3 -> lower/medium/higher density of highly-reflective objects in the room
  uint16_t clientDistType = 0; // 0-possion, 1-trucated-normal, 2-OD-Truncated Normal, 3-OD
  bool mobilityUE = 0; // 0--static, 1--mobile (random walk)
  // bool obsConflictCheck = false; // true--checking obstacle conflicts when allocating obstacles
  double totalSimulationTime = 10.0; // second


  /* Command line argument parser setup. */
  CommandLine cmd;
  cmd.AddValue ("payloadSize", "Application payload size in bytes", payloadSize);
  cmd.AddValue ("dataRate", "Application data rate", dataRate);
  cmd.AddValue ("msduAggregation", "The maximum aggregation size for A-MSDU in Bytes", msduAggregationSize);
  cmd.AddValue ("mpduAggregation", "The maximum aggregation size for A-MPDU in Bytes", mpduAggregationSize);
  cmd.AddValue ("queueSize", "The maximum size of the Wifi MAC Queue", queueSize);
  cmd.AddValue ("scheme", "The access scheme used for channel access (0=SP,1=CBAP)", allocationType);
  cmd.AddValue ("phyMode", "802.11ad PHY Mode", phyMode);
  cmd.AddValue ("verbose", "Turn on all WifiNetDevice log components", verbose);
  cmd.AddValue ("simulationTime", "Simulation time in seconds", simulationTime);
  cmd.AddValue ("pcap", "Enable PCAP Tracing", pcapTracing);
  cmd.AddValue ("x", "ap x", x);
  cmd.AddValue ("y", "ap y", y);
  cmd.AddValue ("ni", "simulation iteration", ni);
  cmd.AddValue ("nii", "simulation iteration ii", nii);
  cmd.AddValue ("z", "ap z", z);
  cmd.AddValue ("centerLocation", "center Location Type", centerLocation);
  // cmd.AddValue ("platformSize", "platform Size", platformSize); 
  cmd.AddValue ("clientRS", "random seed for client", clientRS);
  cmd.AddValue ("clientDistType", "distribution type for client", clientDistType);
  cmd.AddValue ("distRS", "random seed for truncated normal distribution", distRS);
  cmd.AddValue ("depSD", "random seed for dependent distribution", depSD);
  cmd.AddValue ("clientNo", "Number of client", clientNo);
  // cmd.AddValue ("hermesFlag", "0 means static AP scenario, 1 means hermes scenario", hermesFlag);
  // cmd.AddValue ("shapeCategary", "shape Categary", shapeCategary);
  cmd.AddValue ("obsNumber", "obstacle Number", obsNumber);  
  cmd.Parse (argc, argv);


  //----------------------- Scenario Setting -------------------------//

  // two sub-rooms
  /*
  |----------------------------------|
  |               |                  |
  |               |                  |
  |    Room 1   window     Room 2    |
  |               |                  |
  |_________________|___________________| 
  */
  Vector subRoomSize1 = Vector (12.0, 8.0, 3.0);
  Vector subRoomSize2 = Vector (12.0, 8.0, 3.0);

  Box subRoom1 = Box (0, 12.0, 0, 8.0, 0, 3.0);
  Box subRoom2 = Box (12.0, 24.0, 0, 8.0, 0, 3.0);

  // configure the wall and window
  Vector wallSize = Vector (0.2, roomSize.y, roomSize.z);
  Vector wallCenter = Vector (roomSize.x/2.0, roomSize.y/2.0, roomSize.z/2.0);
  Vector windowCenter = Vector (12.0, 4.0, 1.5);
  double windowLength = 1.2;
  double windowWidth = 1.2;
  

  // configure the obstacle layout
  // 1) read the obstacle info from the txt
  string filename;
  double a;
  std::vector<double> obs_temp;
  if (obsNumber > 0)
  	{
  		filename = "/home/guest/YCworkSpace/ns3-802-11ad-ay/multi-room-file/double-room-scenario/fixed_obstacleList.txt";
  		std::ifstream ifs;
  		ifs.open(filename, ios::in);
  		if (!ifs) // no file exists
		{
			NS_LOG_INFO("no file exist!");
			std::cerr << "no file exist! (obs file)" << std::endl;
		}
  		if (ifs.is_open())
		{
			for (; ifs >> a;)
			{
				obs_temp.push_back(a);
			}
			ifs.close();
		}
  		else
		{
			NS_LOG_INFO("File exists but Unable to open");
			std::cerr << "File exists but Unable to open! (obs file)" << std::endl;
		}
  	}
  obsNumber = obs_temp.size()/6;
  // set human obstacle number
  uint16_t obsNumber_human = (uint16_t)(floor(obsNumber*1.0*human_obs_ratio));
  
  Ptr<Obstacle> labScenarios = CreateObject<Obstacle> ();
  labScenarios->SetObstacleNumber(obsNumber);
  labScenarios->SetHumanObstacleNumber(obsNumber_human);

  
  // configure the obstacle material, reflectivity, and scenario channel
  labScenarios->EnableSVChannel(SV_channel); 
  labScenarios->SetReflectedMode(reflectorDenseMode); 
  // set penetration loss mode
  std::vector<double> obstaclePenetrationLoss;
  if (reflectorDenseMode == 1)
  	{
  	  obstaclePenetrationLoss = labScenarios->m_obstaclePenetrationLoss_low;
  	}
  else if (reflectorDenseMode == 2)
  	{
  	  obstaclePenetrationLoss = labScenarios->m_obstaclePenetrationLoss_medium;
  	}
  else
  	{ 
  	  obstaclePenetrationLoss = labScenarios->m_obstaclePenetrationLoss_high;
  	}
  labScenarios->SetPenetrationLossMode (obstaclePenetrationLoss);


  // 2) get the obstacles' dimentions and locations
  std::vector<Box> obsDim;
  if (obsNumber > 0)
  	{
  	  for (uint32_t obsID = 0; obsID < obsNumber; ++obsID)
  	  	{
  	  	  obsDim.push_back(Box (0, 0, 0, 0, 0, 0));
  	  	  obsDim.at(obsID).xMin = obs_temp.at(6*obsID);
		  obsDim.at(obsID).xMax = obs_temp.at(6*obsID + 1);
		  obsDim.at(obsID).yMin = obs_temp.at(6*obsID + 2);
		  obsDim.at(obsID).yMax = obs_temp.at(6*obsID + 3);
		  obsDim.at(obsID).zMin = obs_temp.at(6*obsID + 4);
		  obsDim.at(obsID).zMax = obs_temp.at(6*obsID + 5);
  	  	}
	  // add the fixed obstacles
      labScenarios->AllocateObstacle_KnownBox(obsDim);
  	}
  
  // add wall with a small opening between the room
  labScenarios->AddWallwithWindow(wallSize, wallCenter, windowCenter, windowLength, windowWidth);
       

  // 3) record the obstacle info and setting for two seperated subrooms
  uint16_t obsNumber_human_1 = (uint16_t)(floor(obsNumber_1*1.0*human_obs_ratio));
  uint16_t obsNumber_human_2 = (uint16_t)(floor(obsNumber_2*1.0*human_obs_ratio));
  // room 1
  std::vector<Box> obsDim_room1;
  // if (obsNumber_1 > 0)
  	// {
  		for (uint16_t obsID = 0; obsID < obsNumber_1; ++ obsID)
  		{
  	  		obsDim_room1.push_back(obsDim.at(obsID));
  		}
  		Ptr<Obstacle> room1Scenarios = CreateObject<Obstacle> ();
  		room1Scenarios->SetObstacleNumber(obsNumber_1);
  		room1Scenarios->SetHumanObstacleNumber(obsNumber_human_1);
  		// configure the obstacle material, reflectivity, and scenario channel
  		room1Scenarios->EnableSVChannel(SV_channel_1); 
  		room1Scenarios->SetReflectedMode(reflectorDenseMode_1); 
  		// set penetration loss mode
  		if (reflectorDenseMode_1 == 1)
  		{
  	  		obstaclePenetrationLoss = room1Scenarios->m_obstaclePenetrationLoss_low;
  		}
  		else if (reflectorDenseMode_1 == 2)
  		{
  	  		obstaclePenetrationLoss = room1Scenarios->m_obstaclePenetrationLoss_medium;
  		}
  		else
  		{ 
  	  		obstaclePenetrationLoss = room1Scenarios->m_obstaclePenetrationLoss_high;
  		}
  		room1Scenarios->SetPenetrationLossMode (obstaclePenetrationLoss);
  		// add the fixed obstacles
  		room1Scenarios->AllocateObstacle_KnownBox(obsDim_room1);
  	// }
    
  // room 2
  std::vector<Box> obsDim_room2;
  // if (obsNumber_2 > 0)
  	// {
  		for (uint16_t obsID = obsNumber_1; obsID < obsNumber_1+obsNumber_2; ++ obsID)
  		{
  	  		obsDim_room2.push_back(obsDim.at(obsID));
  		}
  		Ptr<Obstacle> room2Scenarios = CreateObject<Obstacle> ();
  		room2Scenarios->SetObstacleNumber(obsNumber_2);
  		room2Scenarios->SetHumanObstacleNumber(obsNumber_human_2);
  		// configure the obstacle material, reflectivity, and scenario channel
  		room2Scenarios->EnableSVChannel(SV_channel_2); 
  		room2Scenarios->SetReflectedMode(reflectorDenseMode_2); 
  		// set penetration loss mode
  		if (reflectorDenseMode_2 == 1)
  		{
  	  		obstaclePenetrationLoss = room2Scenarios->m_obstaclePenetrationLoss_low;
  		}
  		else if (reflectorDenseMode_2 == 2)
  		{
  	  		obstaclePenetrationLoss = room2Scenarios->m_obstaclePenetrationLoss_medium;
  		}
  		else
  		{ 
  	  		obstaclePenetrationLoss = room2Scenarios->m_obstaclePenetrationLoss_high;
  		}
  		room2Scenarios->SetPenetrationLossMode (obstaclePenetrationLoss);
  		// add the fixed obstacles
  		room2Scenarios->AllocateObstacle_KnownBox(obsDim_room2);
  	// }

  
  // scale parameter due to shell script without float value
  depSD = depSD*0.1;


  /* Global params: no fragmentation, no RTS/CTS, fixed rate for all packets */
  Config::SetDefault ("ns3::WifiRemoteStationManager::FragmentationThreshold", StringValue ("999999"));
  Config::SetDefault ("ns3::WifiRemoteStationManager::RtsCtsThreshold", StringValue ("999999"));


  /**** WifiHelper is a meta-helper: it helps creates helpers ****/
  DmgWifiHelper wifi;
  wifi.SetStandard (WIFI_PHY_STANDARD_80211ad); // follow IEEE 802.11ad standard

  /* Turn on logging */
  if (verbose)
    {
      wifi.EnableLogComponents ();
      LogComponentEnable ("CompareAccessSchemes", LOG_LEVEL_ALL);
    }


  // set up user mobility model
  // 1) configure the specific client locations, read the location info from the txt
  // i) Room 1
  filename = "/home/guest/YCworkSpace/ns3-802-11ad-ay/multi-room-file/double-room-scenario/room1_mobility_locations.txt";
  std::vector<double> Loc_temp1;
  std::ifstream ifs1;
  ifs1.open(filename, ios::in);
  if (!ifs1) // no file exists
	{
		NS_LOG_INFO("no file exist!");
		std::cerr << "no file exist! (mobility file 1)" << std::endl;
	}
  if (ifs1.is_open())
	{
		for (; ifs1 >> a;)
		{
			Loc_temp1.push_back(a);
		}
		ifs1.close();
	}
  else
	{
		NS_LOG_INFO("File exists but Unable to open");
		std::cerr << "File exists but Unable to open! (mobility file 1)" << std::endl;
	}
  int numLocRoom1 = Loc_temp1.size()/3;
  
  // ii) Room 2
  filename = "/home/guest/YCworkSpace/ns3-802-11ad-ay/multi-room-file/double-room-scenario/room2_mobility_locations.txt";
  std::vector<double> Loc_temp2;
  std::ifstream ifs2;
  ifs2.open(filename, ios::in);
  if (!ifs2) // no file exists
	{
		NS_LOG_INFO("no file exist!");
		std::cerr << "no file exist! (mobility file 2)" << std::endl;
	}
  if (ifs2.is_open())
	{
		for (; ifs2 >> a;)
		{
			Loc_temp2.push_back(a);
		}
		ifs2.close();
	}
  else
	{
		NS_LOG_INFO("File exists but Unable to open");
		std::cerr << "File exists but Unable to open! (mobility file 2)" << std::endl;
	}
  int numLocRoom2 = Loc_temp2.size()/3;

  // 2) get the mobile locations
  std::vector<Vector> KnownLoc1;
  if (numLocRoom1 > 0)
  	{
  	  for (int locID = 0; locID < numLocRoom1; ++locID)
  	  	{
  	  	  KnownLoc1.push_back(Vector (0, 0, 0));
  	  	  KnownLoc1.at(locID).x = Loc_temp1.at(3*locID);
		  KnownLoc1.at(locID).y = Loc_temp1.at(3*locID + 1);
		  KnownLoc1.at(locID).z = Loc_temp1.at(3*locID + 2);
  	  	}
  	}
  std::vector<Vector> KnownLoc2;
  if (numLocRoom2 > 0)
  	{
  	  for (int locID = 0; locID < numLocRoom2; ++locID)
  	  	{
  	  	  KnownLoc2.push_back(Vector (0, 0, 0));
  	  	  KnownLoc2.at(locID).x = Loc_temp2.at(3*locID);
		  KnownLoc2.at(locID).y = Loc_temp2.at(3*locID + 1);
		  KnownLoc2.at(locID).z = Loc_temp2.at(3*locID + 2);
  	  	}
  	}

  // 3) get shared locations ID, where the client can access to APs in different rooms
  // i) Room 1
  filename = "/home/guest/YCworkSpace/ns3-802-11ad-ay/multi-room-file/double-room-scenario/room1_shared_locID.txt";
  std::vector<int> share_Loc1;
  std::ifstream ifs3;
  ifs3.open(filename, ios::in);
  if (!ifs3) // no file exists
	{
		NS_LOG_INFO("no file exist!");
		std::cerr << "no file exist! (shared mobility file 1)" << std::endl;
	}
  int b;
  if (ifs3.is_open())
	{
		for (; ifs3 >> b;)
		{
			share_Loc1.push_back(b);
		}
		ifs3.close();
	}
  else
	{
		NS_LOG_INFO("File exists but Unable to open");
		std::cerr << "File exists but Unable to open! (shared mobility file 1)" << std::endl;
	}
  // int numShareLocRoom1 = share_Loc1.size();
  // i) Room 2
  filename = "/home/guest/YCworkSpace/ns3-802-11ad-ay/multi-room-file/double-room-scenario/room2_shared_locID.txt";
  std::vector<int> share_Loc2;
  std::ifstream ifs4;
  ifs4.open(filename, ios::in);
  if (!ifs4) // no file exists
	{
		NS_LOG_INFO("no file exist!");
		std::cerr << "no file exist! (shared mobility file 2)" << std::endl;
	}
  if (ifs4.is_open())
	{
		for (; ifs4 >> b;)
		{
			share_Loc2.push_back(b);
		}
		ifs4.close();
	}
  else
	{
		NS_LOG_INFO("File exists but Unable to open");
		std::cerr << "File exists but Unable to open! (shared mobility file 2)" << std::endl;
	}
  // int numShareLocRoom2 = share_Loc2.size();



  // Configure the AP deployment, locations
  std::vector<Vector> apPosVec; // record APs' positions
    
  // Multiple-AP deployment (support Number of APs = 1 or 2)
  double rl = subRoomSize1.x;
  double rw = subRoomSize1.y;

  // ni is the number of AP, nii is the iith AP
  if (ni==1) // just a single AP deployed in the first (left) sub-room
	{
	  	x = rl/2;
		y = rw/2;
		apPosVec.push_back(Vector (x, y, z));
  	}     
  if (ni==2) // both subRooms have a single AP deployed in the center of the subroom
	{
		apPosVec.push_back(Vector (rl/2, rw/2, z));
		apPosVec.push_back(Vector (roomSize.x - subRoomSize2.x/2.0, subRoomSize2.y/2.0, z));
  	}


  //--------------------- (end) Scenario Setting -------------------------//






  //--------------------- mmWave Network simulation part -------------------------//



  // NS_LOG_INFO("Logging" << verbose);

  // start to record cpu running time
  clock_t startRunTime, endRunTime;
  double totaltime;
  startRunTime=clock();

  uint32_t totalSimInstance = (uint32_t)(totalSimulationTime/simulationTime); // simulation time instances
  
  // for every simulation instance
  for(uint32_t sim = 1; sim <= totalSimInstance; ++sim)
  	{
  	  std::cerr << std:: endl << "Time instant " << sim << std::endl << std:: endl;

	  /* Setting mobility model */
      // 1) clients in room 1, assign the mobility locations (changed every simulation time instance)
      std::vector<Vector> clientPos_1 = labScenarios->IdentifyCLientLocation_KnownLoc(clientNo_1, KnownLoc1);
      // 2) clients in room 2, assign the mobility locations (changed every simulation time instance)
      std::vector<Vector> clientPos_2 = labScenarios->IdentifyCLientLocation_KnownLoc(clientNo_2, KnownLoc2);
      // 3) merge all clients
      std::vector<Vector> clientPos = clientPos_1;
      clientPos.insert(clientPos.end(), clientPos_2.begin(), clientPos_2.end());
      // print the client locations
      for (uint16_t ic = 0; ic < clientPos.size(); ++ic)
  	  {
  	    std::cerr << "clientPos: " << clientPos.at(ic).x << " " << clientPos.at(ic).y << " " << clientPos.at(ic).z << std::endl;
  	  }

	  // for each subroom, every AP
      for (nii = 0; nii < ni; ++nii)
      	{
      	  std::cerr << std:: endl << "Room " << nii + 1 << std::endl;
      	
  
  /**** Set up Channel ****/
  DmgWifiChannelHelper wifiChannelHelper;
  /* Simple propagation delay model */
  wifiChannelHelper.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
  /* Friis model with standard-specific wavelength */
  wifiChannelHelper.AddPropagationLoss ("ns3::FriisPropagationLossModel", "Frequency", DoubleValue (60.48e9));
  Ptr<DmgWifiChannel> wifiChannel = wifiChannelHelper.Create ();

  /**** Setup physical layer ****/
  DmgWifiPhyHelper wifiPhy = DmgWifiPhyHelper::Default ();
  /* Nodes will be added to the channel we set up earlier */
  wifiPhy.SetChannel (wifiChannel);
  /* All nodes transmit at 10 dBm == 10 mW, no adaptation */
  wifiPhy.Set ("TxPowerStart", DoubleValue (10.0));
  wifiPhy.Set ("TxPowerEnd", DoubleValue (10.0));
  wifiPhy.Set ("TxPowerLevels", UintegerValue (1));
  // wifiPhy.Set ("TxGain", DoubleValue (0));
  // wifiPhy.Set ("RxGain", DoubleValue (0));
  /* Set operating channel */
  wifiPhy.Set ("ChannelNumber", UintegerValue (2));
  /* Sensitivity model includes implementation loss and noise figure */
  wifiPhy.Set ("RxNoiseFigure", DoubleValue (10));
  // wifiPhy.Set ("CcaMode1Threshold", DoubleValue (-79));
  // wifiPhy.Set ("EnergyDetectionThreshold", DoubleValue (-79 + 3));
  /* Set the phy layer error model */
  wifiPhy.SetErrorRateModel ("ns3::SensitivityModel60GHz");
  /* Set default algorithm for all nodes to be constant rate */
  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager", "ControlMode", StringValue (phyMode),
                                                                "DataMode", StringValue (phyMode));
  

  /* Make two nodes and set them up with the PHY and the MAC */
  NodeContainer staWifiNode;
  staWifiNode.Create (clientNo);
  NodeContainer apWifiNode;
  apWifiNode.Create (1);

  /* Add a DMG upper mac */
  DmgWifiMacHelper wifiMac = DmgWifiMacHelper::Default ();

  Ssid ssid = Ssid ("Compare");
  wifiMac.SetType ("ns3::DmgApWifiMac",
                   "Ssid", SsidValue(ssid),
                   "BE_MaxAmpduSize", UintegerValue (mpduAggregationSize),
                   "BE_MaxAmsduSize", UintegerValue (msduAggregationSize),
                   "SSSlotsPerABFT", UintegerValue (8), "SSFramesPerSlot", UintegerValue (8),
                   "BeaconInterval", TimeValue (MicroSeconds (102400)),
                   // "BeaconTransmissionInterval", TimeValue (MicroSeconds (600)),
                   // "ATIPresent", BooleanValue (false));
                   "ATIDuration", TimeValue (MicroSeconds (1000)));

  /* Set Analytical Codebook for the DMG Devices */
  wifi.SetCodebook ("ns3::CodebookAnalytical",
                    "CodebookType", EnumValue (SIMPLE_CODEBOOK),
                    "Antennas", UintegerValue (1),
                    "Sectors", UintegerValue (8));

  NetDeviceContainer apDevice;
  apDevice = wifi.Install (wifiPhy, wifiMac, apWifiNode.Get (0));

  wifiMac.SetType ("ns3::DmgStaWifiMac",
                   "Ssid", SsidValue (ssid), "ActiveProbing", BooleanValue (false),
                   "BE_MaxAmpduSize", UintegerValue (mpduAggregationSize),
                   "BE_MaxAmsduSize", UintegerValue (msduAggregationSize));

  NetDeviceContainer staDevice;
  staDevice = wifi.Install (wifiPhy, wifiMac, staWifiNode);

    
  
  MobilityHelper mobility1; // AP's mobility model
  MobilityHelper mobility2; // user's ,obility model
  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
  Vector apPos = apPosVec.at(nii);
  positionAlloc->Add (apPos);	/* PCP/AP */

  if (nii == 0)
  	{
      for (uint16_t clientId = 0; clientId < clientNo_1; clientId++)
        positionAlloc->Add (clientPos_1.at(clientId));  /* DMG STA */
	  // do LoS calculation only for local clients in current AP-deployed room
      room1Scenarios->LoSAnalysis (apPos, clientPos_1, apDimension);
  	}
  else
  	{
  	  for (uint16_t clientId = 0; clientId < clientNo_2; clientId++)
        positionAlloc->Add (clientPos_2.at(clientId));  /* DMG STA */
	  // do LoS calculation only for clients in current AP-deployed room
      room2Scenarios->LoSAnalysis (apPos, clientPos_2, apDimension);
  	}
  
  wifiChannel->SetScenarioModel(labScenarios); // connect scenario to channel (entire scenario level)

  wifiChannel->SetSVChannelEnabler(SV_channel); // (entire scenario level)
  wifiChannel->SetSVChannelReflectedMode(reflectorDenseMode); // (entire scenario level)


  // obstacle density in specific subroom
  double obsDensity = 0;
  if (nii == 0)
  	{
      obsDensity = obsNumber_1*1.0/(subRoomSize1.x*subRoomSize1.y);
  	}
  else
  	{
  	  obsDensity = obsNumber_2*1.0/(subRoomSize2.x*subRoomSize2.y);
  	}
  wifiChannel->SetObsDensity(obsDensity);


  // get the LoS and fadingLoss info from entire scenario level (without unnecessary LoS/fadingLoss calculations)
  if (nii == 0) // the loop of room 1
  	{
  	  // for local clients
      for (uint16_t clientId = 0; clientId < clientNo_1; clientId++)
       {
		  Vector apPos_t = room1Scenarios->GetAPPos(clientId);
		  Vector clientPos_t = room1Scenarios->GetClientPos(clientId);
	      bool losflag_t = room1Scenarios->GetLoSFlag(clientId);
		  double fadingLoss_t = room1Scenarios->GetFadingLoss(clientId);

		  labScenarios->CreatChannelInfo();
		  labScenarios->SetAPPos(apPos_t, clientId);
		  labScenarios->SetClientPos(clientPos_t, clientId);
		  labScenarios->SetLoSFlag(losflag_t, clientId);
		  labScenarios->SetFadingLoss(fadingLoss_t, clientId);		  
       }    
	  // for non-local clients (in room 2)
	  for (uint16_t clientId = 0; clientId < clientNo_2; clientId++)
       {
		  Vector apPos_t = apPos;
		  Vector clientPos_t = clientPos_2.at(clientId);

		  // check if this client in the shared location
		  bool losflag_t = NON_LINE_OF_SIGHT;
		  double fadingLoss_t = (-1.0)*1470*wallSize.x; // 1470 dB/m for brick wall
		  if (share_Loc2.empty() == false)
		  	{
		      for (uint16_t is = 0; is < share_Loc2.size(); ++is)
		  	  {
		  	    if ((clientPos_t.x == KnownLoc2.at(share_Loc2.at(is)).x) && 
			  	    (clientPos_t.y == KnownLoc2.at(share_Loc2.at(is)).y) &&
			  	    (clientPos_t.z == KnownLoc2.at(share_Loc2.at(is)).z))
		  	  	   {
		  	  	     // at shared location of room 2
		  	  	     losflag_t = LINE_OF_SIGHT;
				     fadingLoss_t = (-1.0)*labScenarios->multiPathFadingVarianceGen(); // for consistent with channel setting
				     break;
			  	   }
		  	  }
		  	}

		  labScenarios->CreatChannelInfo();
		  labScenarios->SetAPPos(apPos_t, clientId+clientNo_1);
		  labScenarios->SetClientPos(clientPos_t, clientId+clientNo_1);
		  labScenarios->SetLoSFlag(losflag_t, clientId+clientNo_1);
		  labScenarios->SetFadingLoss(fadingLoss_t, clientId+clientNo_1);		  
       }  
    }
  else // the loop of room 2
  	{
	  // for non-local clients (in room 1)
	  for (uint16_t clientId = 0; clientId < clientNo_1; clientId++)
       {
		  Vector apPos_t = apPos;
		  Vector clientPos_t = clientPos_1.at(clientId);

		  // check if this client in the shared location
		  bool losflag_t = NON_LINE_OF_SIGHT;
		  double fadingLoss_t = (-1.0)*1470*wallSize.x; // 1470 dB/m for brick wall
		  if (share_Loc1.empty() == false)
		  	{
		      for (uint16_t is = 0; is < share_Loc1.size(); ++is)
		  	    {
		  	      if ((clientPos_t.x == KnownLoc1.at(share_Loc1.at(is)).x) && 
			  	       (clientPos_t.y == KnownLoc1.at(share_Loc1.at(is)).y) &&
			  	        (clientPos_t.z == KnownLoc1.at(share_Loc1.at(is)).z))
		  	  	    {
		  	  	      // at shared location of room 2
		  	  	      losflag_t = LINE_OF_SIGHT;
				      fadingLoss_t = (-1.0)*labScenarios->multiPathFadingVarianceGen(); // for consistent with channel setting
				      break;
			  	    }
		  	    }
		  	}

		  labScenarios->CreatChannelInfo();
		  labScenarios->SetAPPos(apPos_t, clientId);
		  labScenarios->SetClientPos(clientPos_t, clientId);
		  labScenarios->SetLoSFlag(losflag_t, clientId);
		  labScenarios->SetFadingLoss(fadingLoss_t, clientId);		  
       }  

	  // for local clients
      for (uint16_t clientId = 0; clientId < clientNo_2; clientId++)
       {
		  Vector apPos_t = room2Scenarios->GetAPPos(clientId);
		  Vector clientPos_t = room2Scenarios->GetClientPos(clientId);
	      bool losflag_t = room2Scenarios->GetLoSFlag(clientId);
		  double fadingLoss_t = room2Scenarios->GetFadingLoss(clientId);

		  labScenarios->CreatChannelInfo();
		  labScenarios->SetAPPos(apPos_t, clientId+clientNo_1);
		  labScenarios->SetClientPos(clientPos_t, clientId+clientNo_1);
		  labScenarios->SetLoSFlag(losflag_t, clientId+clientNo_1);
		  labScenarios->SetFadingLoss(fadingLoss_t, clientId+clientNo_1);		  
       }    
  	}


  // for all clients
   std::vector<bool> losFlag;
  for (uint16_t clientId = 0; clientId < clientNo; clientId++)
    {
      std::pair<double, double> fadingInfo = labScenarios->GetFadingInfo(apPos, clientPos.at(clientId));
      losFlag.push_back(fadingInfo.first);
    }

  
  
  // endRunTime=clock();

  // AP's mobility pattern settings
  mobility1.SetPositionAllocator (positionAlloc);
  mobility1.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility1.Install (apWifiNode);

  // user's mobility pattern settings
  mobility2.SetPositionAllocator (positionAlloc);
  if (mobilityUE == false)
  	{
  	  mobility2.SetMobilityModel ("ns3::ConstantPositionMobilityModel"); // static
  	}
  else
  	{
  	  mobility2.SetMobilityModel ("ns3::RandomWalk2dMobilityModel",
	  	                          "Mode", StringValue("Time"),
	  	                          "Time", StringValue("1s"),
	  	                          "Speed", StringValue("ns3::ConstantRandomVariable[Constant=1]"),
	  	                          "Bounds", RectangleValue(Rectangle(0,roomSize.x,0,roomSize.y))); // random walk
  	}
  
  mobility2.Install (staWifiNode);
  
  

  /* Internet stack*/
  InternetStackHelper stack;
  stack.Install (apWifiNode);
  stack.Install (staWifiNode);

  Ipv4AddressHelper address;
  address.SetBase ("10.0.0.0", "255.255.255.0");
  Ipv4InterfaceContainer apInterface;
  apInterface = address.Assign (apDevice);
  Ipv4InterfaceContainer staInterface;
  staInterface = address.Assign (staDevice);

  /* Populate routing table */
  Ipv4GlobalRoutingHelper::PopulateRoutingTables ();

  /* We do not want any ARP packets */
  PopulateArpCache ();

  ApplicationContainer sourceApplications, sinkApplications;
  uint32_t portNumber = 9;
  for (uint8_t index = 0; index < clientNo; ++index)
    {
      auto ipv4 = staWifiNode.Get (index)->GetObject<Ipv4> ();
      const auto address = ipv4->GetAddress (1, 0).GetLocal ();
      InetSocketAddress sinkSocket (address, portNumber++);
      OnOffHelper src ("ns3::UdpSocketFactory", sinkSocket); 
      src.SetAttribute ("MaxBytes", UintegerValue (0));
      src.SetAttribute ("PacketSize", UintegerValue (payloadSize));
      src.SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=1e6]"));
      src.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0]"));
      src.SetAttribute ("DataRate", DataRateValue (DataRate (dataRate)));
      sourceApplications.Add (src.Install (apWifiNode.Get (0)));
      PacketSinkHelper packetSinkHelper ("ns3::UdpSocketFactory", sinkSocket);
      sinkApplications.Add (packetSinkHelper.Install (staWifiNode.Get (index)));
   }

  // endRunTime=clock();
  
  sinkApplications.Start (Seconds (0.0));
  sinkApplications.Stop (Seconds (simulationTime));
  sourceApplications.Start (Seconds (1.0));
  sourceApplications.Stop (Seconds (simulationTime));

  /* Print Traces */
  if (pcapTracing)
    {
      wifiPhy.SetPcapDataLinkType (DmgWifiPhyHelper::DLT_IEEE802_11_RADIO);
      wifiPhy.EnablePcap ("Traces/AccessPoint", apDevice, false);
      wifiPhy.EnablePcap ("Traces/Station", staDevice, false);
    }

  /*apWifiNetDevice = StaticCast<WifiNetDevice> (apDevice.Get (0));
  staWifiNetDevice = StaticCast<WifiNetDevice> (staDevice.Get (0));
  apWifiMac = StaticCast<DmgApWifiMac> (apWifiNetDevice->GetMac ());
  staWifiMac = StaticCast<DmgStaWifiMac> (staWifiNetDevice->GetMac ());
  staWifiMac->TraceConnectWithoutContext ("Assoc", MakeBoundCallback (&StationAssoicated, staWifiMac));*/

  Simulator::Stop (Seconds (simulationTime + 1));
  Simulator::Run ();
  Simulator::Destroy ();



  /* -------------- Print Results Summary ------------- */
  // std::cerr << i << " " << centerLocation << " "  << clientRS << " "  << apPos << " ";
  // std::copy(clientPos.begin(), clientPos.end(), std::ostream_iterator<Vector>(std::cerr, " "));
  std::copy(losFlag.begin(), losFlag.end(), std::ostream_iterator<bool>(std::cerr, " "));
  std::cerr << std::endl;
 
  	  for (unsigned index = 0; index < sinkApplications.GetN (); ++index)
    	{
    	    // check if connection is across the room, if NLoS, thrp is set as 0 due to the wall blockages
    	    if (((clientPos.at(index).x < subRoom1.xMax)&&(clientPos.at(index).x > subRoom1.xMin)&&(clientPos.at(index).y < subRoom1.yMax)&&(clientPos.at(index).y > subRoom1.yMin)
				&&(apPos.x < subRoom2.xMax)&&(apPos.x > subRoom2.xMin)&&(apPos.y < subRoom2.yMax)&&(apPos.y > subRoom2.yMin))
				 || ((clientPos.at(index).x < subRoom2.xMax)&&(clientPos.at(index).x > subRoom2.xMin)&&(clientPos.at(index).y < subRoom2.yMax)&&(clientPos.at(index).y > subRoom2.yMin)
				&&(apPos.x < subRoom1.xMax)&&(apPos.x > subRoom1.xMin)&&(apPos.y < subRoom1.yMax)&&(apPos.y > subRoom1.yMin)))
    	    	{
    	    	  // std::cerr << "(This is cross-connection case, clientPos: " << clientPos.at(index).x << " " << clientPos.at(index).y << " " << clientPos.at(index).z << " ) ";
    	    	  if (losFlag.at(index) == NON_LINE_OF_SIGHT)
    	    	  	{
    	    	  	   uint64_t totalPacketsThrough = 0; // wall will totally block the signal or association
      				   throughput += ((totalPacketsThrough * 8) / ((simulationTime-1) * 1000000.0)); //Mbit/s
      				   std::cerr << ((totalPacketsThrough * 8) / ((simulationTime-1) * 1000000.0)) << " ";
    	    	  	}
				  else // LoS, cross connection
				  	{
				  	   uint64_t totalPacketsThrough = StaticCast<PacketSink> (sinkApplications.Get (index))->GetTotalRx ();
      				   throughput += ((totalPacketsThrough * 8) / ((simulationTime-1) * 1000000.0)); //Mbit/s
      				   std::cerr << ((totalPacketsThrough * 8) / ((simulationTime-1) * 1000000.0)) << " ";
				  	}
				}
			else
				{
      	    		uint64_t totalPacketsThrough = StaticCast<PacketSink> (sinkApplications.Get (index))->GetTotalRx ();
      				throughput += ((totalPacketsThrough * 8) / ((simulationTime-1) * 1000000.0)); //Mbit/s
      				std::cerr << ((totalPacketsThrough * 8) / ((simulationTime-1) * 1000000.0)) << " ";
				}
    	}
	  	std::cerr << std::endl;

      	}
  	}

  endRunTime=clock();
  totaltime=(double)(endRunTime-startRunTime)/CLOCKS_PER_SEC;
  std::cerr << std::endl << "Running time: " << totaltime << " " << std::endl;
  return 0;
}

