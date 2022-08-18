/*
 * Copyright (c) 2021
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
#include <algorithm>


#include <gsl/gsl_rng.h>

#define PI 3.14159265

#define min(a,b) ((a) < (b) ? (a) : (b))
#define max(a,b) ((a) > (b) ? (a) : (b))


/**
 * Simulation Objective:
 * This script is used to evaluate the performance of office conference room scenario defined in TGad Evaluation document 

 * Network Topology:
 * The scenario consists of 1 AP (STA 0) and 8 STAs, and traffic model includes lightly compressed video, local file transfer, and web browsing.
 * STA2 -> STA1 (lightly compressed video)
 * STA0 -> STA2 (local file transfer)
 * STA3 -> STA5 (local file transfer)
 * STA0 -> STA3 (web browsing)
 * STA4 -> STA0 (local file transfer)
 * STA0 -> STA4 (web browsing)
 * STA0 -> STA5 (web browsing)
 * STA0 -> STA6 (web browsing)
 * STA7 -> STA8 (local file transfer)
 * STA0 -> STA7 (local file transfer)
 *
 * Running Simulation:
 * ./waf --run "TGad_performance_evaluation_office_room"  (disable human blockages)
 * ./waf --run "TGad_performance_evaluation_office_room --humanBlock=true"  (enable human blockages)
 *
 * Simulation Output:
 * 1. LoS/NLoS status (LoS -- 0; NLoS -- 1)
 * 2. Throughput (Mbps)
 */

NS_LOG_COMPONENT_DEFINE ("TGadEvalOfficeConfRoom");



using namespace ns3;
using namespace std;

/**  Application Variables **/
uint64_t totalRx = 0;
double throughput = 0;
uint32_t allocationType = 0;               /* The type of channel access scheme during DTI (CBAP is the default) */

#define UNCOMP_VIDEO 0
#define COMP_VIDEO 1
#define LOCAL_FILE 2
#define WEB_BROWSING 3


int
main(int argc, char *argv[])
{
  LogComponentEnable ("TGadEvalOfficeConfRoom", LOG_LEVEL_ALL);
  //LogComponentEnable ("MacLow", LOG_LEVEL_ALL);
  //LogComponentEnable ("EdcaTxopN", LOG_LEVEL_ALL);
  //LogComponentEnable ("Obstacle", LOG_LEVEL_ALL);
  // LogComponentEnable ("YansWifiChannel", LOG_LEVEL_ALL);
  //LogComponentEnable ("TruncatedNormalDistribution", LOG_LEVEL_ALL);
  // LogComponentEnable ("DmgApWifiMac", LOG_LEVEL_ALL);

  uint32_t payloadSize = 1472;                  /* Application payload size in bytes. */
  string dataRate = "4500Mbps";                 /* Application data rate. */
  double target_rate = 4500;                    /* Mbps. */
  string uncompVideoRate = "3000Mbps";          /* 1080p, 1920*1080 */
  uint32_t msduAggregationSize = 8000; // 7935; /* The maximum aggregation size for A-MSDU in Bytes. */
  uint32_t mpduAggregationSize = 262143;        /* The maximum aggregation size for A-MSPU in Bytes. */
  // uint32_t queueSize = 10000;                    /* Wifi MAC Queue Size. */
  string queueSize = "4000p";                   /* Wifi MAC Queue Size. */
  string phyMode = "DMG_MCS12";                 /* Type of the Physical Layer. */
  bool verbose = false;                         /* Print Logging Information. */
  double simulationTime = 1.1;                  /* Simulation time in seconds. */
  bool pcapTracing = false;                     /* PCAP Tracing is enabled. */
  double x = 4.5;
  double y = 3;
  double z = 3;
  uint16_t clientRS = 1; // 18
  uint16_t distRS = 1;
  uint16_t i = 1; // number of AP
  uint16_t ii = 1; // iith AP
  Vector roomSize = Vector (3.0, 4.5, 3.0);
  Vector apDimension = Vector (0.23, 0.23, 0.12);
  Vector apPos_FOFC = Vector (1.5, 0.5, 2.9 + apDimension.z*0.5); // Vector (3.5, 6.5, 1.5 + apDimension.z*0.5); // Vector (0.1, 6.9, 2.7 + apDimension.z*0.5);
  Vector LaptopDimension = Vector (0.3, 0.21, 0.05);
  Vector LaptopDimension_sc = Vector (0.3, 0.21, 0.22); // with screen open
  Vector ProjectorDimension = Vector (0.5, 0.3, 0.12);
  Vector MobileDeviceDimension = Vector (0.08, 0.05, 0.15);
  uint16_t numSTAs = 8;
  uint16_t numCommPair = 10;
  double depSD = 1;
  uint32_t clientNo = 1; // number of sta, must fixed at 1 for this script
  uint16_t obsNumber = 20; // 22, 43, obstacles
  uint16_t human_obs = 2; // if no human blockage, set as 0.
  bool TGad_channel = true; // enable TGad_channel
  int reflectorDenseMode = 2; // 1/2/3 -> lower/medium/higher density of highly-reflective objects in the room
  uint16_t clientDistType = 0; // 0-possion, 1-trucated-normal, 2-OD-Truncated Normal, 3-OD
  bool mobilityUE = 0; // 0--static, 1--mobile (random walk)
  bool obsConflictCheck = true; // true--checking obstacle conflicts when allocating obstacles
  bool FOFC = true; // true -- allocate fixed obstacles and clients in the scenario; false: randomly generate obstacles and clients
  bool humanBlock = false; // true -- enable a human obstacle that blocks the link btw STB and TV
  uint16_t commID = 1; // range: 1 -- 10
  uint16_t trafficMode = UNCOMP_VIDEO; // by default


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
  cmd.AddValue ("humanBlock", "Enable human obstacle", humanBlock);
  // cmd.AddValue ("commID", "communication pair ID", commID);
  cmd.AddValue ("x", "ap x", x);
  cmd.AddValue ("y", "ap y", y);
  cmd.AddValue ("i", "simulation iteration", i);
  cmd.AddValue ("ii", "simulation iteration ii", ii);
  cmd.AddValue ("z", "ap z", z);
  cmd.AddValue ("clientRS", "random seed for client", clientRS);
  cmd.AddValue ("clientDistType", "distribution type for client", clientDistType);
  cmd.AddValue ("distRS", "random seed for truncated normal distribution", distRS);
  cmd.AddValue ("depSD", "random seed for dependent distribution", depSD);
  cmd.AddValue ("clientNo", "Number of client", clientNo);
  cmd.AddValue ("obsNumber", "obstacle Number", obsNumber);  
  cmd.Parse (argc, argv);


  // all STAs' positions (including AP: ID is 0)
  std::vector<Vector> STApos;
  STApos.push_back(apPos_FOFC); // AP
  STApos.push_back(Vector (1.75, 2.3, 1.0 + ProjectorDimension.z*0.5)); // STA1
  STApos.push_back(Vector (1.9, 1.5, 1.0 + MobileDeviceDimension.z*0.5)); // STA2
  STApos.push_back(Vector (1.35, 3.0, 1.0 + LaptopDimension.z*0.5)); // STA3
  STApos.push_back(Vector (1.3, 2.4, 1.0 + LaptopDimension.z*0.5)); // STA4
  STApos.push_back(Vector (1.25, 1.4, 1.0 + LaptopDimension.z*0.5)); // STA5
  STApos.push_back(Vector (1.55, 1.2, 1.0 + LaptopDimension.z*0.5)); // STA6
  STApos.push_back(Vector (1.85, 3.1, 1.0 + LaptopDimension.z*0.5)); // STA7
  STApos.push_back(Vector (1.6, 3.25, 1.0 + MobileDeviceDimension.z*0.5)); // STA8
  

  for (commID = 1; commID <= numCommPair; ++commID)
  {  
  // furniture-type obstacles
  std::vector<double> xPos, yPos, wObs, lObs, hObs, dirObs, hObs_min;
  std::vector<double> obs_temp;
  if (FOFC == true)
  	{
  // Fixed obstacle setting
  obs_temp = {3.0000,0.1000,0.1000,0.1000,0.1000,0.3000,0.3500,0.3500,0.4200,0.3000,0.3500,0.3500,0.4200,0.3000,0.3500,0.3500,0.4200,0.3000,0.3500,0.3500,0.4200,0.3000,0.3500,0.3500,0.4200,0.3000,0.3500,0.3500,0.4200,0.3000,0.3500,0.3500,0.4200,0.3000,0.3500,0.3500,0.4200,1.4000,0.1000,0.1000,0.1000,0.1000,0.3000,0.0500,0.0500,0.0600,0.3000,0.0500,0.0500,0.0600,0.3000,0.0500,0.0500,0.0600,0.3000,0.0500,0.0500,0.0600,0.3000,0.0500,0.0500,0.0600,0.3000,
              0.0500,0.0500,0.0600,0.3000,0.0500,0.0500,0.0600,0.3000,0.0500,0.0500,0.0600,1.0000,0.9300,0.9300,0.9300,0.9300,0.6000,0.8000,0.8000,0.8600,0.6000,0.8000,0.8000,0.8600,0.6000,0.8000,0.8000,0.8600,0.6000,0.8000,0.8000,0.8600,0.6000,0.8000,0.8000,0.8600,0.6000,0.8000,0.8000,0.8600,0.6000,0.8000,0.8000,0.8600,0.6000,0.8000,0.8000,0.8600,90.0000,0.0000,0.0000,0.0000,0.0000,0.0000,90.0000,90.0000,0.0000,0.0000,90.0000,90.0000,0.0000,0.0000,0.0000,
              0.0000,90.0000,0.0000,0.0000,0.0000,90.0000,0.0000,0.0000,0.0000,90.0000,0.0000,0.0000,0.0000,90.0000,0.0000,0.0000,0.0000,90.0000,0.0000,0.0000,0.0000,90.0000,1.5000,0.9500,2.0500,0.9500,2.0500,1.5000,1.3250,1.6750,1.5000,1.5000,1.3250,1.6750,1.5000,2.7200,2.7200,2.7200,2.9250,2.3500,2.3500,2.3500,2.5550,2.3500,2.3500,2.3500,2.5550,0.6500,0.6500,
              0.6500,0.4450,0.7000,0.7000,0.7000,0.4950,0.5500,0.5500,0.5500,0.3450,2.2500,3.6000,3.6000,0.9000,0.9000,4.0000,4.0000,4.0000,4.2100,0.5000,0.5000,0.5000,0.2900,4.2250,4.3750,4.0500,4.2250,3.2000,3.3500,3.0250,3.2000,1.2000,1.3500,1.0250,1.2000,3.2000,3.3500,3.0250,3.2000,2.6000,2.7500,2.4250,2.6000,1.2000,1.3500,1.0250,1.2000,0.9300,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,
              0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000};

  
  obsNumber = obs_temp.size()/7;
  for (uint16_t io = 0; io < obs_temp.size(); ++io)
  	{
  	  if (io < obsNumber)
  	  	{
          lObs.push_back(obs_temp.at(io));
  	  	}
	  if (io < 2*obsNumber && io >= obsNumber)
  	  	{
          wObs.push_back(obs_temp.at(io));
  	  	}
	  if (io < 3*obsNumber && io >= 2*obsNumber)
  	  	{
          hObs.push_back(obs_temp.at(io));
  	  	}
	  if (io < 4*obsNumber && io >= 3*obsNumber)
  	  	{
          dirObs.push_back(obs_temp.at(io)*PI/180.0); // rad
  	  	}
	  if (io < 5*obsNumber && io >= 4*obsNumber)
  	  	{
          xPos.push_back(obs_temp.at(io));
  	  	}
	  if (io < 6*obsNumber && io >= 5*obsNumber)
  	  	{
          yPos.push_back(obs_temp.at(io));
  	  	}
	  if (io < 7*obsNumber && io >= 6*obsNumber)
  	  	{
          hObs_min.push_back(obs_temp.at(io));
  	  	}
  	}

  	}


  // add self blockages (STA devices)
  std::vector<double> deviceDir = {0, 65.0, 315.0, 315.0, 315.0, 315.0, 45.0, 45.0, 315.0};
  std::vector<double> deviceLength = {apDimension.x, ProjectorDimension.x, MobileDeviceDimension.x, LaptopDimension.x, LaptopDimension.x, LaptopDimension.x, LaptopDimension.x, LaptopDimension.x, MobileDeviceDimension.x};
  std::vector<double> deviceWidth = {apDimension.y, ProjectorDimension.y, MobileDeviceDimension.y, LaptopDimension.y, LaptopDimension.y, LaptopDimension.y, LaptopDimension.y, LaptopDimension.y, MobileDeviceDimension.y};
  std::vector<double> deviceHeight = {2.9+apDimension.z, 1.0+ProjectorDimension.z, 1.0+MobileDeviceDimension.z, 1.0+LaptopDimension_sc.z, 1.0+LaptopDimension_sc.z, 1.0+LaptopDimension_sc.z, 1.0+LaptopDimension_sc.z, 1.0+LaptopDimension_sc.z, 1.0+MobileDeviceDimension.z};
  std::vector<double> deviceHeight_base = {2.9, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
  
  // set human obstacle number
  uint16_t obsNumber_human = 0;
  if (humanBlock == true)
  	{
      obsNumber_human = human_obs;
  	}

  depSD = depSD*0.1;

  /* Global params: no fragmentation, no RTS/CTS, fixed rate for all packets */
  Config::SetDefault ("ns3::WifiRemoteStationManager::FragmentationThreshold", StringValue ("999999"));
  Config::SetDefault ("ns3::WifiRemoteStationManager::RtsCtsThreshold", StringValue ("999999"));

  /**** WifiHelper is a meta-helper: it helps creates helpers ****/
  DmgWifiHelper wifi;
  wifi.SetStandard (WIFI_PHY_STANDARD_80211ad); // set up the standard

  /* Turn on logging */
  if (verbose)
    {
      wifi.EnableLogComponents ();
      LogComponentEnable ("CompareAccessSchemes", LOG_LEVEL_ALL);
    }

  // NS_LOG_INFO("Logging" << verbose);  

  // Set scenario object
  Ptr<Obstacle> labScenarios = CreateObject<Obstacle> ();

  // labScenarios->EnableSVChannel(SV_channel);
  labScenarios->EnableTGadChannel(TGad_channel);
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

  std::vector<Vector> apPosVec; // record APs' positions
  
  
  // ---- AP deployment ---- 
  if (FOFC == true) // with a given STB deployment in the living room
  	{
  	  apPosVec.push_back(apPos_FOFC);
  	}
  else // LoS-optimal deployment
  	{
      apPosVec = labScenarios->AllocateOptAP(roomSize, i);
  	}

  x = apPosVec.at(ii - 1).x;
  y = apPosVec.at(ii - 1).y;
  z = apPosVec.at(ii - 1).z;


   /**** Set up Channel ****/
  DmgWifiChannelHelper wifiChannelHelper;
  /* Simple propagation delay model */
  wifiChannelHelper.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
  wifiChannelHelper.AddPropagationLoss ("ns3::FriisPropagationLossModel", "Frequency", DoubleValue (60.48e9));

  Ptr<DmgWifiChannel> wifiChannel = wifiChannelHelper.Create ();

  /**** Setup physical layer ****/
  DmgWifiPhyHelper wifiPhy = DmgWifiPhyHelper::Default ();
  /* Nodes will be added to the channel we set up earlier */
  wifiPhy.SetChannel (wifiChannel);
  /* Nodes will be added to the channel we set up earlier */
  // wifiPhy.SetChannel (wifiChannel.Create ());
  /* All nodes transmit at 10 dBm == 10 mW, no adaptation */
  wifiPhy.Set ("TxPowerStart", DoubleValue (10));
  wifiPhy.Set ("TxPowerEnd", DoubleValue (10));
  wifiPhy.Set ("TxPowerLevels", UintegerValue (1));
  /* Set operating channel */
  wifiPhy.Set ("ChannelNumber", UintegerValue (2));
  /* Sensitivity model includes implementation loss and noise figure */
  wifiPhy.Set ("RxNoiseFigure", DoubleValue (10));
  /* Set the phy layer error model */
  wifiPhy.SetErrorRateModel ("ns3::SensitivityModel60GHz");
  // wifiPhy.SetErrorRateModel ("ns3::DmgErrorModel",
                            //  "FileName", StringValue ("DmgFiles/ErrorModel/LookupTable_1458.txt"));

  /* Set default algorithm for all nodes to be constant rate */
  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager", "ControlMode", StringValue (phyMode),
                                                                "DataMode", StringValue (phyMode));
  
  /* Make two nodes and set them up with the PHY and the MAC */
	NodeContainer rxWifiNode;
	rxWifiNode.Create (clientNo);
	NodeContainer txWifiNode;
	txWifiNode.Create (1);
  
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
					 // "ATIPresent", BooleanValue (false)
					 "ATIDuration", TimeValue (MicroSeconds (1000)));
  
	/* Set Analytical Codebook for the DMG Devices */
	wifi.SetCodebook ("ns3::CodebookAnalytical",
					  "CodebookType", EnumValue (SIMPLE_CODEBOOK),
					  "Antennas", UintegerValue (1),
					  "Sectors", UintegerValue (8));
  
	NetDeviceContainer txDevice;
	txDevice = wifi.Install (wifiPhy, wifiMac, txWifiNode.Get (0));
  
	wifiMac.SetType ("ns3::DmgStaWifiMac",
					 "Ssid", SsidValue (ssid), "ActiveProbing", BooleanValue (false),
					 "BE_MaxAmpduSize", UintegerValue (mpduAggregationSize),
					 "BE_MaxAmsduSize", UintegerValue (msduAggregationSize));
  
	NetDeviceContainer rxDevice;
	rxDevice = wifi.Install (wifiPhy, wifiMac, rxWifiNode);
  
  
  // define the communication entity ID
  std::vector<uint16_t> commEntity;
  Vector txdeviceDimension;
  if (commID == 1)
  	{
  	  commEntity = {2, 1}; // STA2 -> STA1
      txdeviceDimension = MobileDeviceDimension;
	  trafficMode = COMP_VIDEO;
  	}
  else if (commID == 2)
  	{
  	  commEntity = {0, 2}; // AP -> STA2
  	  txdeviceDimension = apDimension;
	  trafficMode = LOCAL_FILE;
  	}
  else if (commID == 3)
  	{
  	  commEntity = {3, 5}; // STA3 -> STA5
	  txdeviceDimension = LaptopDimension;
	  trafficMode = LOCAL_FILE;
  	}
  else if (commID == 4)
  	{
  	  commEntity = {0, 3}; // AP -> STA3
	  txdeviceDimension = apDimension;
	  trafficMode = WEB_BROWSING;
  	}
  else if (commID == 5)
  	{
  	  commEntity = {4, 0}; // STA4 -> AP
	  txdeviceDimension = LaptopDimension;
	  trafficMode = LOCAL_FILE;
  	}
  else if (commID == 6)
  	{
  	  commEntity = {0, 4}; // AP -> STA4
	  txdeviceDimension = apDimension;
	  trafficMode = WEB_BROWSING;
  	}
  else if (commID == 7)
  	{
  	  commEntity = {0, 5}; // AP -> STA5
	  txdeviceDimension = apDimension;
	  trafficMode = WEB_BROWSING;
  	}
  else if (commID == 8)
  	{
  	  commEntity = {0, 6}; // AP -> STA6
	  txdeviceDimension = apDimension;
	  trafficMode = WEB_BROWSING;
  	}
  else if (commID == 9)
  	{
  	  commEntity = {7, 8}; // STA7 -> STA8
	  txdeviceDimension = LaptopDimension;
	  trafficMode = LOCAL_FILE;
  	}
  else if (commID == 10)
  	{
  	  commEntity = {0, 7}; // AP -> STA7
	  txdeviceDimension = apDimension;
	  trafficMode = LOCAL_FILE;
  	}
  else // default is commID = 1
  	{
  	  commEntity = {2, 1}; // STA2 -> STA1
  	  txdeviceDimension = MobileDeviceDimension;
	  trafficMode = COMP_VIDEO;
  	}
  // add devices (potential blockages)
  for (uint16_t k = 1; k <= numSTAs; ++k)
  	{
  	  std::vector<uint16_t>::iterator itr = std::find(commEntity.begin(), commEntity.end(), k);
	  if (itr != commEntity.end())
	  	{
	  	  continue;
	  	}
	  lObs.push_back(deviceLength.at(k));
	  wObs.push_back(deviceWidth.at(k));
	  hObs.push_back(deviceHeight.at(k));
	  dirObs.push_back(deviceDir.at(k)*PI/180.0); // rad
	  xPos.push_back(STApos.at(k).x);
	  yPos.push_back(STApos.at(k).y);
	  hObs_min.push_back(deviceHeight_base.at(k));

	  obsNumber = obsNumber + 1;
  	}

  // allocate furniture-type and self-device obstacles
  labScenarios->SetObstacleNumber(obsNumber);
  if (FOFC == true)
  	{
  	  labScenarios->AllocateObstacle_Fixed_withHB(Box (x-apDimension.x/2, x+apDimension.x/2, y-apDimension.y/2, y+apDimension.y/2, z-apDimension.z, z+apDimension.z), roomSize, clientRS, xPos, yPos, wObs, lObs, hObs, hObs_min, dirObs);
  	}
  else // RORC
  	{
  	  labScenarios->AllocateObstacle(Box (x-apDimension.x/2, x+apDimension.x/2, y-apDimension.y/2, y+apDimension.y/2, z-apDimension.z, z+apDimension.z), roomSize, clientRS);
  	}    

  // allocate human obstacles
  labScenarios->SetObsConflictCheck(obsConflictCheck);
  labScenarios->SetHumanObstacleNumber(obsNumber_human);
  if (humanBlock == true) // add human blockages
  	{
  	   labScenarios->AllocateObstacle_human(Box (x-apDimension.x/2, x+apDimension.x/2, y-apDimension.y/2, y+apDimension.y/2, z-apDimension.z, z+apDimension.z), roomSize, clientRS);
  	}

  /* Setting mobility model */
  MobilityHelper mobility1; // tx, AP's mobility model
  MobilityHelper mobility2; // rx, user's ,obility model
  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
  Vector txPos = STApos.at(commEntity.at(0)); // Vector (x, y, z);

  // allocate fixed rx client locations
  std::vector<Vector> rxPos;
  if (FOFC == true)
  	{
  	  rxPos.push_back(STApos.at(commEntity.at(1)));
  	}
  else // RORC
  	{
  	  rxPos = labScenarios->IdentifyCLientLocation (clientRS, distRS, depSD, clientNo, clientDistType);
  	}
  

  // LoS analysis and channel setting
  positionAlloc->Add (txPos);	/* PCP/AP */
  std::vector<bool> losFlag;
  std::vector<std::vector<bool> > losFlag_mul(i, std::vector<bool>(clientNo)); // only for multi-user case
  for (uint16_t clientId = 0; clientId < clientNo; clientId++)
    positionAlloc->Add (rxPos.at(clientId));  /* DMG STA */
  labScenarios->LoSAnalysis (txPos, rxPos, txdeviceDimension);
  wifiChannel->SetScenarioModel(labScenarios);
  wifiChannel->SetTGadChannelEnabler(TGad_channel);
  wifiChannel->SetSVChannelReflectedMode(reflectorDenseMode);

  // obstacle density
  double obsDensity = obsNumber*1.0/(roomSize.x*roomSize.y);
  wifiChannel->SetObsDensity(obsDensity);

  for (uint16_t clientId = 0; clientId < clientNo; clientId++)
    {
      std::pair<double, double> fadingInfo = labScenarios->GetFadingInfo(txPos, rxPos.at(clientId));
      losFlag.push_back(fadingInfo.first);
    }

  // mobility pattern settings
  mobility1.SetPositionAllocator (positionAlloc);
  mobility1.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility1.Install (txWifiNode);

  // mobility pattern settings
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
  
  mobility2.Install (rxWifiNode);

  /* Internet stack*/
  InternetStackHelper stack;
  stack.Install (txWifiNode);
  stack.Install (rxWifiNode);

  Ipv4AddressHelper address;
  address.SetBase ("10.0.0.0", "255.255.255.0");
  Ipv4InterfaceContainer txInterface;
  txInterface = address.Assign (txDevice);
  Ipv4InterfaceContainer rxInterface;
  rxInterface = address.Assign (rxDevice);

  /* Populate routing table */
  Ipv4GlobalRoutingHelper::PopulateRoutingTables ();

  /* We do not want any ARP packets */
  PopulateArpCache ();

  ApplicationContainer sourceApplications, sinkApplications;
  uint32_t portNumber = 9;
  for (uint8_t index = 0; index < clientNo; ++index)
    {
	  if (trafficMode == COMP_VIDEO)
	  	{
	  	  double IAT = 1.0/4080; // inter-arrival time (s)
          double mu_sl = 15.798; // KBytes
          double sigma_sl = 1.35; // KBytes
          double b_sl = 515; // Mbps
          double p_sl = target_rate;
		  double slice_mu = mu_sl*(p_sl/b_sl);
		  double t_mu = slice_mu*8.0/(p_sl*1000);
		  double slice_sigma = sigma_sl*(p_sl/b_sl);
		  double t_sigma = slice_sigma*8.0/(p_sl*1000);
		  double tx_on_max = 92.16*8.0/(p_sl*1000);
		  /*
	  	  UdpEchoServerHelper echoServer (portNumber);
		  sourceApplications = echoServer.Install (txWifiNode.Get (0));

          UdpEchoClientHelper echoClient (rxInterface.GetAddress (0), portNumber);
          echoClient.SetAttribute ("MaxPackets", UintegerValue (3100));
          echoClient.SetAttribute ("Interval", TimeValue (Seconds (IAT)));
          echoClient.SetAttribute ("PacketSize", UintegerValue (92));
          // echoClient.SetAttribute ("PacketSize", StringValue("ns3::NormalRandomVariable[Mean=138.04|Variance=11.8|Bound=92.16]"));
          sinkApplications = echoClient.Install (rxWifiNode.Get (index));
          */
          std::string IAT_off = std::string("ns3::ConstantRandomVariable[Constant=") + std::to_string(IAT) + std::string("]");
          std::string tx_on = std::string("ns3::NormalRandomVariable[Mean=") + std::to_string(t_mu) + std::string("|Variance=") + std::to_string(t_sigma) + std::string("|Bound=") + std::to_string(tx_on_max) + std::string("]");
		  
          auto ipv4 = rxWifiNode.Get (index)->GetObject<Ipv4> ();
          const auto address = ipv4->GetAddress (1, 0).GetLocal ();
          InetSocketAddress sinkSocket (address, portNumber++);
          OnOffHelper src ("ns3::UdpSocketFactory", sinkSocket); 
          src.SetAttribute ("MaxBytes", UintegerValue (0));
          src.SetAttribute ("PacketSize", UintegerValue (payloadSize));
          // src.SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=0.00016384]"));
          src.SetAttribute ("OnTime", StringValue(tx_on));
		  src.SetAttribute ("OffTime", StringValue (IAT_off));
          src.SetAttribute ("DataRate", DataRateValue (DataRate (dataRate)));
          sourceApplications.Add (src.Install (txWifiNode.Get (0)));
          PacketSinkHelper packetSinkHelper ("ns3::UdpSocketFactory", sinkSocket);
          sinkApplications.Add (packetSinkHelper.Install (rxWifiNode.Get (index)));
		  
	  	}
	  else if (trafficMode == LOCAL_FILE)
	  	{		  
          std::string socketType = "ns3::TcpSocketFactory";  /* Socket Type (TCP/UDP) */
          uint32_t packetSize = 1500;                        /* MSDU payload size in bytes. */
          std::string tcpVariant = "NewReno";                /* TCP Variant Type. */
          uint32_t bufferSize = 65535;                       /* TCP Receive Buffer Size. */

		  /* Wifi MAC Queue Parameters */
          ChangeQueueSize (queueSize);
		  
          /*** Configure TCP Options ***/
          ConfigureTcpOptions (tcpVariant, packetSize, bufferSize);
		  Config::SetDefault ("ns3::TcpSocket::SegmentSize", UintegerValue (1564));
		  Config::SetDefault ("ns3::TcpSocket::RcvBufSize", UintegerValue (bufferSize));
		  Config::SetDefault ("ns3::TcpSocket::DelAckTimeout", TimeValue (Seconds (0.05)));
		  Config::SetDefault ("ns3::TcpSocket::InitialCwnd", UintegerValue (1));
		  Config::SetDefault ("ns3::TcpSocket::DelAckCount", UintegerValue (3));
		  Config::SetDefault ("ns3::TcpSocketBase::WindowScaling", BooleanValue (true));
		  Config::SetDefault ("ns3::TcpSocketBase::Sack", BooleanValue (false));
		  Config::SetDefault ("ns3::TcpSocket::TcpNoDelay", BooleanValue (true)); // Set to true to disable Nagle's algorithm
		  Config::SetDefault ("ns3::TcpSocketBase::MinRto", TimeValue (Seconds (1.0)));
		  Config::SetDefault ("ns3::TcpSocket::ConnTimeout", TimeValue (Seconds (3.0)));
		  Config::SetDefault ("ns3::TcpSocket::PersistTimeout", TimeValue (Seconds (64.0)));
		  Config::SetDefault ("ns3::TcpSocketBase::UseEcn", StringValue ("Off"));

		  
		  // Install applications: BulkAPP (use FTP-like application)
          PacketSinkHelper sinkHelper (socketType, InetSocketAddress (Ipv4Address::GetAny (), 9999));
          sinkApplications.Add (sinkHelper.Install (rxWifiNode.Get (index)));

		  Address dest (InetSocketAddress (rxInterface.GetAddress (index), 9999));
		  BulkSendHelper src (socketType, dest);
          sourceApplications= src.Install (txWifiNode.Get (0));

		  /*
          std::string socketType = "ns3::TcpSocketFactory";
		  Address dest (InetSocketAddress (rxInterface.GetAddress (index), 9999));
          OnOffHelper src (socketType, dest);
          // src.SetAttribute ("MaxPackets", UintegerValue (0));
		  src.SetAttribute ("MaxBytes", UintegerValue (0));
          src.SetAttribute ("PacketSize", UintegerValue (payloadSize));
          src.SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=1e6]"));
          src.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0]"));
          src.SetAttribute ("DataRate", DataRateValue (DataRate (dataRate)));
          sourceApplications.Add (src.Install (txWifiNode.Get (0)));
          // onoff = StaticCast<OnOffApplication> (srcApp.Get (0));
          PacketSinkHelper packetSinkHelper (socketType, InetSocketAddress (Ipv4Address::GetAny (), 9999));
          // ApplicationContainer sinkApp = sinkHelper.Install (apWifiNode);
		  sinkApplications.Add (packetSinkHelper.Install (rxWifiNode.Get (index)));
		  */
		  
	  	}
	  else if (trafficMode == WEB_BROWSING)
	  	{
	  	  Ipv4Address serverAddress = txInterface.GetAddress (0);
	  	  // Create HTTP server helper
          ThreeGppHttpServerHelper serverHelper (serverAddress);

          // Install HTTP server
          sourceApplications = serverHelper.Install (txWifiNode.Get (0));
          Ptr<ThreeGppHttpServer> httpServer = sourceApplications.Get (0)->GetObject<ThreeGppHttpServer> ();

		  // Setup HTTP variables for the server
          PointerValue varPtr;
          httpServer->GetAttribute ("Variables", varPtr);
          Ptr<ThreeGppHttpVariables> httpVariables = varPtr.Get<ThreeGppHttpVariables> ();
		  // main object size, max and min are 2MB and 100B by default
          httpVariables->SetMainObjectSizeMean (10710); // bytes
          httpVariables->SetMainObjectSizeStdDev (25032); // bytes

		  // embedded object size, max and min are 2MB and 50B by default
          httpVariables->SetEmbeddedObjectSizeMean (7758); // bytes
          httpVariables->SetEmbeddedObjectSizeStdDev (126168); // bytes

		  // number of embedded objects per page
		  httpVariables->SetNumOfEmbeddedObjectsMax (53);
		  httpVariables->SetNumOfEmbeddedObjectsShape (5.64); // Pareto distribution

		  // reading time
		  httpVariables->SetReadingTimeMean (Seconds (30.0)); // mean (second)

		  // parsing time
		  httpVariables->SetParsingTimeMean (Seconds (0.13)); // mean (second)

		  // HTTP request size
		  httpVariables->SetRequestSize (350); // bytes

		  // HTTP high MTU prob
		  // httpVariables->SetHighMtuProbability (1.0); // prob for higher MTU
		  Config::SetDefault ("ns3::ThreeGppHttpVariables::HighMtuProbability", DoubleValue (1.0));

		  // HTTP high MTU size
		  // httpVariables->SetHighMtuSize  (1500); // bytes
		  Config::SetDefault ("ns3::ThreeGppHttpVariables::HighMtuSize", UintegerValue (1500));

		  // Create HTTP client helper
          Ipv4Address clientAddress = rxInterface.GetAddress (index);
          ThreeGppHttpClientHelper clientHelper (clientAddress);
		  // ThreeGppHttpClientHelper clientHelper (serverAddress);

          // Install HTTP client
          sinkApplications = clientHelper.Install (rxWifiNode.Get (index));
          Ptr<ThreeGppHttpClient> httpClient = sinkApplications.Get (0)->GetObject<ThreeGppHttpClient> ();
          
	  	}
	  else // UNCOMP_VIDEO
	  	{
	  	  auto ipv4 = rxWifiNode.Get (index)->GetObject<Ipv4> ();
          const auto address = ipv4->GetAddress (1, 0).GetLocal ();
          InetSocketAddress sinkSocket (address, portNumber++);
          OnOffHelper src ("ns3::UdpSocketFactory", sinkSocket); 
          src.SetAttribute ("MaxBytes", UintegerValue (0));
          src.SetAttribute ("PacketSize", UintegerValue (payloadSize));
          src.SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=1e6]"));
          src.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0]"));
          src.SetAttribute ("DataRate", DataRateValue (DataRate (uncompVideoRate)));
          sourceApplications.Add (src.Install (txWifiNode.Get (0)));
          PacketSinkHelper packetSinkHelper ("ns3::UdpSocketFactory", sinkSocket);
          sinkApplications.Add (packetSinkHelper.Install (rxWifiNode.Get (index)));
	  	}
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
      wifiPhy.EnablePcap ("tgad_conf_Traces_AccessPoint", txDevice, false);
      wifiPhy.EnablePcap ("tgad_conf_Traces_Station", rxDevice, false);
    }
   

  Simulator::Stop (Seconds (simulationTime + 1));
  Simulator::Run ();
  Simulator::Destroy ();

  /* Print Results Summary */
  // std::cerr << i << " " << centerLocation << " "  << clientRS << " "  << apPos << " ";
  // std::copy(clientPos.begin(), clientPos.end(), std::ostream_iterator<Vector>(std::cerr, " "));
  // std::cerr << "Communication pair #" << commID << ": ";
  std::cerr << "STA" << commEntity.at(0) << " -> " << "STA" << commEntity.at(1) << ": ";
  std::copy(losFlag.begin(), losFlag.end(), std::ostream_iterator<bool>(std::cerr, " "));
  for (unsigned index = 0; index < sinkApplications.GetN (); ++index)
    {
      	uint64_t totalPacketsThrough = StaticCast<PacketSink> (sinkApplications.Get (index))->GetTotalRx ();
      	throughput += ((totalPacketsThrough * 8) / ((simulationTime-1) * 1000000.0)); //Mbit/s
      	std::cerr << ((totalPacketsThrough * 8) / ((simulationTime-1) * 1000000.0)) << " ";
    }
  
  std::cerr << std::endl;
  }
  
  return 0;
}

