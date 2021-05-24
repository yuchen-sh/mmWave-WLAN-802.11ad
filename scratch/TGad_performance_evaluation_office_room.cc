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

/**
 * Simulation Objective:
 * This script is used to evaluate the performance of office conference scenario defined in TGad Evaluation document 

 * Network Topology:
 * The scenario consists of 8 STAs and 1 AP, and traffic models are described as follow:
 *
 *
 *
 * Running Simulation:
 * ./waf --run "TGad_performance_evaluation_office_room"
 *
 * Simulation Output:
 * 1. Throughput (Mbps)
 */

NS_LOG_COMPONENT_DEFINE ("TGadEvalLivingRoom");



using namespace ns3;
using namespace std;

/**  Application Variables **/
uint64_t totalRx = 0;
double throughput = 0;
uint32_t allocationType = 0;               /* The type of channel access scheme during DTI (CBAP is the default) */


/*
void
CalculateThroughput (Ptr<PacketSink> sink, uint64_t lastTotalRx, double averageThroughput)
{
  Time now = Simulator::Now ();                                         // Return the simulator's virtual time. 
  double cur = (sink->GetTotalRx() - lastTotalRx) * (double) 8/1e5;     // Convert Application RX Packets to MBits. 
  std::cout << now.GetSeconds () << '\t' << cur << std::endl;
  lastTotalRx = sink->GetTotalRx ();
  averageThroughput += cur;
  Simulator::Schedule (MilliSeconds (100), &CalculateThroughput, sink, lastTotalRx, averageThroughput);
}
*/



int
main(int argc, char *argv[])
{
  LogComponentEnable ("TGadEvalLivingRoom", LOG_LEVEL_ALL);
  //LogComponentEnable ("MacLow", LOG_LEVEL_ALL);
  //LogComponentEnable ("EdcaTxopN", LOG_LEVEL_ALL);
  //LogComponentEnable ("Obstacle", LOG_LEVEL_ALL);
  // LogComponentEnable ("YansWifiChannel", LOG_LEVEL_ALL);
  //LogComponentEnable ("TruncatedNormalDistribution", LOG_LEVEL_ALL);
  // LogComponentEnable ("DmgApWifiMac", LOG_LEVEL_ALL);

  uint32_t payloadSize = 1472;                  /* Application payload size in bytes. */
  string dataRate = "4500Mbps";                 /* Application data rate. */
  uint32_t msduAggregationSize = 8000; // 7935; /* The maximum aggregation size for A-MSDU in Bytes. */
  uint32_t mpduAggregationSize = 262143;        /* The maximum aggregation size for A-MSPU in Bytes. */
  uint32_t queueSize = 1000;                    /* Wifi MAC Queue Size. */
  string phyMode = "DMG_MCS12";                 /* Type of the Physical Layer. */
  bool verbose = false;                         /* Print Logging Information. */
  double simulationTime = 1.5; // 1.5 , 1.125   /* Simulation time in seconds. */
  bool pcapTracing = false;                     /* PCAP Tracing is enabled. */
  double x = 4.5;
  double y = 3;
  double z = 3;
  uint16_t clientRS = 1; // 18
  uint16_t distRS = 1;
  uint16_t i = 1; // number of AP
  uint16_t ii = 1; // iith AP
  // uint16_t shapeCategary = 0;
  // uint16_t centerLocation = 1;
  // double platformSize = 30;
  Vector roomSize = Vector (7.0, 7.0, 3.0);
  // Vector trackSize = Vector(0, 0.065, 0.047); // x dimension is a parameter to be changed
  // double moveStep = 0.1;
  Vector apDimension = Vector (0.23, 0.23, 0.12);
  Vector STADimension = Vector (0, 0, 0);
  Vector apPos_FOFC = Vector (1.5, 0.5, 2.9 + apDimension.z*0.5); // Vector (3.5, 6.5, 1.5 + apDimension.z*0.5); // Vector (0.1, 6.9, 2.7 + apDimension.z*0.5);
  double depSD = 1;
  uint32_t clientNo = 1; // number of sta, must fixed at 1 for this script
  // bool hermesFlag = 0;  // 0--Multiple static AP, 1--mobile AP
  uint16_t obsNumber = 20; // 22, 43, furniture-type obstacles
  double human_obs_ratio = 0; // if no human blockage, set as 0 // 0.5
  // bool SV_channel = false; // enable SV channel
  bool TGad_channel = true; // enable TGad_channel
  int reflectorDenseMode = 2; // 1/2/3 -> lower/medium/higher density of highly-reflective objects in the room
  uint16_t clientDistType = 0; // 0-possion, 1-trucated-normal, 2-OD-Truncated Normal, 3-OD
  // bool mobilityUE = 0; // 0--static, 1--mobile (random walk)
  bool obsConflictCheck = false; // true--checking obstacle conflicts when allocating obstacles
  bool FOFC = true; // true -- allocate fixed obstacles and clients in the scenario; false: randomly generate obstacles and clients

  std::vector<double> xPos, yPos, wObs, lObs, hObs, dirObs, hObs_min;
  // double a;
  // string filename_obs = "obs_info/case_fixed_obs_living_room.txt";
  // string filename_client = "UE_info/case_UE_pos_living_room.txt";
  std::vector<double> obs_temp;
  if (FOFC == true)
  	{
  // Fixed obstacle setting
  // read the obstacle info from the txt
  /*
  std::ifstream ifs;
  ifs.open(filename_obs, ios::in);
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
  */
  obs_temp = {3.0000,0.1000,0.1000,0.1000,0.1000,0.3000,0.3500,0.3500,0.4200,0.3000,0.3500,0.3500,0.4200,0.3000,0.3500,0.3500,0.4200,0.3000,0.3500,0.3500,0.4200,0.3000,0.3500,0.3500,0.4200,0.3000,0.3500,0.3500,0.4200,0.3000,0.3500,0.3500,0.4200,0.3000,0.3500,0.3500,0.4200,1.4000,0.1000,0.1000,0.1000,0.1000,0.3000,0.0500,0.0500,0.0600,0.3000,0.0500,0.0500,0.0600,0.3000,0.0500,0.0500,0.0600,0.3000,0.0500,0.0500,0.0600,0.3000,0.0500,0.0500,0.0600,0.3000,0.0500,0.0500,0.0600,0.3000,0.0500,0.0500,0.0600,0.3000,0.0500,0.0500,0.0600,
              90.0000,0.0000,0.0000,0.0000,0.0000,0.0000,90.0000,90.0000,0.0000,0.0000,90.0000,90.0000,0.0000,0.0000,0.0000,0.0000,90.0000,0.0000,0.0000,0.0000,90.0000,0.0000,0.0000,0.0000,90.0000,0.0000,0.0000,0.0000,90.0000,0.0000,0.0000,0.0000,90.0000,0.0000,0.0000,0.0000,90.0000,
              1.5000,0.9500,2.0500,0.9500,2.0500,1.5000,1.3250,1.6750,1.5000,1.5000,1.3250,1.6750,1.5000,2.7200,2.7200,2.7200,2.9250,2.3500,2.3500,2.3500,2.5550,2.3500,2.3500,2.3500,2.5550,0.6500,0.6500,0.6500,0.4450,0.7000,0.7000,0.7000,0.4950,0.5500,0.5500,0.5500,0.3450,2.2500,3.6000,
              3.6000,0.9000,0.9000,4.0000,4.0000,4.0000,4.2100,0.5000,0.5000,0.5000,0.2900,4.2250,4.3750,4.0500,4.2250,3.2000,3.3500,3.0250,3.2000,1.2000,1.3500,1.0250,1.2000,3.2000,3.3500,3.0250,3.2000,2.6000,2.7500,2.4250,2.6000,1.2000,1.3500,1.0250,1.2000,0.9300,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000};
  
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
          dirObs.push_back(obs_temp.at(io));
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
  cmd.AddValue ("i", "simulation iteration", i);
  cmd.AddValue ("ii", "simulation iteration ii", ii);
  cmd.AddValue ("z", "ap z", z);
  // cmd.AddValue ("centerLocation", "center Location Type", centerLocation);
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

  // set human obstacle number
  uint16_t obsNumber_human = (uint16_t)(floor(obsNumber*1.0*human_obs_ratio));
  // uint16_t obsNumber_human = (uint16_t)(floor(43*1.0*human_obs_ratio));

  // scale parameter due to shell script without float value
  // platformSize = platformSize*0.1;
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
  
  /**** Set up Channel ****/
  DmgWifiChannelHelper wifiChannelHelper;
  /* Simple propagation delay model */
  wifiChannelHelper.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
  // if ((TGad_channel == true) && (SV_channel == false))
  	// {
  	   /*log-distance based model with 60GHz (802.11ad/ay) wavelength */
	   // wifiChannelHelper.AddPropagationLoss ("ns3::LogDistancePropagationLossModel", "ReferenceLoss", DoubleValue (0.0),
	                                                                                //  "Exponent", DoubleValue (2.23),
	                                                                                //  "ReferenceDistance", DoubleValue (3.97887e-4));
  	// }
  // else
  	// {
       /* Friis model with standard-specific wavelength */
       wifiChannelHelper.AddPropagationLoss ("ns3::FriisPropagationLossModel", "Frequency", DoubleValue (60.48e9));
  	// }


  // ------------------------ Define different channels ------------------------------- //
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

  /* Set default algorithm for all nodes to be constant rate */
  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager", "ControlMode", StringValue (phyMode),
                                                                "DataMode", StringValue (phyMode));

																
  /* Make two nodes and set them up with the PHY and the MAC */
  // NodeContainer staWifiNode;
  // staWifiNode.Create (clientNo);
  // NodeContainer apWifiNode;
  // apWifiNode.Create (1);
  NodeContainer wifiNodes;
  wifiNodes.Create (4);
  Ptr<Node> apWifiNode = wifiNodes.Get (0);
  Ptr<Node> STA1Node = wifiNodes.Get (1);
  Ptr<Node> STA2Node = wifiNodes.Get (2);
  Ptr<Node> STA4Node = wifiNodes.Get (3);
  // Ptr<Node> eastNode = wifiNodes.Get (4);

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

  NetDeviceContainer apDevice;
  apDevice = wifi.Install (wifiPhy, wifiMac, apWifiNode);

  wifiMac.SetType ("ns3::DmgStaWifiMac",
                   "Ssid", SsidValue (ssid), "ActiveProbing", BooleanValue (false),
                   "BE_MaxAmpduSize", UintegerValue (mpduAggregationSize),
                   "BE_MaxAmsduSize", UintegerValue (msduAggregationSize));

  NetDeviceContainer staDevices;
  // staDevice = wifi.Install (wifiPhy, wifiMac, staWifiNode);
  staDevices = wifi.Install (wifiPhy, wifiMac, NodeContainer (STA1Node, STA2Node, STA4Node));

  // Set Obstacles and track location
  Ptr<Obstacle> labScenarios = CreateObject<Obstacle> ();
  labScenarios->SetObstacleNumber(obsNumber);
  labScenarios->SetHumanObstacleNumber(obsNumber_human);

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
  
  
  // ---- STB deployment ---- 
  // double rl = roomSize.x;
  // double rw = roomSize.y;
  if (FOFC == true) // with a given STB deployment in the living room
  	{
  	  apPosVec.push_back(apPos_FOFC);
  	  // apPosVec.push_back(Vector (3.5,4.8, 1.5 + apDimension.z*0.5));
  	}
  else // LoS-optimal deployment
  	{
      apPosVec = labScenarios->AllocateOptAP(roomSize, i);
  	}

  x = apPosVec.at(ii - 1).x;
  y = apPosVec.at(ii - 1).y;
  z = apPosVec.at(ii - 1).z;

  if (FOFC == true)
  	{
  	  labScenarios->AllocateObstacle_Fixed_withHB(Box (x-apDimension.x/2, x+apDimension.x/2, y-apDimension.y/2, y+apDimension.y/2, z-apDimension.z, z+apDimension.z), roomSize, clientRS, xPos, yPos, wObs, lObs, hObs, hObs_min, dirObs);
  	}
  else // RORC
  	{
  	  labScenarios->AllocateObstacle(Box (x-apDimension.x/2, x+apDimension.x/2, y-apDimension.y/2, y+apDimension.y/2, z-apDimension.z, z+apDimension.z), roomSize, clientRS);
  	}
  // labScenarios->AllocateObstacle(Box (x-apDimension.x/2, x+apDimension.x/2, y-apDimension.y/2, y+apDimension.y/2, z-apDimension.z, z+apDimension.z), roomSize, clientRS);
  // AllocateObstacle_Fixed(Box railLocation, Vector roomSize, uint16_t clientRS, std::vector<double> xPos, std::vector<double> yPos, std::vector<double> wObs, std::vector<double> lObs, std::vector<double> hObs, std::vector<double> dirObs)
  // labScenarios->AllocateObstacle_Fixed(Box (x-apDimension.x/2, x+apDimension.x/2, y-apDimension.y/2, y+apDimension.y/2, z-apDimension.z, z+apDimension.z), roomSize, clientRS, xPos, yPos, wObs, lObs, hObs, dirObs);  



  // start to record cpu running time
  // clock_t startRunTime, endRunTime;
  // double totaltime;
  // startRunTime=clock();


  /* Setting mobility model */
  MobilityHelper mobility; // WifiNode's mobility model
  // MobilityHelper mobility2; // user's ,obility model
  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
  Vector apPos = Vector (x, y, z);

  // allocate fixed client locations
  // std::vector<Vector> clientPos = labScenarios->IdentifyCLientLocation (clientRS, distRS, depSD, clientNo, clientDistType);
  std::vector<Vector> clientPos;
  if (FOFC == true)
  	{
        // std::vector<Vector> clientPos;
  		std::vector<double> client_temp;
		/*
  		std::ifstream ifs1;
  		ifs1.open(filename_client, ios::in);
  		if (!ifs1) // no file exists
		{
			NS_LOG_INFO("no file exist!");
			std::cerr << "no file exist! (client file)" << std::endl;
		}
  		// double a;
  		if (ifs1.is_open())
		{
			for (; ifs1 >> a;)
			{
				client_temp.push_back(a);
			}
			ifs1.close();
		}
  		else
		{
			NS_LOG_INFO("File exists but Unable to open");
			std::cerr << "File exists but Unable to open! (client file)" << std::endl;
		}
		*/
        client_temp = {1.75, 2.3, 1.0,  // projector position, STA1
                       1.90, 1.5, 1.0,  // STA2
                       1.30, 2.4, 1.0   // STA4
                       };
		
  		uint32_t FixedUENumber = client_temp.size()/3;
  		for (uint32_t iu = 0; iu < FixedUENumber; ++iu)
  			{
  		      Vector thisUE = Vector (client_temp.at(iu*3), client_temp.at(iu*3 + 1), client_temp.at(iu*3 + 2));
  		      clientPos.push_back(thisUE); // only for one-user case
  			}
  	}
  else // RORC
  	{
  	  clientPos = labScenarios->IdentifyCLientLocation (clientRS, distRS, depSD, clientNo, clientDistType);
  	}
  

  // LoS analysis and channel setting
  positionAlloc->Add (apPos);	/* PCP/AP */
  std::vector<bool> losFlag;
  // std::vector<std::vector<bool> > losFlag_mul(i, std::vector<bool>(clientNo)); // only for multi-user case
  for (uint16_t clientId = 0; clientId < clientNo; clientId++)
    positionAlloc->Add (clientPos.at(clientId));  /* DMG STAs */

  // do LoS analysis for different physical links
  std::vector<Vector> clientPos_L1;
  clientPos_L1.push_back(clientPos.at(2));
  labScenarios->LoSAnalysis (apPos, clientPos_L1, apDimension); // STA4 -> ap

  std::vector<Vector> clientPos_L2;
  clientPos_L2.push_back(clientPos.at(0));
  labScenarios->LoSAnalysis (clientPos.at(1), clientPos_L2, STADimension); // STA2 -> STA1

  // record LoS/NLoS results
  std::pair<double, double> fadingInfo; 
  fadingInfo = labScenarios->GetFadingInfo(apPos, clientPos.at(2));
  losFlag.push_back(fadingInfo.first);
  fadingInfo = labScenarios->GetFadingInfo(clientPos.at(1), clientPos.at(0));
  losFlag.push_back(fadingInfo.first);
  
  
  wifiChannel->SetScenarioModel(labScenarios);
  // wifiChannel->SetSVChannelEnabler(SV_channel);
  wifiChannel->SetTGadChannelEnabler(TGad_channel);
  wifiChannel->SetSVChannelReflectedMode(reflectorDenseMode);
  labScenarios->SetObsConflictCheck(obsConflictCheck);

  // obstacle density
  double obsDensity = obsNumber*1.0/(roomSize.x*roomSize.y);
  wifiChannel->SetObsDensity(obsDensity);


  /*
  // For multi-user, multi-AP cases
  if ((clientNo > 1)&&(i > 1))
  	{
  	  for (uint16_t i_ap = 0; i_ap < i; ++i_ap)
  	  	{
  	  	  losFlag_mul.push_back(labScenarios->LoSAnalysis_MultiAP (apPosVec[i_ap], clientPos, apDimension));
  	  	}
  	}
  */
  
  
  // endRunTime=clock();

  // AP's mobility pattern settings
  mobility.SetPositionAllocator (positionAlloc);
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (wifiNodes);
  

  /* Internet stack*/
  InternetStackHelper stack;
  stack.Install (wifiNodes);
  // stack.Install (staWifiNode);

  Ipv4AddressHelper address;
  address.SetBase ("10.0.0.0", "255.255.255.0");
  Ipv4InterfaceContainer apInterface;
  apInterface = address.Assign (apDevice);
  Ipv4InterfaceContainer staInterfaces;
  staInterfaces = address.Assign (staDevices);

  /* Populate routing table */
  Ipv4GlobalRoutingHelper::PopulateRoutingTables ();

  /* We do not want any ARP packets */
  PopulateArpCache ();

  /*
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
   */



  /*** Install Applications ***/

  /* Install UDP Server on sink Nodes */
  PacketSinkHelper sinkHelper ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), 9999));
  ApplicationContainer sinks = sinkHelper.Install (NodeContainer (STA1Node, apWifiNode));

  /** Install UDP Server on src Nodes for STA2 **/
  uint64_t STA1NodeLastTotalRx = 0;
  double STA1NodeAverageThroughput = 0;
  /* Install UDP Transmiter on the STA2 Node (Transmit to the STA1 Node) */
  ApplicationContainer srcApp1;
  OnOffHelper src1 ("ns3::UdpSocketFactory", InetSocketAddress (staInterfaces.GetAddress (1), 9999));
  src1.SetAttribute ("MaxBytes", UintegerValue (0));
  src1.SetAttribute ("PacketSize", UintegerValue (payloadSize));
  src1.SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=1e6]"));
  src1.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0]"));
  src1.SetAttribute ("DataRate", DataRateValue (DataRate (dataRate)));
  srcApp1 = src1.Install (STA2Node);
  srcApp1.Start (Seconds (1.0));

  /** Install UDP Server on src Nodes for STA4 **/
  uint64_t APNodeLastTotalRx = 0;
  double APNodeAverageThroughput = 0;
  /* Install UDP Transmiter on the STA4 Node (Transmit to the AP Node) */
  ApplicationContainer srcApp2;
  OnOffHelper src2 ("ns3::UdpSocketFactory", InetSocketAddress (staInterfaces.GetAddress (2), 9999));
  src2.SetAttribute ("MaxBytes", UintegerValue (0));
  src2.SetAttribute ("PacketSize", UintegerValue (payloadSize));
  src2.SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=1e6]"));
  src2.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0]"));
  src2.SetAttribute ("DataRate", DataRateValue (DataRate (dataRate)));
  srcApp2 = src2.Install (STA4Node);
  srcApp2.Start (Seconds (1.5));


  // endRunTime=clock();
  
  
  sinks.Start (Seconds (0.0));
  sinks.Stop (Seconds (simulationTime));
  // sourceApplications.Start (Seconds (1.0));
  srcApp1.Stop (Seconds (simulationTime));
  srcApp2.Stop (Seconds (simulationTime));


 
  /* Schedule Throughput Calulcations */
  // Simulator::Schedule (Seconds (1.1), &CalculateThroughput, StaticCast<PacketSink> (sinks.Get (0)),
                    //   STA1NodeLastTotalRx, STA1NodeAverageThroughput);

  // Simulator::Schedule (Seconds (1.1), &CalculateThroughput, StaticCast<PacketSink> (sinks.Get (1)),
                    //   APNodeLastTotalRx, APNodeAverageThroughput);
  

  /* Print Traces */
  if (pcapTracing)
    {
      wifiPhy.SetPcapDataLinkType (DmgWifiPhyHelper::DLT_IEEE802_11_RADIO);
      wifiPhy.EnablePcap ("Traces/AccessPoint", apDevice, false);
      wifiPhy.EnablePcap ("Traces/Station", staDevices, false);
    }

  /*apWifiNetDevice = StaticCast<WifiNetDevice> (apDevice.Get (0));
  staWifiNetDevice = StaticCast<WifiNetDevice> (staDevice.Get (0));
  apWifiMac = StaticCast<DmgApWifiMac> (apWifiNetDevice->GetMac ());
  staWifiMac = StaticCast<DmgStaWifiMac> (staWifiNetDevice->GetMac ());
  staWifiMac->TraceConnectWithoutContext ("Assoc", MakeBoundCallback (&StationAssoicated, staWifiMac));*/

  Simulator::Stop (Seconds (simulationTime + 1));
  Simulator::Run ();
  Simulator::Destroy ();

  // Print Results Summary
  // std::cerr << i << " " << centerLocation << " "  << clientRS << " "  << apPos << " ";
  // std::copy(clientPos.begin(), clientPos.end(), std::ostream_iterator<Vector>(std::cerr, " "));
  std::copy(losFlag.begin(), losFlag.end(), std::ostream_iterator<bool>(std::cerr, " "));
  for (unsigned index = 0; index < sinks.GetN (); ++index)
    {
      	uint64_t totalPacketsThrough = StaticCast<PacketSink> (sinks.Get (index))->GetTotalRx ();
      	throughput += ((totalPacketsThrough * 8) / ((simulationTime-1) * 1000000.0)); //Mbit/s
      	std::cerr << ((totalPacketsThrough * 8) / ((simulationTime-1) * 1000000.0)) << " ";
		// std::cerr << totalPacketsThrough*1.0/payloadSize << " ";
		// double distance = std::sqrt((clientPos.at(index).x - apPos.x)*(clientPos.at(index).x - apPos.x)+(clientPos.at(index).y - apPos.y)*(clientPos.at(index).y - apPos.y)+(clientPos.at(index).z - apPos.z)*(clientPos.at(index).z - apPos.z));
		// std::cerr << distance << " ";
    }

  // endRunTime=clock();
  // totaltime=(double)(endRunTime-startRunTime)/CLOCKS_PER_SEC;
  // std::cerr << totaltime << " ";
  
  std::cerr << std::endl;
  
  return 0;
}

