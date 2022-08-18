/*
 * Copyright (c) 2021
 * Author: Yuchen Liu*/

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
 * This script is used to evaluate the performance of living room scenario defined in TGad Evaluation document 

 * Network Topology:
 * The scenario consists of 1 set-top box (STB) and 1 TV, and traffic model is uncompressed videos.
 *
 *
 *
 * Running Simulation:
 * ./waf --run "TGad_performance_evaluation_living_room --humanBlock=false"  (LoS channel)
 * ./waf --run "TGad_performance_evaluation_living_room --humanBlock=true"  (NLoS channel)
 *
 * Simulation Output:
 * 1. LoS/NLoS status (LoS -- 0; NLoS -- 1)
 * 2. Throughput (Mbps)
 */

NS_LOG_COMPONENT_DEFINE ("TGadEvalLivingRoom");



using namespace ns3;
using namespace std;

/**  Application Variables **/
uint64_t totalRx = 0;
double throughput = 0;
uint32_t allocationType = 0;               /* The type of channel access scheme during DTI (CBAP is the default) */

int
main(int argc, char *argv[])
{
  LogComponentEnable ("TGadEvalLivingRoom", LOG_LEVEL_ALL);

  uint32_t payloadSize = 1472;                  /* Application payload size in bytes. */
  string dataRate = "4500Mbps";                 /* Application data rate. */
  string uncompVideoRate = "3000Mbps";          /* 1080p, 1920*1080 */
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
  Vector roomSize = Vector (7.0, 7.0, 3.0);
  Vector apDimension = Vector (0.23, 0.23, 0.12);
  Vector apPos_FOFC = Vector (5.3, 6.35, 1.5 + apDimension.z*0.5); // Vector (3.5, 6.5, 1.5 + apDimension.z*0.5); // Vector (0.1, 6.9, 2.7 + apDimension.z*0.5);
  double depSD = 1;
  uint32_t clientNo = 1; // number of sta, must fixed at 1 for this script
  uint16_t obsNumber = 20; // 22, 43, obstacles
  double human_obs_ratio = 0; // if no human blockage, set as 0. NOTE: only used for RORC mode
  bool TGad_channel = true; // enable TGad_channel
  int reflectorDenseMode = 2; // 1/2/3 -> lower/medium/higher density of highly-reflective objects in the room
  uint16_t clientDistType = 0; // 0-possion, 1-trucated-normal, 2-OD-Truncated Normal, 3-OD
  bool mobilityUE = 0; // 0--static, 1--mobile (random walk)
  bool obsConflictCheck = false; // true--checking obstacle conflicts when allocating obstacles
  bool FOFC = true; // true -- allocate fixed obstacles and clients in the scenario; false: randomly generate obstacles and clients
  bool humanBlock = false; // true -- enable a human obstacle that blocks the link btw STB and TV



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


  std::vector<double> xPos, yPos, wObs, lObs, hObs, dirObs, hObs_min;
  std::vector<double> obs_temp;
  if (FOFC == true)
  	{
  // Fixed obstacle setting
  obs_temp = {2.0, 0.7, 0.7, 2.24, 2.0, 0.7, 0.7, 2.24, 1.5, 0.05, 0.05, 0.05, 0.05, 0.6, 0.7, 0.7, 0.84, 1.6, 
              0.7, 0.12, 0.12, 0.14, 0.7, 0.12, 0.12, 0.14, 0.6, 0.05, 0.05, 0.05, 0.05, 0.6, 0.12, 0.12, 0.14, 0.5,
              0.8, 1.2, 1.2, 1.26, 0.8, 1.2, 1.2, 1.26, 1.1, 1.03, 1.03, 1.03, 1.03, 0.8, 1.2, 1.2, 1.26, 1.5, 
              90, 0, 0, 90, 0, 90, 90, 0, 90, 0, 0, 0, 0, 45, 135, 135, 45, 0,
              2.6, 2.6, 2.6, 2.18, 4.3, 3.24, 5.36, 4.3, 4.3, 4.1, 4.5, 4.1, 4.5, 5.36, 5, 5.72, 5.36, 4.5,
              3.5, 2.44, 4.56, 3.5, 1.65, 1.65, 1.65, 1.23, 3.5, 4.15, 4.15, 2.85, 2.85, 5.1, 5.1, 5.1, 5.47, 6.25,
              0, 0, 0, 0, 0, 0, 0, 0, 1.03, 0, 0, 0, 0, 0, 0, 0, 0, 0};

  if (humanBlock == true)// add one human obstacle
  	{
      obs_temp = {2.0, 0.7, 0.7, 2.24, 2.0, 0.7, 0.7, 2.24, 1.5, 0.05, 0.05, 0.05, 0.05, 0.6, 0.7, 0.7, 0.84, 1.6, 0.5,
              0.7, 0.12, 0.12, 0.14, 0.7, 0.12, 0.12, 0.14, 0.6, 0.05, 0.05, 0.05, 0.05, 0.6, 0.12, 0.12, 0.14, 0.5, 0.2,
              0.8, 1.2, 1.2, 1.26, 0.8, 1.2, 1.2, 1.26, 1.1, 1.03, 1.03, 1.03, 1.03, 0.8, 1.2, 1.2, 1.26, 1.5, 1.75,
              90, 0, 0, 90, 0, 90, 90, 0, 90, 0, 0, 0, 0, 45, 135, 135, 45, 0, 45,
              2.6, 2.6, 2.6, 2.18, 4.3, 3.24, 5.36, 4.3, 4.3, 4.1, 4.5, 4.1, 4.5, 5.36, 5, 5.72, 5.36, 4.5, 6.2,
              3.5, 2.44, 4.56, 3.5, 1.65, 1.65, 1.65, 1.23, 3.5, 4.15, 4.15, 2.85, 2.85, 5.1, 5.1, 5.1, 5.47, 6.6, 5.05,
              0, 0, 0, 0, 0, 0, 0, 0, 1.03, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  	}
  
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

  

  // set human obstacle number
  uint16_t obsNumber_human = (uint16_t)(floor(obsNumber*1.0*human_obs_ratio));
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
                   // "ATIPresent", BooleanValue (false)
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

  if (FOFC == true)
  	{
  	  labScenarios->AllocateObstacle_Fixed_withHB(Box (x-apDimension.x/2, x+apDimension.x/2, y-apDimension.y/2, y+apDimension.y/2, z-apDimension.z, z+apDimension.z), roomSize, clientRS, xPos, yPos, wObs, lObs, hObs, hObs_min, dirObs);
  	}
  else // RORC
  	{
  	  labScenarios->AllocateObstacle(Box (x-apDimension.x/2, x+apDimension.x/2, y-apDimension.y/2, y+apDimension.y/2, z-apDimension.z, z+apDimension.z), roomSize, clientRS);
  	}


  /* Setting mobility model */
  MobilityHelper mobility1; // AP's mobility model
  MobilityHelper mobility2; // user's ,obility model
  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
  Vector apPos = Vector (x, y, z);

  // allocate fixed client locations
  std::vector<Vector> clientPos;
  if (FOFC == true)
  	{
        // std::vector<Vector> clientPos;
  		std::vector<double> client_temp;
        client_temp = {7.0, 3.5, 1.5}; // TV position

  		Vector thisUE = Vector (client_temp.at((clientRS - 1)*3), client_temp.at((clientRS - 1)*3 + 1), client_temp.at((clientRS - 1)*3 + 2));
  		clientPos.push_back(thisUE); // only for one-user case
  	}
  else // RORC
  	{
  	  clientPos = labScenarios->IdentifyCLientLocation (clientRS, distRS, depSD, clientNo, clientDistType);
  	}
  

  // LoS analysis and channel setting
  positionAlloc->Add (apPos);	/* PCP/AP */
  std::vector<bool> losFlag;
  std::vector<std::vector<bool> > losFlag_mul(i, std::vector<bool>(clientNo)); // only for multi-user case
  for (uint16_t clientId = 0; clientId < clientNo; clientId++)
    positionAlloc->Add (clientPos.at(clientId));  /* DMG STA */
  labScenarios->LoSAnalysis (apPos, clientPos, apDimension);
  wifiChannel->SetScenarioModel(labScenarios);
  wifiChannel->SetTGadChannelEnabler(TGad_channel);
  wifiChannel->SetSVChannelReflectedMode(reflectorDenseMode);
  labScenarios->SetObsConflictCheck(obsConflictCheck);

  // obstacle density
  double obsDensity = obsNumber*1.0/(roomSize.x*roomSize.y);
  wifiChannel->SetObsDensity(obsDensity);

  for (uint16_t clientId = 0; clientId < clientNo; clientId++)
    {
      std::pair<double, double> fadingInfo = labScenarios->GetFadingInfo(apPos, clientPos.at(clientId));
      losFlag.push_back(fadingInfo.first);
    }

  // For multi-user, multi-AP cases
  if ((clientNo > 1)&&(i > 1))
  	{
  	  for (uint16_t i_ap = 0; i_ap < i; ++i_ap)
  	  	{
  	  	  losFlag_mul.push_back(labScenarios->LoSAnalysis_MultiAP (apPosVec[i_ap], clientPos, apDimension));
  	  	}
  	}

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
      src.SetAttribute ("DataRate", DataRateValue (DataRate (uncompVideoRate)));
      sourceApplications.Add (src.Install (apWifiNode.Get (0)));
      PacketSinkHelper packetSinkHelper ("ns3::UdpSocketFactory", sinkSocket);
      sinkApplications.Add (packetSinkHelper.Install (staWifiNode.Get (index)));
   }

  
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


  Simulator::Stop (Seconds (simulationTime + 1));
  Simulator::Run ();
  Simulator::Destroy ();

  /* Print Results Summary */
  std::copy(losFlag.begin(), losFlag.end(), std::ostream_iterator<bool>(std::cerr, " "));
  for (unsigned index = 0; index < sinkApplications.GetN (); ++index)
    {
      	uint64_t totalPacketsThrough = StaticCast<PacketSink> (sinkApplications.Get (index))->GetTotalRx ();
      	throughput += ((totalPacketsThrough * 8) / ((simulationTime-1) * 1000000.0)); //Mbit/s
      	std::cerr << ((totalPacketsThrough * 8) / ((simulationTime-1) * 1000000.0)) << " ";
    }
  
  std::cerr << std::endl;
  return 0;
}

