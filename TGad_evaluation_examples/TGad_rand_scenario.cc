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
 * This script is used to evaluate the performance of TGad scenarios with random obstacles and random clients

 * Running Simulation:
 * ./waf --run "TGad_Rand_scenario"
 *
 * Simulation Output:
 * client#1: LoS/NLoS status (LoS -- 0; NLoS -- 1) client#2: LoS/NLoS status (LoS -- 0; NLoS -- 1) ...
 * client#1: Throughput (Mbps) client#2: Throughput (Mbps) ...
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
  uint16_t clientRS = 1;
  uint16_t distRS = 1;
  uint16_t i = 1; // number of AP
  uint16_t ii = 1; // iith AP
  Vector roomSize = Vector (12.0, 8.0, 3.0); // room configuration
  Vector apDimension = Vector (0.23, 0.23, 0.12);
  Vector apPos_FOFC = Vector (6.0, 4.0, 3.0); // AP's position
  double depSD = 1;
  uint32_t clientNo = 2; // number of sta
  uint16_t obsNumber = 20; // furniture-type obstacles
  double human_obs_ratio = 0; // if no human blockage, set as 0 
  bool TGad_channel = true; // enable TGad_channel
  int reflectorDenseMode = 2; // 1/2/3 -> lower/medium/higher density of highly-reflective objects in the room
  uint16_t clientDistType = 0; // 0-possion, 1-trucated-normal, 2-OD-Truncated Normal, 3-OD
  bool mobilityUE = 0; // 0--static, 1--mobile (random walk)
  bool obsConflictCheck = false; // true--checking obstacle conflicts when allocating obstacles
  // bool FOFC = false; // true -- allocate fixed obstacles and clients in the scenario; false: randomly generate obstacles and clients


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
  cmd.AddValue ("clientRS", "random seed for client", clientRS);
  cmd.AddValue ("clientDistType", "distribution type for client", clientDistType);
  cmd.AddValue ("distRS", "random seed for truncated normal distribution", distRS);
  cmd.AddValue ("depSD", "random seed for dependent distribution", depSD);
  cmd.AddValue ("clientNo", "Number of client", clientNo);
  cmd.AddValue ("obsNumber", "obstacle Number", obsNumber);  
  cmd.Parse (argc, argv);
  

  // set human obstacle number
  uint16_t obsNumber_human = (uint16_t)(floor(obsNumber*1.0*human_obs_ratio));

  // scale parameter due to shell script without float value
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
  apPosVec.push_back(apPos_FOFC);

  x = apPosVec.at(ii - 1).x;
  y = apPosVec.at(ii - 1).y;
  z = apPosVec.at(ii - 1).z;


  // ---- Random obstacle distribution
  labScenarios->AllocateObstacle(Box (x-apDimension.x/2, x+apDimension.x/2, y-apDimension.y/2, y+apDimension.y/2, z-apDimension.z, z+apDimension.z), roomSize, clientRS);


  /* Setting mobility model */
  MobilityHelper mobility1; // AP's mobility model
  MobilityHelper mobility2; // user's ,obility model
  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
  Vector apPos = Vector (x, y, z);

  // ---- randomly allocate client locations
  std::vector<Vector> clientPos;
  clientPos = labScenarios->IdentifyCLientLocation (clientRS, distRS, depSD, clientNo, clientDistType);
  

  // LoS analysis and channel setting
  positionAlloc->Add (apPos);	/* PCP/AP */
  std::vector<bool> losFlag;
  std::vector<std::vector<bool> > losFlag_mul(i, std::vector<bool>(clientNo)); // only for multi-user case
  for (uint16_t clientId = 0; clientId < clientNo; clientId++)
    positionAlloc->Add (clientPos.at(clientId));  /* DMG STA */
  labScenarios->LoSAnalysis (apPos, clientPos, apDimension);
  wifiChannel->SetScenarioModel(labScenarios);
  wifiChannel->SetTGadChannelEnabler(TGad_channel);
  labScenarios->SetObsConflictCheck(obsConflictCheck);

  // obstacle density
  double obsDensity = obsNumber*1.0/(roomSize.x*roomSize.y);
  wifiChannel->SetObsDensity(obsDensity);

  for (uint16_t clientId = 0; clientId < clientNo; clientId++)
    {
      std::pair<double, double> fadingInfo = labScenarios->GetFadingInfo(apPos, clientPos.at(clientId));
      losFlag.push_back(fadingInfo.first);
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


  Simulator::Stop (Seconds (simulationTime + 1));
  Simulator::Run ();
  Simulator::Destroy ();

  /* Print Results Summary */
  // std::cerr << i << " " << centerLocation << " "  << clientRS << " "  << apPos << " ";
  // std::copy(clientPos.begin(), clientPos.end(), std::ostream_iterator<Vector>(std::cerr, " "));
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

