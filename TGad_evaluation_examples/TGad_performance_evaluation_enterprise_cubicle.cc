/*
 * Copyright (c) 2021
 * Author: Yuchen */

#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/wifi-module.h"
#include "ns3/point-to-point-module.h"
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
 * This script is used to evaluate the performance of enterprise cubicle scenario defined in TGad Evaluation document 

 * Network Topology:
 * The scenario consists of 1 AP (STA 0) and 3 STAs, and traffic model includes lightly compressed video, local file transfer, web browsing, and hard disk file transfer.
 * STA2 -> STA1 (lightly compressed video)
 * STA0 -> STA2 (local file transfer)
 * STA2 -> STA3 (local file transfer)
 * STA0 -> STA2 (web browsing)
 * STA3 -> STA2 (hard disk file)

 * Running Simulation:
 * ./waf --run "TGad_performance_evaluation_enterprise_cubicle"
 *
 * Simulation Output:
 * 1. LoS/NLoS status (LoS -- 0; NLoS -- 1)
 * 2. Throughput (Mbps)
 */

NS_LOG_COMPONENT_DEFINE ("TGadEvalCubicleRoom");



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
#define HARD_DISK_FILE 4


int
main(int argc, char *argv[])
{
  LogComponentEnable ("TGadEvalCubicleRoom", LOG_LEVEL_ALL);
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
  double simulationTime = 1.2;                  /* Simulation time in seconds. */
  bool pcapTracing = false;                     /* PCAP Tracing is enabled. */
  double x = 4.5;
  double y = 3;
  double z = 3;
  uint16_t clientRS = 1; // 18
  uint16_t distRS = 1;
  uint16_t i = 1; // number of AP
  uint16_t ii = 1; // iith AP
  Vector roomSize = Vector (11.0, 13.0, 3.0);
  Vector apDimension = Vector (0.23, 0.23, 0.12);
  Vector apPos_FOFC = Vector (2.8, 6.0, 2.9 + apDimension.z*0.5);
  Vector serverPos = Vector (0.0, 0.0, 0.0);
  Vector LaptopDimension_sc = Vector (0.3, 0.21, 0.23);
  Vector MonitorDimension = Vector (0.5, 0.16, 0.30);
  Vector HardDriveDimension = Vector (0.2, 0.1, 0.08);
  uint16_t numSTAs = 3;
  uint16_t numCommPair = 5; // 5
  double depSD = 1;
  uint32_t clientNo = 1; // number of sta, must fixed at 1 for this script
  uint16_t obsNumber = 20; // obstacle number when FOFC is set as false
  bool TGad_channel = true; // enable TGad_channel
  int reflectorDenseMode = 2; // 1/2/3 -> lower/medium/higher density of highly-reflective objects in the room
  uint16_t clientDistType = 0; // 0-possion, 1-trucated-normal, 2-OD-Truncated Normal, 3-OD
  bool mobilityUE = 0; // 0--static, 1--mobile (random walk)
  bool obsConflictCheck = true; // true--checking obstacle conflicts when allocating obstacles
  bool FOFC = true; // true -- allocate fixed obstacles and clients in the scenario; false: randomly generate obstacles and clients
  uint16_t commID = 1; // range: 1 -- 5
  uint16_t trafficMode = UNCOMP_VIDEO; // by default
  double additionalTime = 0;
  double re = 0.0; // final result
  string socketType = "ns3::UdpSocketFactory";    /* Socket Type (TCP/UDP). */
  Ptr<PacketSink> sink;
  // uint16_t RNG = 1000;


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
  STApos.push_back(Vector (1.42, 10.70, 0.95 + MonitorDimension.z*0.5)); // STA1
  STApos.push_back(Vector (2.35, 10.75, 0.95 + LaptopDimension_sc.z*0.5)); // STA2
  STApos.push_back(Vector (1.40, 9.40, 0.95 + HardDriveDimension.z*0.5)); // STA3


  for (commID = 1; commID <= numCommPair; ++commID)
  {  
  // furniture-type obstacles
  std::vector<double> xPos, yPos, wObs, lObs, hObs, dirObs, hObs_min;
  std::vector<double> obs_temp;
  if (FOFC == true)
  	{
  // Fixed obstacle setting
  obs_temp = {1.8000,1.8000,1.8000,2.5000,1.8000,1.1000,0.3000,0.3500,0.3500,0.4200,1.8000,2.5000,1.8000,1.8000,1.8000,1.1000,0.3000,0.3500,0.3500,0.4200,1.8000,1.8000,1.8000,2.5000,1.8000,1.1000,0.3000,0.3500,0.3500,0.4200,1.8000,2.5000,1.8000,1.9000,1.8000,1.1000,0.3000,0.3500,0.3500,0.4200,1.8000,1.8000,1.8000,2.5000,1.8000,1.1000,0.3000,0.3500,0.3500,0.4200,1.8000,2.5000,1.8000,1.8000,1.8000,1.1000,0.3000,0.3500,0.3500,0.4200,
              1.8000,1.8000,1.8000,2.5000,1.8000,1.1000,0.3000,0.3500,0.3500,0.4200,1.8000,2.5000,1.8000,1.9000,1.8000,1.1000,0.3000,0.3500,0.3500,0.4200,1.8000,1.8000,1.8000,2.5000,1.8000,1.1000,0.3000,0.3500,0.3500,0.4200,1.8000,2.5000,1.8000,1.8000,1.8000,1.1000,0.3000,0.3500,0.3500,0.4200,1.8000,1.8000,1.8000,2.5000,1.8000,1.1000,0.3000,0.3500,0.3500,0.4200,1.8000,2.5000,1.8000,1.9000,1.8000,1.1000,0.3000,0.3500,0.3500,0.4200,
              1.8000,1.8000,1.8000,2.5000,1.8000,1.1000,0.3000,0.3500,0.3500,0.4200,1.8000,2.5000,1.8000,1.8000,1.8000,1.1000,0.3000,0.3500,0.3500,0.4200,1.8000,1.8000,1.8000,2.5000,1.8000,1.1000,0.3000,0.3500,0.3500,0.4200,1.8000,2.5000,1.8000,1.9000,1.8000,1.1000,0.3000,0.3500,0.3500,0.4200,0.1000,0.1000,0.1000,0.1000,0.6000,0.6000,0.3000,0.0500,0.0500,0.0600,0.1000,0.1000,0.1000,0.1000,0.6000,0.6000,0.3000,0.0500,0.0500,0.0600,
              0.1000,0.1000,0.1000,0.1000,0.6000,0.6000,0.3000,0.0500,0.0500,0.0600,0.1000,0.1000,0.1000,0.1000,0.6000,0.6000,0.3000,0.0500,0.0500,0.0600,0.1000,0.1000,0.1000,0.1000,0.6000,0.6000,0.3000,0.0500,0.0500,0.0600,0.1000,0.1000,0.1000,0.1000,0.6000,0.6000,0.3000,0.0500,0.0500,0.0600,0.1000,0.1000,0.1000,0.1000,0.6000,0.6000,0.3000,0.0500,0.0500,0.0600,0.1000,0.1000,0.1000,0.1000,0.6000,0.6000,0.3000,0.0500,0.0500,0.0600,
              0.1000,0.1000,0.1000,0.1000,0.6000,0.6000,0.3000,0.0500,0.0500,0.0600,0.1000,0.1000,0.1000,0.1000,0.6000,0.6000,0.3000,0.0500,0.0500,0.0600,0.1000,0.1000,0.1000,0.1000,0.6000,0.6000,0.3000,0.0500,0.0500,0.0600,0.1000,0.1000,0.1000,0.1000,0.6000,0.6000,0.3000,0.0500,0.0500,0.0600,0.1000,0.1000,0.1000,0.1000,0.6000,0.6000,0.3000,0.0500,0.0500,0.0600,0.1000,0.1000,0.1000,0.1000,0.6000,0.6000,0.3000,0.0500,0.0500,0.0600,
              0.1000,0.1000,0.1000,0.1000,0.6000,0.6000,0.3000,0.0500,0.0500,0.0600,0.1000,0.1000,0.1000,0.1000,0.6000,0.6000,0.3000,0.0500,0.0500,0.0600,1.6000,1.6000,1.6000,1.6000,0.9000,0.9000,0.6000,0.8000,0.8000,1.1000,1.6000,1.6000,1.6000,1.6000,0.9000,0.9000,0.6000,0.8000,0.8000,1.1000,1.6000,1.6000,1.6000,1.6000,0.9000,0.9000,0.6000,0.8000,0.8000,1.1000,1.6000,1.6000,1.6000,1.6000,0.9000,0.9000,0.6000,0.8000,0.8000,1.1000,
              1.6000,1.6000,1.6000,1.6000,0.9000,0.9000,0.6000,0.8000,0.8000,1.1000,1.6000,1.6000,1.6000,1.6000,0.9000,0.9000,0.6000,0.8000,0.8000,1.1000,1.6000,1.6000,1.6000,1.6000,0.9000,0.9000,0.6000,0.8000,0.8000,1.1000,1.6000,1.6000,1.6000,1.6000,0.9000,0.9000,0.6000,0.8000,0.8000,1.1000,1.6000,1.6000,1.6000,1.6000,0.9000,0.9000,0.6000,0.8000,0.8000,1.1000,1.6000,1.6000,1.6000,1.6000,0.9000,0.9000,0.6000,0.8000,0.8000,1.1000,
              1.6000,1.6000,1.6000,1.6000,0.9000,0.9000,0.6000,0.8000,0.8000,1.1000,1.6000,1.6000,1.6000,1.6000,0.9000,0.9000,0.6000,0.8000,0.8000,1.1000,1.6000,1.6000,1.6000,1.6000,0.9000,0.9000,0.6000,0.8000,0.8000,1.1000,1.6000,1.6000,1.6000,1.6000,0.9000,0.9000,0.6000,0.8000,0.8000,1.1000,1.6000,1.6000,1.6000,1.6000,0.9000,0.9000,0.6000,0.8000,0.8000,1.1000,1.6000,1.6000,1.6000,1.6000,0.9000,0.9000,0.6000,0.8000,0.8000,1.1000,
              0.0000,90.0000,0.0000,90.0000,90.0000,0.0000,0.0000,90.0000,90.0000,0.0000,0.0000,90.0000,0.0000,90.0000,90.0000,0.0000,0.0000,90.0000,90.0000,0.0000,0.0000,90.0000,0.0000,90.0000,90.0000,0.0000,0.0000,90.0000,90.0000,0.0000,0.0000,90.0000,0.0000,90.0000,90.0000,0.0000,0.0000,90.0000,90.0000,0.0000,0.0000,90.0000,0.0000,90.0000,90.0000,0.0000,0.0000,90.0000,90.0000,0.0000,0.0000,90.0000,0.0000,90.0000,90.0000,0.0000,0.0000,90.0000,90.0000,0.0000,
              0.0000,90.0000,0.0000,90.0000,90.0000,0.0000,0.0000,90.0000,90.0000,0.0000,0.0000,90.0000,0.0000,90.0000,90.0000,0.0000,0.0000,90.0000,90.0000,0.0000,0.0000,90.0000,0.0000,90.0000,90.0000,0.0000,0.0000,90.0000,90.0000,0.0000,0.0000,90.0000,0.0000,90.0000,90.0000,0.0000,0.0000,90.0000,90.0000,0.0000,0.0000,90.0000,0.0000,90.0000,90.0000,0.0000,0.0000,90.0000,90.0000,0.0000,0.0000,90.0000,0.0000,90.0000,90.0000,0.0000,0.0000,90.0000,90.0000,0.0000,
              0.0000,90.0000,0.0000,90.0000,90.0000,0.0000,0.0000,90.0000,90.0000,0.0000,0.0000,90.0000,0.0000,90.0000,90.0000,0.0000,0.0000,90.0000,90.0000,0.0000,0.0000,90.0000,0.0000,90.0000,90.0000,0.0000,0.0000,90.0000,90.0000,0.0000,0.0000,90.0000,0.0000,90.0000,90.0000,0.0000,0.0000,90.0000,90.0000,0.0000,1.9000,1.0500,1.9000,2.8500,1.4000,2.2500,2.0000,1.8250,2.1750,2.0000,3.7000,2.8500,3.7000,4.6500,4.3000,3.4500,3.7000,3.5250,3.8750,3.7000,
              1.9000,1.0500,1.9000,2.8500,1.4000,2.2500,2.0000,1.8250,2.1750,2.0000,3.7000,2.8500,3.7000,4.6500,4.3000,3.4500,3.7000,3.5250,3.8750,3.7000,1.9000,1.0500,1.9000,2.8500,1.4000,2.2500,2.0000,1.8250,2.1750,2.0000,3.7000,2.8500,3.7000,4.6500,4.3000,3.4500,3.7000,3.5250,3.8750,3.7000,1.9000,1.0500,1.9000,2.8500,1.4000,2.2500,2.0000,1.8250,2.1750,2.0000,3.7000,2.8500,3.7000,4.6500,4.3000,3.4500,3.7000,3.5250,3.8750,3.7000,
              6.9000,6.0500,6.9000,7.8500,6.4000,7.2500,7.0000,6.8250,7.1750,7.0000,8.7000,7.8500,8.7000,9.6500,9.3000,8.4500,8.7000,8.5250,8.8750,8.7000,6.9000,6.0500,6.9000,7.8500,6.4000,7.2500,7.0000,6.8250,7.1750,7.0000,8.7000,7.8500,8.7000,9.6500,9.3000,8.4500,8.7000,8.5250,8.8750,8.7000,6.9000,6.0500,6.9000,7.8500,6.4000,7.2500,7.0000,6.8250,7.1750,7.0000,8.7000,7.8500,8.7000,9.6500,9.3000,8.4500,8.7000,8.5250,8.8750,8.7000,
              6.9000,6.0500,6.9000,7.8500,6.4000,7.2500,7.0000,6.8250,7.1750,7.0000,8.7000,7.8500,8.7000,9.6500,9.3000,8.4500,8.7000,8.5250,8.8750,8.7000,3.5500,1.9000,1.0500,2.3000,1.9000,1.4000,2.1000,2.1000,2.1000,2.2800,3.5500,2.2500,1.0500,1.9000,1.9000,1.4000,2.1000,2.1000,2.1000,2.2800,6.0500,5.1000,3.5500,4.8000,5.1000,5.7000,4.8800,4.8800,4.8800,4.7600,6.0500,4.7500,3.5500,5.1500,5.1000,5.7000,4.8800,4.8800,4.8800,4.7600,
              8.5500,6.9000,6.0500,7.3000,6.9000,6.4000,7.1000,7.1000,7.1000,7.2800,8.5500,7.2500,6.0500,6.9000,6.9000,6.4000,7.1000,7.1000,7.1000,7.2800,11.0500,10.1000,8.5500,9.8000,10.1000,10.7000,9.8800,9.8800,9.8800,9.7600,11.0500,9.7500,8.5500,10.1500,10.1000,10.7000,9.8800,9.8800,9.8800,9.7600,3.5500,1.9000,1.0500,2.3000,1.9000,1.4000,2.1000,2.1000,2.1000,2.2800,3.5500,2.2500,1.0500,1.9000,1.9000,1.4000,2.1000,2.1000,2.1000,2.2800,
              6.0500,5.1000,3.5500,4.8000,5.1000,5.7000,4.8800,4.8800,4.8800,4.7600,6.0500,4.7500,3.5500,5.1500,5.1000,5.7000,4.8800,4.8800,4.8800,4.7600,8.5500,6.9000,6.0500,7.3000,6.9000,6.4000,7.1000,7.1000,7.1000,7.2800,8.5500,7.2500,6.0500,6.9000,6.9000,6.4000,7.1000,7.1000,7.1000,7.2800,11.0500,10.1000,8.5500,9.8000,10.1000,10.7000,9.8800,9.8800,9.8800,9.7600,11.0500,9.7500,8.5500,10.1500,10.1000,10.7000,9.8800,9.8800,9.8800,9.7600,
              0.0000,0.0000,0.0000,0.0000,0.8500,0.8500,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.8500,0.8500,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.8500,0.8500,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.8500,0.8500,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.8500,0.8500,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.8500,0.8500,0.0000,0.0000,0.0000,0.0000,
              0.0000,0.0000,0.0000,0.0000,0.8500,0.8500,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.8500,0.8500,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.8500,0.8500,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.8500,0.8500,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.8500,0.8500,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.8500,0.8500,0.0000,0.0000,0.0000,0.0000,
              0.0000,0.0000,0.0000,0.0000,0.8500,0.8500,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.8500,0.8500,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.8500,0.8500,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.8500,0.8500,0.0000,0.0000,0.0000,0.0000};
  
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


  depSD = depSD*0.1;
  
  // add self blockages (STA devices)
  std::vector<double> deviceDir = {0, 45.0, 135.0, 80.0};
  std::vector<double> deviceLength = {apDimension.x, MonitorDimension.x, LaptopDimension_sc.x, HardDriveDimension.x};
  std::vector<double> deviceWidth = {apDimension.y, MonitorDimension.y, LaptopDimension_sc.y, HardDriveDimension.y};
  std::vector<double> deviceHeight = {2.9+apDimension.z, 0.95+MonitorDimension.z, 0.95+LaptopDimension_sc.z, 0.95+HardDriveDimension.z};
  std::vector<double> deviceHeight_base = {2.9, 0.95, 0.95, 0.95};

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
  if (FOFC == true)
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

  /* Set default algorithm for all nodes to be constant rate */
  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager", "ControlMode", StringValue (phyMode),
                                                                "DataMode", StringValue (phyMode));
  

  /*  --- Infrastructure mode --- AP-to-STA --- */															
  if ((commID == 2) || (commID == 4)) // infrastructure mode
  	{

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
  if (commID == 2)
  	{
  	  commEntity = {0, 2}; // AP -> STA2
  	  txdeviceDimension = apDimension;
	  trafficMode = LOCAL_FILE;
  	}
  else if (commID == 4)
  	{
  	  commEntity = {0, 2}; // AP -> STA2
	  txdeviceDimension = apDimension;
	  trafficMode = WEB_BROWSING;
  	}
  else // default is commID = 1
  	{
  	  commEntity = {2, 1}; // STA2 -> STA1
  	  txdeviceDimension = LaptopDimension_sc;
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
  labScenarios->SetHumanObstacleNumber(0);

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

		  /// HTTP high MTU prob
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
      	throughput += ((totalPacketsThrough * 8) / ((simulationTime-1 + additionalTime) * 1000000.0)); //Mbit/s
      	re = ((totalPacketsThrough * 8) / ((simulationTime-1 + additionalTime) * 1000000.0));
      	std::cerr << re << " ";
    }
  
  std::cerr << std::endl;
  	}


  /*   ---------- ad-hoc STA-to-STA ---------------- */
  
  else // ad-hoc mode
  {

  /* Set Analytical Codebook for the DMG Devices */
  wifi.SetCodebook ("ns3::CodebookAnalytical",
                    "CodebookType", EnumValue (SIMPLE_CODEBOOK),
                    "Antennas", UintegerValue (1),
                    "Sectors", UintegerValue (8));
																
  /* Make two nodes and set them up with the PHY and the MAC */
  Ptr<Node> serverNode;
  Ptr<Node> apWifiNode;
  Ptr<Node> staWifiNode;

   /* Make three nodes and set them up with the PHY and the MAC */
  NodeContainer wifiNodes;
  wifiNodes.Create (3);
  serverNode = wifiNodes.Get (0);
  apWifiNode = wifiNodes.Get (1);
  staWifiNode = wifiNodes.Get (2);


  /* Create Backbone network */
  PointToPointHelper p2pHelper;
  p2pHelper.SetDeviceAttribute ("DataRate", StringValue ("10Gbps"));
  p2pHelper.SetChannelAttribute ("Delay", TimeValue (NanoSeconds (20)));
  // p2pHelper.SetQueue ("ns3::DropTailQueue", "MaxPackets", UintegerValue (1000));

  NetDeviceContainer serverDevices;
  serverDevices = p2pHelper.Install (serverNode, apWifiNode);

  /* Add a DMG Ad-Hoc MAC */
  DmgWifiMacHelper wifiMac = DmgWifiMacHelper::Default ();
  wifiMac.SetType ("ns3::DmgAdhocWifiMac",
                   "BE_MaxAmpduSize", UintegerValue (mpduAggregationSize), //Enable A-MPDU with the maximum size allowed by the standard.
                   "BE_MaxAmsduSize", UintegerValue (msduAggregationSize));

  NetDeviceContainer apDevice;
  apDevice = wifi.Install (wifiPhy, wifiMac, apWifiNode);

  NetDeviceContainer staDevice;
  staDevice = wifi.Install (wifiPhy, wifiMac, staWifiNode);	  
  
  // define the communication entity ID
  std::vector<uint16_t> commEntity;
  Vector txdeviceDimension;
  if (commID == 1)
  	{
  	  commEntity = {2, 1}; // STA2 -> STA1
      txdeviceDimension = LaptopDimension_sc;
	  trafficMode = COMP_VIDEO;
  	}
  else if (commID == 3)
  	{
  	  commEntity = {2, 3}; // STA2 -> STA3
	  txdeviceDimension = LaptopDimension_sc;
	  trafficMode = LOCAL_FILE;
	  // trafficMode = COMP_VIDEO;
  	}
  else if (commID == 5)
  	{
  	  commEntity = {3, 2}; // STA3 -> STA2
	  txdeviceDimension = HardDriveDimension;
	  trafficMode = HARD_DISK_FILE;
  	}
  else // default is commID = 1
  	{
  	  commEntity = {2, 1}; // STA2 -> STA1
  	  txdeviceDimension = LaptopDimension_sc;
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
  labScenarios->SetHumanObstacleNumber(0);

  /* Setting mobility model, Initial Position 1 meter apart */
  MobilityHelper mobility;
  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
  positionAlloc->Add (serverPos); // Backbone Server.
  
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

  // set up adhoc mode
  bool adhocMode = true;
  wifiChannel->SetAdHocMode(adhocMode);

  for (uint16_t clientId = 0; clientId < clientNo; clientId++)
    {
      std::pair<double, double> fadingInfo = labScenarios->GetFadingInfo(txPos, rxPos.at(clientId));
      losFlag.push_back(fadingInfo.first);
    }

  mobility.SetPositionAllocator (positionAlloc);
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (wifiNodes);
  

  /* Install Internet Stack */
  InternetStackHelper stack;
  stack.Install (wifiNodes);

  Ipv4AddressHelper address;
  address.SetBase ("10.0.0.0", "255.255.255.0");
  Ipv4InterfaceContainer serverInterface;
  serverInterface = address.Assign (serverDevices);
  address.NewNetwork ();
  Ipv4InterfaceContainer apInterface;
  apInterface = address.Assign (apDevice);
  Ipv4InterfaceContainer staInterface;
  staInterface = address.Assign (staDevice);

  /* Populate routing table */
  Ipv4GlobalRoutingHelper::PopulateRoutingTables ();

  /* We do not want any ARP packets */
  PopulateArpCache ();


  // app layer
  ApplicationContainer sourceApplications, sinkApplications;
  // uint32_t portNumber = 9;
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

          std::string IAT_off = std::string("ns3::ConstantRandomVariable[Constant=") + std::to_string(IAT) + std::string("]");
          std::string tx_on = std::string("ns3::NormalRandomVariable[Mean=") + std::to_string(t_mu) + std::string("|Variance=") + std::to_string(t_sigma) + std::string("|Bound=") + std::to_string(tx_on_max) + std::string("]");
		  
          // auto ipv4 = rxWifiNode.Get (index)->GetObject<Ipv4> ();
          // const auto address = ipv4->GetAddress (1, 0).GetLocal ();
          // InetSocketAddress sinkSocket (address, portNumber++);
          
          /* Install TCP/UDP Server on the server side */
          PacketSinkHelper sinkHelper (socketType, InetSocketAddress (Ipv4Address::GetAny (), 9999));
          sinkApplications = sinkHelper.Install (serverNode);
          sink = StaticCast<PacketSink> (sinkApplications.Get (0));

		  /* Install TCP/UDP Transmitter on the DMD AD-HOC Client */
          Address dest (InetSocketAddress (serverInterface.GetAddress (0), 9999));
		  
          // OnOffHelper src ("ns3::UdpSocketFactory", sinkSocket); 
		  OnOffHelper src (socketType, dest);
          src.SetAttribute ("MaxBytes", UintegerValue (0));
          src.SetAttribute ("PacketSize", UintegerValue (payloadSize));
          // src.SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=0.00016384]"));
          src.SetAttribute ("OnTime", StringValue(tx_on));
		  src.SetAttribute ("OffTime", StringValue (IAT_off));
          src.SetAttribute ("DataRate", DataRateValue (DataRate (dataRate)));
		  
		  sourceApplications = src.Install (staWifiNode);
		  
	  	}
	  else if (trafficMode == LOCAL_FILE)
	  	{		  
          socketType = "ns3::TcpSocketFactory";  /* Socket Type (TCP/UDP) */
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
          sinkApplications.Add (sinkHelper.Install (serverNode));

		  Address dest (InetSocketAddress (serverInterface.GetAddress (0), 9999));
		  BulkSendHelper src (socketType, dest);
          sourceApplications= src.Install (staWifiNode);		  
	  	}
	  else if (trafficMode == WEB_BROWSING)
	  	{
	  	  Ipv4Address serverAddress = staInterface.GetAddress (0);
	  	  // Create HTTP server helper
          ThreeGppHttpServerHelper serverHelper (serverAddress);

          // Install HTTP server
          sourceApplications = serverHelper.Install (staWifiNode);
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

		  /// HTTP high MTU prob
		  // httpVariables->SetHighMtuProbability (1.0); // prob for higher MTU
		  Config::SetDefault ("ns3::ThreeGppHttpVariables::HighMtuProbability", DoubleValue (1.0));

		  // HTTP high MTU size
		  // httpVariables->SetHighMtuSize  (1500); // bytes
		  Config::SetDefault ("ns3::ThreeGppHttpVariables::HighMtuSize", UintegerValue (1500));

		  // Create HTTP client helper
          Ipv4Address clientAddress = serverInterface.GetAddress (0);
          ThreeGppHttpClientHelper clientHelper (clientAddress);
		  // ThreeGppHttpClientHelper clientHelper (serverAddress);

          // Install HTTP client
          sinkApplications = clientHelper.Install (serverNode);
          Ptr<ThreeGppHttpClient> httpClient = sinkApplications.Get (0)->GetObject<ThreeGppHttpClient> ();
          
	  	}
	  else if (HARD_DISK_FILE)
	  	{
	  	  // Create the RNG with a non-uniform distribution in sampling mode for transaction data size
		  Ptr<EmpiricalRandomVariable> x_e2 = CreateObject<EmpiricalRandomVariable> ();  
		  x_e2->CDF (4*1024*8.0/(target_rate*1000000), 0.04); 
		  x_e2->CDF (32*1024*8.0/(target_rate*1000000), 0.08); 
		  x_e2->CDF (64*1024*8.0/(target_rate*1000000), 0.4);
		  x_e2->CDF (96*1024*8.0/(target_rate*1000000), 0.43);
		  x_e2->CDF (128*1024*8.0/(target_rate*1000000), 1.0);

		  
	  	  // additional time for READ
          double READtimeInterval = 0.001; // 1ms
		  double READtime = 256 * 8.0 /(target_rate*1000000);
		  
	  	  // RngSeedManager::SetSeed (RNG);
		  // Create the RNG with a non-uniform distribution in sampling mode for inter-arrival time
		  Ptr<EmpiricalRandomVariable> x_e1 = CreateObject<EmpiricalRandomVariable> ();  
          double est_onTime = x_e2->GetValue(); // second

		  x_e1->CDF (max(0.00004 + READtimeInterval + READtime - est_onTime, 0), 0.02); 
		  x_e1->CDF (max(0.0005 + READtimeInterval + READtime - est_onTime, 0), 0.1); 
		  x_e1->CDF (max(0.0006 + READtimeInterval + READtime - est_onTime, 0), 0.16);
		  x_e1->CDF (max(0.0007 + READtimeInterval + READtime - est_onTime, 0), 0.17);
		  x_e1->CDF (max(0.0008 + READtimeInterval + READtime - est_onTime, 0), 0.19);
		  x_e1->CDF (max(0.0009 + READtimeInterval + READtime - est_onTime, 0), 0.21);
		  x_e1->CDF (max(0.0010 + READtimeInterval + READtime - est_onTime, 0), 0.22);
		  x_e1->CDF (max(0.0011 + READtimeInterval + READtime - est_onTime, 0), 0.3);
		  x_e1->CDF (max(0.0015 + READtimeInterval + READtime - est_onTime, 0), 0.31);
		  x_e1->CDF (max(0.0020 + READtimeInterval + READtime - est_onTime, 0), 0.33);
		  x_e1->CDF (max(0.0040 + READtimeInterval + READtime - est_onTime, 0), 0.34);
		  x_e1->CDF (max(0.0050 + READtimeInterval + READtime - est_onTime, 0), 0.35);
		  x_e1->CDF (max(0.0060 + READtimeInterval + READtime - est_onTime, 0), 0.36);
		  x_e1->CDF (max(0.0070 + READtimeInterval + READtime - est_onTime, 0), 0.37);
		  x_e1->CDF (max(0.0080 + READtimeInterval + READtime - est_onTime, 0), 0.38);
		  x_e1->CDF (max(0.0090 + READtimeInterval + READtime - est_onTime, 0), 0.39);
		  x_e1->CDF (max(0.01 + READtimeInterval + READtime - est_onTime, 0), 0.4);
		  x_e1->CDF (max(0.011 + READtimeInterval + READtime - est_onTime, 0), 0.41);
		  x_e1->CDF (max(0.012 + READtimeInterval + READtime - est_onTime, 0), 0.42);
		  x_e1->CDF (max(0.018 + READtimeInterval + READtime - est_onTime, 0), 0.43);
		  x_e1->CDF (max(0.019 + READtimeInterval + READtime - est_onTime, 0), 0.44);
		  x_e1->CDF (max(0.025 + READtimeInterval + READtime - est_onTime, 0), 0.46);
		  x_e1->CDF (max(0.029 + READtimeInterval + READtime - est_onTime, 0), 0.47);
		  x_e1->CDF (max(0.030 + READtimeInterval + READtime - est_onTime, 0), 0.48);
		  x_e1->CDF (max(0.031 + READtimeInterval + READtime - est_onTime, 0), 0.49);
		  x_e1->CDF (max(0.032 + READtimeInterval + READtime - est_onTime, 0), 0.5);
		  x_e1->CDF (max(0.033 + READtimeInterval + READtime - est_onTime, 0), 0.51);
		  x_e1->CDF (max(0.034 + READtimeInterval + READtime - est_onTime, 0), 0.52);
		  x_e1->CDF (max(0.035 + READtimeInterval + READtime - est_onTime, 0), 0.53);
		  x_e1->CDF (max(0.036 + READtimeInterval + READtime - est_onTime, 0), 0.54);
		  x_e1->CDF (max(0.037 + READtimeInterval + READtime - est_onTime, 0), 0.55);
		  x_e1->CDF (max(0.038 + READtimeInterval + READtime - est_onTime, 0), 0.56);
		  x_e1->CDF (max(0.039 + READtimeInterval + READtime - est_onTime, 0), 0.57);
		  x_e1->CDF (max(0.040 + READtimeInterval + READtime - est_onTime, 0), 0.58);
		  x_e1->CDF (max(0.041 + READtimeInterval + READtime - est_onTime, 0), 0.59);
		  x_e1->CDF (max(0.042 + READtimeInterval + READtime - est_onTime, 0), 0.6);
		  x_e1->CDF (max(0.043 + READtimeInterval + READtime - est_onTime, 0), 0.61);
		  x_e1->CDF (max(0.044 + READtimeInterval + READtime - est_onTime, 0), 0.63);
		  x_e1->CDF (max(0.045 + READtimeInterval + READtime - est_onTime, 0), 0.65);
		  x_e1->CDF (max(0.046 + READtimeInterval + READtime - est_onTime, 0), 0.67);
		  x_e1->CDF (max(0.047 + READtimeInterval + READtime - est_onTime, 0), 0.69);
		  x_e1->CDF (max(0.048 + READtimeInterval + READtime - est_onTime, 0), 0.71);
		  x_e1->CDF (max(0.049 + READtimeInterval + READtime - est_onTime, 0), 0.72);
		  x_e1->CDF (max(0.050 + READtimeInterval + READtime - est_onTime, 0), 0.84);
		  x_e1->CDF (max(0.051 + READtimeInterval + READtime - est_onTime, 0), 1.0);
		  
          /* Install Simple TCP/UDP Server on the server side */
          PacketSinkHelper sinkHelper (socketType, InetSocketAddress (Ipv4Address::GetAny (), 9999));
          sinkApplications = sinkHelper.Install (serverNode);
          sink = StaticCast<PacketSink> (sinkApplications.Get (0));

		  /* Install TCP/UDP Transmitter on the DMD Ad-Hoc Client */
          Address dest (InetSocketAddress (serverInterface.GetAddress (0), 9999));
		  
          // OnOffHelper src ("ns3::UdpSocketFactory", sinkSocket); 
		  OnOffHelper src (socketType, dest);
		  
          src.SetAttribute ("MaxBytes", UintegerValue (0));
		  src.SetAttribute ("DataRate", DataRateValue (DataRate (dataRate)));

		  sourceApplications = src.Install (staWifiNode);		  
		  src.GetOffTimeFromEmpRand(staWifiNode, x_e1);
		  
	  	}
	  else // UNCOMP_VIDEO
	  	{
          /* Install Simple TCP/UDP Server on the server side */
          PacketSinkHelper sinkHelper (socketType, InetSocketAddress (Ipv4Address::GetAny (), 9999));
          sinkApplications = sinkHelper.Install (serverNode);
          sink = StaticCast<PacketSink> (sinkApplications.Get (0));

		  /* Install TCP/UDP Transmitter on the DMD AD-HOC Client */
          Address dest (InetSocketAddress (serverInterface.GetAddress (0), 9999));
		  
          // OnOffHelper src ("ns3::UdpSocketFactory", sinkSocket); 
		  OnOffHelper src (socketType, dest);
		
          src.SetAttribute ("MaxBytes", UintegerValue (0));
          src.SetAttribute ("PacketSize", UintegerValue (payloadSize));
          src.SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=1e6]"));
          src.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0]"));
          src.SetAttribute ("DataRate", DataRateValue (DataRate (uncompVideoRate)));
          sourceApplications = src.Install (staWifiNode);
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
      // wifiPhy.EnablePcap ("tgad_conf_Traces_AccessPoint", txDevice, false);
      // wifiPhy.EnablePcap ("tgad_conf_Traces_Station", rxDevice, false);
    }
   

  Simulator::Stop (Seconds (simulationTime + 1));
  Simulator::Run ();
  Simulator::Destroy ();

  /* Print Results Summary */
  std::cerr << "STA" << commEntity.at(0) << " -> " << "STA" << commEntity.at(1) << ": ";
  std::copy(losFlag.begin(), losFlag.end(), std::ostream_iterator<bool>(std::cerr, " "));
  for (unsigned index = 0; index < sinkApplications.GetN (); ++index)
    {
      	uint64_t totalPacketsThrough = StaticCast<PacketSink> (sinkApplications.Get (index))->GetTotalRx ();
      	throughput += ((totalPacketsThrough * 8) / ((simulationTime-1 + additionalTime) * 1000000.0)); //Mbit/s
      	re = ((totalPacketsThrough * 8) / ((simulationTime-1 + additionalTime) * 1000000.0));
      	std::cerr << re << " ";
    }
  
  std::cerr << std::endl;
  }	
  
  }
  
  return 0;
}

