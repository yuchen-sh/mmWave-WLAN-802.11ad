/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2015-2019 IMDEA Networks Institute
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
 * Author: Hany Assasa <hany.assasa@gmail.com>
           Yuchen Liu <yuchen.liu@gmail.com>
 */

#include "ns3/simulator.h"
#include "ns3/log.h"
#include "ns3/pointer.h"
#include "ns3/net-device.h"
#include "ns3/node.h"
#include "ns3/propagation-loss-model.h"
#include "ns3/propagation-delay-model.h"
#include "ns3/mobility-model.h"
#include "dmg-wifi-channel.h"
#include "wifi-utils.h"
#include <fstream>
#include "wifi-ppdu.h"
#include "wifi-psdu.h"
#include <ns3/random-variable-stream.h>
#include <random>
#include <cmath>
#include <vector>
#include "ns3/core-module.h" // for random seed

#define PI 3.14159265

#define min(a,b) ((a) < (b) ? (a) : (b))
#define max(a,b) ((a) > (b) ? (a) : (b))


namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("DmgWifiChannel");

NS_OBJECT_ENSURE_REGISTERED (DmgWifiChannel);

TypeId
DmgWifiChannel::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::DmgWifiChannel")
    .SetParent<Channel> ()
    .SetGroupName ("Wifi")
    .AddConstructor<DmgWifiChannel> ()
    .AddAttribute ("PropagationLossModel", "A pointer to the propagation loss model attached to this channel.",
                   PointerValue (),
                   MakePointerAccessor (&DmgWifiChannel::m_loss),
                   MakePointerChecker<PropagationLossModel> ())
    .AddAttribute ("PropagationDelayModel", "A pointer to the propagation delay model attached to this channel.",
                   PointerValue (),
                   MakePointerAccessor (&DmgWifiChannel::m_delay),
                   MakePointerChecker<PropagationDelayModel> ())
    /* New trace sources for DMG PLCP */
    .AddTraceSource ("PhyActivityTracker",
                     "Trace source for transmitting/receiving PLCP field (PHY Tracker).",
                     MakeTraceSourceAccessor (&DmgWifiChannel::m_phyActivityTrace),
                     "ns3::DmgWifiChannel::PhyActivityTracedCallback")
  ;
  return tid;
}

DmgWifiChannel::DmgWifiChannel ()
  : m_itfFlag (0),
    m_blockage (0),
    m_packetDropper (0),
    m_experimentalMode (false),
    m_SVChannel (false),
    m_TGadChannel (false),
    m_reflectorDenseMode (1),
    m_obsDensity (0),
    // m_itfFlag (0),
    m_adhocMode (false),
    m_seq (false)
{
  NS_LOG_FUNCTION (this);
}

DmgWifiChannel::~DmgWifiChannel ()
{
  NS_LOG_FUNCTION (this);
  m_phyList.clear ();
}

void
DmgWifiChannel::SetPropagationLossModel (const Ptr<PropagationLossModel> loss)
{
  NS_LOG_FUNCTION (this << loss);
  m_loss = loss;
}

void
DmgWifiChannel::SetPropagationDelayModel (const Ptr<PropagationDelayModel> delay)
{
  NS_LOG_FUNCTION (this << delay);
  m_delay = delay;
}

// Yuchen
void 
DmgWifiChannel::SetChannelStatus (bool channelStatus)
{
  m_channelStatus = channelStatus;
}

void 
DmgWifiChannel::SetScenarioModel (Ptr<Obstacle> scenario)
{
  m_scenario = scenario;
}

void 
DmgWifiChannel::SetSVChannelEnabler (bool SVChannel)
{
  m_SVChannel = SVChannel;
}

void 
DmgWifiChannel::SetTGadChannelEnabler (bool TGadChannel)
{
  m_TGadChannel = TGadChannel;
}


void 
DmgWifiChannel::SetSVChannelReflectedMode (int reflectedMode)
{ 
  m_reflectorDenseMode = reflectedMode;
}


void 
DmgWifiChannel::SetObsDensity (double obsDensity)
{
  m_obsDensity = obsDensity;
}

void 
DmgWifiChannel::SetAdHocMode (bool adhocMode)
{
  m_adhocMode = adhocMode;
}

void 
DmgWifiChannel::SetSeqSim (bool seq)
{
  m_seq = seq;
}



void
DmgWifiChannel::AddBlockage (double (*blockage)(), Ptr<DmgWifiPhy> srcWifiPhy, Ptr<DmgWifiPhy> dstWifiPhy)
{
  m_blockage = blockage;
  m_srcWifiPhy = srcWifiPhy;
  m_dstWifiPhy = dstWifiPhy;
}

void
DmgWifiChannel::RemoveBlockage (void)
{
  m_blockage = 0;
  m_srcWifiPhy = 0;
  m_dstWifiPhy = 0;
}

void
DmgWifiChannel::AddPacketDropper (bool (*dropper)(), Ptr<DmgWifiPhy> srcWifiPhy, Ptr<DmgWifiPhy> dstWifiPhy)
{
  m_packetDropper = dropper;
  m_srcWifiPhy = srcWifiPhy;
  m_dstWifiPhy = dstWifiPhy;
}

void
DmgWifiChannel::RemovePacketDropper (void)
{
  m_packetDropper = 0;
  m_srcWifiPhy = 0;
  m_dstWifiPhy = 0;
}

void
DmgWifiChannel::RecordPhyActivity (uint32_t srcID, uint32_t dstID, Time duration, double power,
                                   PLCP_FIELD_TYPE fieldType, ACTIVITY_TYPE activityType) const
{
  m_phyActivityTrace (srcID, dstID, duration, power, fieldType, activityType);
}

void
DmgWifiChannel::LoadReceivedSignalStrengthFile (std::string fileName, Time updateFreuqnecy)
{
  NS_LOG_FUNCTION (this << "Loading Received signal strength file" << fileName);
  std::ifstream file;
  file.open (fileName.c_str (), std::ifstream::in);
  NS_ASSERT_MSG (file.good (), " file not found");
  std::string line;
  double strength;
  while (std::getline (file, line))
    {
      std::stringstream value (line);
      value >> strength;
      m_receivedSignalStrength.push_back (strength);
    }
  file.close ();
  m_currentSignalStrengthIndex = 0;
  m_experimentalMode = true;
  m_updateFrequency = updateFreuqnecy;
  /* Schedule Update event */
  Simulator::Schedule (m_updateFrequency, &DmgWifiChannel::UpdateSignalStrengthValue, this);
}

void
DmgWifiChannel::UpdateSignalStrengthValue (void)
{
  NS_LOG_FUNCTION (this);
  m_currentSignalStrengthIndex++;
  if (m_currentSignalStrengthIndex == m_receivedSignalStrength.size ())
    {
      m_currentSignalStrengthIndex = 0;
    }
  Simulator::Schedule (m_updateFrequency, &DmgWifiChannel::UpdateSignalStrengthValue, this);
}

/*
void 
DmgWifiChannel::SetItfFlag (int16_t itfFlag)
{
  m_itfFlag = itfFlag;
}


int16_t 
DmgWifiChannel::GetItfFlag (void)
{
  return m_itfFlag;
}
*/


/* Saleh-Valenzuela Channel for 60 GHz 802.11ad indoor scenario */
// this version is going to be deprecated
double 
DmgWifiChannel::SVChannelGain_dep(int reflectorDenseMode, Ptr<DmgWifiPhy> sender, Ptr<DmgWifiPhy> receiver, double txPowerDbm, double obsDensity) const
{

  int lambda_K; // cluster density
  int lambda_ray; // density of rays within each cluster
  uint16_t granularity = 10;
  double G_sv = 0.0; // return value
  bool obsRel = true;
 
  // based on the assumption of very narrow beams of the directional antennas
  lambda_K = 3; 
  lambda_ray = 8;
  /*
  if (reflectorDenseMode == 1) // lower density of highly-reflective objects in the room
  	{
  	  lambda_K = 3;
	  lambda_ray = 10;
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
       // RngSeedManager::SetSeed (2); // random seed setting
       Ptr<UniformRandomVariable> clusterNoUV = CreateObject<UniformRandomVariable> ();
       clusterNoUV->SetAttribute ("Min", DoubleValue (0.0));
       clusterNoUV->SetAttribute ("Max", DoubleValue (1.0));
       double probability = clusterNoUV->GetValue ();
	   // std::cerr << "Pro " << probability << std::endl;
	   
       if (probability < poissonProb)
  	    {
  	  	   num_cluster++;
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
  Ptr<MobilityModel> senderMobility = sender->GetMobility ();
  // NS_ASSERT (senderMobility != 0);
  // uint32_t j = 0; /* Phy ID */
  Vector sender_pos = senderMobility->GetPosition ();
  // Ptr<DirectionalAntenna> senderAnt = sender->GetDirectionalAntenna ();
  Ptr<Codebook> senderCodebook = sender->GetCodebook ();
  // Ptr<Directional60GhzAntenna> senderAnt60ghz = CreateObject<Directional60GhzAntenna> ();
  // double rxPowerDbm;
  // Time delay; /* Propagation delay of the signal */
  Ptr<MobilityModel> receiverMobility;
  receiverMobility = receiver->GetMobility ()->GetObject<MobilityModel> ();
  double azimuthTx = CalculateAzimuthAngle (sender_pos, receiverMobility->GetPosition ());
  double azimuthRx = CalculateAzimuthAngle (receiverMobility->GetPosition (), sender_pos);
  // Antenna gain, get from IEEE 802.11ad direction antenna model
  // double ref_dB_bias = 0.0;
  double Gtx_dB = 23.18; // 5.57; // init, 17.59; // 14.58 (32 antenna array); very narrow beam,64 antenna array of AP, based on "Capacity of Multi-Connectivity mmWave Systems with Dynamic Blockage and Directional Antennas"
  double Grx_dB = 0.0; // init, 7.20; // 5.57; // 4 antenna array of client device
  // double Gtx_dB = senderCodebook->GetTxGainDbi (azimuthTx);
  // double Grx_dB = receiver->GetCodebook ()->GetRxGainDbi (azimuthRx);
  double Gtx_dB_ref = senderCodebook->GetTxGainDbi (azimuthTx); // Gtx_dB;  // senderAnt->GetTxGainDbi (azimuthTx) - ref_dB_bias;
  /*
  Gtx_dB_ref = max(Gtx_dB_ref, Gtx_dB);
  if (Gtx_dB_ref < 0)
  	{
  	   Gtx_dB_ref = 0;
    }
  */
  // double Grx_dB_ref = receiver->GetDirectionalAntenna ()->GetRxGainDbi (azimuthRx);
  double Grx_dB_ref = receiver->GetCodebook ()->GetRxGainDbi (azimuthRx);
  // Grx_dB_ref = max(Grx_dB_ref, Grx_dB); // just for test
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
  Vector receiver_pos = receiverMobility->GetPosition ();
  double h2 = receiver_pos.z;

  // seperation distance between tranceiver
  double L = CalculateDistance(sender_pos, receiver_pos);

  // LoS status, 0 -- Line of Sight, 1 -- NLoS
  bool LoSStatus =  m_scenario->GetFadingInfo(sender_pos, receiverMobility->GetPosition ()).first;

  // penetration loss + shadowing variable (only for NLoS case)
  // double X_pen_dB = m_scenario->GetFadingInfo(sender_pos, receiverMobility->GetPosition ()).second;

  // reflection coefficient (determined by reflection density mode)
  double mean_r0, var_r0;
  if (reflectorDenseMode == 1) // lower density of highly-reflective objects in the room
  	{
  	  mean_r0 = 0.35;
	  var_r0 = 0.05;
  	}
  else if (reflectorDenseMode == 2) // medium density of highly-reflective objects in the room
    {
  	  mean_r0 = 0.6; // 0.6;
	  var_r0 = 0.05;
  	}
  else // higher density of highly-reflective objects in the room
    {
  	  mean_r0 = 0.85;
	  var_r0 = 0.05; // 0.05
    }
  //follow the truncated normal distribution
  // RngSeedManager::SetSeed (2);
  Ptr<NormalRandomVariable> reff = CreateObject<NormalRandomVariable> ();
  reff->SetAttribute ("Mean", DoubleValue (mean_r0));
  reff->SetAttribute ("Variance", DoubleValue (var_r0));
  double R0 = reff->GetValue ();
  // in case beyond 0~1, or deviate the mean so far
  double rth = 0.2;
  if (R0 < 0 || R0 > 1 || (R0 -mean_r0) >= rth || (mean_r0 - R0) >= rth)
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
  double coeff = 1.53; // based on the experiment (DYB'21) 3.0/2; // 2/3;
  // RngSeedManager::SetSeed (2);
  Ptr<UniformRandomVariable> sFprob = CreateObject<UniformRandomVariable> ();
  sFprob->SetAttribute ("Min", DoubleValue (0.0));
  sFprob->SetAttribute ("Max", DoubleValue (1.0));
  double rdP = sFprob->GetValue ();
  double rou_b;
  if (obsRel == false)
  	{
  	  strongRefProb = 1.0;
	  coeff = 1.0; // turn off the reflection probability measurement
  	}
  if(rdP <= strongRefProb*coeff)
  	{
  	   rou_b = std::sqrt(Gtx_ref*Grx_ref)*R0;
  	}
  else
  	{
       // RngSeedManager::SetSeed (2);
       Ptr<NormalRandomVariable> reff_loss = CreateObject<NormalRandomVariable> ();
  	   reff_loss->SetAttribute ("Mean", DoubleValue (-10.0)); // Alexander Maltsev's experiment
       reff_loss->SetAttribute ("Variance", DoubleValue (5.0)); // Alexander Maltsev's experiment
       double Rl_dB = reff_loss->GetValue ();
	   if ((Rl_dB > 0) || (Rl_dB < -20.0)) // in case of generating inf values
	   	{
	   	   Rl_dB = -10.0;
	   	}
       double Rl_val = std::pow(10.0,Rl_dB/10);
	   rou_b = std::sqrt(Gtx_ref*Grx_ref)*Rl_val;
  	}
  
  double rou_2 = (rou_a+rou_b*std::sin(phi_r))*(rou_a+rou_b*std::sin(phi_r)) + (rou_b*std::cos(phi_r))*(rou_b*std::cos(phi_r));
  double rou_ref_2 = rou_b*rou_b;
  double rou2; // LoS or strongest reflection gain
  // std::cerr << "rou_LoS " << rou_2 << " rou_NLoS " << rou_ref_2 << std::endl;
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
  double G_min = txPowerDbm - 96.0; // set a min threshold to prevent the MissAck bug (for 11ad Single-carrier PHY)

  for (int i = 0; i < num_cluster; ++i)
  	{
  	  for (int j = 0; j < num_ray; ++j)
  	  	{
  	  	  sum_E = sum_E + E_tap_W[i][j];
  	  	}
  	}
  
  
  G = (lambda_w/(4*PI*L))*(lambda_w/(4*PI*L))*rou2 * sum_E;

  double G_dB;
  if (G <= 0)
  	{
  	  G_dB = -1000.0; // set as a -inf
  	}
  else
  	{
      G_dB = 10.0*std::log10(G);
  	}

  G_sv = max(G_min, G_dB);

  return G_sv;
}

double 
DmgWifiChannel::SVChannelGain(int reflectorDenseMode, Ptr<DmgWifiPhy> sender, Ptr<DmgWifiPhy> receiver, double txPowerDbm, double obsDensity, bool channelStatus) const
{

  int lambda_K; // cluster density
  int lambda_ray; // density of rays within each cluster
  uint16_t granularity = 10;
  double G_sv = 0.0; // return value
  bool obsRel = true;
 
  // based on the assumption of very narrow beams of the directional antennas
  lambda_K = 3; 
  lambda_ray = 8;
  /*
  if (reflectorDenseMode == 1) // lower density of highly-reflective objects in the room
  	{
  	  lambda_K = 3;
	  lambda_ray = 10;
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
       // RngSeedManager::SetSeed (2); // random seed setting
       Ptr<UniformRandomVariable> clusterNoUV = CreateObject<UniformRandomVariable> ();
       clusterNoUV->SetAttribute ("Min", DoubleValue (0.0));
       clusterNoUV->SetAttribute ("Max", DoubleValue (1.0));
       double probability = clusterNoUV->GetValue ();
	   // std::cerr << "Pro " << probability << std::endl;
	   
       if (probability < poissonProb)
  	    {
  	  	   num_cluster++;
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
  Ptr<MobilityModel> senderMobility = sender->GetMobility ();
  // NS_ASSERT (senderMobility != 0);
  // uint32_t j = 0; /* Phy ID */
  Vector sender_pos = senderMobility->GetPosition ();
  // Ptr<DirectionalAntenna> senderAnt = sender->GetDirectionalAntenna ();
  Ptr<Codebook> senderCodebook = sender->GetCodebook ();
  // Ptr<Directional60GhzAntenna> senderAnt60ghz = CreateObject<Directional60GhzAntenna> ();
  // double rxPowerDbm;
  // Time delay; /* Propagation delay of the signal */
  Ptr<MobilityModel> receiverMobility;
  receiverMobility = receiver->GetMobility ()->GetObject<MobilityModel> ();
  double azimuthTx = CalculateAzimuthAngle (sender_pos, receiverMobility->GetPosition ());
  double azimuthRx = CalculateAzimuthAngle (receiverMobility->GetPosition (), sender_pos);
  // Antenna gain, get from IEEE 802.11ad direction antenna model
  // double ref_dB_bias = 0.0;
  double Gtx_dB = 23.18; // 5.57; // init, 17.59; // 14.58 (32 antenna array); very narrow beam,64 antenna array of AP, based on "Capacity of Multi-Connectivity mmWave Systems with Dynamic Blockage and Directional Antennas"
  double Grx_dB = 0.0; // init, 7.20; // 5.57; // 4 antenna array of client device
  // double Gtx_dB = senderCodebook->GetTxGainDbi (azimuthTx);
  // double Grx_dB = receiver->GetCodebook ()->GetRxGainDbi (azimuthRx);
  double Gtx_dB_ref = senderCodebook->GetTxGainDbi (azimuthTx); // Gtx_dB;  // senderAnt->GetTxGainDbi (azimuthTx) - ref_dB_bias;
  /*
  Gtx_dB_ref = max(Gtx_dB_ref, Gtx_dB);
  if (Gtx_dB_ref < 0)
  	{
  	   Gtx_dB_ref = 0;
    }
  */
  // double Grx_dB_ref = receiver->GetDirectionalAntenna ()->GetRxGainDbi (azimuthRx);
  double Grx_dB_ref = receiver->GetCodebook ()->GetRxGainDbi (azimuthRx);
  // Grx_dB_ref = max(Grx_dB_ref, Grx_dB); // just for test
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
  Vector receiver_pos = receiverMobility->GetPosition ();
  double h2 = receiver_pos.z;

  // seperation distance between tranceiver
  double L = CalculateDistance(sender_pos, receiver_pos);

  // LoS status, 0 -- Line of Sight, 1 -- NLoS
  // bool LoSStatus =  m_scenario->GetFadingInfo(sender_pos, receiverMobility->GetPosition ()).first;
  bool LoSStatus = channelStatus;

  // penetration loss + shadowing variable (only for NLoS case)
  // double X_pen_dB = m_scenario->GetFadingInfo(sender_pos, receiverMobility->GetPosition ()).second;

  // reflection coefficient (determined by reflection density mode)
  double mean_r0, var_r0;
  if (reflectorDenseMode == 1) // lower density of highly-reflective objects in the room
  	{
  	  mean_r0 = 0.35;
	  var_r0 = 0.05;
  	}
  else if (reflectorDenseMode == 2) // medium density of highly-reflective objects in the room
    {
  	  mean_r0 = 0.6; // 0.6;
	  var_r0 = 0.05;
  	}
  else // higher density of highly-reflective objects in the room
    {
  	  mean_r0 = 0.85;
	  var_r0 = 0.05; // 0.05
    }
  //follow the truncated normal distribution
  // RngSeedManager::SetSeed (2);
  Ptr<NormalRandomVariable> reff = CreateObject<NormalRandomVariable> ();
  reff->SetAttribute ("Mean", DoubleValue (mean_r0));
  reff->SetAttribute ("Variance", DoubleValue (var_r0));
  double R0 = reff->GetValue ();
  // in case beyond 0~1, or deviate the mean so far
  double rth = 0.2;
  if (R0 < 0 || R0 > 1 || (R0 -mean_r0) >= rth || (mean_r0 - R0) >= rth)
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
  double coeff = 1.53; // based on the experiment (DYB'21) 3.0/2; // 2/3;
  // RngSeedManager::SetSeed (2);
  Ptr<UniformRandomVariable> sFprob = CreateObject<UniformRandomVariable> ();
  sFprob->SetAttribute ("Min", DoubleValue (0.0));
  sFprob->SetAttribute ("Max", DoubleValue (1.0));
  double rdP = sFprob->GetValue ();
  double rou_b;
  if (obsRel == false)
  	{
  	  strongRefProb = 1.0;
	  coeff = 1.0; // turn off the reflection probability measurement
  	}
  if(rdP <= strongRefProb*coeff)
  	{
  	   rou_b = std::sqrt(Gtx_ref*Grx_ref)*R0;
  	}
  else
  	{
       // RngSeedManager::SetSeed (2);
       Ptr<NormalRandomVariable> reff_loss = CreateObject<NormalRandomVariable> ();
  	   reff_loss->SetAttribute ("Mean", DoubleValue (-10.0)); // Alexander Maltsev's experiment
       reff_loss->SetAttribute ("Variance", DoubleValue (5.0)); // Alexander Maltsev's experiment
       double Rl_dB = reff_loss->GetValue ();
	   if ((Rl_dB > 0) || (Rl_dB < -20.0)) // in case of generating inf values
	   	{
	   	   Rl_dB = -10.0;
	   	}
       double Rl_val = std::pow(10.0,Rl_dB/10);
	   rou_b = std::sqrt(Gtx_ref*Grx_ref)*Rl_val;
  	}
  
  double rou_2 = (rou_a+rou_b*std::sin(phi_r))*(rou_a+rou_b*std::sin(phi_r)) + (rou_b*std::cos(phi_r))*(rou_b*std::cos(phi_r));
  double rou_ref_2 = rou_b*rou_b;
  double rou2; // LoS or strongest reflection gain
  // std::cerr << "rou_LoS " << rou_2 << " rou_NLoS " << rou_ref_2 << std::endl;
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
  double G_min = txPowerDbm - 96.0; // set a min threshold to prevent the MissAck bug (for 11ad Single-carrier PHY)

  for (int i = 0; i < num_cluster; ++i)
  	{
  	  for (int j = 0; j < num_ray; ++j)
  	  	{
  	  	  sum_E = sum_E + E_tap_W[i][j];
  	  	}
  	}
  
  
  G = (lambda_w/(4*PI*L))*(lambda_w/(4*PI*L))*rou2 * sum_E;

  double G_dB;
  if (G <= 0)
  	{
  	  G_dB = -1000.0; // set as a -inf
  	}
  else
  	{
      G_dB = 10.0*std::log10(G);
  	}

  G_sv = max(G_min, G_dB);

  return G_sv;
}



void
DmgWifiChannel::Send (Ptr<DmgWifiPhy> sender, Ptr<const WifiPpdu> ppdu, double txPowerDbm) const
{
  NS_LOG_FUNCTION (this << sender << ppdu << txPowerDbm);
  Ptr<MobilityModel> senderMobility = sender->GetMobility ();
  NS_ASSERT (senderMobility != 0);
  for (PhyList::const_iterator i = m_phyList.begin (); i != m_phyList.end (); i++)
    {
      if (sender != (*i))
        {
          //For now don't account for inter channel interference nor channel bonding
          if ((*i)->GetChannelNumber () != sender->GetChannelNumber ())
            {
              continue;
            }

          /* Packet Dropper */
          if ((m_packetDropper != 0) && ((m_srcWifiPhy == sender) && (m_dstWifiPhy == (*i))))
            {
              if (m_packetDropper ())
                {
                  continue;
                }
            }

          Vector sender_pos = senderMobility->GetPosition ();
          Ptr<Codebook> senderCodebook = sender->GetCodebook ();
          Ptr<MobilityModel> receiverMobility = (*i)->GetMobility ()->GetObject<MobilityModel> ();
          Time delay = m_delay->GetDelay (senderMobility, receiverMobility);
          double rxPowerDbm;
          double azimuthTx = CalculateAzimuthAngle (sender_pos, receiverMobility->GetPosition ());
          double azimuthRx = CalculateAzimuthAngle (receiverMobility->GetPosition (), sender_pos);
          double gtx = senderCodebook->GetTxGainDbi (azimuthTx);        // Sender's antenna gain in dBi.
          double grx = (*i)->GetCodebook ()->GetRxGainDbi (azimuthRx);  // Receiver's antenna gain in dBi.

          NS_LOG_DEBUG ("POWER: azimuthTx=" << azimuthTx
                        << ", azimuthRx=" << azimuthRx
                        << ", txPowerDbm=" << txPowerDbm
                        << ", RxPower=" << m_loss->CalcRxPower (txPowerDbm, senderMobility, receiverMobility)
                        << ", Gtx=" << gtx
                        << ", Grx=" << grx);

          if (m_experimentalMode)
            {
              rxPowerDbm = m_receivedSignalStrength[m_currentSignalStrengthIndex];
            }
          else
            {
              if (m_adhocMode == false)
              	{
                     if ((m_SVChannel == false) && (m_TGadChannel == false)) // Jian-Liu Channel (WiMove'21)
                     {
                        // do obstacle and LoS analysis first
                        double fadingLoss = m_scenario->checkLoS(sender_pos, receiverMobility->GetPosition ()).second;
                  	    rxPowerDbm = m_loss->CalcRxPower (txPowerDbm, senderMobility, receiverMobility) +
                                 gtx + grx + fadingLoss;
                        // std::cerr << "path loss is: " << m_loss->CalcRxPower (txPowerDbm, senderMobility, receiverMobility) << std::endl;
						// NS_LOG_DEBUG ("rxPowerDbm" << rxPowerDbm << " fading " << m_scenario->GetFadingInfo(sender_pos, receiverMobility->GetPosition ()).second);
                     }
			         else if ((m_SVChannel == true) && (m_TGadChannel == false)) // S-V 11ad channel
              	     {
                        // do obstacle and LoS analysis first
                        if (m_scenario->GetMultiRoomFlag() == false)
                        {
                          bool channelStatus = m_scenario->checkLoS(sender_pos, receiverMobility->GetPosition ()).first;						  
						  Ptr<DmgWifiPhy> receiver = *i;
				  		  double G_sv_channel = SVChannelGain(m_reflectorDenseMode, sender, receiver, txPowerDbm, m_obsDensity, channelStatus);
				  		  rxPowerDbm = txPowerDbm + G_sv_channel;
				  		  NS_LOG_INFO("rxPowerDbm" << rxPowerDbm << " fading " << G_sv_channel);
						  // std::cerr << "rxPowerDbm" << rxPowerDbm << " fading " << G_sv_channel << std::endl;						  
                        }
						else
						{
						  uint16_t channelStatus_withWall = m_scenario->checkLoS_withWall(sender_pos, receiverMobility->GetPosition ()).first;						  
                          bool channelStatus = m_scenario->checkLoS(sender_pos, receiverMobility->GetPosition ()).first;
						  Ptr<DmgWifiPhy> receiver = *i;
				  		  double G_sv_channel = SVChannelGain(m_reflectorDenseMode, sender, receiver, txPowerDbm, m_obsDensity, channelStatus);
				  		  rxPowerDbm = txPowerDbm + G_sv_channel;
						  if (channelStatus_withWall > 1)
						  	rxPowerDbm = -1000.0; // zero the signal strength if blocked by the wall
				  		  NS_LOG_INFO("rxPowerDbm" << rxPowerDbm << " fading " << G_sv_channel);
						  // std::cerr << "rxPowerDbm" << rxPowerDbm << " fading " << G_sv_channel << std::endl;		
						}
              	     }
                     else if ((m_TGadChannel == true) && (m_SVChannel == false)) // TGad channel
				     {
				  	    // double pathloss_db = m_loss->CalcRxPower (txPowerDbm, senderMobility, receiverMobility) - txPowerDbm;
                        // pathloss_db = max(pathloss_db, -98.0); // based on TGad document
                        // do obstacle and LoS analysis first
                        if (m_scenario->GetMultiRoomFlag() == false)
                        {
                          bool channelStatus = m_scenario->checkLoS(sender_pos, receiverMobility->GetPosition ()).first;
                          // double LoSSign =  m_scenario->GetFadingInfo(sender_pos, receiverMobility->GetPosition ()).first;
						  double LoSSign = 0;
						  if (channelStatus == NON_LINE_OF_SIGHT)
						   {
							 LoSSign = 1.0;
						   }
						  rxPowerDbm = m_loss->CalcRxPower (txPowerDbm, senderMobility, receiverMobility) +                                   // propagation loss
                                       min(14.0, gtx) +                            // Sender's antenna gain.
                                       min(14.0, grx) +                            // receiver's antenna gain
                                       LoSSign*(-1.0)*20.0;                        // 10 dB additional loss for NLoS case
                          // rxPowerDbm = 10.3 - 81.5212;
						  // std::cerr << "path loss is: " << m_loss->CalcRxPower (txPowerDbm, senderMobility, receiverMobility) << std::endl;
						  // std::cerr << "rxPowerDbm is: " << rxPowerDbm << std::endl; 
						  // NS_LOG_DEBUG ("rxPowerDbm" << rxPowerDbm << " fading " << m_scenario->GetFadingInfo(sender_pos, receiverMobility->GetPosition ()).second);
                        }
						else
						{
						  uint16_t channelStatus_withWall = m_scenario->checkLoS_withWall(sender_pos, receiverMobility->GetPosition ()).first;
						  if (channelStatus_withWall == 0) // LoS
						   {
							 rxPowerDbm = m_loss->CalcRxPower (txPowerDbm, senderMobility, receiverMobility) +                                   // propagation loss
                                       min(14.0, gtx) +                            // Sender's antenna gain.
                                       min(14.0, grx);                            // receiver's antenna gain                                       
						   }
						  else if (channelStatus_withWall == 1) // NLoS
						   {
						     rxPowerDbm = m_loss->CalcRxPower (txPowerDbm, senderMobility, receiverMobility) +                                   // propagation loss
                                       min(14.0, gtx) +                            // Sender's antenna gain.
                                       min(14.0, grx) +                            // receiver's antenna gain
                                       (-1.0)*20.0;
						   }
						  else // Wall_NLoS
						   {
						     rxPowerDbm = m_loss->CalcRxPower (txPowerDbm, senderMobility, receiverMobility) +                                   // propagation loss
                                       min(14.0, gtx) +                            // Sender's antenna gain.
                                       min(14.0, grx) +                            // receiver's antenna gain
                                       (-1.0)*1000.0;                              // zero down the signal strength
						   }
						  // std::cerr << "rxPowerDbm: " << rxPowerDbm << std::endl;
                          // rxPowerDbm = 10.3 - 81.5212;
						  // std::cerr << "path loss is: " << m_loss->CalcRxPower (txPowerDbm, senderMobility, receiverMobility) << std::endl;
						  // std::cerr << "rxPowerDbm is: " << rxPowerDbm << std::endl; 
						  // NS_LOG_DEBUG ("rxPowerDbm" << rxPowerDbm << " fading " << m_scenario->GetFadingInfo(sender_pos, receiverMobility->GetPosition ()).second);
						}
				     }
			         else // default log-distance based channel, rarely used for dmg since no LoS/NloS analysis is performed
			  	     {
                        rxPowerDbm = m_loss->CalcRxPower (txPowerDbm, senderMobility, receiverMobility) + gtx + grx;
			  	     }

              	}
			  else
			  	{
			  	     if (m_TGadChannel == true)
			  	   	 {
			  	   	    uint8_t numTxSector = senderCodebook->GetTotalNumberOfTransmitSectors();
					    uint8_t numRxSector = senderCodebook->GetTotalNumberOfReceiveSectors();
					    uint8_t numAntenna = senderCodebook->GetTotalNumberOfAntennas();
			  	   	    gtx = senderCodebook->GetMaxGainDbi(numTxSector, numAntenna); // Sender's antenna gain in dBi.
                        grx = (*i)->GetCodebook ()->GetMaxGainDbi(numRxSector, numAntenna);  // Receiver's antenna gain in dBi.
			  	   	    // double LoSSign =  m_scenario->GetFadingInfo(sender_pos, receiverMobility->GetPosition ()).first;
                        // do obstacle and LoS analysis first
                        bool channelStatus = m_scenario->checkLoS(sender_pos, receiverMobility->GetPosition ()).first;
                        double LoSSign = 0;
						if (channelStatus == NON_LINE_OF_SIGHT)
							{
							  LoSSign = 1.0;
							}
						rxPowerDbm = m_loss->CalcRxPower (txPowerDbm, senderMobility, receiverMobility) +                                   // propagation loss
                                     min(14.0, gtx) +                            // Sender's antenna gain.
                                     min(14.0, grx) +                            // receiver's antenna gain
                                     LoSSign*(-1.0)*10.0;                        // 10 dB additional loss for NLoS case
			  	   	  }
				     else // default log-distance based channel, rarely used
				   	  {
				   	    rxPowerDbm = m_loss->CalcRxPower (txPowerDbm, senderMobility, receiverMobility) + gtx + grx;
				   	  }
			  	}
			  /*
			  // interference analysis, for the use in next interference calculation
			  int16_t itfFlag_t = m_scenario->GetInterferenceInfo(sender_pos, receiverMobility->GetPosition ()).first;
			  if (m_seq == true)
			  	{
			      if (itfFlag_t > 0) // suffer interference effects
			  	  {
				     double distToSrc = m_scenario->GetInterferenceInfo(sender_pos, receiverMobility->GetPosition ()).second;
				     // assume log-distance path loss for interference calculation
				     double itfPowerDbm = txPowerDbm + gtx + grx - (32.4 + 17.3*std::log10(distToSrc) + 20*std::log10(60.48));
				     double itf_w = std::pow(10, itfPowerDbm/10.0) / 1000.0;
				     double rx_w = std::pow(10, rxPowerDbm/10.0) / 1000.0;
				     static const double BOLTZMANN = 1.3803e-23;
                     uint16_t channelWidth = 1760; // MHz
                     //Nt is the power of thermal noise in W
                     double Nt = BOLTZMANN * 290 * channelWidth * 1e6;
                     //receiver noise Floor (W) which accounts for thermal noise and non-idealities of the receiver
                     double noiseFloor = 10 * Nt; // W
                     double rxPower_modf = noiseFloor * rx_w / (noiseFloor + itf_w); // W
                     rxPowerDbm = 10.0 * std::log10(rxPower_modf * 1000.0); // dBm, with interference effects
			  	  }
			      // m_itfFlag = itfFlag_t;
			  	}
			  else
			  	{
			  	  // for common or parallel cases
			  	  m_itfFlag = itfFlag_t;
			  	}
              */
			  
			  // m_srcWifiPhy->SetItfFlag(itfFlag_t);
			  // m_dstWifiPhy->SetItfFlag(itfFlag_t);
			  // this->SetItfFlag(itfFlag_t);
			  
		  }

          /* External Attenuator */
          if (m_blockage &&
              ((m_srcWifiPhy == sender && m_dstWifiPhy == (*i)) ||
               (m_srcWifiPhy == (*i) && m_dstWifiPhy == sender)))
            {
              rxPowerDbm += m_blockage ();
              NS_LOG_DEBUG ("RxPower [dBm] with blockage=" << rxPowerDbm);
            }

          NS_LOG_DEBUG ("propagation: txPower=" << txPowerDbm << "dbm, rxPower=" << rxPowerDbm << "dbm, " <<
                        "distance=" << senderMobility->GetDistanceFrom (receiverMobility) << "m, delay=" << delay);
          Ptr<WifiPpdu> copy = Copy (ppdu);
          Ptr<NetDevice> dstNetDevice = (*i)->GetDevice ();
          uint32_t dstNode;
          if (dstNetDevice == 0)
            {
              dstNode = 0xffffffff;
            }
          else
            {
              dstNode = dstNetDevice->GetNode ()->GetId ();
            }

          Simulator::ScheduleWithContext (dstNode,
                                          delay, &DmgWifiChannel::Receive,
                                          (*i), copy, rxPowerDbm);

          /* PHY Activity Monitor */
          uint32_t srcNode = sender->GetDevice ()->GetNode ()->GetId ();
          if (sender->GetStandard () == WIFI_PHY_STANDARD_80211ad)
            {
              RecordPhyActivity (srcNode, dstNode, ppdu->GetTxDuration (), txPowerDbm + gtx, PLCP_80211AD_PREAMBLE_HDR_DATA, TX_ACTIVITY);
              Simulator::Schedule (delay, &DmgWifiChannel::RecordPhyActivity, this,
                                   srcNode, dstNode, ppdu->GetTxDuration (), rxPowerDbm, PLCP_80211AD_PREAMBLE_HDR_DATA, RX_ACTIVITY);
            }
          else if (sender->GetStandard () == WIFI_PHY_STANDARD_80211ay)
            {
              RecordPhyActivity (srcNode, dstNode, ppdu->GetTxDuration (), txPowerDbm + gtx, PLCP_80211AY_PREAMBLE_HDR_DATA, TX_ACTIVITY);
              Simulator::Schedule (delay, &DmgWifiChannel::RecordPhyActivity, this,
                                   srcNode, dstNode, ppdu->GetTxDuration (), rxPowerDbm, PLCP_80211AY_PREAMBLE_HDR_DATA, RX_ACTIVITY);
            }


        }
    }
}

void
DmgWifiChannel::SendAgcSubfield (Ptr<DmgWifiPhy> sender, double txPowerDbm, WifiTxVector txVector) const
{
  NS_LOG_FUNCTION (this << sender << txPowerDbm << txVector);
  Ptr<MobilityModel> senderMobility = sender->GetMobility ()->GetObject<MobilityModel> ();
  NS_ASSERT (senderMobility != 0);
  Ptr<MobilityModel> receiverMobility;
  uint32_t j = 0; /* Phy ID */
  Time delay; /* Propagation delay of the signal */
  for (PhyList::const_iterator i = m_phyList.begin (); i != m_phyList.end (); i++, j++)
    {
      if (sender != (*i))
        {
          // For now don't account for inter channel interference.
          if ((*i)->GetChannelNumber () != sender->GetChannelNumber ())
            {
              continue;
            }

          receiverMobility = (*i)->GetMobility ()->GetObject<MobilityModel> ();
          delay = m_delay->GetDelay (senderMobility, receiverMobility);
          Ptr<Codebook> senderCodebook = sender->GetCodebook ();
          double azimuthTx = CalculateAzimuthAngle (senderMobility->GetPosition (), receiverMobility->GetPosition ());
          double gtx = senderCodebook->GetTxGainDbi (azimuthTx);

          Ptr<Object> dstNetDevice = m_phyList[j]->GetDevice ();
          uint32_t dstNode;	/* Destination node (Receiver) */
          if (dstNetDevice == 0)
            {
              dstNode = 0xffffffff;
            }
          else
            {
              dstNode = dstNetDevice->GetObject<NetDevice> ()->GetNode ()->GetId ();
            }

          /* PHY Activity Monitor */
          RecordPhyActivity (sender->GetDevice ()->GetNode ()->GetId (), dstNode,
                             AGC_SF_DURATION, txPowerDbm + gtx, PLCP_80211AD_AGC_SF, TX_ACTIVITY);
          Simulator::ScheduleWithContext (dstNode, delay, &DmgWifiChannel::ReceiveAgcSubfield, this, j,
                                          sender, txVector, txPowerDbm, gtx);
        }
    }
}

void
DmgWifiChannel::SendTrnCeSubfield (Ptr<DmgWifiPhy> sender, double txPowerDbm, WifiTxVector txVector) const
{
  NS_LOG_FUNCTION (this << sender << txPowerDbm << txVector);
  Ptr<MobilityModel> senderMobility = sender->GetMobility ()->GetObject<MobilityModel> ();
  NS_ASSERT (senderMobility != 0);
  Ptr<MobilityModel> receiverMobility;
  uint32_t j = 0; /* Phy ID */
  Time delay; /* Propagation delay of the signal */
  for (PhyList::const_iterator i = m_phyList.begin (); i != m_phyList.end (); i++, j++)
    {
      if (sender != (*i))
        {
          // For now don't account for inter channel interference.
          if ((*i)->GetChannelNumber () != sender->GetChannelNumber ())
            {
              continue;
            }

          receiverMobility = (*i)->GetMobility ()->GetObject<MobilityModel> ();
          delay = m_delay->GetDelay (senderMobility, receiverMobility);
          Ptr<Codebook> senderCodebook = sender->GetCodebook ();
          double azimuthTx = CalculateAzimuthAngle (senderMobility->GetPosition (), receiverMobility->GetPosition ());
          double gtx = senderCodebook->GetTxGainDbi (azimuthTx);

          Ptr<Object> dstNetDevice = m_phyList[j]->GetDevice ();
          uint32_t dstNode;	/* Destination node (Receiver) */
          if (dstNetDevice == 0)
            {
              dstNode = 0xffffffff;
            }
          else
            {
              dstNode = dstNetDevice->GetObject<NetDevice> ()->GetNode ()->GetId ();
            }

          /* PHY Activity Monitor */
          RecordPhyActivity (sender->GetDevice ()->GetNode ()->GetId (), dstNode,
                             TRN_CE_DURATION, txPowerDbm + gtx, PLCP_80211AD_TRN_CE_SF, TX_ACTIVITY);
          Simulator::ScheduleWithContext (dstNode, delay, &DmgWifiChannel::ReceiveTrnCeSubfield, this, j,
                                          sender, txVector, txPowerDbm, gtx);
        }
    }
}

void
DmgWifiChannel::SendTrnSubfield (Ptr<DmgWifiPhy> sender, double txPowerDbm, WifiTxVector txVector) const
{
  NS_LOG_FUNCTION (this << sender << txPowerDbm << txVector);
  Ptr<MobilityModel> senderMobility = sender->GetMobility ()->GetObject<MobilityModel> ();
  NS_ASSERT (senderMobility != 0);
  Ptr<MobilityModel> receiverMobility;
  uint32_t j = 0; /* Phy ID */
  Time delay; /* Propagation delay of the signal */
  for (PhyList::const_iterator i = m_phyList.begin (); i != m_phyList.end (); i++, j++)
    {
      if (sender != (*i))
        {
          // For now don't account for inter-channel interference.
          if ((*i)->GetChannelNumber () != sender->GetChannelNumber ())
            {
              continue;
            }

          receiverMobility = (*i)->GetMobility ()->GetObject<MobilityModel> ();
          delay = m_delay->GetDelay (senderMobility, receiverMobility);
          Ptr<Codebook> senderCodebook = sender->GetCodebook ();
          double azimuthTx = CalculateAzimuthAngle (senderMobility->GetPosition (), receiverMobility->GetPosition ());
          double gtx = senderCodebook->GetTxGainDbi (azimuthTx);

          Ptr<Object> dstNetDevice = m_phyList[j]->GetDevice ();
          uint32_t dstNode;	/* Destination node (Receiver) */
          if (dstNetDevice == 0)
            {
              dstNode = 0xffffffff;
            }
          else
            {
              dstNode = dstNetDevice->GetObject<NetDevice> ()->GetNode ()->GetId ();
            }

          /* PHY Activity Monitor */
          if (sender->GetStandard () == WIFI_PHY_STANDARD_80211ad)
            {
              RecordPhyActivity (sender->GetDevice ()->GetNode ()->GetId (), dstNode,
                                 TRN_SUBFIELD_DURATION, txPowerDbm + gtx, PLCP_80211AD_TRN_SF, TX_ACTIVITY);
            }
          else
            {
              RecordPhyActivity (sender->GetDevice ()->GetNode ()->GetId (), dstNode,
                                 txVector.edmgTrnSubfieldDuration, txPowerDbm + gtx, PLCP_80211AY_TRN_SF, TX_ACTIVITY);
            }

          Simulator::ScheduleWithContext (dstNode, delay, &DmgWifiChannel::ReceiveTrnSubfield, this, j,
                                          sender, txVector, txPowerDbm, gtx);
        }
    }
}

void
DmgWifiChannel::Receive (Ptr<DmgWifiPhy> phy, Ptr<WifiPpdu> ppdu, double rxPowerDbm)
{
  NS_LOG_FUNCTION (phy << ppdu << rxPowerDbm);
  // Do no further processing if signal is too weak
  // Current implementation assumes constant RX power over the PPDU duration
  if ((rxPowerDbm + phy->GetRxGain ()) < phy->GetRxSensitivity ())
    {
      NS_LOG_INFO ("Received signal too weak to process: " << rxPowerDbm << " dBm");
      return;
    }
  std::vector<double> rxPowerW;
  rxPowerW.push_back (DbmToW (rxPowerDbm + phy->GetRxGain ()));
  phy->StartReceivePreamble (ppdu, rxPowerW);
}

double
DmgWifiChannel::ReceiveSubfield (uint32_t i, Ptr<DmgWifiPhy> sender, WifiTxVector txVector,
                                 double txPowerDbm, double txAntennaGainDbi,
                                 Time duration, PLCP_FIELD_TYPE type) const
{
  NS_LOG_FUNCTION (this << i << sender << txVector << txPowerDbm << txAntennaGainDbi);
  /* Calculate SNR upon the receiption of the TRN Field */
  Ptr<MobilityModel> senderMobility = sender->GetMobility ()->GetObject<MobilityModel> ();
  Ptr<MobilityModel> receiverMobility = m_phyList[i]->GetMobility ()->GetObject<MobilityModel> ();
  NS_ASSERT ((senderMobility != 0) && (receiverMobility != 0));
  double azimuthRx = CalculateAzimuthAngle (receiverMobility->GetPosition (), senderMobility->GetPosition ());
  double rxPowerDbm;

  NS_LOG_DEBUG ("POWER: Gtx=" << txAntennaGainDbi
                << ", Grx=" << m_phyList[i]->GetCodebook ()->GetRxGainDbi (azimuthRx));

  rxPowerDbm = m_loss->CalcRxPower (txPowerDbm, senderMobility, receiverMobility) +
               txAntennaGainDbi +                                           // Sender's antenna gain.
               m_phyList[i]->GetCodebook ()->GetRxGainDbi (azimuthRx);      // Receiver's antenna gain.

  /* PHY Activity Monitor */
  RecordPhyActivity (sender->GetDevice ()->GetNode ()->GetId (),
                     m_phyList[i]->GetDevice ()->GetNode ()->GetId (),
                     duration, rxPowerDbm, type, RX_ACTIVITY);

  /* External Attenuator */
  if ((m_blockage != 0) && (m_srcWifiPhy == sender) && (m_dstWifiPhy == m_phyList[i]))
    {
      rxPowerDbm += m_blockage ();
    }

  NS_LOG_DEBUG ("propagation: txPower=" << txPowerDbm << "dbm, rxPower=" << rxPowerDbm << "dbm");

  return rxPowerDbm;
}

void
DmgWifiChannel::ReceiveAgcSubfield (uint32_t i, Ptr<DmgWifiPhy> sender, WifiTxVector txVector,
                                    double txPowerDbm, double txAntennaGainDbi) const
{
  NS_LOG_FUNCTION (this << i << sender << txVector << txPowerDbm << txAntennaGainDbi);
  double rxPowerDbm = ReceiveSubfield (i, sender, txVector, txPowerDbm, txAntennaGainDbi,
                                       AGC_SF_DURATION, PLCP_80211AD_AGC_SF);
  m_phyList[i]->StartReceiveAgcSubfield (txVector, rxPowerDbm);
}

void
DmgWifiChannel::ReceiveTrnCeSubfield (uint32_t i, Ptr<DmgWifiPhy> sender, WifiTxVector txVector,
                                      double txPowerDbm, double txAntennaGainDbi) const
{
  NS_LOG_FUNCTION (this << i << sender << txVector << txPowerDbm << txAntennaGainDbi);
  double rxPowerDbm = ReceiveSubfield (i, sender, txVector, txPowerDbm, txAntennaGainDbi,
                                       TRN_CE_DURATION, PLCP_80211AD_TRN_CE_SF);
  m_phyList[i]->StartReceiveCeSubfield (txVector, rxPowerDbm);
}

void
DmgWifiChannel::ReceiveTrnSubfield (uint32_t i, Ptr<DmgWifiPhy> sender, WifiTxVector txVector,
                                    double txPowerDbm, double txAntennaGainDbi) const
{
  NS_LOG_FUNCTION (this << i << sender << txVector << txPowerDbm << txAntennaGainDbi);
  double rxPowerDbm;
  if (sender->GetStandard () == WIFI_PHY_STANDARD_80211ad)
    {
      rxPowerDbm = ReceiveSubfield (i, sender, txVector, txPowerDbm, txAntennaGainDbi,
                                       TRN_SUBFIELD_DURATION, PLCP_80211AD_TRN_SF);
    }
  else
    {
      rxPowerDbm = ReceiveSubfield (i, sender, txVector, txPowerDbm, txAntennaGainDbi,
                            txVector.edmgTrnSubfieldDuration, PLCP_80211AY_TRN_SF);
    }
  /* Report the received SNR to the higher layers. */
  if ( sender->GetStandard () == WIFI_PHY_STANDARD_80211ad)
    {
      m_phyList[i]->StartReceiveTrnSubfield (txVector, rxPowerDbm);
    }
  else
    {
      m_phyList[i]->StartReceiveEdmgTrnSubfield (txVector,rxPowerDbm);
    }
}

std::size_t
DmgWifiChannel::GetNDevices (void) const
{
  return m_phyList.size ();
}

Ptr<NetDevice>
DmgWifiChannel::GetDevice (std::size_t i) const
{
  return m_phyList[i]->GetDevice ()->GetObject<NetDevice> ();
}

void
DmgWifiChannel::Add (Ptr<DmgWifiPhy> phy)
{
  NS_LOG_FUNCTION (this << phy);
  m_phyList.push_back (phy);
}

int64_t
DmgWifiChannel::AssignStreams (int64_t stream)
{
  NS_LOG_FUNCTION (this << stream);
  int64_t currentStream = stream;
  currentStream += m_loss->AssignStreams (stream);
  return (currentStream - stream);
}

} //namespace ns3
