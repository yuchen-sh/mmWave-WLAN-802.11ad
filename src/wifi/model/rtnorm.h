/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2018 YUbing
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

#ifndef RTNORM_H
#define RTNORM_H

#include <vector>
#include <gsl/gsl_rng.h>
#include <ns3/object.h>

namespace ns3 {
#ifdef HAVE_GSL
class TruncatedNormalDistribution : public Object
{
public:
  TruncatedNormalDistribution ();
  virtual ~TruncatedNormalDistribution ();

  // Compute y_l from y_k
  double yl (int k);
  // Rejection algorithm with a truncated exponential proposal
  double rtexp(gsl_rng *gen, double a, double b);
  // Pseudorandom numbers from a truncated Gaussian distribution
  // The Gaussian has parameters mu (default 0) and sigma (default 1)
  // and is truncated on the interval [a,b].
  // Returns the random variable x and its probability p(x).
  std::pair<double, double> rtnorm (gsl_rng *gen, double a, double b, const double mu=0, const double sigma=1);

};

#endif
} //namespace ns3

#endif /* RTNORM_H */
