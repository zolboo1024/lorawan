/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2017 University of Padova
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
 * Author: Davide Magrin <magrinda@dei.unipd.it>
 *         Zolboo Erdenebaatar <erdenebz@dickinson.edu>
 */

#ifndef SECURE_NETWORK_CONTROLLER_H
#define SECURE_NETWORK_CONTROLLER_H

#include "ns3/object.h"
#include "ns3/packet.h"
#include "ns3/secure-network-status.h"
#include "ns3/secure-network-controller-components.h"

namespace ns3 {
namespace lorawan {

class SecureNetworkStatus;
class SecureNetworkControllerComponent;

/**
 * This class collects a series of components that deal with various aspects
 * of managing the network, and queries them for action when a new packet is
 * received or other events occur in the network.
 */
class SecureNetworkController : public Object
{
public:
  static TypeId GetTypeId (void);

  SecureNetworkController ();
  SecureNetworkController (Ptr<SecureNetworkStatus> networkStatus);
  virtual ~SecureNetworkController ();

  /**
   * Add a new SecureNetworkControllerComponent
   */
  void Install (Ptr<SecureNetworkControllerComponent> component);

  /**
   * Method that is called by the NetworkServer when a new packet is received.
   *
   * \param packet The newly received packet.
   */
  void OnNewPacket (Ptr<Packet const> packet);

  /**
   * Method that is called by the NetworkScheduler just before sending a reply
   * to a certain End Device.
   */
  void BeforeSendingReply (Ptr<SecureEndDeviceStatus> endDeviceStatus);

private:
  Ptr<SecureNetworkStatus> m_status;
  std::list<Ptr<SecureNetworkControllerComponent> > m_components;
};

} /* namespace ns3 */

}
#endif /* SECURE_NETWORK_CONTROLLER_H */
