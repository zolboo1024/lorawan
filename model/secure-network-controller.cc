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

#include "secure-network-controller.h"

namespace ns3 {
namespace lorawan {

NS_LOG_COMPONENT_DEFINE ("SecureNetworkController");

NS_OBJECT_ENSURE_REGISTERED (SecureNetworkController);

TypeId
SecureNetworkController::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::SecureNetworkController")
    .AddConstructor<SecureNetworkController> ()
    .SetGroupName ("lorawan");
  return tid;
}

SecureNetworkController::SecureNetworkController ()
{
  NS_LOG_FUNCTION_NOARGS ();
}

SecureNetworkController::SecureNetworkController (Ptr<SecureNetworkStatus> networkStatus) :
  m_status (networkStatus)
{
  NS_LOG_FUNCTION_NOARGS ();
}

SecureNetworkController::~SecureNetworkController ()
{
  NS_LOG_FUNCTION_NOARGS ();
}

void
SecureNetworkController::Install (Ptr<SecureNetworkControllerComponent> component)
{
  NS_LOG_FUNCTION (this);
  m_components.push_back (component);
}

void
SecureNetworkController::OnNewPacket (Ptr<Packet const> packet)
{
  NS_LOG_FUNCTION (this << packet);

  // NOTE As a future optimization, we can allow components to register their
  // callbacks and only be called in case a certain MAC command is contained.
  // For now, we call all components.

  // Inform each component about the new packet
  for (auto it = m_components.begin (); it != m_components.end (); ++it)
    {
      (*it)->OnReceivedPacket (packet,
                               m_status->GetEndDeviceStatus (packet),
                               m_status);
    }
}

void
SecureNetworkController::BeforeSendingReply (Ptr<SecureEndDeviceStatus> endDeviceStatus)
{
  NS_LOG_FUNCTION (this);

  // Inform each component about the imminent reply
  for (auto it = m_components.begin (); it != m_components.end (); ++it)
    {
      (*it)->BeforeSendingReply (endDeviceStatus, m_status);
    }
}

}
}
