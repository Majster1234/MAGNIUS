"""Wybór konkretnego adresu IPv4 fizycznego interfejsu LAN."""

from ipaddress import IPv4Address, IPv4Network
import socket

import psutil


def lan_address(interface, subnet, addresses=None):
    network = IPv4Network(subnet)
    interfaces = psutil.net_if_addrs() if addresses is None else addresses
    candidates = [item.address for item in interfaces.get(interface, [])
                  if item.family == socket.AF_INET and IPv4Address(item.address) in network]
    if len(candidates) != 1 or IPv4Address(candidates[0]).is_loopback:
        raise RuntimeError(
            f'Interfejs {interface} musi mieć dokładnie jeden adres IPv4 w {subnet}; '
            'serwer nie zostanie otwarty na innych interfejsach.')
    return candidates[0]
