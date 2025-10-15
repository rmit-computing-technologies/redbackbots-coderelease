#!/usr/bin/python3
"""
Simple script to set wireless profile of a robot
"""

import sys
import os
import argparse
import filecmp
import platform
import configparser

PROFILES = "/home/nao/Profiles"
CONFIGS = "/home/nao/config/Robots"

# Main entry point
if __name__ == "__main__":
    # Process args
    parser = argparse.ArgumentParser(description='Set Wireless Profile')
    parser.add_argument('-p', dest="profile", type=str, help="Choose wireless profile to set")
    parser.add_argument('-ip', dest="ip", type=str, help="Manually set IP address of robot")

    args = parser.parse_args()

    if args.ip:
        print("Setting IP address is not supported in this version of the script.")
        sys.exit(1)

    if len(sys.argv) == 1:
        parser.print_help()
        sys.exit()

    if not os.path.exists(f"{PROFILES}/{args.profile}"):
        print(f"Profile {args.profile} does not exist")
        parser.print_help()
        sys.exit()

    hostname = platform.node()
    config_parser = configparser.ConfigParser()
    with open(f"{CONFIGS}/{hostname}/network.cfg", "r", encoding="UTF-8") as file:
        config_parser.read_string(f"[root]\n{file.read()}")

    # TODO: make this less gross
    lan_ip = config_parser["root"].get("lan").replace('"','').replace(";","")
    wlan_ip = config_parser["root"].get("wlan").replace('"','').replace(";","")

    print(f"Setting the following ips: {lan_ip=} {wlan_ip=}")

    with open(f"{PROFILES}/{args.profile}", 'r', encoding="UTF-8") as file:
        profile = file.read()

    with open(f"{PROFILES}/default.yaml.base", 'r', encoding="UTF-8") as file:
        base = file.read().replace("WIRED_IP_PLACEHOLDER", lan_ip)

    with open(f"{PROFILES}/default.yaml.wifi", 'r', encoding="UTF-8") as file:
        wifi = file.read().replace("WIRELESS_IP_PLACEHOLDER", wlan_ip)

    with open("/etc/netplan/default.yaml.staging", 'w', encoding="UTF-8") as file:
        file.write(base)
        file.write(wifi)
        file.write(profile)

    if filecmp.cmp("/etc/netplan/default.yaml.staging", "/etc/netplan/default.yaml"):
        print("No changes to netplan configuration, skipping apply.")
    else:
        os.rename("/etc/netplan/default.yaml.staging", "/etc/netplan/default.yaml")
        os.system("netplan apply")
        print("Netplan configuration updated and applied.")
