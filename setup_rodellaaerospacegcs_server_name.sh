#!/bin/bash

# setup_rodellaaerospacegcs_server_name.sh
# This script installs Avahi and configures the system hostname
# for the Rodella Aerospace GCS application

# Text colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# Must run as root
if [ "$(id -u)" -ne 0 ]; then
    echo -e "${RED}This script must be run as root! Try:${NC}"
    echo -e "${YELLOW}sudo ./setup_rodellaaerospacegcs_minimal.sh${NC}"
    exit 1
fi

# Configuration
HOSTNAME="rodellaaerospacegcs"

echo -e "${YELLOW}=========================================================${NC}"
echo -e "${YELLOW}       Rodella Aerospace GCS Basic Setup Script          ${NC}"
echo -e "${YELLOW}=========================================================${NC}"

# Step 1: Update and install dependencies
echo -e "\n${YELLOW}[1/3] Updating package lists and installing Avahi...${NC}"
apt-get update
apt-get install -y avahi-daemon

if [ $? -ne 0 ]; then
    echo -e "${RED}Failed to install Avahi daemon. Please check your internet connection and try again.${NC}"
    exit 1
fi

# Enable and start Avahi
systemctl enable avahi-daemon
systemctl start avahi-daemon

echo -e "${GREEN}✓ Avahi installed and enabled successfully${NC}"

# Step 2: Set hostname
echo -e "\n${YELLOW}[2/3] Setting hostname to $HOSTNAME...${NC}"
hostnamectl set-hostname $HOSTNAME

# Check if it worked
if [ "$(hostname)" != "$HOSTNAME" ]; then
    echo -e "${RED}Failed to set hostname with hostnamectl. Trying alternative method...${NC}"
    echo "$HOSTNAME" > /etc/hostname
    hostname $HOSTNAME
fi

echo -e "${GREEN}✓ Hostname set to $HOSTNAME${NC}"

# Step 3: Update hosts file
echo -e "\n${YELLOW}[3/3] Updating hosts file...${NC}"
# Check if entry already exists
if grep -q "127.0.1.1.*$HOSTNAME" /etc/hosts; then
    # Update existing entry
    sed -i "s/127.0.1.1.*/127.0.1.1\t$HOSTNAME/g" /etc/hosts
else
    # Add new entry
    echo -e "127.0.1.1\t$HOSTNAME" >> /etc/hosts
fi

# Restart Avahi to apply changes
systemctl restart avahi-daemon

echo -e "${GREEN}✓ Hosts file updated${NC}"

# Final verification
echo -e "\n${YELLOW}Verifying configuration...${NC}"
echo -e "Hostname: $(hostname)"
echo -e "Avahi status: $(systemctl is-active avahi-daemon)"

echo -e "\n${GREEN}=====================================================${NC}"
echo -e "${GREEN}    Basic Hostname Configuration Complete!            ${NC}"
echo -e "${GREEN}=====================================================${NC}"
echo -e "${YELLOW}Your server will be accessible at:${NC}"
echo -e "${GREEN}http://$HOSTNAME.local:5000${NC}"
echo -e "\n${YELLOW}Remember to run your server with:${NC}"
echo -e "${GREEN}gunicorn --worker-class eventlet --workers=3 --bind 0.0.0.0:5000 functionality_added_v8:app${NC}"
echo -e "${GREEN}=====================================================${NC}"
