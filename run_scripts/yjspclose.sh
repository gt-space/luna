#!/bin/bash

# Step 1: Close all open local terminal windows (gnome-terminal or any terminal emulator)
echo "Closing all local terminal windows..."
wmctrl -l | grep "gnome-terminal" | awk '{print $1}' | xargs -I {} wmctrl -i -c {}

# Alternatively, use pkill to kill all terminal processes by name
# pkill gnome-terminal  # Uncomment to use pkill instead

# Step 2: Close any SSH sessions
echo "Killing any SSH sessions..."
# We use pkill to terminate ssh processes.
pkill -f "ssh"

# Step 3: Kill all processes with the name 'servo'
echo "Killing all processes with the name 'servo'..."
pkill -f "servo"

# Step 4: Kill all processes with the name 'flight'
echo "Killing all processes with the name 'flight'..."
pkill -f "flight"

echo "All local terminals closed, SSH sessions killed, and 'servo' processes terminated."
