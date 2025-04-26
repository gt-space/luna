#!/bin/bash

# Check if at least one argument is passed
if [ "$#" -eq 0 ]; then
    echo "Usage: $0 <2-digit ID1> <2-digit ID2> ..."
    exit 1
fi

# Initial positions for terminal windows
x_offset=0
y_offset=0
window_width=800
window_height=600

# Loop through all passed arguments
for arg in "$@"; do
    # Ensure the argument is a 2-digit number
    if [[ ! "$arg" =~ ^[0-9][0-9]$ ]]; then
        echo "Error: $arg is not a valid 2-digit number."
        continue
    fi

    # Generate the ID (e.g., SAM01 for 01)
    id="SAM$arg"

    # Prepare the command (e.g., for SAM01 it will be the SSH command)
    command="ssh debian@sam-$arg.local"

    # Open a new terminal with the prepared command and title set to $id
    gnome-terminal --title="$id" -- bash -ic "echo '$id'; echo 'Command: $command'; exec bash" &

    # Give extra time for the terminal to open and initialize (5 seconds)
    sleep 5  # Increased sleep time to ensure the terminal is fully ready

    # Get the window ID of the most recently opened terminal
    window_id=$(wmctrl -l | grep "$id" | tail -n 1 | awk '{print $1}')

    # Check if we found the window ID
    if [ -n "$window_id" ]; then
        # Position the terminal window using wmctrl
        wmctrl -i -r "$window_id" -e "0,$x_offset,$y_offset,$window_width,$window_height"
    else
        echo "Error: Could not find window for $id"
    fi

    # Update offsets for the next terminal (to avoid overlap)
    x_offset=$((x_offset + window_width + 20))  # Move horizontally by the window width + 20 pixels
    if [ $x_offset -ge 1920 ]; then  # If we exceed screen width (1920 pixels for example)
        x_offset=0
        y_offset=$((y_offset + window_height + 20))  # Move to the next row
    fi
done