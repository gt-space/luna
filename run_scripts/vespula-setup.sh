#!/bin/bash

# Open terminal 1 and run a command
gnome-terminal -- bash -c "echo 'servo'; cd luna/servo; cargo run --release -- serve; exec bash" 

# Check if at least one argument is passed
if [ "$#" -eq 0 ]; then
    echo "Usage: $0 <2-digit ID1> <2-digit ID2> <bms> <flight> ..."
    exit 1
fi

# Loop through all passed arguments
for arg in "$@"; do
    # Open a new terminal and run the command
    gnome-terminal -- bash -c "
    echo 'Argument: $arg'
    if [[ $arg =~ ^[0-9]{2}$ ]]; then
        # If the argument is two digits
        echo 'SSHing into debian@sam$arg.local'
        read -p 'Press Enter to run the SSH command...'
        ssh debian@SAM$arg.local
    elif [[ $arg == 'flight' ]]; then
        # If the argument is 'flight'
        echo 'SSHing into ubuntu.flight.local'
        read -p 'Press Enter to run the SSH command...'
        ssh ubuntu.flight.local
    else
        # For any other argument
        echo 'SSHing into debian@$arg.local'
        read -p 'Press Enter to run the SSH command...'
        ssh debian@$arg.local
    fi
    exec bash"
done
