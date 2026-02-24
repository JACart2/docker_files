#!/bin/bash

# Script to create users from a file with admin perms
# Usage: sudo ./bulk-create-users.sh <usernames_file>

if [[ $EUID -ne 0 ]]; then
	echo "This script must be run as root (use sudo)"
	exit 1
fi

# check arguments
if [ "$#" -ne 1 ]; then
	echo "Usage: sudo $0 <usernames_file>"
	echo "Example: sudo $0 usernames.txt"
fi

USERNAMES_FILE="$1"

if [ ! -f "$USERNAMES_FILE" ]; then
	echo "Error: File '$USERNAMES_FILE' not found"
	exit 1
fi

while IFS= read -r username || [ -n "$username" ]; do
	# skip empty lines and comments
	[[ -z "$username" || "$username" =~ ^[[:space:]]*# ]] && continue

	# remove leading/trailing whitespace
	username=$(echo "$username" | xargs)
	echo "Processing user: $username"

	# check if user already exists
	if id "$username" &>/dev/null; then
		echo "Warning: User '$username' already exists. Skipping..."
		continue
	fi

	# create user with home directory, bash shell, and force password change on first login
	useradd -m -s /bin/bash "$username"

	if [ $? -ne 0 ]; then
		echo "Error: failed to create user: '$username'"
		continue
	fi

	# Add user to sudo group for admin perms
	usermod -aG sudo "$username"

	# Add user to netops group for ip perms
	usermod -aG netops "$username"

	# Add user to docker group
	usermod -aG docker "$username"

	echo "User '$username' successfully created. They will be prompted for password on first login"

	# Get User's home directory for home setup
	USER_HOME=$(eval echo ~$username)

	# Run the setup commands as the user
	echo "Running setup commands for user: $username"

	# Execute commands as the user
	su - "$username" << EOF
mkdir -p ~/dev_ws/src
cd ~/dev_ws/src
git clone https://github.com/JACart2/ai-navigation.git
cd ~/
git clone https://github.com/JACart2/docker_files.git
EOF

	# Remove password and force user to set it on first login
	passwd -d "$username" # Delete/remove password
	passwd -e "$username" # Expire the password to force change on the first login

	# Set ownership of home directory (just for safety)
	chown -R "$username:$username" "$USER_HOME"

	echo "succesfully populated home directory of user: '$username'"
	echo "---"

done < "$USERNAMES_FILE"

echo "User creation process complete!"
echo ""
echo "Note: users will be prompted to set a password on first login."
echo "Docker group permission will be active after users log out and back in"
