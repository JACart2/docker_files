# Run this one time when first creating the dev_ws folder on a new computer

# Get packages -------------------------------------------------------------------------------------------
echo "Cloning repos"
mkdir -p src
cd src
git clone --branch display_global_path https://github.com/JACart2/ai-navigation.git && \
	echo "Repos have been set up. This workspace will be build by the docker entry script every time it runs."
