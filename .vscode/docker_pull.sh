echo $OSTYPE

if [[ "$OSTYPE" == "msys"* ]]; then
   docker pull ghcr.io/cwrurobotics/rov-24:docker
else
   sudo docker pull ghcr.io/cwrurobotics/rov-24:docker
fi