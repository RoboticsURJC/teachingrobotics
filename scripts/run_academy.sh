#!/bin/sh

# Default: cpu and offline
gpu_mode="false"
nvidia="false"
base_path_offline="compose_cfg/"
compose_file="user_humble_cpu"
base_path_online="https://raw.githubusercontent.com/JdeRobot/RoboticsAcademy/humble-devel/compose_cfg/"
RA_ex_db_path_online="https://raw.githubusercontent.com/JdeRobot/RoboticsAcademy/humble-devel/database/exercises/db.sql"
RA_django_db_path_online="https://raw.githubusercontent.com/JdeRobot/RoboticsAcademy/humble-devel/database/django_auth.sql"
RI_db_path_online="https://raw.githubusercontent.com/JdeRobot/RoboticsInfrastructure/database/database/universes.sql"


# Function to clean up the containers
cleanup() {
  echo "Cleaning up..."
  if [ "$nvidia" = "true" ]; then
    docker compose --compatibility down
  else
    docker compose down
  fi
  rm docker-compose.yaml
  
  exit 0
}

# Loop through the arguments using a while loop
while getopts ":g:n  " opt; do
  case $opt in
    g) gpu_mode="true" ;; 
    n) nvidia="true" ;;
    \?) echo "Invalid option: -$OPTARG" >&2 ;;   # If an invalid option is provided, print an error message
  esac
done

# Set up trap to catch interrupt signal (Ctrl+C) and execute cleanup function
trap 'cleanup' INT

# Set the compose file
if [ "$gpu_mode" = "true" ]; then
  compose_file="user_humble_gpu"
fi
if [ "$nvidia" = "true" ]; then
  compose_file="user_humble_nvidia"
fi

# Check the mode
if [ -d compose_cfg ]; then
  # Offline mode
  cp $base_path_offline$compose_file.yaml docker-compose.yaml
else
  # Online mode: download all databases
  curl -sL $base_path_online$compose_file.yaml -o docker-compose.yaml
  # Create folders
  mkdir -p database
  mkdir -p database/exercises
  mkdir -p RoboticsInfrastructure
  mkdir -p RoboticsInfrastructure/database
  # Download Db
  curl -sL $RA_ex_db_path_online -o database/exercises/db.sql
  curl -sL $RA_django_db_path_online -o database/django_auth.sql
  curl -sL $RI_db_path_online -o RoboticsInfrastructure/database/universes.sql
fi

# Execute docker compose
if [ "$nvidia" = "true" ]; then
  docker compose --compatibility up
else
  docker compose up
fi
