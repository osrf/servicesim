#!/usr/bin/env bash

usage()
{
cat << EOF
OPTIONS:
   -h        Help
   -d        Download latest template files
   -p [arg]  Custom file prefix, consists of the file path and name without
             extension. Defaults to "service"
EOF
exit
}

# Default values
DOWNLOAD=false
PREFIX="service"

GetOpts()
{
  argv=()
  while [ $# -gt 0 ]
  do
    opt=$1
    shift
    case ${opt} in
        -p)
          if [ $# -eq 0 -o "${1:0:1}" = "-" ]
          then
            echo "Missing file prefix after -p"
          else
            PREFIX="$1"
          fi
          shift
          ;;
        -d)
          DOWNLOAD=true
          ;;
        *)
          usage
          argv+=(${opt})
          ;;
    esac
  done
}

GetOpts $*

if $DOWNLOAD; then

  echo -e "\e[92mDownloading template files...\e[39m"

  # Clear previous files
  rm *.erb *.yaml

  # Download world erb
  wget https://bitbucket.org/osrf/servicesim/raw/default/servicesim_competition/worlds/actor_collisions.erb
  wget https://bitbucket.org/osrf/servicesim/raw/default/servicesim_competition/worlds/actor_idle.erb
  wget https://bitbucket.org/osrf/servicesim/raw/default/servicesim_competition/worlds/actor_trajectory.erb
  wget https://bitbucket.org/osrf/servicesim/raw/default/servicesim_competition/worlds/back_entrance.erb
  wget https://bitbucket.org/osrf/servicesim/raw/default/servicesim_competition/worlds/bathroom_furniture.erb
  wget https://bitbucket.org/osrf/servicesim/raw/default/servicesim_competition/worlds/chair.erb
  wget https://bitbucket.org/osrf/servicesim/raw/default/servicesim_competition/worlds/conference_table.erb
  wget https://bitbucket.org/osrf/servicesim/raw/default/servicesim_competition/worlds/config.yaml
  wget https://bitbucket.org/osrf/servicesim/raw/default/servicesim_competition/worlds/cubicle_corner.erb
  wget https://bitbucket.org/osrf/servicesim/raw/default/servicesim_competition/worlds/cubicle_island.erb
  wget https://bitbucket.org/osrf/servicesim/raw/default/servicesim_competition/worlds/cubicles.erb
  wget https://bitbucket.org/osrf/servicesim/raw/default/servicesim_competition/worlds/debug_visuals.erb
  wget https://bitbucket.org/osrf/servicesim/raw/default/servicesim_competition/worlds/front_entrance.erb
  wget https://bitbucket.org/osrf/servicesim/raw/default/servicesim_competition/worlds/idle_near_entrance.erb
  wget https://bitbucket.org/osrf/servicesim/raw/default/servicesim_competition/worlds/idle_near_fridge.erb
  wget https://bitbucket.org/osrf/servicesim/raw/default/servicesim_competition/worlds/meeting_room_large_furniture.erb
  wget https://bitbucket.org/osrf/servicesim/raw/default/servicesim_competition/worlds/meeting_room_small_furniture.erb
  wget https://bitbucket.org/osrf/servicesim/raw/default/servicesim_competition/worlds/office_chair.erb
  wget https://bitbucket.org/osrf/servicesim/raw/default/servicesim_competition/worlds/office_large_furniture.erb
  wget https://bitbucket.org/osrf/servicesim/raw/default/servicesim_competition/worlds/office_small_furniture.erb
  wget https://bitbucket.org/osrf/servicesim/raw/default/servicesim_competition/worlds/private_cafe.erb
  wget https://bitbucket.org/osrf/servicesim/raw/default/servicesim_competition/worlds/refreshment_area_furniture.erb
  wget https://bitbucket.org/osrf/servicesim/raw/default/servicesim_competition/worlds/room.erb
  wget https://bitbucket.org/osrf/servicesim/raw/default/servicesim_competition/worlds/service.world.erb
  wget https://bitbucket.org/osrf/servicesim/raw/default/servicesim_competition/worlds/spawn_urdf.launch.erb
  wget https://bitbucket.org/osrf/servicesim/raw/default/servicesim_competition/worlds/trajectory_back.erb
  wget https://bitbucket.org/osrf/servicesim/raw/default/servicesim_competition/worlds/trajectory_front.erb
  wget https://bitbucket.org/osrf/servicesim/raw/default/servicesim_competition/worlds/walls.erb

  echo -e "\e[92m... finished downloading\e[39m"
fi

echo -e "\e[92mGenerating files...\e[39m"
erb urdf_launch=$PREFIX.launch service.world.erb > $PREFIX.world
echo -e "\e[92m... generated\e[39m"

