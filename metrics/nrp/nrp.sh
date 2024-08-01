#!/bin/bash
# shellcheck disable=SC2016 # ignore variable expansion in single quotes warning

# NRP DEVELOPER, VERY IMPORTANT: increment the VERSION on every commit, since it will force an update on users' side

VERSION=1.26

#===================#
# UTILITY FUNCTIONS #
#===================#


auto_update_script() {
  set +e
  echo -e "${BLUE}Checking for script updates${NC}"

  local script_name=$1
  local script_path="/tmp/$script_name"
  local script_URL=https://neurorobotics.net/downloads/nrp_installer.sh

  wget --no-check-certificate "$script_URL" -O "$script_path" 2>/dev/null
  if [ -f "$script_path" ]
   then
    new_version=$(grep -o -P "(?<=VERSION=)[0-9]+\.[0-9]+" "$script_path")
    [ -z "$new_version" ] && echo -e "${GREEN}$script_URL ${RED}doesn't have a valid version number${NC}" && return 1

    version_check $VERSION "$new_version"
    check=$?

    if [ "$check" -eq 9 ]; then
      echo -e "${YELLOW}A newer version of this script has been found. Update (recommended)? [y/n] ${NC}"

      read -r yn
      case $yn in
        [Yy]* ) mv -f "$script_path" ./"$script_name"
                chmod 755 ./"$script_name"
                echo -e "${GREEN}$script_name has been updated and will exit now. Just re-run it.${NC}"
                exit
                ;;
        * ) ;;
      esac
    else
      echo -e "${GREEN}$script_name is up-to-date.${NC}"
    fi
  fi
  set -e
}

# Check the existance of the container by its name
container_exists() {
  container=$1
  if [ -z "$1" ]; then
    echo -e "${RED}Cannot check the existence of the container without its name. Failed.${NC}"
    # Exit with "Invalid argument" error
    exit 22
  fi

  if [ -z "$($DOCKER_CMD ps -aqf "name=^$container$")" ]; then 
    return 1
  else
    return 0
  fi
}

restart() {
  #######################################
  # Restarts the containers
  # USAGE: restart - tries to restart all 
  #        containers from nrp_containers
  #        restart <container> - tries
  #        to restart specified container
  #######################################
  container=$1
  if [ -z "$container" ]; then
    for item in "${nrp_containers[@]}"; do
      if container_exists "$item"; then
        restart "$item"
      fi
    done
  elif container_exists "${container}"; then
    echo -e "${BLUE}Restarting $container${NC}"
    $DOCKER_CMD restart "$container" && $DOCKER_CMD exec "$container" bash -c "sudo /etc/init.d/supervisor start"
    if [ "$container" == "proxy" ]
    then
      $DOCKER_CMD exec proxy bash -c 'sudo service haproxy restart > /dev/null 2>&1'
    fi
    echo -e "${RED}$container ${GREEN}container has now been restarted.${NC}"
  else
    echo -e "${RED}Failed to restart ${RED}$container container: the existing container is not found. Try running ${BLUE}install <VERSION>${RED} first.${NC}"
    exit 2
  fi
}

stop() {
  container=$1
  if container_exists "${container}"; then
    echo -e "${BLUE}Stopping $container${NC}"
    $DOCKER_CMD stop "$container"
    echo -e "${RED}$container ${GREEN}container has now been stopped.${NC}"
  fi
}

delete() {
  container=$1
  while true; do
    echo -e "${YELLOW}$container container will be deleted. You will lose any data you may have changed inside it (but not your experiments and models). Can we proceed? [y/n] ${NC}"
    read -r yn
    case $yn in
      [Yy]* )
              echo -e "${BLUE}Deleting $container container${NC}"
              $DOCKER_CMD rm -f "$container"
              echo -e "${GREEN}Successfully deleted old $container container.${NC}"
              return 0;;
      [Nn]* ) return 1;;
      * ) echo "Please answer yes or no.";;
    esac
  done
}

restore() {
  container=$1
  version=$2

  if [ -z "$container" ]; then
    for item in "${nrp_containers[@]}"; do
      if container_exists "$item"; then
        restore "$item"
      fi
    done
  elif container_exists "${container}"; then
    # get current image of $container
    container_image=$($DOCKER_CMD ps -a -f"name=^$container$" --format='{{.Image}}')

    images[$container]=$($DOCKER_CMD inspect --format='{{.Config.Image}}' "$container")

    # use tag $version, if any. Default to running container tag and set image name, otherwise
    nrp_version_tag=${version:-$( echo "${container_image}" | cut -d ":" -f2)}

    stop "$container"

    if delete "$container"
      then start "$container"
    else
      restart "$container"
    fi

  else
    echo -e "${RED}Failed to restore ${RED}$container container: the existing installation is not found. Try running ${BLUE}install <VERSION>${RED} first.${NC}"
    exit 2
  fi
}

start() {
  container=$1
  port=${ports[${container}]}
  image=${images[${container}]}

  check_port "${port}"
  echo -e "${BLUE}Starting ${RED}$container ${BLUE}container on port ${port} using image ${image}${NC}"

  [[ "$is_mac" = false ]] && iparg=--ip="${IPs[$container]}"

  notebook_port_arg=""
  # if notebook_port is not empty and container is nrp
  if [ -n "$notebook_port" ] && [ "$container" = "nrp" ]
  then
    notebook_port_arg="-p $notebook_port:$notebook_port"
  fi
  
  [ "$container" = "nrp" ] && [ "$nrp_version_tag" = "legacy" ] && container_port=8080 || container_port="${port}"

  # container is not running yet, HOME and HBP must be known in advance
  local cont_HOME="/home/bbpnrsoa"
  local cont_HBP="${cont_HOME}/nrp/src"

  # ${notebook_port_arg} should be without quotes
  # shellcheck disable=SC2086
  $DOCKER_CMD run -itd \
    -p "${port}":"${container_port}" ${notebook_port_arg} \
    --net="${networkName}" \
    "${iparg}" \
    -v nrp_user_data:${cont_HOME}/.opt/nrpStorage \
    -v nrp_models:${cont_HBP}/Models \
    -v nrp_experiments:${cont_HBP}/Experiments \
    --name "$container" "${image}"
  eval "setup_${container}"
  echo -e "${RED}$container ${GREEN}container is now up and running.${NC}"
}

pull_images() {

  #######################################
  # Pulls the images specified
  # in the "images" array (required)
  # for the NRP components specified
  # in "nrp_containers" array (required)
  # USAGE: pull_images
  #######################################

  for item in "${nrp_containers[@]}"; do
    echo -e "${BLUE}Pulling ${item} image, this won't take much time..${NC}"
    $DOCKER_CMD pull "${images[$item]}"
    echo -e "${GREEN}Successfully downloaded ${item} image.${NC}"
  done
}

create_containers () {
  local nrp_version_tag="$1"

  # check if "${networkName}" exists
  netID=$(docker network ls -f name="${networkName}" -q)
  # if network doesn't exist, try to create it
  if [ -z "$netID" ]; then
    # exit in case the network can't be created
    # --opt...mtu=1450 is added to allow run script in cloud environments
    if ! $DOCKER_CMD network create -d bridge --subnet "$subnet" --gateway "$hostIP" --opt com.docker.network.driver.mtu=1450 "${networkName}";
    then
      echo -e "${RED}Cannot create Docker network, if address is in use, try to use --subnet command for the script.${NC}"
      exit 1
    fi
  # if network exist, check that it has desired subnet
  else
    targetSUBNET=$($DOCKER_CMD network inspect "${netID}" --format='{{range .IPAM.Config}}{{print .Subnet}}{{end}}')
    if [ "$subnet" = "$targetSUBNET" ];
    then
      echo -e "${BLUE}The ${networkName} Docker network already exists.${NC}"
    else
      echo -e "${RED}The ${networkName} Docker network already exists, but it has a different target subnet: ${targetSUBNET}${NC}"
      echo -e "${RED}Try to use --subnet command for the script in order to change the subnet address.${NC}"
      exit 1
    fi
  fi

  # configure volumes
  $DOCKER_CMD volume create nrp_models
  $DOCKER_CMD volume create nrp_experiments
  $DOCKER_CMD volume create nrp_user_data

  # start the containers or restore them if any exists
  for container in "${nrp_containers[@]}"
  do
    # if there is a container named $container
    if container_exists "${container}"
    then
      echo -e "The container $container already exists."
      if delete "$container"; then
        start "$container"
      fi
    else
      start "$container"
    fi
  done

  # echo -e "${BLUE}Removing old unused images${NC}"
  # $DOCKER_CMD system prune
}

print_success_message () {

    [ "$nrp_version_tag" == "legacy" ] && container="frontend" || container="proxy"

    echo -e "${GREEN}
    Congratulations! The NRP platform is now installed on your computer.
    ${NC}
    
    You can check everything works by going to ${PURPLE}http://localhost:${ports[${container}]}/#/esv-private ${NC}or if you used the --ip option: ${PURPLE}http://${external_IPs[${container}]}:${ports[${container}]}/#/esv-private ${NC}by using your browser and signing in with the following credentials:

    username : nrpuser
    password : password

    If you need any help please use our forum: ${PURPLE}https://forum.humanbrainproject.eu/c/neurorobotics${NC}"
    echo
}

function clone_models_experiments_repo () {

  for repo in "Models" "Experiments"
  do
    echo -e "${BLUE}Cloning $repo, this may take a while${NC}"
    $DOCKER_CMD exec nrp bash -c '{ cd $HBP/"$1" && git config remote.origin.fetch +refs/heads/"$2":refs/remotes/origin/"$2" && sudo git checkout "$3" && sudo git pull; } || { cd $HBP && sudo ls -A1 "$1"/ | xargs rm -rf && sudo git clone --progress --branch="$3" https://bitbucket.org/hbpneurorobotics/"$1".git "$1"/; }' -- "$repo" "$1" "$2"
    # set ownership
    $DOCKER_CMD exec nrp bash -c 'sudo chown -R $USER:$(id -gn) $HBP/$1' -- $repo
  done
}

function sed_in_container() {
  #######################################
  # Run sed in a specified container
  # The arguments "sed script" and "file path" can use variables
  # from the container's environment (e.g. '$HBP')
  #
  # USAGE: sed_in_container <string> <string> <string>
  # ARGUMENTS:
  #   1 - container
  #   2 - sed script
  #   3 - file path
  # OUTPUTS:
  #   Nothing
  #######################################
  $DOCKER_CMD exec "$1" bash -c 'eval script="$1" input_file="$2"; /bin/sed -e $script -i $input_file' -- "$2" "$3"
}

setup_nrp() {

  echo -e "${BLUE}Setting up ${RED}nrp ${BLUE}container${NC}"

  $DOCKER_CMD exec nrp bash -c 'echo "127.0.0.1 $(uname -n)" | sudo tee --append /etc/hosts'
  # From inside the backend container, localhost should point to docker host
  $DOCKER_CMD exec nrp bash -c "echo '$hostIP localhost' | sudo tee --append /etc/hosts"
  set +e
  case $nrp_version_tag in
      legacy)
        branch_refspec="master*"
        branch="master18"
        ;;
      latest)
        branch_refspec="master"
        branch="master"
        ;;
      development)
        branch_refspec="development"
        branch="development"
        ;;
      *)
        echo -e "${RED}[ERROR] Wrong version specified ${NC}"
        exit
        ;;
  esac

  clone_models_experiments_repo "$branch_refspec" "$branch"
  # configure storage IP address
  # Virtual coach
  sed_in_container "nrp" "s/localhost:${ports[frontend]}/${IPs[frontend]}:${ports[frontend]}/" '$HBP/VirtualCoach/hbp_nrp_virtual_coach/pynrp/config.json'
  # ExDBackend
  if [ "$nrp_version_tag" != "legacy" ]; then
    sed_in_container "nrp" "s/localhost:${ports[proxy]}/${IPs[proxy]}:${ports[proxy]}/" '$HBP/ExDBackend/hbp_nrp_commons/hbp_nrp_commons/workspace/Settings.py'
  else
    sed_in_container "nrp" "s/localhost:${ports[frontend]}/${IPs[frontend]}:${ports[frontend]}/" '$HBP/ExDBackend/hbp_nrp_commons/hbp_nrp_commons/workspace/Settings.py'
  fi

  set -e
  echo -e "${BLUE}Setting rendering mode to CPU${NC}"
  $DOCKER_CMD exec nrp bash -c '/$HBP/user-scripts/rendering_mode cpu'

  echo -e "${BLUE}Generating low resolution textures${NC}"
  $DOCKER_CMD exec nrp bash -c 'python $HBP/user-scripts/generatelowrespbr.py'

  # make models available to gazebo
  $DOCKER_CMD exec nrp bash -c 'export NRP_MODELS_DIRECTORY=$HBP/Models && mkdir -p $HOME/.gazebo/models && $HBP/Models/create-symlinks.sh' 2>&1 | grep -v "HBP-NRP"

  # generate XML schema bindings
  $DOCKER_CMD exec nrp bash -c 'cd $HBP && source $HOME/.opt/platform_venv/bin/activate && pyxbgen -u Experiments/bibi_configuration.xsd -m bibi_api_gen && pyxbgen -u Experiments/ExDConfFile.xsd -m exp_conf_api_gen && pyxbgen -u Models/environment_model_configuration.xsd -m environment_conf_api_gen && pyxbgen -u Models/robot_model_configuration.xsd -m robot_conf_api_gen && deactivate' 2>&1 | grep -v "WARNING"
  $DOCKER_CMD exec nrp bash -c 'gen_file_path=$HBP/ExDBackend/hbp_nrp_commons/hbp_nrp_commons/generated && filepaths=$HOME/nrp/src && sudo cp $filepaths/bibi_api_gen.py $gen_file_path &&  sudo cp $filepaths/exp_conf_api_gen.py $gen_file_path && sudo cp $filepaths/robot_conf_api_gen.py $gen_file_path && sudo cp $filepaths/environment_conf_api_gen.py $gen_file_path'

  # start NRP services
  $DOCKER_CMD exec nrp bash -c "sudo /etc/init.d/supervisor start"
  echo -e "${GREEN}Finished setting up ${RED}nrp ${GREEN}container.${NC}"
}

setup_proxy() {
  echo -e "${BLUE}Setting up ${RED}proxy ${BLUE}container${NC}"

  $DOCKER_CMD exec proxy bash -c 'cp $HBP/user-scripts/config_files/nrpBackendProxy/config.json.sample.docker $HBP/nrpBackendProxy/config.json'
  sed_in_container proxy "s/\<nrp_IP\>/${IPs[nrp]}/" '$HBP/nrpBackendProxy/config.json'
  sed_in_container proxy "s=\<nrp_port\>=${ports[nrp]}=" '$HBP/nrpBackendProxy/config.json'
  sed_in_container proxy "s=\<proxy_ip\>=${external_IPs[proxy]}=" '$HBP/nrpBackendProxy/config.json'
  sed_in_container proxy "s=\<proxy_port\>=${ports[proxy]}=" '$HBP/nrpBackendProxy/config.json'


  # Prepare Haproxy config files
  $DOCKER_CMD exec proxy bash -c 'sudo mkdir -p /etc/haproxy/lua_scripts ; sudo cp $HBP/user-scripts/config_files/haproxy/local_docker/backend_extractor.lua /etc/haproxy/lua_scripts/ ;'
  $DOCKER_CMD exec proxy bash -c 'sudo cp $HBP/user-scripts/config_files/haproxy/local_docker/haproxy.cfg /etc/haproxy/'
  $DOCKER_CMD exec proxy bash -c 'sudo curl  https://raw.githubusercontent.com/haproxytech/haproxy-lua-cors/master/lib/cors.lua -o /etc/haproxy/lua_scripts/cors.lua'

# Remove exit on fail, as if the user exists already we dont care.
  set +e
  $DOCKER_CMD exec proxy bash -c 'source $HBP/user-scripts/nrp_variables 2> /dev/null && $HBP/user-scripts/add_new_database_storage_user -u nrpuser -p password -s > /dev/null 2>&1'
  set -e
  
  $DOCKER_CMD exec proxy bash -c 'sudo /etc/init.d/supervisor start'

  $DOCKER_CMD exec proxy bash -c 'sudo service rsyslog start > /dev/null 2>&1 ; sudo service haproxy start > /dev/null 2>&1'

  echo -e "${GREEN}Finished setting up ${RED}proxy ${GREEN}container.${NC}"
}

setup_frontend() {
  echo -e "${BLUE}Setting up ${RED}frontend ${BLUE}container${NC}"

  if [ "$nrp_version_tag" != "legacy" ]; then
    sed_in_container frontend "s/localhost/${external_IPs[proxy]}/" '$HBP/ExDFrontend/dist/config.json'

    sed_in_container frontend "s/:9000/${ports[frontend]}/" '$HOME/.local/etc/nginx/conf.d/frontend.conf'

  else
    if $is_win; then
      sed_in_container frontend "s/localhost/${external_IPs[frontend]}/" '$HBP/ExDFrontend/dist/config.json'
    else
      sed_in_container frontend "s/localhost/${hostIP}/" '$HBP/ExDFrontend/dist/config.json'
    fi

    sed_in_container frontend "s=:8080=:${ports[nrp]}=" '$HBP/nrpBackendProxy/config.json'


    # Add internalIp if it's not added yet
    $DOCKER_CMD exec frontend bash -c '/bin/sed -e '\'"/\"localhost\": {/,/\"gzweb\": {/c\"localhost\": {\n\t\t\"internalIp\": \"http:\/\/${IPs[nrp]}:8080\",\n\t\"gzweb\": {"\'' -i $HBP/nrpBackendProxy/config.json'
    # Remove exit on fail, as if the user exists already we dont care.
    set +e
    $DOCKER_CMD exec frontend bash -c 'source $HBP/user-scripts/nrp_variables 2> /dev/null && $HBP/user-scripts/add_new_database_storage_user -u nrpuser -p password -s > /dev/null 2>&1'
    set -e
  fi

  $DOCKER_CMD exec frontend bash -c "sudo /etc/init.d/supervisor start"

  echo -e "${GREEN}Finished setting up ${RED}frontend ${GREEN}container.${NC}"
}


uninstall() {
  # Dont fail on errors
  set +e
  echo -e "${BLUE}Removing NRP docker containers, volumes and network.${NC}"
  for container in "${nrp_containers[@]}"; do
    if container_exists "${container}"; then
      $DOCKER_CMD stop "${container}"
      $DOCKER_CMD rm "${container}"
    fi
  done
  $DOCKER_CMD network rm "${networkName}"
  $DOCKER_CMD volume rm {nrp_models,nrp_experiments}

  echo -e "${YELLOW}Would you also like to remove your personal experiments and models? [y/n] ${NC}"
  read -r yn
  case $yn in
    [Yy]* ) echo -e "${BLUE}Removing NRP user data${NC}" && $DOCKER_CMD volume rm nrp_user_data;;
    * ) echo "personal experiments and models are kept";;
  esac

  while true; do
    echo -e "${YELLOW}Would you like to remove the NRP Docker images? [y/n] ${NC}"
    read -r yn
    case $yn in
      [Yy]* ) break;;
      [Nn]* ) echo -e "${GREEN}The NRP has been successfully removed from your system.${NC}" && exit;;
      * ) echo "Please answer yes or no.";;
    esac
  done

  echo -e "${BLUE}Removing NRP docker images. This may take a while.${NC}"
  IMAGEStoDELETE="$($DOCKER_CMD images --filter=reference="${NRP_DOCKER_REGISTRY}"/nrp --filter=reference="${NRP_DOCKER_REGISTRY}"/nrp_frontend --filter=reference="${NRP_DOCKER_REGISTRY}"/nrp_proxy --format '{{.ID}}')"
  # ${IMAGEStoDELETE} should be without quotes
  # shellcheck disable=SC2086
  [ -z "$IMAGEStoDELETE" ] || $DOCKER_CMD rmi -f ${IMAGEStoDELETE} && echo -e "${BLUE}No images to delete${NC}" 
  echo -e "${GREEN}NRP Docker images have been successfully removed.${NC}"
  set -e

  echo -e "${GREEN}All traces of the NRP images and user data have been successfully removed from your system.${NC}"
}

connect() {
  container=$1

  echo -e "${BLUE}Opening new terminal into ${container} container${NC}"
  thecmd="bash -c \"echo -e \\\"${RED}You are inside the $container container. Advanced users only.\nCTRL+D to exit.\nIf you mess up everything, you can restore this container\nwith the reset command of the install script.\n${NC}\\\"; $DOCKER_CMD exec -it $container bash\""
  if [ -z "$(which gnome-terminal)" ]
  then
    echo -e "${GREEN}No gnome-terminal installed. Defaulting to bash.${NC}"
    bash -c "$thecmd"
  else
    gnome-terminal -e "$thecmd" &
  fi
}

check_port() {
  port=$1
  echo -e "${BLUE}Checking ports${NC}"
  set +e
  is_port=$(netstat -tuplen 2>/dev/null | awk '{ print $4 }' | grep ":$port")
  if [ -n "$is_port" ]
  then
    echo -e "${RED}[ERROR] The port $port is in currently in use. If you would like to install the NRP please find the process using this port and stop it:${NC}"
    echo -e "$is_port"
    exit
  fi
  echo -e "${GREEN}Port $port is available.${NC}"
  set -e
}

version_check() {
   [ -z "$1" ] || [ -z "$2" ] && return 9
   [ "$1" == "$2" ] && return 10

   ver1front=$(echo "$1" | cut -d "." -f -1)
   ver1back=$(echo "$1" | cut -d "." -f 2-)

   ver2front=$(echo "$2" | cut -d "." -f -1)
   ver2back=$(echo "$2" | cut -d "." -f 2-)

   if [ "$ver1front" != "$1" ] || [ "$ver2front" != "$2" ]; then
       [ "$ver1front" -gt "$ver2front" ] && return 11
       [ "$ver1front" -lt "$ver2front" ] && return 9

       [ "$ver1front" == "$1" ] || [ -z "$ver1back" ] && ver1back=0
       [ "$ver2front" == "$2" ] || [ -z "$ver2back" ] && ver2back=0
       version_check "$ver1back" "$ver2back"
       return $?
   else
           [ "$1" -gt "$2" ] && return 11 || return 9
   fi
}

subnet_check() {
  if [[ "$subnet" =~ ^[0-9]{1,3}\.[0-9]{1,3}\.[0-9]{1,3}\.0$ ]]; then
    subnet="$subnet"/16
  elif [[ "$subnet" =~ ^[0-9]{1,3}\.[0-9]{1,3}\.[0-9]{1,3}\.[0]{1}\/[0-9]{1,2}$ ]]; then
    # shellcheck disable=SC2269
    subnet="$subnet"
  else 
    echo -e "${RED}[ERROR] Wrong --subnet value: $subnet, expected network mask X.Y.Z.0/A or X.Y.Z.0${NC}"
    exit 1
  fi
  # ${subnet//./ } should not be in quotes
  # shellcheck disable=SC2206
  A=(${subnet//./ })
  highBits="${A[0]}.${A[1]}.${A[2]}"
  echo -e "${BLUE}Using Docker subnet $subnet${NC}"
}

update() {
      local nrp_version_tag="$1"

      # https://hbpneurorobotics.atlassian.net/browse/NRRPLT-8335
      # Before performing the update steps, the NRP containers should be running
      pull_images && create_containers "$nrp_version_tag"

      # Create backup of nrpStorage in the same path as nrp_installer.sh
      echo -e "${BLUE}Creating a backup of your nrpStorage${NC}"
      mkdir -p /tmp/nrp_storage.tmp

      local cont_HOME
      cont_HOME=$($DOCKER_CMD exec nrp bash -c 'printf $HOME')
      $DOCKER_CMD cp nrp:"$cont_HOME/.opt/nrpStorage" /tmp/nrp_storage.tmp/

      local backup_date
      backup_date="$(date +%d.%m.%y-%H:%I)"
      local backup_name=NRP_STORAGE_"$backup_date".bak.zip

      pushd /tmp/nrp_storage.tmp/ >/dev/null
      zip -r "$HOME"/"$backup_name" ./* 1> /dev/null || { echo ERROR; exit 1; }
      popd >/dev/null

      echo -e "${GREEN}$backup_name created in $HOME${NC}"

      # Process the public models (NRRPLT-7911)
      process_models

      # Run refactor_storage converting pre-3.2 experiments to NRP 3.2 experiment layout
      rfc_stor=$($DOCKER_CMD exec -it frontend bash -c 'cd $HBP/nrpBackendProxy; grep -q "refactor_storage" package.json' && echo "yes")
      if [ "$rfc_stor" == "yes" ]
      then
        $DOCKER_CMD exec frontend bash -c 'cd $HBP/nrpBackendProxy; npm run refactor_storage'
      fi
}

process_models() {
  #######################################
  # Process the public "template" models (NRRPLT-7911)
  # - Copy them into nrpStorage
  # - Add them to the models database
  # USAGE: process_models
  # OUTPUTS:
  #   Nothing
  #######################################
  # Run copy-to-storage and update_template_models if they exists
  cp_stor=$($DOCKER_CMD exec nrp bash -c 'test -f $HBP/Models/copy-to-storage.sh; exit $?' && echo "yes")
  if [ "$cp_stor" == "yes" ]
  then
    echo -e "${BLUE}Copying public template models to your storage${NC}"
    $DOCKER_CMD exec nrp bash -c '$HBP/Models/copy-to-storage.sh' 2>&1
    update_models=$($DOCKER_CMD exec proxy bash -c 'cd $HBP/nrpBackendProxy; grep -q "update_template_models" package.json' && echo "yes")
    if [ "$update_models" == "yes" ]
    then
      $DOCKER_CMD exec proxy bash -c 'cd $HBP/nrpBackendProxy; npm run update_template_models'
    fi
  fi
}

#==========================#
# END OF UTILITY FUNCTIONS #
#==========================#

#Fail on errors
set -e

is_win=false
is_mac=false

if grep -qEi "(Microsoft|WSL)" /proc/version; then
  exe=".exe";
  is_win=true;
elif uname -a | grep -q "Darwin"; then
  is_mac=true;
fi

declare highBits="172.19.0"
declare subnet="${highBits}.0/16"

networkName="nrpnet"

NRP_DOCKER_REGISTRY="${NRP_DOCKER_REGISTRY:-hbpneurorobotics}"

# # containers' ports
declare -A ports
ports=(
  [nrp]="8090"
  [frontend]="9002"
  [proxy]="9000"
)
frontend_legacy_port="9000"
notebook_port=8888

unset external_IP

# Declare images massive
declare -A images

# Declare nrp components
declare -a nrp_containers=(nrp frontend proxy)

DOCKER_CMD="docker"$exe
CMD=""

# the log machine and logging procedure is deprecated so this line is commented
# nrp_proxy_url="http://148.187.97.48"

#Colours
RED="\033[01;31m"
GREEN="\033[01;32m"
PURPLE="\033[01;35m"
BLUE="\033[01;34m"
YELLOW="\033[01;33m"
NC="\033[00m"

usage="
Usage: $(basename "$0") COMMAND VERSION

A script to install and start the Neurorobotics Platform using docker images.

Options:
    -h                   Show this help text

    -s/--sudo            Use docker with sudo
    --subnet X.Y.Z.0/A   Change the Docker network subnet address (default: X.Y.Z.0/A - ${subnet})
    -np/--notebook_port  Specify exposed port to be used by Jupyter notebook, Default port number is 8888, see NRP Tutorials >> Virtual Coach Tutorial >> Launching an Experiment from the Virtual Coach
    --debug              Add verbose output to the script for debugging."
if ! $is_mac
then
usage="$usage
    -i/--ip <ip_address> The IP address of the machine the images are installed on.
                         Use this option when you would like to access the NRP outside the machine its installed on."
fi
usage="$usage
Commands:
    restart_backend         Restart the backend container
    restart_frontend        Restart the frontend container
    restart_proxy           Restart the proxy container
    restart                 Restart backend and frontend containers
    update  [<VERSION>]     if no VERSION is specified it updates to latest stable version
                            if VERSION is development fetches latest updates from development branch of repositories
    install <VERSION>       Installs specified version the NRP
    uninstall               Uninstall the NRP
    stop                    Stops the nrp containers
    start                   Starts nrp containers which have previously been stopped
    reset_backend           Restores the backend container
    reset_frontend          Restores the frontend container
    reset_proxy             Restores the proxy container
    reset                   Restores the backend and frontend containers
    connect_frontend        Connect to the frontend container (Opens in a new terminal)
    connect_backend         Connect to the backend container (Opens in a new terminal)
    connect_proxy           Connect to the proxy container (Opens in a new terminal)

VERSION:
    latest - Installs the latest release of the NRP (Python 3 compatible)
    legacy - Installs the legacy release of the NRP (v3.0.5 - Python 2.7 compatible)
    ${BLUE}NOTE: Not specifying the VERSION value will install latest version${NC}

${BLUE}Please note:${NC}
This script requires that the package 'docker' is already installed
At least 15GB of disk space will be needed in order to install the NRP images${NC}
If you want to use a version other than your already installed one, follow these steps:
  1- Save all your experiments and data. You can download a complete zip file for each of your cloned experiments by
     pressing Export button in My experiments section.
  2- Uninstall your version by running './nrp_installer.sh uninstall'
  3- Install another version by running './nrp_installer.sh install \$VERSION'
  4- You can import previously exported experiments by pressing the \"Import Zip\" button in \"My Experiments\".
     WARNING: Python scripts written in Python 2.7 for NRP-legacy may not be compatible with the NRP-latest.
     In fact, NRP-latest is not backward compatible with NRP-legacy.
     Porting your Python scripts to Python 3 is required.
"

# Argument parsing
while [[ $# -gt 0 ]]
do
key="$1"
case $key in
    -h|--help)
      echo -e "$usage"
      exit
    ;;
    -np|--notebook_port)
      re='^[0-9]+$'
      if ! [[ $2 =~ $re ]] ; then
        echo -e "${RED}[ERROR] Please specify a port number (-np)${NC}"
        echo ""
        exit
      else
        notebook_port=$2
        shift
      fi
      shift
    ;;
    -s|--sudo)
      DOCKER_CMD="sudo docker"
      shift
    ;;
    --subnet)
      subnet="$2"
      shift
      shift
    ;;
    --debug)
      set -x
      shift
    ;;
    -i|--ip)
      external_IP="$2"
      shift
      shift
    ;;

    restart_backend)
       CMD="restart nrp"
       shift
     ;;
    restart_frontend)
       CMD="restart frontend"
       shift
     ;;
    restart_proxy)
       CMD="restart proxy"
       shift
     ;;
    restart)
       CMD="restart"
       shift
     ;;
    start)
       CMD="restart"
       shift
     ;;
    update)
      # the log machine and logging procedure is deprecated so this line is commented
      # set +e && curl -X POST ${nrp_proxy_url}/proxy/activity_log/update --max-time 5; set -e # logs each update event via the NRP proxy server

      # Specifying development after update means you want to fetch latest updates from development branch
      # Not specifying anything means you want to upgrade to the latest version
      if [ "$2" == "development" ]
      then
        nrp_version_tag="$2"
        shift
      else
        nrp_version_tag="latest"
      fi

      images=(
        [nrp]="${NRP_DOCKER_REGISTRY}/nrp:$nrp_version_tag"
        [frontend]="${NRP_DOCKER_REGISTRY}/nrp_frontend:$nrp_version_tag"
        [proxy]="${NRP_DOCKER_REGISTRY}/nrp_proxy:$nrp_version_tag"
      )
      CMD="update $nrp_version_tag && print_success_message"
      shift
    ;;
    install)
      # the log machine and logging procedure is deprecated so this line is commented
      # set +e && curl -X POST ${nrp_proxy_url}/proxy/activity_log/install --max-time 10; set -e # logs each install event via the NRP proxy server
      nrp_version_tag="$2"
      if [[ "$nrp_version_tag" =~ ^(legacy|latest|development)$ ]] ; then
        images=(
          [nrp]="${NRP_DOCKER_REGISTRY}/nrp:$nrp_version_tag"
          [frontend]="${NRP_DOCKER_REGISTRY}/nrp_frontend:$nrp_version_tag"
          [proxy]="${NRP_DOCKER_REGISTRY}/nrp_proxy:$nrp_version_tag"
        )
      else
        echo -e "${RED}[ERROR] Wrong version specified ${NC}"
        echo -e "$usage"
        exit
      fi
      if [ "$nrp_version_tag" == "legacy" ]; then
        nrp_containers=(nrp frontend)
        ports[frontend]="${frontend_legacy_port}"
      fi
      # after creating the containers we need to update models database (NRRPLT-7911)
      CMD="pull_images && create_containers $nrp_version_tag && process_models && print_success_message"
      shift # $2
      shift # $1
    ;;
    uninstall)
      CMD="uninstall"
      shift
    ;;
    stop)
      CMD="stop nrp && stop frontend && stop proxy"
      shift
    ;;
    reset_backend)
      CMD="restore nrp"
      shift
    ;;
    reset_proxy)
      CMD="restore proxy"
      shift
    ;;
    reset_frontend)
      CMD="restore frontend"
      shift
    ;;
    reset)
       CMD="restore"
       shift
     ;;
    connect_backend)
      CMD="connect nrp"
      shift
    ;;
    connect_frontend)
      CMD="connect frontend"
      shift
    ;;
    connect_proxy)
      CMD="connect proxy"
      shift
    ;;

    *)
     echo Unknown option "$key"
     echo -e "$usage"
     exit
     ;;
esac
done

if [ -z "$CMD" ]
then
  echo -e "${RED}[ERROR] Please provide a command to execute${NC}"
  echo ""
  echo -e "$usage"
  exit
fi

# set up subnet and highBits
subnet_check

# containers' hostname
declare -A IPs
if $is_mac
then
  IPs=(
    [nrp]="host.docker.internal"
    [frontend]="host.docker.internal"
    [proxy]="host.docker.internal"
  )
else
  IPs=(
    [nrp]="${highBits}.3"
    [frontend]="${highBits}.2"
    [proxy]="${highBits}.4"
  )
fi

# containers' external IP address
# in windows use localhost to connect from host to containers. refer to :
# https://hbpneurorobotics.atlassian.net/browse/NUIT-175
declare -A external_IPs
if $is_win
then
  external_IPs=(
    [nrp]="${external_IP:-localhost}"
    [frontend]="${external_IP:-localhost}"
    [proxy]="${external_IP:-localhost}"
  )
else
  external_IPs=(
    [nrp]="${external_IP:-${IPs[nrp]}}"
    [frontend]="${external_IP:-${IPs[frontend]}}"
    [proxy]="${external_IP:-${IPs[proxy]}}"
  )
fi

declare hostIP="${highBits}.1"


auto_update_script "$(basename "$0")"

# run parsed command
eval "$CMD"

# Reset terminal colour back to normal
echo -e "${NC}"
