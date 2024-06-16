mkdir ./tmp
curl -L https://github.com/osrf/gazebo_models/archive/refs/heads/master.zip -o ./tmp/gazebo_models.zip
unzip ./tmp/gazebo_models.zip -d ./tmp


# on docker
# mkdir -p ~/.gazebo/models/
# mv ./tmp/gazebo_models-master/* ~/.gazebo/models/