#!/usr/bin/env bash

#系统镜像版本
DOCKER_REPO=cuidarchan/test_v1.0
VERSION=v2.0
ENV_IMG=${DOCKER_REPO}:${VERSION}
echo $ENV_IMG

# 当前工作路径, 如catkin_ws的绝对路径
WORKDIR_PATH=$(cd "$(dirname "$0")/../../../";pwd)
echo $WORKDIR_PATH

#删除已经命名的calib_docker容器
docker ps -a --format "{{.Names}}" | grep 'calib_docker' 1>/dev/null
if [ $? == 0 ]; then
    echo "Starting stop calib_docker container"
    docker stop calib_docker 1>/dev/null
    docker rm -v -f calib_docker 1>/dev/null
fi

# 打开calib_docker，并挂载相应的文件
docker run -it --name calib_docker \
    -v $WORKDIR_PATH/:/catkin_ws/ \
    --env USER=cuidarchan \
    --hostname in_calib_docker \
    -w /catkin_ws/ \
    --net host \
    -e VNC_RESOLUTION=1820x880 \
 	-v /tmp/.X11-unix:/tmp/.X11-unix \
    -e GDK_SCALE \
    -e GDK_DPI_SCALE \
    $ENV_IMG \

# UTF-8加入到环境变量，解析中文字符 
#docker exec squirrel_docker bash -c "echo 'export LANG="C.UTF-8"' >> ~/.bashrc;" 

# 提示语句
str="[docker start succeed!]"
echo -e "\033[32m ${str}\033[0m"
echo "Now you can enter with: bash docker_into.sh"