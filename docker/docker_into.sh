#!/usr/bin/env bash

DOCKER_REPO=cuidarchan/test_v1.0
VERSION=v2.0
ENV_IMG=${DOCKER_REPO}:${VERSION}
echo $ENV_IMG

docker exec -it calib_docker /bin/bash