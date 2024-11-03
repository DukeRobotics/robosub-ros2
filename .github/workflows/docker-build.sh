#!/bin/bash
# shellcheck disable=SC2035

docker buildx bake --file docker-compose.yml --set *.cache-from=type=local,src=/tmp/.buildx-cache --set *.cache-to=type=local,dest=/tmp/.buildx-cache
docker compose -f docker-compose.yml up -d