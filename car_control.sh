#!/bin/bash

docker compose -f ./scripts/docker-compose_car_control.yml up -d

cleanup() {
    echo "Shutting down docker compose services..."
    docker ps -aq --filter "name=scripts*" | xargs docker rm -f
    exit 0
}

trap cleanup SIGINT

echo "Press Ctrl+C to stop..."

echo "Listening to docker-compose_car_control.yml logs. Press Ctrl+C to stop..."
docker compose -f ./scripts/docker-compose_car_control.yml logs -f &
wait