
docker build -t logic-lynx-steering-v0.0.1 -f Dockerfile.armhf .
docker save  logic-lynx-steering-v0.0.1 > logic-lynx-steering-v0.0.1.tar
