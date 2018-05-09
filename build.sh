
name="logic-lynx-steering-v0.0.1"
tag=$(git log -1 --pretty=%h)
img="$name:$tag"
echo $img
docker build -t $img -f Dockerfile.armhf .

docker save  $img > $img.tar
