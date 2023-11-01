https://www.tutorialspoint.com/docker/docker_images.htm#

docker run hello-world 


## run interactively
docker run -it centos /bin/bash

# press CTRL+P+Q to detach from it. 


## images 
docker images  #get list of images 
docker rmi <image id>  #remove an image 
docker inspect <imageID> # print all details as json


## containers 
docker ps -a
docker top <containerID>
docker stop <containerID> 
docker rm <containerID> # remove a container
docker stats <containerID> 



##
docker attach <containerID> 
docker pause <containerID> 
docker unpause <containerID>
docker kill <containerID> 
docker run <imageID> 
# press CTRL+P+Q to detach from it. 



## 
sudo service docker stop 
sudo service docker start 



## HOWTO docker file 
mkdir nginx-image 
cd nginx-image 
touch Dockerfile #then put your contents in this file
cd ..
docker build nginx-image/ -t nginx:1.0



## filesystem 
docker run -v $HOME/Downloads:/Downloads -it centos /bin/bash #maps local files on the docker
 
 
## networking 
docker network ls 


## Commit current state
// docker ps -> to get the currently running images 
docker commit container_id my-custom-image:tag
  

## Stopped containers
docker ps -a # lists all containers
docker container prune


## Tag a local image 
```
docker tag my-custom-image:v1.2 mpkuse/my-g2o-image
```


```
docker login 
docker push mpkuse/my-g2o-image
```