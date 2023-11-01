# G2o Examples 

I have a custom docker image with baselayer as ubuntu and g2o installed.
I try to develop some usable g2o codes. 

docker run -it my-custom-image:v1.1


## Docker config 
Base layer is Ubuntu22.04. 
- build essentials gcc, git, cmake 
- Eigen3
- g2o version from oct24 2023 


## Running the code base with docker
Make sure you have docker installed on your host pc. 


### Clone this repo to your local machine 
```
cd $HOME/Downloads 
git clone https://github.com/mpkuse/g2o_tutorial
```

### Run Docker 
<<<<<<< HEAD
=======
I have a [docker image](https://hub.docker.com/r/mpkuse/my-g2o-image) with g2o and Eigen installed on it. 
>>>>>>> f3c71a0 (first commit)
Mount the repo in the docker at `/code` and run the docker. 
```
docker run -v $HOME/Downloads/g2o_tutorial:/code -it mpkuse/my-g2o-image
```

Additional terminals 
```
docker ps #to get container id
docker exec -it <CONTAINER_ID> /bin/bash
```

### Compile code (in docker)
Run the docker and in the docker's terminal execute this
```
$(docker) cd /code 
$(docker) mkdir build 
$(docker) cd build 

$(docker) cmake .. 

$(docker) make 
$(docker) ./simple_optimize 
```