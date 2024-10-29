# WALL-E
Wildlife Activity Life Explorer
The main repository for the New Mexico Tech Wildlife Monitoring Project

# INSTALLATION

This system is designed to run in Docker on linux.  

Install docker  
<https://docs.docker.com/engine/install/ubuntu/>  

To use docker without sudo, run the following commands  
```
sudo groupadd docker
sudo usermod -aG docker $USER
newgrp docker
```

Ensure you have an SSH key set up with Github (optional)
<https://docs.github.com/en/authentication/connecting-to-github-with-ssh/adding-a-new-ssh-key-to-your-github-account>  

Clone this repository. To commit to the repository, clone with the ssh address. Don't push to main unless you helped write this README (make your own branch).
```
git clone https://github.com/NMT-Wildlife-Monitoring-Project/WALL-E.git
```

Build the docker image  
`docker build -t ros_walle WALL-E`  

# USAGE

Run the docker image  
`docker run -it --name ros_walle ros_walle`  
To open another terminal in the same container run  
`docker exec -it ros_walle bash`  


