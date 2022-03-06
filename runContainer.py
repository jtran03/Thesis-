import os
import time
# Build
# yes it's a little tedious but it makes sure changes to dockerfile propagate among team members
os.system("docker build -f Dockerfile.build . -t pizzabotsim")

os.system("docker container stop pizzabotsim")
os.system("docker container rm pizzabotsim")
os.system("docker run -it -d --name pizzabotsim -p 5900:5900 pizzabotsim ")
# wait for server to become ready
os.system("docker exec -it pizzabotsim ""nohup /openvnc.sh""")
os.system("docker exec -it pizzabotsim /bin/bash")
os.system("docker container stop pizzabotsim")
# dont rm in case we want to pull stuff out of it in emergency
