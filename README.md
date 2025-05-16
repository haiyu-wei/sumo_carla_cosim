### Environments
CARLA 0.9.15 with Ubuntu 22.04 
https://carla.readthedocs.io/en/latest/build_linux/

You need to build CARLA, otherwise you will not able to change the custom map. Linux is strongly recommanded, as using Windows will cause lots of bugs.

SUMO 1.23.1 
https://sumo.dlr.de/docs/Downloads.php

You will also need netedit to modify the road network and the traffic demand. https://sumo.dlr.de/docs/Netedit/index.html


### Run Co-Simulation
```
python3 run_synchronization.py demand1.sumocfg --tls-manager sumo --sumo-gui
```

`demand1.sumocfg` is just a sample demand sumo configuration, you can change it using netedit.
