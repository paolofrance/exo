## Run exo

### Preliminaries

Make sure the odrive, motors and harware components are set up properly. For odrive configuration, see [config README](./config/README.md).

### compile the packages

#### shared memory library

``` console
    cd shared-memory-1.0
    mkdir build
    cd build
    cmake ..
    make
```

#### odrive master

``` console
    cd odrive_master
    mkdir build
    cd build
    cmake ..
    make
```

#### controller package

``` console
    cd controller
    mkdir build
    cd build
    cmake ..
    make
```

#### perception

```bash
cd detection
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```
to compile the receiver:

``` console
    g++ -o udprecieve udprecieve.cpp
```

run it 
``` console
    ./udprecieve
```

