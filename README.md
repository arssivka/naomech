# RDot Naomech
This is the system that allows you to control Nao locally or remote.

### Example

For example, you can write python script like this to set the hardness of motors:

```python
 proxy = xmlrpclib.ServerProxy("http://your.nao.ip:8080")
 proxy.joints.hardness(0.6)
```

### Why?

The creation of this system was motivated by our wish to minimize the usage of Aldebaran's system and bad compatibility of
Nao's system with ROS. For example when you are trying to move mototrs by zero angle using aldebaran's system, mototrs will
move. Also the cameras were really slow.

### Installation

1. You need ctc installed on your system. If you haven't already go to the aldebaran's site and download 32-bit or 64-bit version according
to your system: [Aldebaran's site][aldbr]
2. Extract it wherever you like.
3. Now you need to install new drivers provided by BHuman. Follow their unstructions [here][bh].
4. Clone [this][rep] repository:

  ```sh
  $ git clone https://github.com/arssivka/naomech
  ```
5. Copy the script ctc_restructurer.sh to your ctc folder and run it.

  ```sh
  $ bash path/to/ctc/ctc_restructurer.sh
  ```
6. Now you can build the project. For that you need to run script naomech/tools/scripts/install.sh with parameters:
  - --ip IP of your Nao
  - --sources-path Path to source files
  - -b Path where project will be built
  - -i Path where project will be installed
  - --ctc Path to your ctc
  
  ```sh
  $ bash tools/scripts/install.sh --ip ip.of.your.nao --sources-path /path/to/naomech/ -b /path/to/build/ -i /path/to/intsall/ --ctc /path/to/ctc/
  ```
  
  This script will also copy all the needed files to the robot.
  
7. Connect to your Nao via ssh:

  ```sh
  $ ssh nao@your.nao.ip
  ```
  
  Create file /home/nao/.profile with the following content:
  
  ```
  export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/home/nao/naomech/lib
  ```
  
  Also add following
  
  ```
  /home/nao/naomech/lib/libhwcontroller.so
  ```
  
  to the file /home/nao/naoqi/preferences/autoload.ini
  
  Restart the nao.
  Now you can run the system.
  
  ```sh
  $ ./home/nao/naomech/bin/controller
  ```



[aldbr]: <https://community.aldebaran.com/en/resources/software/>
[rep]: <https://github.com/arssivka/naomech>
[bh]: <https://github.com/bhuman/BKernel>
