# RDot Naomech
This is the system that allows you to control Nao locally or remote.

### Introduction

We are the developers which trying to prepare the team of Nao robots for playing football. After expierencing some problems 
(Nao can't walk strigh! wtf, aldebaran?! -_-) we've diceded to develop this 'bicycle'. We are not professionals, so if you think that 
what you see here is strange, it's normal. You are free to use this code, but try to use minimum of it and don't forget to add link
to the authors.

### Example

For example, you can write python script like this to set the hardness of motors:

```python
 proxy = xmlrpclib.ServerProxy("http://your.nao.ip:8080")
 proxy.joints.hardness(0.6)
```

### Why?

The creation of this system was motivated by our wish to minimize the usage of Aldebaran's system and bad compatibility of
Nao's system with ROS. For example when you are trying to move mototrs by zero angle using aldebaran's system, mototrs will
move. Also the cameras were really slow. As you will see in the installation section, we have script that changes ctc structure. It is made to use only CMake without qibuild for building the project. Also we wanted to refuse the usage of ALValue, because it's slow and 
we wanted to manage our data manually.

### Installation (This is for linux systems only!)

1. You need ctc installed on your system. If you haven't already go to the aldebaran's site and download 32-bit or 64-bit version according
to your system: [Aldebaran's site][aldbr]
2. Extract it wherever you like. Let it be the folder ~/ctc for example.
3. Now you need to install new drivers provided by BHuman. Follow their unstructions [here][BHumanKernel].
4. Clone [this][rep] repository:

  ```sh
  $ git clone https://github.com/arssivka/naomech
  ```
5. Copy the script ctc_restructurer.sh to your ctc folder and run it. This is made to provide the work of CMake without qibuild.

  ```sh
  $ bash path/to/ctc/ctc_restructurer.sh
  ```
  
5. Connect to your Nao via ssh and create folder naomech:

  ```sh
  $ ssh nao@your.nao.ip
  $ mkdir naomech
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
  Note: if you have any idea how to improve this, feel free to share that idea with us! ;)
  
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
  
  If everything is ok, you will see something like this:
  
  ```sh
  [I] 8529 qimessaging.session: Session listener created on tcp://0.0.0.0:0
  [I] 8529 qimessaging.transportserver: TransportServer will listen on: tcp://127.0.0.1:47123
  [I] 8529 qimessaging.transportserver: TransportServer will listen on: tcp://192.168.0.8:47123
  ```
  
### We are using:
 
  * [XML-RPC] For providing the remote control
  * [BHumanKernel] - new drivers
 
### License
  This project is under BSD license. See [license][lic] file for details.
  
### Note
  Project is still under development. A lot of fixes and changes can be done at any time.

[aldbr]: <https://community.aldebaran.com/en/resources/software/>
[rep]: <https://github.com/arssivka/naomech>
[BHumanKernel]: <https://github.com/bhuman/BKernel>
[XML-RPC]: <https://github.com/ensc/xmlrpc-c>
[lic]: <https://github.com/arssivka/naomech/blob/master/LICENSE>
