#!/usr/bin/env bash
# Script builds all sources and copy it to the robot

PROGRAM=$0

if [[ $# != 10 ]]; then
    STOP=true
fi

# Parse arguments
while [[ $# > 1 ]]
do
    key="$1"

    case $key in
        --ip)
        ROBOT_IP="$2"
        shift # past argument
        ;;
        -b|--build-path)
        BUILD_PATH=`realpath $2` || "$2"
        shift # past argument
        ;;
        -i|--install-path)
        INSTALL_PATH=`realpath $2` || "$2"
        shift # past argument
        ;;
        --toolchain-path|--ctc)
        TOOLCHAIN_PATH=`realpath $2` || "$2"
        shift
        ;;
        --sources-path)
        SOURCES_PATH=`realpath $2` || "$2"
        shift
        ;;
        -h|--help)
        HELP=true
        STOP=true
        ;;
        *)
                # unknown option
        ;;
    esac
    shift # past argument or value
done

# Check paths
FILES="$SOURCES_PATH $BUILD_PATH $INSTALL_PATH $TOOLCHAIN_PATH $SOURCES_PATH/CMakeLists.txt $TOOLCHAIN_PATH/toolchain-atom.cmake"

for FILE in ${FILES}; do
    if [ ! -f $FILE -a ! -d $FILE ]; then
        echo "ERROR: Path $FILE doesn't exist"
        HELP=true
        STOP=true
    fi
done

# Print help
if [ "$HELP" = true ] ; then
    echo "Usage: $PROGRAM --ip IP --sources-path PATH --build-path PATH --install-path PATH --ctc PATH"
fi

# Break script
if [ "$STOP" = true ] ; then
    exit 1
fi

# Build controller
cd $BUILD_PATH
cmake $SOURCES_PATH \
         -DCMAKE_TOOLCHAIN_FILE=$TOOLCHAIN_PATH/toolchain-atom.cmake \
         -DCMAKE_PREFIX_PATH=$TOOLCHAIN_PATH \
         -DCMAKE_INSTALL_PREFIX=$INSTALL_PATH \
          & make install \
          & ssh nao@$ROBOT_IP '[ -d naomech ] || mkdir ~/naomech' \
          & scp -r $INSTALL_PATH/* nao@$ROBOT_IP:~/naomech/