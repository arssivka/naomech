#!/bin/sh
# You should run it in nao ctc directory

for i in acl alsa archive attr avahi boost bzip2 c_ares cross curl dbus dbus-glib dl eigen3 ffi ffmpeg flac fuse-2.8.6 glib2 gstreamer gstreamer-farsight iconv jpeg libevent libftdi1 liblttng_ust libnaoqi liburcu lircclient lttng_tools octomap ogg opencv openni2 openssl pam png pthread pulseaudio python qt rt samplerate sndfile soundtouch tbb telepathy-farsight telepathy-glib tiff udev urg usb usb_1 v4l vorbis xml2 xz_utils zbar zlib
do
    rsync -avh $i/* ./
    rm -r $i
done

mv ./include/boost-1_55/* ./include
rmdir ./include/boost-1_55

find ./lib/pkgconfig/ -type f -exec sed -i 's:/usr:/opt/ctc:g' {} +
find ./share/cmake/ -name "*.cmake" -type f -exec sed -i 's:${_boost_root}/include/boost-1_55:${_boost_root}/include/:g' {} +
sed -i 's:set(ALDE_CTC_CROSS   "${CMAKE_CURRENT_LIST_DIR}/cross"):set(ALDE_CTC_CROSS   "${CMAKE_CURRENT_LIST_DIR}"):g' ./cross-config.cmake
