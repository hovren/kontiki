FROM ubuntu:bionic

RUN apt-get update  &&              \
    apt-get install -y              \
        python3-dev                 \
        python3-setuptools          \
        python3-wheel               \
        python3-pip                 \
        python3-pytest              \
        python3-h5py                \
        python3-scipy               \
        build-essential             \
        cmake                       \
        git                         \
        libceres-dev                \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /root/source &&                                                    \
    git clone https://github.com/strasdat/Sophus.git /root/source/Sophus &&     \
    cd /root/source/Sophus &&                                                   \
    git checkout 00f3fd91c153ef04 &&                                            \
    mkdir build && cd build &&                                                  \
    cmake .. -DBUILD_TESTS=OFF &&                                               \
    make && make install &&                                                     \
    rm -rf /root/source/


