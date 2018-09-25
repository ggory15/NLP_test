FROM eur0c.laas.fr:5000/gepetto/buildfarm/robotpkg:16.04

RUN apt-get update -qqy \
 && apt-get install -qqy \
    libyaml-cpp-dev \
    robotpkg-qpoases \
 && rm -rf /var/lib/apt/lists/*
