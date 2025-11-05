
. ./variables.sh

$NS3_SOURCE_DIR/ns3 configure -G Ninja \
  --enable-examples \
  --enable-modules='core;network;internet;mobility;lte;propagation;spectrum;buildings;config-store;netanim;'\
  --disable-modules='' \
  --disable-tests --disable-gsl --disable-mpi \

$NS3_SOURCE_DIR/ns3 build
