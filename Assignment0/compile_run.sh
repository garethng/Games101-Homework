mkdir -p ./build
pushd ./build
cmake ..
make
echo "***************************************************************************************"
./Transformation
popd
