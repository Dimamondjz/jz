# install osqp
cd osqp
mkdir build
cd build
cmake -G "Unix Makefiles" ..
cmake --build .
sudo cmake --build . --target install
cd ../..
cd yaml-cpp
mkdir build
cd build
cmake ..
make
sudo make install