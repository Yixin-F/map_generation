rm -rf build
sleep 3


mkdir build
cd build
sleep 3
cmake ..
sleep 3
make 
./map surround.pcd 1.pcd 1 0 0 0 2.pcd 0 1 0 0 3.pcd 0 0 1 0 4.pcd 0 0 0 1 5.pcd 1 1 1 1 


