cd build
rm -rf *
cmake ..
cmake --build /mnt/f/g101/build --config Release --target all -- -j 10
cd ..
./build/Rasterizer

