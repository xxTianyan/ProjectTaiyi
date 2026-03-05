### How to build tiny VBD demo
```
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --target tiny_vbd_demo -j
./build/tiny_vbd_demo
```

The demo runs a hanging-cloth simulation and writes OBJ snapshots to `output/frame_XXXX.obj`.

### Optional: build legacy Taiyi app (renderer)
```
cmake -S . -B build -DBUILD_LEGACY_TAIYI=ON
cmake --build build --target Taiyi -j
```
