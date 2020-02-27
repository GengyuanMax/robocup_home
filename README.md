The metapackage **rhome_perception** has been tested with Ubuntu 16.04 and ROS Kinetic. It might work with other versions as well.

# Preparation

## Requirements

*  Flann (tested with 1.8.4)
*  PCL (tested with 1.9.1)
*  TensorFlowCC
*  OpenBlas

### tensorflow c++ API compiling
1. install Bazel (0.19.2 version tested) 
download proper version from https://github.com/bazelbuild/bazel/releases
```
sudo apt-get install pkg-config zip g++ zlib1g-dev unzip python
chmod +x bazel-<version>-installer-linux-x86_64.sh
./bazel-<version>-installer-linux-x86_64.sh --user
export PATH="$PATH:$HOME/bin"
```

2. clone tensorflow source codes
```
git clone --recursive git@github.com:tensorflow/tensorflow.git
cd tensorflow
git checkout r1.13    # tensorflow 1.13.1 tested 
./configure   	# configure installation options
```

3. Compiling with bazel
```
bazel build --config=monolithic --config=cuda //tensorflow:libtensorflow_cc.so	# compiling w/ CUDA
bazel build --config=monolithic //tensorflow:libtensorflow_cc.so	# compiling w/o CUDA
```

4. Compiling third-party libraries
```
cd tensorflow
cd tensorflow/contrib/makefile/build_all_linux.sh
```

5. Install
```
sudo cp bazel-bin/tensorflow/libtensorflow_*.so /usr/local/lib

sudo mkdir -p /usr/local/include/google/tensorflow
sudo cp -r tensorflow /usr/local/include/google/tensorflow
sudo find /usr/local/include/google/tensorflow/tensorflow -type f ! -name "*.h" -delete	#delete unnecessary files
 
sudo cp bazel-genfiles/tensorflow/core/framework/*.h /usr/local/include/google/tensorflow/core/framework
sudo cp bazel-genfiles/tensorflow/core/lib/core/*.h /usr/local/include/google/tensorflow/core/lib/core
sudo cp bazel-genfiles/tensorflow/core/protobuf/*.h /usr/local/include/google/tensorflow/core/protobuf
sudo cp bazel-genfiles/tensorflow/core/util/*.h /usr/local/include/google/tensorflow/core/util
sudo cp bazel-genfiles/tensorflow/core/ops/*.h /usr/local/include/google/tensorflow/core/ops
sudo cp -r third-party /usr/local/include/google/tensorflow/
sudo rm -r /usr/local/include/google/tensorflow/third-party/py
```

Also you can manually add the library path manually in CMakeLists and edit the corresponding lines
```
link_directories(path_to_tensorflow/tensorflow/bazel-bin/tensorflow)  # library path
include_directories(                                        # header file path
   path_to_tensorflow/
   path_to_tensorflow/bazel-genfiles
   path_to_tensorflow/bazel-bin/tensorflow
   path_to_tensorflow/tensorflow/contrib/makefile/downloads/absl
   /usr/local/include/eigen3
   )
 ```

## Deep learning pre-trained models
face_recognition https://drive.google.com/file/d/1R-bFuoGqbLHKUv-HoXIEe847mqwvHAnK/view?usp=sharing

emotion https://drive.google.com/file/d/1VhcPvf2lywXTVVmbj-sAuF4xqpTECCNg/view?usp=sharing

gender https://drive.google.com/file/d/1XUetizHgIbgTDBn7umNCE9SUEq75BlZY/view?usp=sharing

hand detection https://drive.google.com/file/d/1b-3YH-cXV4yAJdh-rxMT4VvKlTU7oH5_/view?usp=sharing

handpose classification https://drive.google.com/file/d/1oSuWNTdPdJ6SL7xGYlRRdF6DYbHMubl5/view?usp=sharing

## Help


