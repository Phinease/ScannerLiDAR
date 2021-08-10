# ScannerLiDAR

**ScannerLiDAR is a scanner of Intel Realsense Lidar based on Realsense SDK examples. It offers a GUI of variable functions like export PLY, record, toggle color source.
For further post-processing, some scripts are provided to review the output, ICP registration, refinement and conversion to other formats.
More commercial softwares are recommended in the end.**

## Contents

* [Getting Started](#Getting-Started)
* [Usage](#Usage)
* [Conversion to DSM\DEM or LAS](#Conversion-to-DSMDEM-or-LAS)
## Getting Started

### Hardware
You may need a decent GPU for realtime processing and a high-speed disk for recording. Windows is the preferable platform, but linux is equaly runnable.

### Prerequisites
Make sure you have installed all of the following prerequisites on your development machine:
* Anaconda env with python 3.8 (any other versions might NOT be compatible with open3d)
* [Realsense SDK](https://github.com/IntelRealSense/librealsense/releases) (The documentary of official website is outdated, please check github repository)
* pyrealsense2
```
pip install pyrealsense2
```
* pyglet
```
pip install --upgrade --user pyglet
```

* opencv-python
```
pip install opencv-python
```
* open3d
```
pip install open3d
```

## Usage

### ScanColorGPU or ScanColor
```
python ScanColorGPU.py 
```
Mouse:
```
    Drag with left button to rotate around pivot (thick small axes),
    with right button to translate and the wheel to zoom.
```

Keyboard:

| Key    | Function                                                    |
| ------ | ----------------------------------------------------------- |
| p      | Pause                                                       |
| r      | Reset View                                                  |
| d      | Cycle through decimation values                             |
| z      | Toggle point scaling                                        |
| x      | Toggle point distance attenuation                           |
| c      | Toggle color source                                         |
| l      | Toggle lighting                                             |
| f      | Toggle depth post-processing                                |
| s      | Save PNG (./outX.png)                                       |
| e      | Export points to ply (./outX.ply) and Save PNG (./outX.png) |
| espace | start/stop record (./name.bag)                              |
| q/ESC  | Quit                                                        |

#### Notes:

Using deprecated OpenGL (FFP lighting, matrix stack...) however, draw calls
are kept low with pyglet.graphics.* which uses glDrawArrays internally.

Normals calculation is done with numpy on CPU which is rather slow, should really
be done with shaders but was omitted for several reasons - brevity, for lowering
dependencies (pyglet doesn't ship with shader support & recommends pyshaders)
and for reference.

### ViewAll
```
python ViewAll.py ./PATH_TO_DIR
```
View all ply files of the directory, one by one first, then merged together.

### ViewOneByOne
```
python ViewOneByOne.py ./PATH_TO_DIR
```
View all ply files of the directory, one by one.

### Refinement
```
python Refinement.py ./PATH_TO_PLY_FILE
```
View original file first then apply the filter(voxel_down_sample, uniform_down_sample, remove_statistical_outlier, remove_radius_outlier).

### CombinePointCloud
```
python CombinePointCloud.py ./PATH_TO_DIR
...
Enter the name:
Saved
```
Use ICP algorithme in open3d to align point clouds together and save it in the name entered.

### BagReader
```
python BagReader.py ./PATH_TO_BAG_FILE
```
BagReader can read bag file frame by frame for futher functionality.

## Conversion to DSM\DEM or LAS

### Non-commercial software
* [LAStools](https://rapidlasso.com/) for conversion between PLY and LAS, LAS and DEM
* [Realsense Tools](https://github.com/IntelRealSense/librealsense/tree/master/tools)
* [CloudCompare](https://www.danielgm.net/cc/)


### Commercial software
* [LiDar360](https://greenvalleyintl.com/?LiDAR360/) for transformation, ICP registration, PLY to DSM\DEM directly, classify ground points
