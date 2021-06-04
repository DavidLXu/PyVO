# PyVO
A Python Visual Odometry, supports RGB-D and Stereo dataset.

This project was greatly inspired by [14 lectures on visual SLAM](https://github.com/gaoxiang12/slambook) by Gao Xiang, without which this work wouldn't be possible.

3rd-party libraries in use: opencv, numpy, scipy, matplotlib, sophuspy.

To install opencv, numpy, scipy, and matplotlib:
```
pip install opencv-python
pip install numpy
pip install scipy
pip install matplotlib
```
To install sophuspy, go to [this website](https://github.com/craigstar/SophusPy).

Please begin with `run_vo_stereo.py`. Comment line 169-171 to disable realtime plotting for better performance.

Modify the parameters in `config.py` according to specific dataset.

视觉里程计毕业设计核心代码
