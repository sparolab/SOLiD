<div align="center">
  <h1>SOLiD</h1>
  <a href="https://github.com/sparolab/solid/tree/master/"><img src="https://img.shields.io/badge/-C++-blue?logo=cplusplus" /></a>
  <a href="https://github.com/sparolab/solid/tree/master"><img src="https://img.shields.io/badge/Python-3670A0?logo=python&logoColor=ffdd54" /></a>
  <a href="https://sparolab.github.io/research/solid/"><img src="https://github.com/sparolab/Joint_ID/blob/main/fig/badges/badge-website.svg" alt="Project" /></a>
  <a href="https://ieeexplore.ieee.org/abstract/document/10629042"><img src="https://img.shields.io/badge/Paper-PDF-yellow" alt="Paper" /></a>
  <a href="https://arxiv.org/abs/2408.07330"><img src="https://img.shields.io/badge/arXiv-2408.07330-b31b1b.svg?style=flat-square" alt="Arxiv" /></a>
  <a href="https://www.alphaxiv.org/abs/2408.07330"><img src="https://img.shields.io/badge/alphaXiv-2408.07330-darkred" alt="alphaXiv" /></a>
  <a href="https://www.youtube.com/watch?v=4sAWWfZTwLs"><img src="https://badges.aleen42.com/src/youtube.svg" alt="YouTube" /></a>
  <br />
  <br />

**[IEEE RA-L]** This repository is the official code for Narrowing your FOV with **SOLiD**: Spatially Organized and Lightweight Global Descriptor for FOV-constrained LiDAR Place Recognition.

  <a href="https://scholar.google.com/citations?user=t5UEbooAAAAJ&hl=ko" target="_blank">Hogyun Kim</a><sup></sup>,
  <a href="https://scholar.google.com/citations?user=wL8VdUMAAAAJ&hl=ko" target="_blank">Jiwon Choi</a><sup></sup>,
  <a href="https://scholar.google.com/citations?user=UPg-JuQAAAAJ&hl=ko" target="_blank">Taehu Sim</a><sup></sup>,
  <a href="https://scholar.google.com/citations?user=9mKOLX8AAAAJ&hl=ko" target="_blank">Giseop Kim</a><sup></sup>,
  <a href="https://scholar.google.com/citations?user=W5MOKWIAAAAJ&hl=ko" target="_blank">Younggun Cho</a><sup>†</sup>

**[Spatial AI and Robotics Lab (SPARO)](https://sites.google.com/view/sparo/%ED%99%88?authuser=0&pli=1)**

  <p align="center"><img src="fig/solid_example.png") alt="animated" width="75%" /></p>
  
</div>

## NEWS
* [November, 2024] Now, [Distributed-SOLiD-SLAM](https://github.com/sparolab/Distributed-SOLiD-SLAM.git) code is released!!
* [September, 2024] SOLiD is introduced in [HeLiPR-Place-Recognition Toolbox](https://github.com/minwoo0611/HeLiPR-Place-Recognition.git)!!
* [August, 2024] Now, the [SOLiD-A-LOAM](https://github.com/sparolab/SOLiD-A-LOAM.git) code is released!!
* [August, 2024] Now, the [SOLiD-PyICP-SLAM](https://github.com/sparolab/SOLiD-PyICP-SLAM) code is released!!
* [August, 2024] The SOLiD is added in [awesome-lidar-place-recognition](https://github.com/hogyun2/awesome-lidar-place-recognition)!!
* [August, 2024] Now, the SOLiD code is released!!
* [July, 2024] SOLiD is accepted in RA-L!!

## TODO
* [ ] Integrated with KISS-ICP: [SOLiD-KISS-ICP](https://github.com/sparolab/SOLiD-LOAM-LIVOX.git)
* [ ] Integrated with LIO-SAM: [SOLiD-LIO-SAM](https://github.com/sparolab/SOLiD-LIO-SAM)
* [ ] Integrated with FAST-LIO2: [SOLiD-FAST-LIO2](https://github.com/sparolab/SOLiD-FAST-LIO2)
* [ ] Integrated with LT-mapper

## Note
* SOLiD can be integrated with various LiDAR odometry including solid-state LiDAR and Situations (i.e. intra-session, inter-session, and multi-robot).
	* Intra-session SLAM 
		* Integrated with A-LOAM: [SOLiD-A-LOAM](https://github.com/sparolab/SOLiD-A-LOAM.git)
		* Integrated with a basic ICP odometry: [SOLiD-PyICP-SLAM](https://github.com/sparolab/SOLiD-PyICP-SLAM)
			* This implementation is fully Python-based so slow but for educational purposes.
   	* Inter-session SLAM
   		*  (TBD) Integrated with LT-mapper
	* Multi-Robot SLAM
		* Integrated with DiSCo SLAM: [Distributed-SOLiD-SLAM](https://github.com/sparolab/Distributed-SOLiD-SLAM.git)

## What are the problems with traditional LiDAR Place Recognition?
* The traditional method uses a bird eye view and overlooks vertical information.
* Also, because it focuses on performance, it is difficult to apply in real-time on an onboard computer.
  <p align="center"><img src="fig/problem.png") alt="animated" width="50%" /></p>

## What is the SOLiD?
* SOLiD is a lightweight and fast LiDAR global descriptor for FOV constraints situations that are limited through fusion with other sensors or blocked by robot/sensor operators including mechanical components or solid-state LiDAR (e.g. Livox).
  <p align="center"><img src="fig/method.png") alt="animated" width="75%" /></p>

## How to use the SOLiD?
* Python version
	* If you use the other LiDAR sensor, you modify the parameters in parser of test.py and lidar file structure in utils/point_module.py.
	    ```
	    $ git clone https://github.com/sparolab/SOLiD.git
	    $ cd python
	    $ python3 test.py
	    ```

* Cpp version
	* If you use the other LiDAR sensor, you modify the parameters in include/solid_module.h.
	  ```
	    $ git clone https://github.com/sparolab/SOLiD.git
	    $ cd cpp
	    $ mkdir build
	    $ cd build
	    $ cmake ..
	    $ make
	    $ ./test_solid
	  ```

## Utils
* If you want to clip the points, you can use python/bin/point_clipper.py
  ```
    $ git clone https://github.com/sparolab/SOLiD.git
    $ cd python/bin/
    $ python3 point_clipper.py
  ```

## Supplementary
* [Arxiv](https://arxiv.org/abs/2408.07330#)
* [Paper](https://ieeexplore.ieee.org/abstract/document/10629042)
* [Video](https://www.youtube.com/watch?v=4sAWWfZTwLs)
* [Project page](https://sites.google.com/view/lidar-solid)

## Main Contribution
* [Hogyun Kim](https://scholar.google.com/citations?user=t5UEbooAAAAJ&hl=ko)
* [Jiwon Choi](https://scholar.google.com/citations?user=wL8VdUMAAAAJ&hl=ko)
* [Taehu Sim](https://scholar.google.com/citations?user=UPg-JuQAAAAJ&hl=ko)
* [Giseop Kim](https://scholar.google.com/citations?user=9mKOLX8AAAAJ&hl=ko)
* [Younggun Cho](https://scholar.google.com/citations?user=W5MOKWIAAAAJ&hl=ko)

## QnA
* If you have a question, you utilize a [alphaXiv](https://www.alphaxiv.org/abs/2408.07330) and comment here.
  
## Citation
  ```
	@article{kim2024narrowing,
	  title={Narrowing your FOV with SOLiD: Spatially Organized and Lightweight Global Descriptor for FOV-constrained LiDAR Place Recognition},
	  author={Kim, Hogyun and Choi, Jiwon and Sim, Taehu and Kim, Giseop and Cho, Younggun},
	  journal={IEEE Robotics and Automation Letters},
	  year={2024},
	  publisher={IEEE}
	}
  ```


## Contact
* Hogyun Kim (hg.kim@inha.edu)

## License
* For academic usage, the code is released under the BSD 3.0 license. For any commercial purpose, please contact the authors.
  
## Contributors 
<a href="https://github.com/sparolab/SOLiD/graphs/contributors">
  <img src="https://contrib.rocks/image?repo=sparolab/SOLiD" />
</a>
