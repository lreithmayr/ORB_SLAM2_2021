# Modification of [ORB SLAM 2](https://github.com/raulmur/ORB_SLAM2) by Mur-Artal et. al.

* **Concurrent construction** of and localization in grid map in **seperate thread**.   
  * Adapting the grid mapping algorithm by [Singh and Jahani](https://github.com/abhineet123/ORB_SLAM2) to run in a fourth thread of OS2 and publish a ROS OccupancyGrid message.

**Changelog and Additions:**

* Tested under **Ubuntu 20.04**
* PR #970 implemented: Support for **OpenCV 4** (tested with OpenCV 4.5.4)
* Support for **C++11 and higher** implemented (tested with GCC 9.3.0)
  * **Pangolin v0.6** now supported
  * **Eigen 3.4** supported
* **Camera calibration** using the ROS calibration toolbox 
* **Map saving and loading** thanks to [Alkaid Benetnash]("https://github.com/Alkaid-Benetnash/ORB_SLAM2")

## Related Publications:

[Monocular] Raúl Mur-Artal, J. M. M. Montiel and Juan D. Tardós. **ORB-SLAM: A Versatile and Accurate Monocular SLAM System**. *IEEE Transactions on Robotics,* vol. 31, no. 5, pp. 1147-1163, 2015. (**2015 IEEE Transactions on Robotics Best Paper Award**). **[PDF](http://webdiis.unizar.es/~raulmur/MurMontielTardosTRO15.pdf)**.

[Stereo and RGB-D] Raúl Mur-Artal and Juan D. Tardós. **ORB-SLAM2: an Open-Source SLAM System for Monocular, Stereo and RGB-D Cameras**. *IEEE Transactions on Robotics,* vol. 33, no. 5, pp. 1255-1262, 2017. **[PDF](https://128.84.21.199/pdf/1610.06475.pdf)**.

[DBoW2 Place Recognizer] Dorian Gálvez-López and Juan D. Tardós. **Bags of Binary Words for Fast Place Recognition in Image Sequences**. *IEEE Transactions on Robotics,* vol. 28, no. 5, pp.  1188-1197, 2012. **[PDF](http://doriangalvez.com/php/dl.php?dlp=GalvezTRO12.pdf)**
