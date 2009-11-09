# An OpenSteer CUDA port #

This is the source code of a [CUDA](http://www.nvidia.com/object/cuda_home.html)-enhanced version of [OpenSteer](http://opensteer.sf.net).

## Requirements ##

* Install CUDA 2.3 drivers, tools and SDK

* In /Developer/GPU Computing/C run `make i386=1 x86_64=0`

* Make sure to run the Application with the `LD_LIBRARY_PATH` environment variable set to `/usr/local/cuda/lib`

## Important notes ##

* As I don't have any Windows or Linux computers I wasn't able to update the project files for these two platforms. So far, only the **Xcode project files for Mac OS X are working!**

* This project is part of my bachelor's thesis which can be found [here](http://www.stiefels.net/wp-content/uploads/2009/08/Parallelizing-the-OpenSteer-toolkit-using-GPUs.pdf).

* The *nvcc* compiler is expected to live in /usr/local/cuda/bin

* All \*.cu files are automatically compiled using *nvcc* (defined by build rule).
