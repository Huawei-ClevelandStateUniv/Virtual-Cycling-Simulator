## Manually compile IPOPT and Cyipopt

Before start, Anaconda (https://www.anaconda.com/download/#linux) is recommended be installed. Also make sure you installed the following packages.

These packages can be installed through ''conda install <package name>'' if you installed Anaconda first.

pkg-config

numpy

scipy

cython

six

future

wget

This whole install process is followed Matthias's Cyipopt documentation: https://github.com/matthias-k/cyipopt

 
First Install Ipopt

Download ipopt package that you want to installed from this address https://www.coin-or.org/download/source/Ipopt/.

Extract ipopt and put it at home dictionary (recommended) , and download the external code using 'wget'

 

export IPOPTDIR='path to ipopt'/Ipopt-3.12.4/
cd $IPOPTDIR/ThirdParty/Blas \
./get.Blas
cd ../Lapack && ./get.Lapack
cd ../ASL && ./get.ASL

cd ../Mumps && ./get.Mumps

cd ../Metis && ./get.Metis

HSL can only be get from office website (http://www.hsl.rl.ac.uk/ipopt/), academic license is free but needs about 1 day to process.

Here is the HSL source code that I downloaded. You can directly use it if you want.

Extract HSL source code after you get it. Rename the extracted file to 'coinhsl' and copy it in the HSL folder inside Ipopt-**/ThirdParty.

Then Install Ipopt using ./configure, make, and install commands. Please check this web page as reference https://www.coin-or.org/Ipopt/documentation/node14.html

Type in the following command in your terminal:
cd $IPOPTDIR/build
$IPOPTDIR/configure --prefix=/opt/ipopt/
make
make test

make install

Set environment path:

export IPOPTPATH="/home/huawei/Ipopt-3.12.10/build"

export PKG_CONFIG_PATH=$PKG_CONFIG_PATH:$IPOPTPATH/lib/pkgconfig

export PATH=$PATH:$IPOPTPATH/bin

Get help from this web-page if you got error at this step. https://stackoverflow.com/questions/13428910/how-to-set-the-environmental-variable-ld-library-path-in-linux
Connect ipopt with python using cyipopt

Download cyipopt-0.1.7 from this github: https://github.com/matthias-k/cyipopt

Extract cyipopt-0.1.7 and put it to home dictionary.

Write following commands in terminal:

 cd cyipopt-0.1.7

python setup.py install
if everthing works well you can run the test code now
python -c "import ipopt"

python examplehs071.py
cd test

Caution 1: Python may cannot find Ipopt while loading it. This can be solved by define LD_LIBRARY_PATH

export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/huawei/Ipopt-3.12.10/build/lib

Caution 2: Ipopt may cannot load MA27, MA57, MA86 solvers in HSL package. This can be solved by link libcoinhsl.so and libhsl.so.

ln libcoinhsl.so libhsl.so

Finally, you will see the optimization results shown in command window: Optimization been successfully solved.

