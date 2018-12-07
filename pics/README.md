## Manually compile IPOPT and Cyipopt

Before start, make sure the pip is installed. If not, please use '' sudo apt install python-pip''.

Then use ''pip'' install the following packages:

''pip2 install pkgconfig numpy scipy cython six future wget''

 
### First Compile Ipopt

1. Download ipopt verison that you would like to have from this page https://www.coin-or.org/download/source/Ipopt/. Here I choiced Ipopt-12.11

    Extract ipopt and put it at home dictionary (recommended) .

2. Download the external code using 'wget', example code is as follow:

    Create the Ipopt directionary using the following code:
    ''export IPOPTDIR=Path to ipopt/Ipopt-3.12.4''

    Get the third party solvers for ipopt using wget:

    ''cd $IPOPTDIR/ThirdParty/Blas
    ./get.Blas
    cd ../Lapack && ./get.Lapack
    cd ../ASL && ./get.ASL
    cd ../Mumps && ./get.Mumps
    cd ../Metis && ./get.Metis''

    In order to use ma27, ma57, ma86 solvers, HSL source are needed. HSL can only be get from office website (http://www.hsl.rl.ac.uk/ipopt/), academic license is free but needs about 1 day to process.

    Extract HSL source code after you get it. Rename the extracted file to 'coinhsl' and copy it in the HSL folder inside Ipopt-   
    3.**.**/ThirdParty/HSL.

3. Install Ipopt using ./configure, make, and install commands. 

    I used ipopt documents as reference, if you got an error in this process, please check here https://www.coin-or.org/Ipopt/documentation/node14.html

    Type in the following command in your terminal:
    cd $IPOPTDIR/build
    $IPOPTDIR/configure --prefix=/opt/ipopt/
    make
    make test
    make install

    Set environment path:
    
    ''export IPOPTPATH="/home/huawei/Ipopt-3.12.11/build"  ''  make sure this one starts with root path
    
    ''export PKG_CONFIG_PATH=$PKG_CONFIG_PATH:$IPOPTPATH/lib/pkgconfig
    export PATH=$PATH:$IPOPTPATH/bin''

    Get help from this web-page if you got error at this step. https://stackoverflow.com/questions/13428910/how-to-set-the-environmental-variable-ld-library-path-in-linux

### Connect ipopt with python using cyipopt

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

