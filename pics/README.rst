Manually compile IPOPT and Cyipopt (On Ubuntu 18.04, python 2.7.15)
-------------------------------------

Before start, make sure ``pip`` is installed. If not, please install it.

    ``sudo apt install python-pip``

Then use ``pip`` install the following packages:

    $ pip2 install pkgconfig numpy scipy cython six future wget
    
    
* Compile Ipopt

1. Download ipopt source code, choose the verison that you would like to have from this page <https://www.coin-or.org/download/source/Ipopt/>.

Here I used ``Ipopt-3.12.11``

Extract ipopt and put it in the floder you want (I put it at ``Home`` folder).

Create the Ipopt directory: ``export IPOPTDIR=/home/huawei/Ipopt-3.12.11``

2. Download external packages using ``wget``::

    cd $IPOPTDIR/ThirdParty/Blas
    ./get.Blas
    cd ../Lapack && ./get.Lapack
    cd ../ASL && ./get.ASL
    cd ../Mumps && ./get.Mumps
    cd ../Metis && ./get.Metis

To use ``ma27, ma57, ma86`` solvers, ``HSL`` package are needed. ``HSL`` can be get from its official website <http://www.hsl.rl.ac.uk/ipopt/>, academic license is free.

Extract ''HSL'' source code after you get it. Rename the extracted folder to ``coinhsl`` and copy it in the HSL folder: ``Ipopt-3.12.11/ThirdParty/HSL``

3. Install Ipopt using ``./configure, make, make install`` commands::
    
    $ mkdir $IPOPTDIR/build
    $ cd $IPOPTDIR/build 
    $ IPOPTDIR/configure
    $ make 
    $ make test 
    $ make install

I used the ipopt document as reference, and it descrips the compile process in a very detail way. 
If you get errors in this process, please check it <https://www.coin-or.org/Ipopt/documentation/node14.html>

4. Set environment path::

    $ export IPOPTPATH="/home/huawei/Ipopt-3.12.11/build" # make sure this one starts with the Home path
    $ export PKG_CONFIG_PATH=$PKG_CONFIG_PATH:$IPOPTPATH/lib/pkgconfig 
    $ export PATH=$PATH:$IPOPTPATH/bin

Get help from this web-page if you get errors in setting environments. <https://stackoverflow.com/questions/13428910/how-to-set-the-environmental-variable-ld-library-path-in-linux>


2. Compile cyipopt

1. Download ``cyipopt`` source code from this github: <https://github.com/matthias-k/cyipopt>

Extract cyipopt and put it at the place you want (I put it at Home directory).

Change the path of terminal to cyipopt: ``cd ~/cyipopt``

2. check that everything linked correctly with ``ldd`` ::

    $ ldd build/lib.linux-x86_64-2.7/cyipopt.so
    linux-vdso.so.1 (0x00007ffe895e1000)
    libipopt.so.1 => /home/huawei/Ipopt-3.12.11/build/lib/libipopt.so.1 (0x00007f74efc2a000)
    libc.so.6 => /lib/x86_64-linux-gnu/libc.so.6 (0x00007f74ef839000)
    libcoinmumps.so.1 => /home/huawei/Ipopt-3.12.11/build/lib/libcoinmumps.so.1 (0x00007f74ef4ae000)
    libcoinhsl.so.1 => /home/huawei/Ipopt-3.12.11/build/lib/libcoinhsl.so.1 (0x00007f74ef169000)
    liblapack.so.3 => /usr/lib/x86_64-linux-gnu/liblapack.so.3 (0x00007f74ee8cb000)
    libblas.so.3 => /usr/lib/x86_64-linux-gnu/libblas.so.3 (0x00007f74ee65e000)
    libdl.so.2 => /lib/x86_64-linux-gnu/libdl.so.2 (0x00007f74ee45a000)
    libstdc++.so.6 => /usr/lib/x86_64-linux-gnu/libstdc++.so.6 (0x00007f74ee0d1000)
    libm.so.6 => /lib/x86_64-linux-gnu/libm.so.6 (0x00007f74edd33000)
    /lib64/ld-linux-x86-64.so.2 (0x00007f74f02c0000)
    libgcc_s.so.1 => /lib/x86_64-linux-gnu/libgcc_s.so.1 (0x00007f74edb1b000)
    libcoinmetis.so.1 => /home/huawei/Ipopt-3.12.11/build/lib/libcoinmetis.so.1 (0x00007f74ed8ca000)
    libgfortran.so.4 => /usr/lib/x86_64-linux-gnu/libgfortran.so.4 (0x00007f74ed4eb000)


3. Compile cyipopt using the command: ``python setup.py install``

If there is no error, then you have compiled ``cyipopt`` succefully 

Before try the test code, add Ipopt ``lib`` path to ``LD_LIBRARY_PATH``::

    $ export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/huawei/Ipopt-3.12.11/build/lib

To make this path works for all terminal, it can be added to ``.bashrc`` ::

    $ echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/your/custom/path/' >> ~/.bashrc

4. Now, you can run the test code::

    $ cd test
    $ python -c "import ipopt"
    $ python examplehs071.py

If it could be run successfully, the optimization will start with the following descriptions::

    ******************************************************************************
    This program contains Ipopt, a library for large-scale nonlinear optimization.
     Ipopt is released as open source code under the Eclipse Public License (EPL).
             For more information visit http://projects.coin-or.org/Ipopt
    ******************************************************************************

    This is Ipopt version 3.12.11, running with linear solver ma27.

        
   
