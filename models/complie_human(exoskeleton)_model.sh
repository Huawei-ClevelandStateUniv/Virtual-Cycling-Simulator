#!/bin/bash

# Author: Huawei Wang
# Complie c code model
# Date: 11-20-2018

python Q_pydy_codegen_setup.py build_ext --inplace
python dQdq_pydy_codegen_setup.py build_ext --inplace
python dQdu_pydy_codegen_setup.py build_ext --inplace
python dQda_pydy_codegen_setup.py build_ext --inplace
python dQdf_pydy_codegen_setup.py build_ext --inplace
