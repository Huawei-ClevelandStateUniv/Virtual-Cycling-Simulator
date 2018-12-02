#!/bin/bash

# Author: Huawei Wang
# Complie c code model
# Date: 11-20-2018


python human_exoskeleton_model_generate_c_code.py
python Q_pydy_codegen_setup.py build_ext --inplace
python dQdq_pydy_codegen_setup.py build_ext --inplace
python dQdqd_pydy_codegen_setup.py build_ext --inplace
python dQdqdd_pydy_codegen_setup.py build_ext --inplace
python dQdf_pydy_codegen_setup.py build_ext --inplace
