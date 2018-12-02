#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Nov 19 16:06:29 2018

This code is to generate c code of symbolic dynamic equations and derivatives.

@author: huawei
"""

from pydy.codegen.cython_code import CythonMatrixGenerator
from human_exoskeleton_modeling_sympy import PlanarCyclingModel4Segments
import sympy as sm
import sympy.physics.mechanics as me

sym_kwargs = {'positive': True, 'real': True}
me.dynamicsymbols._t = sm.symbols('t', **sym_kwargs)

# import code to generate symbolic dynamic equations
sys = PlanarCyclingModel4Segments()
sys._setup_problem()
eoms_equ, kane= sys._generate_eoms()

# load symbolic state parameters, specific parameters, and constants
q = kane.q[:]
u = kane.u[:]
udot = sys.time_varying_a
T = sys.time_varying_T
F = sys.time_varying_F
constants = sys.parameter_strings

# write symbolic dynamic euqations into c code as well as the python interface
g = CythonMatrixGenerator([q, u, udot, T, F, constants], [eoms_equ], prefix='Q_pydy_codegen')
setup_py, cython_src, c_header, c_src = g.doprint()
g.write()

# write the derivative of symbolic dynamic euqations with respect to joint angles into c code as well as the python interface
dfdq = eoms_equ.jacobian(q)
g1 = CythonMatrixGenerator([q, u, udot, T, F, constants], [dfdq], prefix='dQdq_pydy_codegen')
g1.write()

# write derivative of symbolic dynamic euqations with respect to joint velocities into c code as well as the python interface
dfdu = eoms_equ.jacobian(u)
g2 = CythonMatrixGenerator([q, u, udot, T, F, constants], [dfdu], prefix='dQdqd_pydy_codegen')
g2.write()

# write derivative of symbolic dynamic euqations with respect to joint accelerations into c code as well as the python interface
dfda = eoms_equ.jacobian(udot)
g3 = CythonMatrixGenerator([q, u, udot, T, F, constants], [dfda], prefix='dQdqdd_pydy_codegen')
g3.write()

# write derivative of symbolic dynamic euqations with respect to external force into c code as well as the python interface
dfdf = eoms_equ.jacobian(list(F))
g5 = CythonMatrixGenerator([q, u, udot, T, F, constants], [dfdf], prefix='dQdf_pydy_codegen')
g5.write()
