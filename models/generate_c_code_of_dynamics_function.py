#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Nov 19 16:06:29 2018

@author: huawei
"""

from pydy.codegen.cython_code import CythonMatrixGenerator
from sympy.physics.mechanics.functions import find_dynamicsymbols
from dynamics_modeling_4dof_sympy import PlanarCyclingModel4Segments
import sympy as sm
import sympy.physics.mechanics as me

sym_kwargs = {'positive': True, 'real': True}
me.dynamicsymbols._t = sm.symbols('t', **sym_kwargs)

sys = PlanarCyclingModel4Segments()
sys._setup_problem()
eoms_equ, kane= sys._generate_eoms()

q = kane.q[:]
u = kane.u[:]
udot = sys.time_varying_a
T = sys.time_varying_T
F = sys.time_varying_F
constants = sys.parameter_strings

g = CythonMatrixGenerator([q, u, udot, T, F, constants], [eoms_equ], prefix='Q_pydy_codegen')
setup_py, cython_src, c_header, c_src = g.doprint()
g.write()
g.compile()

dfdq = eoms_equ.jacobian(q)
g1 = CythonMatrixGenerator([q, u, udot, T, F, constants], [dfdq], prefix='dQdq_pydy_codegen')
g1.write()
g1.compile()

dfdu = eoms_equ.jacobian(u)
g2 = CythonMatrixGenerator([q, u, udot, T, F, constants], [dfdu], prefix='dQdu_pydy_codegen')
g2.write()
g2.compile()

dfda = eoms_equ.jacobian(udot)
g3 = CythonMatrixGenerator([q, u, udot, T, F, constants], [dfda], prefix='dQda_pydy_codegen')
g3.write()
g3.compile()

dfdf = eoms_equ.jacobian(list(F))
g5 = CythonMatrixGenerator([q, u, udot, T, F, constants], [dfdf], prefix='dQdf_pydy_codegen')
g5.write()
g5.compile()

# -------
