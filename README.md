# ICECUBE-VREPToolbox

Haopeng Hu

2018.04.12

A toolbox designed for V-REP remote API (MATLAB) applications.

# Introduction

The ICECUBE-VREPToolbox is designed for acceleration of V-REP + MATLAB applications development. You DO NOT have to learn Lua language to make V-REP simulations tick! Some V-REP scenes are built exclusively for MATLAB remote operation that can be found in the repository.

However, it is still under development.

# Attributes

 - The toolbox has been tested on MATLAB2016b together with V-REP PRO EDU 3.5.0 (Windows 10 x64).

 - Peter Corke's Robotics Toolbox is required for some functions (but not necessary).

# Install

1. Just clone the master branch anywhere you want.
2. Cut the file 'ICECUBE_init.m' out to your project's repository and modify it. 

# Demo

 1. Open the "UR5plusRG2" scene.

 2. Run "HelloICECUBE.m".

 See 'Demos' for more demos.

# Attention

For most applications, you do not have to include the whole package into your project. What you may need are:

 - ICECUBE_VREPToolbox

 - ICECUBE_VREPToolbox\vrepTools

 - ICECUBE_VREPToolbox\robotTools\

 Enjoy it for your cool summer!