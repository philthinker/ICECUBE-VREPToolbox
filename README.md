# ICECUBE-VREPToolbox

Haopeng Hu

2018.05.12

A toolbox designed for V-REP remote API (MATLAB) applications.

v2.0 beta (**It CANNOT WORK NOW** Turn to branch *ICECUBEv10-CoolSummer* for a stable version)

# Introduction

The ICECUBE-VREPToolbox is designed for acceleration of V-REP + MATLAB applications development. You DO NOT have to learn Lua language to make V-REP simulations tick! Some V-REP scenes are built exclusively for MATLAB remote operation that can be found in the repository.

# Attributes

 - The toolbox has been tested on MATLAB2016b together with V-REP PRO EDU 3.5.0 (Windows 10 x64).

 - Peter Corke's Robotics Toolbox is required for some functions (but not necessary).

 - Note that the ICECUBEv2.0 and ICECUBEv1.0 are NOT mutually compatible.

# Install

Just clone the master branch anywhere you want. Move 'ICECUBE_init.m' out of ICECUBE-VREPToolbox and trim the file. 

# Demo

 1. Open the "UR5plusRG2" scene.

 2. Run "Demos\HelloICECUBE.m".

 See 'Demos' for more demos.

# Attention

For most applications, you do not have to include the whole package into your project. What you may need are:

 - ICECUBE_VREPToolbox

 - ICECUBE_VREPToolbox\vrepTools

 - ICECUBE_VREPToolbox\robotTools\

 Sprite is our lucky beverage!