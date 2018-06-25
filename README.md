# ICECUBE-VREPToolbox

Haopeng Hu

2018.05.12

A toolbox designed for V-REP remote API (MATLAB & Python) applications.

v2.2 beta - 2018.06.25 updated

## Introduction

The ICECUBE-VREPToolbox is designed for acceleration of V-REP + MATLAB applications development. You DO NOT have to learn Lua language to make V-REP simulations tick! Some V-REP scenes are built exclusively for MATLAB remote operation that can be found in the repository.

- The toolbox has been tested on **MATLAB2016b** together with **V-REP PRO EDU 3.5.0** (Windows 10 x64).

- Peter Corke's Robotics Toolbox is required for some functions (but not necessary).

- Note that the ICECUBEv2.X and ICECUBEv1.0 are NOT mutually compatible.

- The functions of Python APIs are limited now. They are tested on **Python 3.6**.

- Some of the functions in ICECUBE v2.1 are not supported.

## Install

- Copy the codes in the folder 'Lua' to the V-REP scene.

- For MATLAB users, clone the master branch anywhere you want. Move 'loadICECUBE.m' out of ICECUBE-VREPToolbox and trim the file based on your requirement.

- For Python users, what you need is just the folder "Python".

## Demo

 1. Open the "PickAndPlace_v2.ttt" scene.

 2. Run "Demos\HelloICECUBE.m".

 Refer to 'Demos\' for more demos.

 *No sprite, no summer!*