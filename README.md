# ICECUBE-VREPToolbox

Haopeng Hu

2018.05.12

A toolbox designed for V-REP remote API MATLAB applications.

v2.3.0 beta - 2018.06.30 updated

## Introduction

The ICECUBE-VREPToolbox is designed for acceleration of V-REP + MATLAB applications development. You DO NOT have to learn Lua language to make V-REP simulations tick! Some V-REP scenes are built exclusively for MATLAB remote operation that can be found in the repository.

- The toolbox has been tested on **MATLAB2016b**.

- Note that the ICECUBEv2.X and ICECUBEv1.0 are NOT mutually compatible.

- Some of the functions in ICECUBE v2.1 are not supported.

- It supports all the system APIs in ICECUBE v2.2.

## Install

- Copy the codes in the folder 'Lua' to the V-REP scene.

- For MATLAB users, clone the master branch anywhere you want. Move 'loadICECUBE.m' out of ICECUBE-VREPToolbox and trim the file based on your requirement.

- For ICECUBE v2.2 users, make sure to follow the sequence: **ICECUBE_init -> ICECUBE_start -> ICECUBE_stop -> ICECUBE_delete** whenever you use the toolbox.

- For ICECUBE v2.3 users, follow the sequence: **ICECUBE() -> ICECUBE.start() -> ICECUBE.stop() -> ICECUBE.delele()** whenever you use the toolbox.

- Do not mix the ICECUBE v2.2 APIs and ICECUBE v2.3 APIs.

## Demo

 1. Open the "PickAndPlace_v2.ttt" scene.

 2. Run "Demos\HelloICECUBE.m".

 Refer to 'Demos\' for more demos.

 *No sprite, no summer!*