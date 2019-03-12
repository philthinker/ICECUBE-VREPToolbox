# ICECUBE-VREPToolbox

Haopeng Hu

2019.03.09

A toolbox designed for V-REP remote API MATLAB applications.

v2.4.1

## Introduction

The ICECUBE-VREPToolbox is designed for acceleration of V-REP + MATLAB applications development. You DO NOT have to learn Lua language to make V-REP simulations tick! Some V-REP scenes are built exclusively for MATLAB remote operation that can be found in the repository.

- The toolbox has been tested on **MATLAB2016b**.

- Functions in earlier ICECUBE are not supported.

## Install

- Copy the codes in the folder 'Lua' to the V-REP scene.

- For MATLAB users, clone the master branch anywhere you want. Move 'loadICECUBE.m' out of ICECUBE-VREPToolbox and trim the file based on your requirement.

- For ICECUBE v2.3 users, follow the sequence: **ICECUBE() -> ICECUBE.start() -> ICECUBE.stop() -> ICECUBE.delele()** whenever you use the toolbox.


## Demo

There are two demos as well as two V-REP scenes available:

 1. Open the "UR5PickAndPlace.ttt" scene and run "Demo\HelloICECUBE_PickAndPlace.m".

 2. Open the "UR5PegInHole.ttt" scene and run "Demo\HelloICECUBE_PegInHole.m".

 3. Run "magicTools\MovementPrimitive\DMP_demo.m" the learn the use of @DMP

 *No sprite, no summer!*