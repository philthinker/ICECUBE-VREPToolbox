# ICECUBE-VREPToolbox

Haopeng Hu

2019.07.22

A toolbox designed for V-REP remote API MATLAB applications.

v3.1.0

## Introduction

The ICECUBE-VREPToolbox is designed for acceleration of V-REP + MATLAB applications development. You DO NOT have to learn Lua language to make V-REP simulations tick! Some V-REP scenes are built exclusively for MATLAB remote operation that can be found in the repository.

- The toolbox has been tested on **MATLAB2018b** and **V-REP 3.6.2 EDU**.

- Functions in earlier version may not be supported.

## Install

- Copy the codes in the folder 'Lua' to the V-REP scene.

- For MATLAB users, clone the master branch anywhere you want. Run 'loadICECUBE.m' to trim the files based on your requirement.

- Always follow the sequence: **ICECUBE() -> ICECUBE.start() -> ICECUBE.stop() -> ICECUBE.delele()** whenever you use the toolbox.

## Demo

There are two demos as well as two V-REP scenes available:

 1. Open the "Demo\Scenes\UR5PickAndPlace.ttt" scene and run "Demo\HelloICECUBE_PickAndPlace.m".

 2. Open the "Demo\Scenes\UR5PegInHole.ttt" scene and run "Demo\HelloICECUBE_PegInHole.m".

### Rooibos goes well with nights of silence