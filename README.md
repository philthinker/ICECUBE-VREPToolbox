# ICECUBE-VREPToolbox

Haopeng Hu

2019.12.04

A toolbox designed for CoppeliaSim remote API MATLAB applications.

v4.0.0
 
## Introduction

The ICECUBE-VREPToolbox is designed for acceleration of V-REP + MATLAB applications development. You DO NOT have to learn Lua language to make V-REP simulations tick! Some CoppeliaSim scenes are built exclusively for MATLAB remote operation that can be found in the repository.

- The toolbox has been tested on **MATLAB2019b** and **CoppeliaSim 4.0.0 EDU**.

## Install

- Copy the codes in the folder 'Lua' to the CoppeliaSim scene.

- For MATLAB users, clone the master branch anywhere you want. Run 'loadICECUBE.m' to trim the files based on your requirement.

- Always follow the sequence: **ICECUBE() -> ICECUBE.start() -> ICECUBE.stop() -> ICECUBE.delele()** whenever you use the toolbox.

## Demos

There are two demos as well as two CoppeliaSim scenes available:

 1. Open the "Demo\Scenes\UR5PickAndPlace.ttt" scene and run "Demo\HelloICECUBE_PickAndPlace.m".

 2. Open the "Demo\Scenes\UR5PegInHole.ttt" scene and run "Demo\HelloICECUBE_PegInHole.m".

### Let's enjoy some Sushi with Sake!