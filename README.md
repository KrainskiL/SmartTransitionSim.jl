# SmartTransitionSim.jl
Smart cars transition multi-agent simulator created in Julia

**Documentation**

[![](https://img.shields.io/badge/docs-latest-blue.svg)](https://krainskil.github.io/SmartTransitionSim.jl/latest)

**Build status**

[![Build Status](https://travis-ci.org/KrainskiL/SmartTransitionSim.jl.svg?branch=master)](https://travis-ci.org/KrainskiL/SmartTransitionSim.jl)
[![codecov](https://img.shields.io/codecov/c/gh/KrainskiL/SmartTransitionSim.jl.svg)](https://codecov.io/gh/KrainskiL/SmartTransitionSim.jl)


This is a repository containig a simulation model accompying the paper:

**Multi-agent routing simulation with partial smart vehicles penetration**

*by Bogumił Kamiński, Łukasz Kraiński, Atefeh (Atty) Mashatan Paweł Prałat and Przemyslaw Szufel*

*submitted the Journal of Advanced Transportation*


## Agent Based Simulation Framework for modelling transport systems  with partial smart vehicles penetration**


The framework was optimized in terms of performance using. Major performance tweaks include:
- Yen's algorithm is based on custom, fast A-star implementation---5 times performance improvement over a standard implementation, 
- routes calculated by the $k$-shortest paths algorithm are saved for future re-use (memoization technique)---leading to up to 15 times faster simulation execution in comparison to no-memoization,
- simulations use common, separately generated agents pools---halved overall running time.

We have designed the simulation tool in such a way that the simulations can be executed in a distributed fashion. Additionally the simulation model has been adjusted to work with KissCluster software [KissCluster](https://github.com/pszufe/KissCluster) that can be used to manage the distributed simulation execution and the data collection process in the Amazon Web Service cloud.


In order to run the simulation please run the following julia commnads:

```julia
using Pkg
Pkg.add(PackageSpec(url="https://github.com/KrainskiL/SmartTransitionSim.jl"))
```

Once the simulation package with its dependencies is installed get the run_sweep_v4_NoSerial-1.jl (available in the `sweep` project subfolder) to actually run the simulations.
In order to to pararelize the simulation over a computational cluster you need to use external software. I order to start simulation run the command:
```bash
julia run_sweep_v4_NoSerial-1.jl 1
```
where `1` is the value that will be parsed as the sweep parameter

