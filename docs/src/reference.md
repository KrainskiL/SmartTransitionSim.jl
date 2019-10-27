Reference
=========

```@meta
CurrentModule = RSUOptimization
DocTestSetup = quote
    using SmartTransitionSim
end
```

Agent generation
----------------------
```@docs
Rect
Agent
pick_random_node
get_nodes_set
generate_agents
```

Rerouting
----------------------
```@docs
k_shortest_path_rerouting!
yen_a_star
```

Traffic model
----------------------
```@docs
get_max_densities
update_weights!
update_weights_and_events!
traffic_constants
init_traffic_variables
next_edge
update_event_agent!
```

Simulation
-----------------------
```@docs
simulation_run
gather_statistics
```
