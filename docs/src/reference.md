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
generate_agents
```

Rerouting
----------------------
```@docs
k_shortest_path_rerouting!
```

Traffic model
----------------------
```@docs
get_max_densities
update_weights!
traffic_constants
init_traffic_variables
next_edge
update_event_agent!
update_agents_position!
```
