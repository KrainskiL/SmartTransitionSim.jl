Reference
=========

```@meta
CurrentModule = RSUOptimization
DocTestSetup = quote
    using RSUOptimization
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

RSU location optimization
----------------------
```@docs
RSU
get_agent_coordinates
calculate_RSU_location
adjust_RSU_availability!
adjust_RSU_utilization!
gather_statistics
```

Rerouting
----------------------
```@docs
k_shortest_path_rerouting!
```

Vehicular communication
----------------------
```@docs
send_weights_update
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
