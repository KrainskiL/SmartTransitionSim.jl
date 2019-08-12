module SmartTransitionSim

#packages
using OpenStreetMapX
using StatsBase
using SparseArrays
using Base.Iterators
using LightGraphs
using DataFrames

#types
export Rect, Agent

#functions
#generate_agents.jl
export generate_agents, pick_random_node
#rerouting.jl
export k_shortest_path_rerouting!
#simulations.jl
export simulation_run, gather_statistics
#traffic_model.jl
export get_max_densities, traffic_constants, init_traffic_variables, next_edge
export update_weights!, update_event_agent!, update_agents_position!

#files
include("types.jl")
include("generate_agents.jl")
include("rerouting.jl")
include("simulations.jl")
include("traffic_model.jl")

end
