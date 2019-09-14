using Test
using OpenStreetMapX
using SmartTransitionSim
using Random

test_map = OpenStreetMapX.get_map_data("reno_east3.osm", use_cache = false)
Rect1 = [Rect((39.50,-119.70),(39.55,-119.74))]
Rect2 = [Rect((39.50,-119.80),(39.55,-119.76))]
KDict = Dict{Tuple{Int,Int},Array{Vector{Int}}}()
AgentsSet = generate_agents(test_map,10,Rect1,Rect2, 0.5, 3, 1.0, KDict)

#generate_agents.jl
@testset "agents" begin

Random.seed!(0);
@test rand(Int) == -4635026124992869592

@test in(pick_random_node(test_map, Rect1, false), keys(test_map.nodes))
@test in(pick_random_node(test_map, [Rect1[1], Rect2[1]], false), keys(test_map.nodes))
@test typeof(pick_random_node(test_map, Rect1, true)) == Array{Int64,1}

@test all([in(x, keys(test_map.nodes)) for x in getfield.(AgentsSet,:start_node)])
@test sum(getfield.(AgentsSet,:smart)) == 5

end

#rerouting.jl
@testset "rerouting" begin

constantNode = AgentsSet[1].route[1]
speeds = OpenStreetMapX.get_velocities(test_map)
k_shortest_path_rerouting!(test_map, KDict, AgentsSet[1], speeds, 3, 1.0, 100)
@test AgentsSet[1].route[1] == constantNode

k_routes = yen_a_star(test_map.g,test_map.v[AgentsSet[1].route[1]],test_map.v[AgentsSet[1].route[end]],speeds,3)
@test length(k_routes.dists) == 3
end

#simulations.jl
@testset "simulations" begin

newAgents = generate_agents(test_map,10,Rect1,Rect2, 0.5, 3, 1.0, KDict)
output = simulation_run("base",test_map, newAgents)
@test length(output) == 3
@test typeof(output) == NamedTuple{(:Steps, :Simtime, :TravelTimes),Tuple{Int64,Float64,Array{Float64,1}}}

ITSOutput = simulation_run("smart",test_map, newAgents, 5.0, KDict, 100, 1.0, 3)

@test length(ITSOutput) == 3
@test typeof(ITSOutput) == NamedTuple{(:Steps, :Simtime, :TravelTimes),Tuple{Int64,Float64,Array{Float64,1}}}

output = simulation_run("base",test_map, AgentsSet)
stats = gather_statistics(getfield.(AgentsSet,:smart),
                    output.TravelTimes,
                    ITSOutput.TravelTimes)

@test length(stats) == 7
@test typeof(stats) == NamedTuple{(:overall_time, :smart_time, :other_time, :avg_base, :avg_overall_V2I, :avg_smart_V2I, :avg_regular_V2I),
                        Tuple{Float64,Float64,Float64,Float64,Float64,Float64,Float64}}
end

#traffic_model.jl
@testset "traffic_model" begin

max_dens = get_max_densities(test_map, 5.0)
max_speeds = OpenStreetMapX.get_velocities(test_map)
densities, speeds = init_traffic_variables(test_map, AgentsSet)
#get_max_densities tests
@test round(sum(max_dens);digits=3) == 206315.887

edge = rand(keys(densities))
update_weights!(speeds, densities, max_dens, max_speeds)
#update_weights! tests
@test speeds[edge[1],edge[2]] < max_speeds[edge[1],edge[2]] && speeds[487,1] == max_speeds[487,1]

#traffic_constants tests
max_d, max_s = traffic_constants(test_map, 5.0)
@test max_d == max_dens && max_s == max_speeds

#init_traffic_variables tests
i_densities, i_speeds = init_traffic_variables(test_map, AgentsSet)
@test sum(values(i_densities)) == 10
@test i_speeds == max_speeds

#next_edge tests
events = next_edge(AgentsSet, max_speeds, test_map.w)
@test typeof(events) == Vector{Float64}
event = next_edge(AgentsSet[1], max_speeds, test_map.w)
@test typeof(event) == Float64

#update_event_agent! tests
Agent1 = deepcopy(AgentsSet[1])
update_event_agent!(AgentsSet[1],event[1], i_densities, test_map.v)
@test AgentsSet[1].edge[1] == Agent1.edge[2]
@test length(AgentsSet[1].route) == length(Agent1.route) - 1

end
