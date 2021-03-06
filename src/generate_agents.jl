##################################
## Setting up agents properties ##
##################################
"""
`pick_random_node` function is used to get starting and ending node of agents.
Nodes are randomly chosen from set of provided areas.

**Input parameters**
* `OSMmap` : OpenStreetMapX mapData object with road network data
* `rects` : vector of Rect types interpreted as a set of rectangle areas
"""
function pick_random_node(OSMmap::OpenStreetMapX.MapData,
                          rects::Vector{Rect})
    nodes_in_rects = Vector{Int}()
    for rect in rects
        p1 = ENU(LLA(rect.p1[1], rect.p1[2]), OSMmap.bounds)
        p2 = ENU(LLA(rect.p2[1], rect.p2[2]), OSMmap.bounds)
        exE = extrema([p1.east, p2.east])
        exN = extrema([p1.north, p2.north])
        for key in keys(OSMmap.v)
            if (exE[1] <= OSMmap.nodes[key].east <= exE[2] &&
                exN[1] <= OSMmap.nodes[key].north <= exN[2])
                push!(nodes_in_rects, key)
            end
        end
    end
    unique_nodes = unique!(nodes_in_rects)
    return rand(unique_nodes)
end

"""
`get_nodes_set` function is used to get set of nodes within passed area.

**Input parameters**
* `OSMmap` : OpenStreetMapX mapData object with road network data
* `rects` : vector of Rect types interpreted as a set of rectangle areas
"""
function get_nodes_set(OSMmap::OpenStreetMapX.MapData,
                          rects::Vector{Rect})
    nodes_in_rects = Vector{Int}()
    for rect in rects
        p1 = ENU(LLA(rect.p1[1], rect.p1[2]), OSMmap.bounds)
        p2 = ENU(LLA(rect.p2[1], rect.p2[2]), OSMmap.bounds)
        exE = extrema([p1.east, p2.east])
        exN = extrema([p1.north, p2.north])
        for key in keys(OSMmap.v)
            if (exE[1] <= OSMmap.nodes[key].east <= exE[2] &&
                exN[1] <= OSMmap.nodes[key].north <= exN[2])
                push!(nodes_in_rects, key)
            end
        end
    end
    unique_nodes = unique!(nodes_in_rects)
    return unique_nodes
end

"""
`generate_agents` function creating vector of agents and returning travel time
for initial routes travelled with maximal speed

**Input parameters**
* `OSMmap` : OpenStreetMapX MapData object with road network data
* `N` : number of agents to be generated
* `StartArea` : vector of areas from which agents randomly pick starting point
* `EndArea` : vector of areas from which agents randomly pick ending point
* `α` : percentage of smart agents
* `k` : number of fastest routes used in k-path algorithm
* `T` : control variable for k-path algorithm probability distribution
* `AvgStartTime` : average agents start time in minutes
* `k_routes_dict` : dictionary with multiple shortest paths (values) between vertices (keys)
* `seed` : set seed for random function
"""
function generate_agents(OSMmap::OpenStreetMapX.MapData,
                        N::Int,
                        StartArea::Vector{Rect},
                        EndArea::Vector{Rect},
                        α::Float64,
                        k::Int = 3,
                        T::Float64 = 1.0,
                        AvgStartTime::Float64 = 0.0,
                        k_routes_dict::Dict{Tuple{Int,Int},Array{Vector{Int}}}=
                        Dict{Tuple{Int,Int},Array{Vector{Int}}}())
    #Initialize empty working variables
    AgentsArr = Vector{Agent}()
    #Indicate smart agents
    N_int= Int(ceil(N*α))
    smart_ind = [trues(N_int); falses(N-N_int)]
    #Obtaining nodes set for given starting and ending areas
    start_set = get_nodes_set(OSMmap, StartArea)
    end_set = get_nodes_set(OSMmap, EndArea)
    #Generating N agents
    for i in 1:N
        dist = Inf
        start_node = end_node = counter =  0
        init_route = Array{Int64,1}()
        while dist == Inf
            start_node = rand(start_set)
            end_node = rand(end_set)
            init_route, dist = OpenStreetMapX.fastest_route(OSMmap, start_node, end_node)[1:2]
            counter +=1
            counter == 100 && error("Route from starting to ending point can't be calculated.")
        end
        #First edge in vertices notation
        firstEdge = (OSMmap.v[init_route[1]], OSMmap.v[init_route[2]])
        sTime = rand()*AvgStartTime*60*2
        NewAgent = Agent(smart_ind[i], start_node, end_node,
                        init_route, sTime, 0.0, firstEdge,
                        ifelse(sTime == 0.0,true,false))
        push!(AgentsArr, NewAgent)
    end
    #Get averages speeds from base scenario - simulating short term memory
    AverageSpeeds = simulation_run(:base, OSMmap,AgentsArr; track_avg_speeds=true).AvgSpeeds
    #Recalculate agents paths with k-shortest path algorithm and avg speeds
    max_speeds = OpenStreetMapX.get_velocities(OSMmap)
    for a in AgentsArr
        k_shortest_path_rerouting!(OSMmap, k_routes_dict, a, AverageSpeeds,max_speeds, k, T, 0)
    end
    return AgentsArr
end
