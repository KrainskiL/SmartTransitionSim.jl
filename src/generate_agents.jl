##################################
## Setting up agents properties ##
##################################
"""
`pick_random_node` function is used to set starting and ending node of agents.
Nodes are randomly chosen from set of rectangles corresponding to areas on map.

**Input parameters**
* `OSMmap` : OpenStreetMapX mapData type object with road network data
* `rects` : vector of Rect types interpreted as a set of rectangle areas
* `NodesSet` : returning object switch
    * `false` return random node from given area
    * `true` return set of nodes in given area for further processing
"""
function pick_random_node(OSMmap::OpenStreetMapX.MapData,
                          rects::Vector{Rect},
                          NodesSet::Bool = false)
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
    NodesSet && return unique_nodes
    return rand(unique_nodes)
end

"""
`generate_agents` function creating vector of agents and returning travel time
for initial routes travelled with maximal speed

**Input parameters**
* `OSMmap` : OpenStreetMapX MapData type object with road network data
* `N` : number of agents to be generated
* `StartArea` : vector of points corresponding to area from which agents randomly pick starting point
* `EndArea` : vector of points corresponding to area from which agents randomly pick ending point
* `α` : percentage of smart agentss
"""
function generate_agents(OSMmap::OpenStreetMapX.MapData,
                        N::Int,
                        StartArea::Vector{Rect},
                        EndArea::Vector{Rect},
                        α::Float64)
    #Initialize empty working variables
    AgentsArr = Vector{Agent}()
    times = Vector{Float64}()
    dists = Vector{Float64}()
    #Indicate smart agents
    N_int= Int(ceil(N*α))
    smart_ind = [trues(N_int); falses(N-N_int)]
    #Obtaining nodes set for given starting and ending areas
    start_set = pick_random_node(OSMmap, StartArea, true)
    end_set = pick_random_node(OSMmap, EndArea, true)
    #Generating N agents
    for i in 1:N
        dist = Inf
        start_node = end_node = counter = time =  0
        init_route = Array{Int64,1}()
        while dist == Inf
            start_node = rand(start_set)
            end_node = rand(end_set)
            init_route, dist, time = OpenStreetMapX.fastest_route(OSMmap, start_node, end_node)
            counter +=1
            counter == 100 && error("Route from starting to ending point can't be calculated.")
        end
        push!(times, time)
        push!(dists, dist)
        #First edge in vertices notation
        firstEdge = [OSMmap.v[init_route[1]], OSMmap.v[init_route[2]]]
        NewAgent = Agent(smart_ind[i], start_node, end_node, init_route, 0.0, firstEdge, true)
        push!(AgentsArr, NewAgent)
    end
    return AgentsArr, times, dists
end
