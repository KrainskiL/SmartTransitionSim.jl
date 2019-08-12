######################################
## Auxilary traffic model functions ##
######################################

"""
`get_max_densities` calculate maximal traffic density (number of agents) on each road in given network

**Input parameters**
* `OSMmap` : MapData type object with road network data
* `density_factor` : road length reserved for one vehicle
"""
function get_max_densities(OSMmap::MapData, density_factor::Float64 = 5.0)
    roads_lanes = Dict{Int64,Int64}()
    for r in OSMmap.roadways
        OpenStreetMapX.haslanes(r) ? lanes = OpenStreetMapX.getlanes(r) : lanes = 1
        roads_lanes[r.id] = lanes
    end
    segments = OpenStreetMapX.find_segments(OSMmap.nodes, OSMmap.roadways, OSMmap.intersections)
    segments = [[OSMmap.v[s.node0], OSMmap.v[s.node1], roads_lanes[s.parent]] for s in segments]
    sparse_lanes = SparseArrays.sparse([x[1] for x in segments],
                                        [x[2] for x in segments],
                                        [x[3] for x in segments],
                                        length(OSMmap.v),length(OSMmap.v))
    return OSMmap.w .* sparse_lanes / density_factor
end

"""
`update_weights!` change speeds in given speed_matrix for edges listed in new_densities

**Input parameters**
* `speed_matrix` : matrix with average speeds on edges
* `new_densities` : dictionary with edges as keys and new traffic density as value
* `V_max` : matrix with maximal speeds on edges
* `V_min` : minimal speed on road
"""
function update_weights!(speed_matrix::SparseMatrixCSC{Float64,Int64},
                        edges::Vector{Vector{Int64}},
                        dens::Dict{Vector{Int64},Int64},
                        max_densities::SparseMatrixCSC{Float64,Int64},
                        V_max::SparseMatrixCSC{Float64,Int64},
                        V_min::Float64 = 1.0)
    speed_factors = Vector{Float64}()
    for edge in edges
        v1, v2 = edge
        old_speed = speed_matrix[v1,v2]
        speed_matrix[v1,v2]  = (V_max[v1,v2] - V_min)* max((1 - dens[edge]/max_densities[v1,v2]), 0.0) + V_min
        push!(speed_factors, old_speed/speed_matrix[v1,v2])
    end
    return speed_factors
end

function update_weights!(speed_matrix::SparseMatrixCSC{Float64,Int64},
                        dens::Dict{Vector{Int64},Int64},
                        max_densities::SparseMatrixCSC{Float64,Int64},
                        V_max::SparseMatrixCSC{Float64,Int64},
                        V_min::Float64 = 1.0)
    for (k,v) in dens
        v1, v2 = k
        speed_matrix[v1,v2]  = (V_max[v1,v2] - V_min)* max((1 - dens[k]/max_densities[v1,v2]), 0.0) + V_min
    end
end

function event_time_correction!(inAgents::Vector{Agent},
    edges::Vector{Vector{Int64}},
    factors::Vector{Float64},
    events::Vector{Float64})
    for (i,e) in enumerate(edges)
        for (j,a) in enumerate(inAgents)
            if a.active && a.edge == e events[j] *= factors[i] end
        end
    end
end
"""
`traffic_constants` create maximal traffic densities and speeds matrices

**Input parameters**
* `OSMmap` : MapData type object with road network data
* `density_factor` : road length reserved for one vehicle
"""
function traffic_constants(OSMmap::MapData,
                           density_factor::Float64 = 5.0)
    #Create maximal densitites matrix
    max_densities = get_max_densities(OSMmap, density_factor)
    #Create maximal speeds matrix
    max_speeds = OpenStreetMapX.get_velocities(OSMmap)
    return max_densities, max_speeds
end

"""
`init_traffic_variables` create data used in calculating velocities change during simulation

**Input parameters**
* `OSMmap` : MapData type object with road network data
* `Agents` : set of agents created with generate_agents function
* `optimization_stats` : switch for creating data used in RSUs optimization algorithm
"""
function init_traffic_variables(OSMmap::MapData,
                                Agents::Vector{Agent})
    #Initital densities on edges
    initial_densities = StatsBase.countmap([a.edge for a in Agents])
    initial_speeds = OpenStreetMapX.get_velocities(OSMmap)
    return initial_densities, initial_speeds
end

"""
`next_edge` returns time for nearest edge switch and ID of agent performing the switch

**Input parameters**
* `Agents` : set of agents created with generate_agents function
* `speeds` : current speeds matrix
* `lengths` : matrix with road lengths
"""
function next_edge(Agent::Agent,
                    speeds::AbstractMatrix,
                    lengths::AbstractMatrix)
    e = Agent.edge
    event_time = lengths[e[1],e[2]]/speeds[e[1],e[2]]
    return event_time
end

function next_edge(Agents::Vector{Agent},
                    speeds::AbstractMatrix,
                    lengths::AbstractMatrix)
    return [next_edge(a,speeds,lengths) for a in Agents]
end
"""
`update_event_agent!` update densities matrix, progress agents to next edge and deactivate agents

**Input parameters**
* `inAgent` : agent connected with occuring event
* `curr_time` : current simulation time
* `densities` : current traffic densitites dictionary
* `vertices_map` : mapping from nodes to vertices
"""
function update_event_agent!(inAgent::Agent,
                            curr_time::Float64,
                            densities::Dict,
                            vertices_map::Dict{Int,Int})
    #Decrease density on previous edge
    p_edge = inAgent.edge
    densities[p_edge] -= 1
    #Update agent  position
    if length(inAgent.route) == 2
        #Disable agent and set travelling time
        inAgent.active = false
        inAgent.travel_time = curr_time
        #Return dictionary with changed density
        return [p_edge]
    else
        #Update agent route and current edge
        inAgent.route = inAgent.route[2:end]
        c_edge = [inAgent.edge[2], vertices_map[inAgent.route[2]]]
        inAgent.edge = c_edge
        #Add density on new edge
        haskey(densities, c_edge) ? densities[c_edge] += 1 : densities[c_edge] = 1
        #Return dictionary with changed densities
        return [p_edge, c_edge]
    end
end
