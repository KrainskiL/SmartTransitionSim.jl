######################################
## Auxilary traffic model functions ##
######################################
"""
`traffic_constants` create maximal traffic densities and speeds matrices

**Input parameters**
* `OSMmap` : OpenStreetMapX MapData object with road network data
* `density_factor` : road length reserved for one vehicle
"""
function traffic_constants(OSMmap::MapData,
                           density_factor::Float64 = 5.0)
    #Create maximal densitites matrix
    max_densities = get_max_densities(OSMmap, density_factor)
    #Create speed limit matrix
    max_speeds = OpenStreetMapX.get_velocities(OSMmap)
    return max_densities, max_speeds
end

"""
`get_max_densities` calculate maximal traffic densities on all edges

**Input parameters**
* `OSMmap` : OpenStreetMapX MapData object with road network data
* `density_factor` : road length reserved for one vehicle
"""
function get_max_densities(OSMmap::MapData,
                           density_factor::Float64 = 5.0)
    roads_lanes = Dict{Int,Int}()
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
`init_traffic_variables` create data used in calculating velocities change during simulation

**Input parameters**
* `OSMmap` : OpenStreetMapX MapData object with road network data
* `Agents` : set of agents created with generate_agents function
"""
function init_traffic_variables(OSMmap::MapData,
                                Agents::Vector{Agent})
    #Dictionary with initital densities
    initial_densities = StatsBase.countmap([a.edge for a in Agents])
    initial_speeds = OpenStreetMapX.get_velocities(OSMmap)
    return initial_densities, initial_speeds
end

"""
`update_weights!` change speeds in given speed_matrix for edges listed in new_densities

**Input parameters**
* `speed_matrix` : matrix with average speeds on edges
* `new_densities` : dictionary with edges as keys and new traffic density as value
* `V_max` : matrix with maximal speeds on edges
* `V_min` : minimal speed on road
"""
function update_weights!(speed_matrix::SparseMatrixCSC{Float64,Int},
                        dens::Dict{Tuple{Int,Int},Int},
                        max_densities::SparseMatrixCSC{Float64,Int},
                        V_max::SparseMatrixCSC{Float64,Int},
                        V_min::Float64 = 1.0)
    for k in keys(dens)
        v1, v2 = k
        speed_matrix[v1,v2]  = (V_max[v1,v2] - V_min)* max((1 - dens[k]/max_densities[v1,v2]), 0.0) + V_min
    end
end

"""
`update_weights_and_events!` update events time and weights in speed matrix

**Input parameters**
* `inAgents` : set of agents created with generate_agents function
* `agents_pos` : vector with agents current position (edges)
* `events` : vector with events time
* `speed_matrix` : matrix with average speeds on edges
* `edges` : one or two edges that took part in an event
* `dens` : dictionary with current density on edges
* `max_densities` : matrix with maximal densities
* `V_max` : matrix with speed limits
* `V_min` : minimal speed allowed on road
"""
function update_weights_and_events!(inAgents::Vector{Agent},
                                    agents_pos::Vector{Tuple{Int,Int}},
                                    events::Vector{Float64},
                                    speed_matrix::SparseMatrixCSC{Float64,Int},
                                    edges::Vector{Tuple{Int,Int}},
                                    dens::Dict{Tuple{Int,Int},Int},
                                    max_densities::SparseMatrixCSC{Float64,Int},
                                    V_max::SparseMatrixCSC{Float64,Int},
                                    V_min::Float64 = 1.0)
    for edge in edges
        v1, v2 = edge
        old_speed = speed_matrix[v1,v2]
        #Update speed matrix
        new_speed = (V_max[v1,v2] - V_min)* max((1 - dens[edge]/max_densities[v1,v2]), 0.0) + V_min
        speed_matrix[v1,v2]  = new_speed
        #Adjust event time for agents in modified edges
        speed_factor = old_speed/new_speed
        # for i in [k for (k,v) in agents_pos if v == edge]
        #     @inbounds events[i] = events[i]*speed_factor
        # end
        for i in findall(x-> x==edge, agents_pos)
            @inbounds events[i] = events[i]*speed_factor
        end
    end
end

"""
`next_edge` returns time required to reach next junction for one agent

**Input parameters**
* `Agent` : single instance of Agent
* `speeds` : current speeds matrix
* `lengths` : road lengths matrix
"""
function next_edge(Agent::Agent,
                    speeds::AbstractMatrix,
                    lengths::AbstractMatrix)
    e = Agent.edge
    event_time = lengths[e[1],e[2]]/speeds[e[1],e[2]]
    return event_time
end

"""
`next_edge` returns time required to reach next junction for set of agents

**Input parameters**
* `Agents` : set of agents created with generate_agents function
* `speeds` : current speeds matrix
* `lengths` : road lengths matrix
"""
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
                            densities::Dict{Tuple{Int,Int}, Int},
                            vertices_map::Dict{Int,Int})
    #Decrease density on previous edge
    p_edge = inAgent.edge
    densities[p_edge] -= 1
    #Update agent  position
    if length(inAgent.route) == 2
        #Disable agent and set travelling time
        inAgent.active = false
        inAgent.travel_time = curr_time
        #Return agent's last edge
        return [p_edge]
    else
        #Update agent route and current edge
        inAgent.route = inAgent.route[2:end]
        c_edge = (inAgent.edge[2], vertices_map[inAgent.route[2]])
        inAgent.edge = c_edge
        #Add density on new edge
        haskey(densities, c_edge) ? densities[c_edge] += 1 : densities[c_edge] = 1
        #Return previous and current agent's edge
        return [p_edge, c_edge]
    end
end
