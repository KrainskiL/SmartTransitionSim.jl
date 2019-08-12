##################################
## Rerouting module ##
##################################

"""
`k_shortest_path_rerouting!` change agents current route according to traffic conditions

**Input parameters**
* `OSMmap` : MapData type object with road network data
* `inAgent` : rerouting agent
* `speeds` : current speeds matrix
* `k` : number of fastest routes returned
* `T` : control variable for probability distribution
"""
function k_shortest_path_rerouting!(OSMmap::MapData,
                                    inAgent::Agent,
                                    speeds::AbstractMatrix,
                                    k::Int64,
                                    T::Float64)
    k_routes = LightGraphs.yen_k_shortest_paths(OSMmap.g, OSMmap.v[inAgent.route[2]],
                                                OSMmap.v[inAgent.end_node], OSMmap.w./speeds, k)
    if k == 1
        new_path = k_routes.paths[1]
    else
        #Normalize k-paths travelling time
        norm_time = k_routes.dists/maximum(k_routes.dists)
        #Calculate probability of being picked for every route
        exp_ntime = exp.(-norm_time/T)
        probs = exp_ntime/sum(exp_ntime)
        #Assign new route
        new_path = sample(k_routes.paths, StatsBase.weights(probs))
    end
    nodes_new_path = map(i-> OSMmap.n[i], new_path)
    inAgent.route = [inAgent.route[1]; nodes_new_path]
end
