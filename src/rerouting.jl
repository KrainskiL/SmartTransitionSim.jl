##################################
## Rerouting module ##
##################################

"""
`k_shortest_path_rerouting!` change agents current route according to traffic conditions

**Input parameters**
* `OSMmap` : OpenStreetMapX MapData object with road network data
* `inAgent` : modified agent
* `speeds` : current speed matrix
* `k` : number of fastest routes returned
* `T` : control variable for probability distribution
"""
function k_shortest_path_rerouting!(OSMmap::MapData,
                                    inAgent::Agent,
                                    speeds::AbstractMatrix,
                                    k::Int64,
                                    T::Float64)
    k_routes = yen_a_star(OSMmap.g, OSMmap.v[inAgent.route[2]],
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

"""
`yen_a_star` is modified version of LightGraphs yen_k_shortest_paths function

**Input parameters**
* `g` : road network graph
* `source` : source vertex
* `target` : target vertex
* `distmx` : weight matrix
* `K` : number of routes to be calculated
"""
function yen_a_star(g::AbstractGraph,
    source::Int64,
    target::Int64,
    distmx::AbstractMatrix=weights(g),
    K::Int64=1)

    dj = OpenStreetMapX.a_star_algorithm(g, source, target, distmx)
    path = dj[1]

    dists = Array{Float64,1}()
    push!(dists, dj[2])
    A = [path]
    B = DataStructures.PriorityQueue()
    gcopy = deepcopy(g)

    for k = 1:(K - 1)
        for j = 1:(length(A[k])-1)
            spurnode = A[k][j]
            rootpath = A[k][1:j]
            edgesremoved = Array{Tuple{Int,Int},1}()
            for ppath in A
                if length(ppath) > j && rootpath == ppath[1:j]
                    u = ppath[j]
                    v = ppath[j + 1]
                    if has_edge(gcopy, u, v)
                        rem_edge!(gcopy, u, v)
                        push!(edgesremoved, (u, v))
                    end
                end
            end
            distrootpath = 0.
            for n = 1:(length(rootpath) - 1)
                u = rootpath[n]
                nei = copy(neighbors(gcopy, u))
                for v in nei
                    rem_edge!(gcopy, u, v)
                    push!(edgesremoved, (u, v))
                end
                v = rootpath[n + 1]
                distrootpath += distmx[u, v]
            end
            djspur = OpenStreetMapX.a_star_algorithm(gcopy, spurnode, target, distmx)
            spurpath = djspur[1]
            if !isempty(spurpath)
                pathtotal = [rootpath[1:(end - 1)]; spurpath]
                distpath  = distrootpath + djspur[2]
                if !haskey(B, pathtotal)
                    enqueue!(B, pathtotal, distpath)
                end
            end
            for (u, v) in edgesremoved
                add_edge!(gcopy, u, v)
            end
        end
        isempty(B) && break
        mindistB = peek(B)[2]
        push!(dists, peek(B)[2])
        push!(A, dequeue!(B))
    end

    return LightGraphs.YenState(dists, A)
end
