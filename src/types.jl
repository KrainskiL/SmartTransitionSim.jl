################################
## Types used in simulation ##
################################
"""
`Rect` type represent rectangle area on map

**Fields**
* `p1` : Latitude-Longitude coordinates of first rectangle corner
* `p2` : Latitude-Longitude coordinates of second rectangle corner
"""
struct Rect
    p1::Tuple{Float64,Float64}
    p2::Tuple{Float64,Float64}
end

"""
`Agent` type stores information about agents

**Fields**
* `smart` : logical value indicating if agent use re-routing service
* `start_node` : starting point of agent's route
* `end_node` : ending point of agent's route
* `route` : array of nodes determining agent's route (may be changed by re-routing)
* `start_time` : time after which agent starts moving
* `travel_time` : time spent in simulation
* `edge` : current edge agent is on
* `active` : indicates if agent is active in simulation
"""
mutable struct Agent
    smart::Bool
    start_node::Int
    end_node::Int
    route::Union{Array{Int,1}, Nothing}
    start_time::Float64
    travel_time::Float64
    edge::Tuple{Int,Int}
    active::Bool
end
