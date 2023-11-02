module ContractionHierarchies

export local_node, shortest_time, HierarchicalMap, HMaps, add_map, remove_map

import Downloads
import Serialization
import SparseArrays

using OpenStreetMapX
import Graphs
import DataStructures
import ProgressMeter

const DEFAULT_VEHICULE = "car"

struct AstarCache{T,U}
    frontier::DataStructures.PriorityQueue{U,T}
    dists::Vector{T}
    parents::Vector{U}
    visited::BitVector
    done::BitVector
end

function AstarCache{T,U}(nvg) where {T,U}
    frontier = DataStructures.PriorityQueue{U,T}()
    dists = fill(typemax(T), nvg)
    parents = zeros(U, nvg)
    visited = falses(nvg)
    done = falses(nvg)
    return AstarCache{T,U}(frontier, dists, parents, visited, done)
end

function Base.empty!(cache::AstarCache{T,U}) where {T,U}
    empty!(cache.frontier)
    fill!(cache.dists, typemax(T))
    fill!(cache.parents, zero(U))
    fill!(cache.visited, false)
    fill!(cache.done, false)
    return
end

function init(cache::AstarCache{T,U}, u::U, d::T, v::T) where {T,U}
    cache.visited[u] = true
    cache.dists[u] = d
    cache.frontier[u] = v
    return
end

function _a_star(
    g::Graphs.AbstractGraph{U},
    istarget::Function,
    distance::Function,
    heuristic::Function,
    final_cost::Function,
    curiosity,
    frontier,
    dists::Vector{T},
    parents,
    visited::BitVector,
    done::BitVector,
) where {T,U}
    best_node = zero(U)
    best_cost = typemax(T)
    while !isempty(frontier)
        u = DataStructures.dequeue!(frontier)
        cost_so_far = dists[u]
        if istarget(u)
            cur_cost = cost_so_far + final_cost(u)
            if iszero(best_node) || cur_cost < best_cost
                best_node = u
                best_cost = cur_cost
            end
            if isone(curiosity)
                break
            end
        end
        if !iszero(best_node) && cost_so_far > best_cost * curiosity
            break
        end
        for v in Graphs.outneighbors(g, u)
            if !done[v]
                dist = distance(u, v)
                path_cost = cost_so_far + dist
                if !visited[v]
                    visited[v] = true
                    parents[v] = u
                    dists[v] = path_cost
                    DataStructures.enqueue!(
                        frontier,
                        v,
                        path_cost + heuristic(v),
                    )
                elseif path_cost < dists[v]
                    parents[v] = u
                    dists[v] = path_cost
                    frontier[v] = path_cost + heuristic(v)
                end
            end
        end
        done[u] = true
    end
    return best_node, best_cost
end

function dijkstra(
    g::Graphs.AbstractGraph{U},
    s::Integer,
    istarget::Function,
    distance::Function,
    ::Type{T},
    cache::Union{Nothing,AstarCache{T,U}}=nothing,
) where {T,U}
    if cache === nothing
        cache = AstarCache{T,U}(Graphs.nv(g))
    else
        empty!(cache)
    end
    init(cache, s, zero(T), zero(T))
    _, _ = _a_star(
        g,
        istarget,
        distance,
        _ -> zero(T),
        _ -> zero(T),
        2.0,
        cache.frontier,
        cache.dists,
        cache.parents,
        cache.visited,
        cache.done,
    )
    return cache
end

function a_star(
    g::Graphs.AbstractGraph{U},
    s::Integer,
    t::Integer,
    distmx::AbstractMatrix{T},
    heuristic::Function,
    cache::Union{Nothing,AstarCache{T,U}}=nothing,
) where {T,U}
    nvg = Graphs.nv(g)
    checkbounds(distmx, Base.OneTo(nvg), Base.OneTo(nvg))
    if cache === nothing
        cache = AstarCache{T,U}(nvg)
    else
        empty!(cache)
    end
    init(cache, s, zero(T), zero(T))
    best_node, best_cost = _a_star(
        g,
        isequal(t),
        (u, v) -> distmx[u, v],
        Base.Fix2(heuristic, t),
        _ -> zero(T),
        one(T),
        cache.frontier,
        cache.dists,
        cache.parents,
        cache.visited,
        cache.done,
    )
    if iszero(best_node)
        return U[], typemax(T)
    else
        return OpenStreetMapX.extract_a_star_route(cache.parents, s, best_node), best_cost
    end
end

struct TMap
    data::MapData
    time::SparseArrays.SparseMatrixCSC{Float64,Int}
    speeds::Dict{Int,Float64}
end

mutable struct HierarchicalMap
    vehicule::String
    fast_map::Union{Nothing,TMap}
    speeds::Dict{Int,Float64}
    local_bounds::Dict{String,OpenStreetMapX.Bounds{OpenStreetMapX.LLA}}
    local_caches::Dict{String,MapData}
end
Base.broadcastable(hmap::HierarchicalMap) = Ref(hmap)

const ROOT = dirname(@__DIR__)
const DATA = joinpath(ROOT, "data")
const LOCAL_BOUNDS = joinpath(DATA, "local_bounds")

function HierarchicalMap(vehicule, fast_map, speeds)
    root = dirname(@__DIR__)
    if isfile(LOCAL_BOUNDS)
        open(LOCAL_BOUNDS) do f
            local_bounds = Serialization.deserialize(f)
        end
    else
        local_bounds = Dict{String,OpenStreetMapX.Bounds{OpenStreetMapX.LLA}}()
    end
    return HierarchicalMap(vehicule, fast_map, speeds, local_bounds, Dict{String,MapData}())
end

function HierarchicalMap(speeds=OpenStreetMapX.SPEED_ROADS_URBAN)
    return HierarchicalMap(DEFAULT_VEHICULE, nothing, OpenStreetMapX.SPEED_ROADS_URBAN)
end

struct HMaps
    hmaps::Dict{String,HierarchicalMap}
end
HMaps() = HMaps(Dict{String,HierarchicalMap}())
HMaps(hmap::HierarchicalMap) = HMaps(Dict(hmap.vehicule => hmap))
function Base.getindex(hmaps::HMaps, ::Nothing)
    return getindex(hmaps, DEFAULT_VEHICULE)
end
function Base.getindex(hmaps::HMaps, vehicule::String)
    if !haskey(hmaps.hmaps, vehicule)
        hmaps.hmaps[vehicule] = HierarchicalMap(vehicule, nothing, OpenStreetMapX.SPEED_ROADS_URBAN)
    end
    return hmaps.hmaps[vehicule]
end

struct LocalNode
    path::String
    node::Int
end

function local_map(hmap::HierarchicalMap, path::String)
    if !haskey(hmap.local_caches, path)
        default = joinpath(DATA, path, "local")
        pbf = vehicule_map(hmap.vehicule, default)
        hmap.local_caches[path] = OpenStreetMapX.get_map_data(pbf)
    end
    return hmap.local_caches[path]
end
function local_tmap(hmap::HierarchicalMap, path::String)
    return tmap(local_map(hmap, path), hmap.speeds)
end

function nearest(nodes, locs, loc)
    best = 0
    dist = Inf
    for node in nodes
        cur = OpenStreetMapX.distance(locs[node], loc)
        if cur < dist
            best = node
            dist = cur
        end
    end
    return best, dist
end

function local_node(hmap::HierarchicalMap, lla::OpenStreetMapX.LLA)
    best_local = nothing
    best_node = 0
    best_dist = Inf
    if isempty(hmap.local_bounds)
        error("There is no local map.")
    end
    for (path, bounds) in hmap.local_bounds
        if OpenStreetMapX.inbounds(lla, bounds)
            map_data = local_map(hmap, path)
            enu = OpenStreetMapX.ENU(lla, map_data.bounds)
            node, dist = nearest(map_data.n, map_data.nodes, enu)
            if best_local === nothing || dist < best_dist
                best_local = path
                best_node = node
                best_dist = dist
            end
        end
    end
    if best_local === nothing
        error("$lla is not part of the bounds of any local map. Download more maps with `add_map`.")
    end
    if best_dist > 1000
        @warn("Closest node to $lla is $best_node in $best_local at distance $best_dist")
    end
    return LocalNode(best_local, best_node)
end

function local_dijkstra(hmap::HierarchicalMap, node::LocalNode)
    fast_map = global_fast(hmap)
    local_map = local_tmap(hmap, node.path)
    is_fast(u) = haskey(fast_map.data.v, local_map.data.n[u])
    cache = dijkstra(
        local_map.data.g,
        local_map.data.v[node.node],
        is_fast,
        (u, v) -> local_map.time[u, v],
        Float64,
        nothing,
    )
    finite = Int[]
    dists = fill(Inf, Graphs.nv(fast_map.data.g))
    for u in eachindex(cache.visited)
        if cache.visited[u] && is_fast(u)
            fast_u = fast_map.data.v[local_map.data.n[u]]
            push!(finite, fast_u)
            dists[fast_u] = cache.dists[u]
        end
    end
    return cache.dists, dists, finite, cache.parents
end

function append_route!(route::Vector{Int}, lmap, parents)
    cur = lmap.v[pop!(route)]
    while !iszero(cur)
        push!(route, lmap.n[cur])
        cur = parents[cur]
    end
end

function shortest_time(hmap::HierarchicalMap, src_dst::Pair{LocalNode,LocalNode})
    src, dst = src_dst
    dists_src, fast_dists_src, finite_src, parents_src = local_dijkstra(hmap, src)
    dists_dst, fast_dists_dst, finite_dst, parents_dst = local_dijkstra(hmap, dst)
    fast_map = hmap.fast_map
    nvg = Graphs.nv(fast_map.data.g)
    max_speed = maximum(values(hmap.speeds))
    heuristic = map(1:nvg) do u
        minimum(finite_dst, init = Inf) do v
            OpenStreetMapX.get_distance(u, v, fast_map.data.nodes, fast_map.data.n) / max_speed + fast_dists_dst[v]
        end
    end
    cache = AstarCache{Float64,Int}(nvg)
    for u in finite_src
        dist = fast_dists_src[u]
        init(cache, u, dist, dist + heuristic[u])
    end
    best_node, best_cost = _a_star(
        fast_map.data.g,
        u -> isfinite(fast_dists_dst[u]),
        (u, v) -> fast_map.time[u, v],
        Base.Fix1(getindex, heuristic),
        Base.Fix1(getindex, fast_dists_dst),
        1.0,
        cache.frontier,
        cache.dists,
        cache.parents,
        cache.visited,
        cache.done,
    )
    src_map = local_map(hmap, src.path)
    # Is it faster to not through highways ?
    if src.path == dst.path && dists_src[src_map.v[dst.node]] < best_cost
        return dists_src[src_map.v[dst.node]]
    elseif iszero(best_node)
    #if iszero(best_node)
        #return U[], typemax(T)
        return typemax(typeof(best_cost))
    else
        cur = best_node
        route = Int[]
        while !iszero(cur)
            push!(route, fast_map.data.n[cur])
            cur = cache.parents[cur]
        end
        lmap = local_map(hmap, src.path)
        append_route!(route, src_map, parents_src)
        reverse!(route)
        dst_map = local_map(hmap, dst.path)
        append_route!(route, dst_map, parents_dst)
        #return route, best_cost
        return best_cost
    end
end

function shortest_time(m::TMap, osm_src_dst::Pair{Int,Int}, args...)
    osm_src, osm_dst = osm_src_dst
    graph_src = m.data.v[osm_src]
    graph_dst = m.data.v[osm_dst]
    max_speed = maximum(values(m.speeds))
    route, time = a_star(
        m.data.g,
        graph_src,
        graph_dst,
        m.time,
        (u, v) -> OpenStreetMapX.get_distance(u, v, m.data.nodes, m.data.n) / max_speed,
        args...,
    )
    return time
end

function shortest_time(hmap::HierarchicalMap, nodes::Vector{LocalNode})
    return [
        shortest_time(hmap, nodes[i] => nodes[j])
        for i in eachindex(nodes), j in eachindex(nodes)
    ]
end

const NODES_TYPE{T} = Union{Pair{T,T},Vector{T}}

function shortest_time(hmap::HierarchicalMap, nodes::Pair{OpenStreetMapX.LLA})
    return shortest_time(hmap, local_node(hmap, nodes.first) => local_node(hmap, nodes.second))
end

function shortest_time(hmap::HierarchicalMap, nodes::Vector{OpenStreetMapX.LLA})
    return shortest_time(hmap, local_node.(hmap, nodes))
end

function shortest_time(src_dst::NODES_TYPE)
    return shortest_time(HMaps()[nothing], src_dst)
end

function shortest_time(map::String, src_dst::NODES_TYPE; speeds = OpenStreetMapX.SPEED_ROADS_URBAN)
    hmap = HierarchicalMap(DEFAULT_VEHICULE, tmap(file, speeds), speeds)
    return shortest_time(hmap, src_dst)
end

function tmap(map_data::MapData, speeds)
    time = OpenStreetMapX.create_weights_matrix(map_data, OpenStreetMapX.network_travel_times(map_data, speeds))
    return TMap(map_data, time, speeds)
end
function tmap(file::String, speeds)
    @info("Reading map data from `$file`.")
    tmap(OpenStreetMapX.get_map_data(file), speeds)
end

# http://download.geofabrik.de/europe/belgium-latest.osm.pbf
# http://download.geofabrik.de/europe/germany/saarland-latest.osm.pbf
"""
    add_map(hmap::HierarchicalMap, args...)

Download the map of the regtion specified by `args` from http://download.geofabrik.de.

For small country like Belgium, specify the continent then the country:
```julia
julia> h = HierarchicalMap()

julia> add_map(h, "europe", "belgium")
```
For larger country, additionally specify a smaller region:
```julia
julia> h = HierarchicalMap()

julia> add_map(h, "europe", "germany", "saarland")
```
See http://download.geofabrik.de for the list of countries and corresponding regions.
"""
function add_map(hmap::HierarchicalMap, args...)
    root = dirname(@__DIR__)
    dir = joinpath(root, "data", args...)
    mkpath(dir)
    progressmeter = nothing
    function progress(total, now)
        if iszero(total)
            return
        end
        if progressmeter === nothing && now < total
            progressmeter = ProgressMeter.Progress(total)
        end
        if progressmeter !== nothing
            ProgressMeter.update!(progressmeter, now)
        end
        # It calls `progress(total, total)` several times at the end which cause the progress bar to be displayed several times.
        if now == total
            progressmeter = nothing
        end
    end
    path = join([args...], "/")
    all_pbf = joinpath(dir, "all.osm.pbf")
    @info("$path: Downloading PBF format [$all_pbf]")
    Downloads.download(
        string("http://download.geofabrik.de/", path, "-latest.osm.pbf"),
        all_pbf;
        progress = progress,
    )

    all_om = joinpath(dir, "all.osm.o5m")
    @info("$path: Converting from PBF to O5M [$all_om]")
    @time run(`osmconvert $all_pbf -o=$all_om`)

    car_om = joinpath(dir, "local.osm.o5m")
    car_filter = joinpath(root, "filters", "car.txt")
    @info("$path: Filtering out non-map data [$car_om]")
    @time run(pipeline(`osmfilter "$all_om" --parameter-file="$car_filter"`, car_om))

    car_pbf = joinpath(dir, "local.osm.pbf")
    @info("$path: Converting from O5M to PBF [$car_pbf]")
    @time run(`osmconvert $car_om -o=$car_pbf`)

    fast_om = joinpath(dir, "fast.osm.o5m")
    fast_filter = joinpath(root, "filters", "fast.txt")
    @info("$path: Filtering out slow ways [$fast_om]")
    @time run(pipeline(`osmfilter "$car_om" --parameter-file="$fast_filter"`, fast_om))
    @info("$path: Getting map data [$car_om.cache]")

    fast_pbf = joinpath(dir, "fast.osm.pbf")
    @info("$path: Converting from O5M to PBF [$fast_pbf]")
    @time run(`osmconvert $fast_om -o=$fast_pbf`)

    rm(string(car_pbf, ".cache"), force=true)
    map_data = OpenStreetMapX.get_map_data(car_pbf)
    hmap.local_caches[path] = map_data
    @info "Bounds: $(map_data.bounds)"
    hmap.local_bounds[path] = map_data.bounds
    open(LOCAL_BOUNDS, "w") do f
        Serialization.serialize(f, hmap.local_bounds)
    end

    invalidate_global_fast(hmap)
    return
end

function remove_map(hmap::HierarchicalMap, args...)
    root = dirname(@__DIR__)
    dir = joinpath(root, "data", args...)
    rm(dir, recursive=true, force=true)
    path = join([args...], "/")
    delete!(hmap.local_caches, path)
    delete!(hmap.local_bounds, path)
    open(LOCAL_BOUNDS, "w") do f
        Serialization.serialize(f, hmap.local_bounds)
    end
    invalidate_global_fast(hmap)
    return
end

function invalidate_global_fast(h::HierarchicalMap)
    h.fast_map = nothing
    root = dirname(@__DIR__)
    fast_pbf = joinpath(root, "data", "global_fast.osm.pbf")
    rm(fast_pbf, force = true)
    fast_pbf_cache = joinpath(root, "data", "global_fast.osm.pbf.cache")
    rm(fast_pbf_cache, force = true)
end

function vehicule_map(vehicule::String, prefix)
    default = prefix * ".osm.pbf"
    if vehicule == DEFAULT_VEHICULE
        return default
    else
        pbf = prefix * "_" * vehicule * ".osm.pbf"
        if !isfile(pbf)
            default_om = prefix * ".osm.o5m"
            om = prefix * "_" * vehicule * ".osm.o5m"
            root = dirname(@__DIR__)
            vehicule_filter = joinpath(root, "filters", vehicule * ".txt")
            if !isfile(vehicule_filter)
                error("Type de vehicule `$vehicule` invalide: pas de filtre `$vehicule_filter`.")
            end
            @info("Filter for vehicule `$vehicule` [$om]")
            @time run(pipeline(`osmfilter "$default_om" --parameter-file="$vehicule_filter"`, om))
            @info("Converting from O5M to PBF [$pbf]")
            @time run(`osmconvert $om -o=$pbf`)
        end
        return pbf
    end
end

function global_fast(hmap::HierarchicalMap)
    if hmap.fast_map === nothing
        root = dirname(@__DIR__)
        fast_pbf = joinpath(root, "data", "global_fast.osm.pbf")
        global_fast_om = joinpath(root, "data", "global_fast.osm.o5m")
        if !isfile(fast_pbf)
            cmd = `osmconvert`
            for p in keys(hmap.local_bounds)
                push!(cmd.exec, joinpath(root, "data", p, "fast.osm.o5m"))
            end
            push!(cmd.exec, "-o=$global_fast_om")
            @info("Merging fast O5M's from local regions [$global_fast_om]")
            @time run(cmd)

            global_fast_pbf = joinpath(root, "data", "global_fast.osm.pbf")
            @info("Converting from O5M to PBF [$global_fast_pbf]")
            @time run(`osmconvert $global_fast_om -o=$global_fast_pbf`)
        end
        hmap.fast_map = tmap(vehicule_map(hmap.vehicule, joinpath(root, "data", "global_fast")), hmap.speeds)
    end
    return hmap.fast_map
end

end # module ContractionHierarchies
