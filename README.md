# Contraction Hierarchies

## Requirements

* Julia v1.6 or more recent
* `osmconvert` v0.8.10 or more recent
* `osmfilter` v1.4.4 or more recent

# Cleanup

When updating, it might be safer to clean up the cached files as they
might not be compatible with the update:
```
$ find . -name '*cache' -delete
```

To compute the shortest time between two coordinates `src` and `dst`
which are `OpenStreetMapX.LLA` in a single map `map.osm.pbf`, use
```julia
shortest_path("map.osm.pbf", src, dst)'
```

Note that this loads the map `map.osm.pbf` and first extract the whole map in a Julia datastructure.
Then, it generates the graph with the data needed for `shortest_path` and store this graph in a file `map.osm.pbf.cache`.
The second call of `shortest_time` with the same map will be much faster as
it will directly read the graph in `map.osm.pbf.cache` and will not need to go through `map.osm.pbf` which contains other metadata that are not used by `shortest_path`.

### Hierarchical map

To use the hierarchical map, use the following:
```julia
julia> shortest_path(src, dest)
```

If the hierarchical map does not contain the right local map, you must first download it with:
```julia
julia> h = HierchicalMap();

julia> add_map(h, "europe", "andorra")
```

To delete a local map, use `remove_map`:
```julia
julia> remove_map(h, "europe", "andorra")
```

Note that `shortest_time` first computes the local paths form `src` to the highways and from `dst` to the highways and then compute the highway path to link them.
When computing multiple `shortest_time` using the same `src` or `dst`, these local path don't need to be computed several times.
In order to avoid computing this several time, compute the local path separately with `local_node`:
```julia
julia> local_src = local_node(h, src)

julia> local_dst = local_node(h, dst)

julia> local_dst = shortest_time(h, local_src, local_dst)
```
When computing all shortest path for a vector of nodes of type `Vector{OpenStreetMapX.LLA}`, the following convenience
function will first compute all local nodes:
```julia
julia> shortest_time(h, nodes)
```
