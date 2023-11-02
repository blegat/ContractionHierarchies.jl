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

The following example shows how to use the hierarchical map:
```julia
julia> using ContractionHierarchies

julia> h = HierarchicalMap();

julia> add_map(h, "europe", "andorra")
[ Info: europe/andorra: Downloading PBF format [/home/blegat/.julia/dev/ContractionHierarchies/data/europe/andorra/all.osm.pbf]
Progress: 100%|███████████████████████████████████████| Time: 0:00:00
[ Info: europe/andorra: Converting from PBF to O5M [/home/blegat/.julia/dev/ContractionHierarchies/data/europe/andorra/all.osm.o5m]
  0.061672 seconds (57 allocations: 2.219 KiB)
[ Info: europe/andorra: Filtering out non-map data [/home/blegat/.julia/dev/ContractionHierarchies/data/europe/andorra/local.osm.o5m]
  0.148796 seconds (27.59 k allocations: 1.892 MiB, 19.03% compilation time)
[ Info: europe/andorra: Converting from O5M to PBF [/home/blegat/.julia/dev/ContractionHierarchies/data/europe/andorra/local.osm.pbf]
  0.074395 seconds (57 allocations: 2.219 KiB)
[ Info: europe/andorra: Filtering out slow ways [/home/blegat/.julia/dev/ContractionHierarchies/data/europe/andorra/fast.osm.o5m]
  0.083372 seconds (66 allocations: 3.078 KiB)
[ Info: europe/andorra: Getting map data [/home/blegat/.julia/dev/ContractionHierarchies/data/europe/andorra/local.osm.o5m.cache]
[ Info: europe/andorra: Converting from O5M to PBF [/home/blegat/.julia/dev/ContractionHierarchies/data/europe/andorra/fast.osm.pbf]
  0.056376 seconds (57 allocations: 2.219 KiB)
[ Info: Saved map data to cache /home/blegat/.julia/dev/ContractionHierarchies/data/europe/andorra/local.osm.pbf.cache
[ Info: Bounds: OpenStreetMapX.Bounds{OpenStreetMapX.LLA}(42.4276, 42.65717, 1.412368, 1.787481)

julia> using OpenStreetMapX

julia> shortest_time(h, LLA(42.5, 1.5) => LLA(42.45, 1.45))
[ Info: Merging fast O5M's from local regions [/home/blegat/.julia/dev/ContractionHierarchies/data/global_fast.osm.o5m]
  0.016116 seconds (41 allocations: 1.297 KiB)
[ Info: Converting from O5M to PBF [/home/blegat/.julia/dev/ContractionHierarchies/data/global_fast.osm.pbf]
  0.076948 seconds (57 allocations: 2.211 KiB)
[ Info: Reading map data from `/home/blegat/.julia/dev/ContractionHierarchies/data/global_fast.osm.pbf`.
[ Info: Saved map data to cache /home/blegat/.julia/dev/ContractionHierarchies/data/global_fast.osm.pbf.cache
769.3974754397674

julia> shortest_time(h, LLA(42.5, 1.5) => LLA(42.45, 1.45))
769.3974754397674
```

We need to first load the local map containing the `OpenStreetMapX.LLA`
coordinates, and then we can compute the shortest path with `shortest_time`.
Note that the loaded maps are cached inside `h` so when calling `shortest_time`
the second time, it is not loaded again.

To delete a local map (stored in the `data` directory), use `remove_map`:
```julia
julia> remove_map(h, "europe", "andorra")

julia> shortest_time(h, LLA(42.5, 1.5) => LLA(42.45, 1.45))
ERROR: There is no local map.
Stacktrace:
 [1] error(s::String)
   @ Base ./error.jl:35
 [2] local_node(hmap::HierarchicalMap, lla::LLA)
   @ ContractionHierarchies ~/.julia/dev/ContractionHierarchies/src/ContractionHierarchies.jl:256
```

Note that `shortest_time` first computes the local paths form `src` to the highways and from `dst` to the highways and then compute the highway path to link them.
When computing multiple `shortest_time` using the same `src` or `dst`, these local path don't need to be computed several times.
In order to avoid computing this several time, compute the local path separately with `local_node`:
```julia
julia> local_src = local_node(h, LLA(42.5, 1.5))
ContractionHierarchies.LocalNode("europe/andorra", 1922592503)

julia> local_dst = local_node(h, LLA(42.45, 1.45))
ContractionHierarchies.LocalNode("europe/andorra", 349145254)

julia> shortest_time(h, local_src => local_dst)
769.3974754397674
```
When computing all shortest path for a vector of nodes of type `Vector{OpenStreetMapX.LLA}`, the following convenience
function will first compute all local nodes:
```julia
julia> shortest_time(h, [LLA(42.5, 1.5), LLA(42.45, 1.45)])
2×2 Matrix{Float64}:
   0.0    769.397
 775.148    0.0
```
