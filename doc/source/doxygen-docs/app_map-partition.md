\page app_map-partition map-partition

# map-partition

Splits a map (a `.simplemap` file, i.e. a sequence of keyframes with robot
poses and observations) into clusters of keyframes that observe the same parts
of the environment, using spectral graph partitioning (normalized cut) of the
similarity graph between keyframes. It is a front-end for
mrpt::slam::CIncrementalMapPartitioner. Typical uses: building submaps for
hierarchical or topological mapping, and analyzing the structure of a map.

## Usage

    map-partition <file.simplemap> [nCut_threshold]

- `file.simplemap`: input map (`.simplemap` or `.simplemap.gz`).
- `nCut_threshold`: optional normalized cut threshold, in the range [0,2]
  (default: 0.9).

## Example

    map-partition modules/mrpt_data/datasets/malaga-cs-fac-building.simplemap.gz 0.8

After partitioning, two windows show the adjacency (similarity) matrix before
and after rearranging it by partitions; press a key on them to continue.

## Configuration file

Optional. If a file named `MAP-PARTITION_CONFIG.ini` exists in the current
directory, the options are loaded from its `[PARAMS]` section (see
mrpt::slam::CIncrementalMapPartitioner::TOptions):

- `partitionThreshold`: normalized cut threshold (overridden by the command
  line argument).
- `forceBisectionOnly`: only split into two parts.
- `simil_method`: `smMETRIC_MAP_MATCHING` (default) or `smOBSERVATION_OVERLAP`.
- `minimumNumberElementsEachCluster`, `maxKeyFrameDistanceToEval`,
  `minDistForCorrespondence`, `minMahaDistForCorrespondence`.
- The metric map used to compare keyframes, defined in `[PARAMS.metricmap]`
  (default: a point cloud).

The options in use are printed at startup.

## Output

All files are written to `./MAP-PARTITION_RESULTS/` (its previous contents
are deleted):

| File | Contents |
|---|---|
| `out_partitions_<map>.txt` | One line per partition, with the indices of its keyframes. |
| `out_part#NNN.simplemap` | One `.simplemap` per partition. |
| `matrix_A.txt`, `matrix_B.txt` | Adjacency matrix, original and rearranged by partitions. |
| `ADJ_MATRIX_BEFORE.png`, `ADJ_MATRIX_AFTER.png` | Images of both matrices. |
| `globalmap_grid*` | Occupancy grid built from the whole map. |
| `seePartitionResults.m` | MATLAB script that draws the partitions over the grid map. |

## See also

- [robot-map-gui](page_app_robot-map-gui.html): to inspect `.simplemap` files.
- mrpt::slam::CIncrementalMapPartitioner, mrpt::maps::CSimpleMap
