# Docs

How-to guides for each demo in this repo. Each guide is self-contained — copy-paste the commands, no extra reading required.

## Per-demo guides

| Guide | What it covers |
|---|---|
| **[01-known-map-demo.md](01-known-map-demo.md)** | Approach 1: known map + A\* + walking simulation. **No LiDAR.** Most-tested demo, this is what to run first. |
| **[02-lidar-mapping.md](02-lidar-mapping.md)** | Approach 2: live LiDAR → 2D occupancy grid → A\* / frontier exploration. Requires LDS-01 hardware. |
| **[03-room-corners.md](03-room-corners.md)** | Helper utility: declare room dimensions, get the four corner positions and the live nearest-corner topic. |

## Reference

| Doc | What it covers |
|---|---|
| **[architecture.md](architecture.md)** | Topic flow per demo, TF chain, coordinate conventions, build order |
