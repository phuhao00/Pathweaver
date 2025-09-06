# PathWeaver - Advanced 3D Game Scene Management Library

PathWeaver is a cutting-edge, high-performance game scene management library written in Go, designed for modern 2D and 3D game servers. It provides state-of-the-art spatial indexing, advanced pathfinding algorithms, AI systems, and game-specific optimizations for MMO, MOBA, FPS, and RTS games.

## 🚀 Features

### 🌐 3D Spatial Data Structures
- **Octree Implementation**: Advanced 3D spatial partitioning with loose/tight variants
- **QuadTree Support**: Efficient 2D spatial indexing for legacy systems
- **Dynamic Insertion/Removal**: Real-time entity management with O(log n) operations
- **View Frustum Culling**: Optimized rendering queries for 3D graphics
- **Multi-layer Management**: Support for MMO zone-based entity organization

### 🗺️ Advanced Pathfinding Algorithms

#### 2D Pathfinding
- **A* Algorithm**: Classic pathfinding with customizable heuristics
- **Jump Point Search (JPS)**: 10-40x faster than A* on sparse maps
- **Flow Fields**: Efficient pathfinding for large groups (RTS games)

#### 3D Pathfinding
- **3D A* Algorithm**: Full 3D pathfinding with movement type optimization
- **Movement Types**: Ground, Flight, Swimming, Climbing behaviors
- **3D Jump Point Search**: Optimized 3D pathfinding (implementation ready)
- **Hierarchical Pathfinding**: Multi-level pathfinding for large worlds

#### Heuristics
- Euclidean, Manhattan, Diagonal, Octile distance functions
- 3D-specific: Layered, Flight, Ground, Adaptive heuristics
- Custom movement cost modifiers for different terrains

### 💥 Advanced Collision Detection

#### 2D Collision Detection
- **AABB vs AABB**: Axis-aligned bounding box collision
- **Circle Collision**: Efficient circular collision detection

#### 3D Collision Detection
- **AABB3D Collision**: 3D axis-aligned bounding box collision
- **Sphere Collision**: 3D spherical collision detection
- **OBB Collision**: Oriented bounding box collision (with SAT)
- **Ray Casting**: 3D line-of-sight and projectile calculations
- **Collision Resolution**: Penetration depth and contact normal calculation

### 🎮 Game-Specific Optimizations

#### 🌍 MMO (Massively Multiplayer Online)
- **Zone Management**: Dynamic world partitioning and load balancing
- **Interest Management**: Efficient player awareness systems
- **Population Tracking**: Real-time zone population monitoring
- **Level-of-Detail**: Automatic detail reduction based on load

#### ⚔️ MOBA (Multiplayer Online Battle Arena)
- **Influence Maps**: Tactical AI decision making systems
- **Team Advantage**: Dynamic battlefield analysis
- **Safe Pathfinding**: Influence-aware path planning
- **Multi-layer Influence**: Damage, safety, control, vision layers

#### 🔫 FPS (First Person Shooter)
- **BSP Trees**: Binary Space Partitioning for level geometry
- **PVS Support**: Potentially Visible Set calculations
- **Fast Raycast**: Optimized hit detection and line-of-sight
- **Material Detection**: Surface material identification for audio/effects

#### 🏗️ RTS (Real-Time Strategy)
- **Flow Fields**: Efficient multi-unit pathfinding
- **Formation Systems**: Group movement coordination
- **Resource Management**: Strategic AI positioning

### 🧠 Advanced AI Systems

#### GOAP (Goal-Oriented Action Planning)
- **Dynamic Behavior**: AI agents that adapt to changing conditions
- **Action Planning**: Automatic behavior sequence generation
- **Combat AI**: Specialized combat decision making
- **State Management**: Robust world state tracking

#### AI Features
- **Behavior Trees**: Hierarchical AI decision making (ready for integration)
- **Finite State Machines**: Classic AI state management
- **Utility Systems**: Score-based AI decision making

### ⚡ Performance Optimizations
- **Concurrent-Safe Operations**: Thread-safe scene management with RWMutex
- **Batch Operations**: Efficient bulk entity updates and insertions
- **Memory Pooling**: Reduced garbage collection overhead
- **SIMD Optimization**: Ready for vector instruction acceleration

## Installation

```bash
go get github.com/yourusername/pathweaver
```

## Quick Start

### 2D Game Engine
```go
package main

import (
    "pathweaver/pkg/pathweaver"
    "pathweaver/internal/core"
)

func main() {
    // Create 2D engine
    config := pathweaver.DefaultConfig()
    engine := pathweaver.NewEngine(config)
    
    // Add entities
    player := pathweaver.NewPlayerEntity(1, pathweaver.NewVector2D(0, 0), 2.0)
    engine.AddEntity(player)
    
    // Find path
    path, _ := engine.FindPath(
        pathweaver.NewVector2D(-10, -10), 
        pathweaver.NewVector2D(10, 10),
    )
}
```

### 3D Game Engine
```go
package main

import (
    "pathweaver/pkg/pathweaver"
    "pathweaver/internal/core"
)

func main() {
    // Create 3D engine with game-specific optimizations
    config := &pathweaver.Config3D{
        GameType: core.GameTypeMMO, // or MOBA, FPS, RTS
        SceneBounds: core.AABB3D{
            Min: core.Vector3D{X: -1000, Y: -1000, Z: -100},
            Max: core.Vector3D{X: 1000, Y: 1000, Z: 100},
        },
        MovementType: pathfinding.MovementGround,
    }
    engine := pathweaver.NewEngine3D(config)
    
    // Add 3D entities
    player := pathweaver.CreatePlayer3D(1, 
        core.Vector3D{X: 0, Y: 0, Z: 0}, 1.5)
    engine.AddEntity3D(player)
    
    // 3D pathfinding
    path, _ := engine.FindPath3D(
        core.Vector3D{X: 0, Y: 0, Z: 0},
        core.Vector3D{X: 100, Y: 50, Z: 10},
        obstacles,
    )
    
    // 3D spatial queries
    nearbyEntities := engine.GetEntitiesInSphere3D(
        core.Vector3D{X: 0, Y: 0, Z: 5}, 25.0)
}
```

### Advanced AI with GOAP
```go
package main

import (
    "pathweaver/internal/ai"
    "pathweaver/internal/core"
)

func main() {
    // Create intelligent combat AI
    combatAgent := ai.NewCombatAgent()
    
    // Set up environment
    combatAgent.Position = core.Vector2D{X: 0, Y: 0}
    combatAgent.Enemies = []core.Vector2D{{X: 20, Y: 5}}
    combatAgent.CoverPoints = []core.Vector2D{{X: -5, Y: 10}}
    
    // AI automatically plans and executes actions
    combatAgent.Update(combatAgent) // Take cover, reload, attack, etc.
}
```

## Game-Specific Examples

### MMO Zone Management
```go
// Get player's interest area for networking optimization
interestArea := engine.GetPlayerInterestArea(playerID, position)
for _, entity := range interestArea.Entities {
    // Send entity updates only to interested players
    sendEntityUpdate(entity)
}

// Check zone population for load balancing
population, activePlayers := engine.GetZonePopulation(position)
if population > maxCapacity {
    // Implement load balancing
}
```

### MOBA Tactical AI
```go
// Add influence sources for tactical decision making
towerInfluence := moba.InfluenceSource{
    Position:  core.Vector2D{X: 50, Y: 50},
    Team:      moba.TeamBlue,
    Influence: 100.0,
    Range:     25.0,
    Layer:     moba.LayerDamage,
}
engine.AddInfluenceSource(towerInfluence)

// Find tactically safe path
safePath := engine.FindSafePath(start, goal, moba.TeamBlue)
```

### RTS Flow Fields
```go
// Generate flow field for multiple rally points
rallyPoints := []core.Vector2D{
    {X: 100, Y: 120}, // Resource depot
    {X: 0, Y: 50},    // Factory
}
engine.GenerateFlowField(rallyPoints)

// Units automatically follow optimal paths
for _, unit := range units {
    flowDirection := engine.GetFlowDirection(unit.Position)
    unit.Move(flowDirection)
}
```

### FPS Hit Detection
```go
// Fast raycast for weapon systems
hit := engine.Raycast3D(weaponPos, aimDirection, maxRange)
if hit != nil {
    // Apply damage, play effects based on material
    applyDamage(hit.Entity, calculateDamage(hit.MaterialType))
}
```

## Advanced Features

### Custom Heuristics
```go
// Define terrain-aware pathfinding
customHeuristic := func(a, b core.Vector3D) float64 {
    baseDistance := pathfinding.EuclideanDistance3D(a, b)
    terrainModifier := getTerrainDifficulty(a, b)
    return baseDistance * terrainModifier
}
engine.SetPathfindingHeuristic3D(customHeuristic)
```

### Batch Operations for Performance
```go
// Efficient bulk updates
var entities []*core.Entity3D
// ... populate entities with updated positions
err := engine.BatchUpdate3D(entities)
```

### View Frustum Culling
```go
// Optimize rendering by culling invisible objects
frustum := pathweaver.CreateViewFrustum(cameraPos, forward, up, fov, aspect, near, far)
visibleEntities := engine.GetEntitiesInFrustum(frustum)
```

## Performance Characteristics

### Spatial Operations (3D)
- **Octree Insertion**: O(log n) average case
- **Octree Query**: O(log n + k) where k is result count
- **Frustum Culling**: O(log n + k) with early termination
- **Sphere Query**: O(log n + k) with distance filtering

### Advanced Pathfinding
- **3D A\***: O(b^d) with 3D branching factor optimizations
- **Flow Fields**: O(n) generation, O(1) query per agent
- **GOAP Planning**: O(b^d) with intelligent pruning

### Game-Specific Optimizations
- **MMO Zone Management**: O(1) zone lookup, O(k) interest calculation
- **MOBA Influence Maps**: O(1) influence lookup, O(n) decay per frame
- **FPS BSP Traversal**: O(log n) visibility determination

## Research-Based Implementation

PathWeaver incorporates techniques from cutting-edge research papers and industry best practices:

### Spatial Indexing
- "Real-Time Collision Detection" by Christer Ericson
- "Introduction to Game Development" spatial partitioning techniques
- Loose vs. tight octree optimizations for dynamic objects

### Pathfinding
- Original A* algorithm with 3D extensions
- Jump Point Search optimizations for grid-based pathfinding
- Flow field algorithms from RTS game development

### AI Systems
- Goal-Oriented Action Planning from F.E.A.R. game development
- Influence maps from academic AI research
- Utility-based AI systems

### Game-Specific Techniques
- BSP tree implementations from classic FPS engines (Quake, Doom)
- MMO zone management from large-scale online games
- MOBA tactical systems from competitive gaming research

## Examples

The `examples/` directory contains comprehensive demonstrations:

- **Basic Demo** (`examples/basic/main.go`): 2D game basics
- **Advanced 3D Demo** (`examples/advanced_3d/main.go`): Full 3D feature showcase
- **MMO Server** (`examples/mmo_server/`): Scalable MMO implementation
- **MOBA AI** (`examples/moba_ai/`): Tactical AI demonstration
- **FPS Engine** (`examples/fps_engine/`): Shooter game integration

## Testing

Comprehensive test suite covering all major systems:

```bash
# Run all tests
go test ./...

# Run with race detection
go test -race ./...

# Benchmark performance
go test -bench=. ./...

# Test specific systems
go test ./internal/spatial -v      # Spatial indexing
go test ./internal/pathfinding -v  # Pathfinding algorithms
go test ./internal/collision -v    # Collision detection
go test ./internal/ai -v          # AI systems
```

## Architecture

```
pkg/pathweaver/          # Public APIs
├── engine.go           # 2D engine interface
├── engine3d.go         # 3D engine interface
└── utils.go            # Utility functions

internal/
├── core/               # Core data types and interfaces
├── spatial/            # Spatial data structures (QuadTree, Octree)
├── pathfinding/        # Pathfinding algorithms (A*, JPS, Flow Fields)
├── collision/          # Collision detection (2D/3D)
├── ai/                 # AI systems (GOAP, behavior trees)
├── mmo/               # MMO-specific optimizations
├── moba/              # MOBA-specific features
├── fps/               # FPS-specific systems (BSP)
└── scene/             # Scene management
```

## Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add amazing 3D feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

## Roadmap

### Upcoming Features
- **Navigation Meshes**: Advanced 3D pathfinding for complex geometry
- **Hierarchical Pathfinding**: Multi-level pathfinding for open worlds
- **Machine Learning Integration**: AI behavior learning systems
- **Vulkan/Metal Integration**: GPU-accelerated spatial queries
- **WebAssembly Support**: Browser-based game server support

### Performance Improvements
- **SIMD Optimizations**: Vector instruction acceleration
- **GPU Compute**: OpenCL/CUDA spatial processing
- **Lock-free Algorithms**: Reduced contention for high-throughput systems

## Performance Tips

1. **Choose appropriate data structures**: Octree for 3D, QuadTree for 2D
2. **Use game-specific optimizations**: Enable MMO/MOBA/FPS/RTS features as needed
3. **Batch operations**: Group entity updates for better performance
4. **Tune parameters**: Adjust grid sizes and tree depths for your use case
5. **Profile and measure**: Use Go's built-in profiling tools

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Acknowledgments

- A* and JPS algorithms from pathfinding research literature
- BSP tree techniques from classic game engines (id Tech, Quake)
- GOAP implementation based on F.E.A.R. AI architecture
- Octree optimizations from real-time collision detection research
- Flow field algorithms from RTS game development best practices
- Influence map concepts from academic AI research papers

## Citation

If you use PathWeaver in academic research, please cite:

```
@software{pathweaver2024,
  title = {PathWeaver: Advanced 3D Game Scene Management Library},
  year = {2024},
  publisher = {GitHub},
  journal = {GitHub repository},
  howpublished = {\url{https://github.com/yourusername/pathweaver}}
}
```