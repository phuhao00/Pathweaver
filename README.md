# PathWeaver - Game Scene Element Management Core Library

PathWeaver is a high-performance, server-side game scene management library written in Go. It provides efficient spatial indexing, advanced pathfinding algorithms, and collision detection for game servers and simulation systems.

## Features

### 🗂️ Spatial Data Structures
- **QuadTree Implementation**: Efficient 2D spatial partitioning for fast spatial queries
- **Dynamic Insertion/Removal**: Real-time entity management with O(log n) operations
- **Range Queries**: Fast area-based and radius-based entity lookups

### 🗺️ Advanced Pathfinding Algorithms
- **A* Algorithm**: Classic pathfinding with customizable heuristics
- **Jump Point Search (JPS)**: Optimized A* for grid-based maps with significant performance improvements
- **Multiple Heuristics**: Euclidean, Manhattan, Diagonal, and Octile distance functions
- **Grid-based Navigation**: Configurable grid size for different game scales

### 💥 Collision Detection System
- **AABB Collision Detection**: Axis-aligned bounding box collision system
- **Raycast Support**: Line-of-sight and projectile trajectory calculations
- **Movement Validation**: Collision-aware entity movement
- **Spatial Query Integration**: Leverages spatial indexing for performance

### ⚡ Performance Optimizations
- **Concurrent-Safe Operations**: Thread-safe scene management with RWMutex
- **Batch Operations**: Efficient bulk entity updates and insertions
- **Type-based Indexing**: Fast entity lookups by type
- **Memory Efficient**: Minimal allocations with object pooling strategies

## Installation

```bash
go get github.com/yourusername/pathweaver
```

## Quick Start

```go
package main

import (
    "fmt"
    "pathweaver/pkg/pathweaver"
    "pathweaver/internal/core"
)

func main() {
    // Create engine configuration
    config := &pathweaver.Config{
        SceneBounds: pathweaver.NewAABB(-100, -100, 100, 100),
        PathfindingGrid: 1.0,
        PathfindingType: pathweaver.PathfindingAStar,
    }
    
    // Create the PathWeaver engine
    engine := pathweaver.NewEngine(config)
    
    // Add entities
    player := pathweaver.NewPlayerEntity(1, pathweaver.NewVector2D(0, 0), 2.0)
    obstacle := pathweaver.NewObstacleEntity(2, pathweaver.NewVector2D(5, 5), 3, 3)
    
    engine.AddEntity(player)
    engine.AddEntity(obstacle)
    
    // Find path around obstacles
    start := pathweaver.NewVector2D(-10, -10)
    goal := pathweaver.NewVector2D(10, 10)
    
    path, err := engine.FindPath(start, goal)
    if err != nil {
        fmt.Printf("Pathfinding failed: %v\n", err)
    } else {
        fmt.Printf("Found path with %d waypoints\n", len(path))
    }
    
    // Perform spatial queries
    nearbyEntities := engine.GetEntitiesInRadius(player.Position, 10.0)
    fmt.Printf("Found %d entities near player\n", len(nearbyEntities))
}
```

## API Reference

### Engine Creation

```go
// Create with default configuration
engine := pathweaver.NewEngine(nil)

// Create with custom configuration
config := &pathweaver.Config{
    SceneBounds: pathweaver.NewAABB(-1000, -1000, 1000, 1000),
    PathfindingGrid: 0.5,
    PathfindingType: pathweaver.PathfindingJPS,
    MaxPathfindingNodes: 50000,
}
engine := pathweaver.NewEngine(config)
```

### Entity Management

```go
// Create entities
player := pathweaver.NewPlayerEntity(1, position, 2.0)
npc := pathweaver.NewNPCEntity(2, position, 2.0)
obstacle := pathweaver.NewObstacleEntity(3, position, 5.0, 3.0)

// Add to scene
engine.AddEntity(player)
engine.AddEntity(npc)
engine.AddEntity(obstacle)

// Update entity
player.Position = newPosition
engine.UpdateEntity(player)

// Remove entity
engine.RemoveEntity(player.ID)
```

### Spatial Queries

```go
// Get entities in area
bounds := pathweaver.NewAABB(0, 0, 10, 10)
entities := engine.GetEntitiesInArea(bounds)

// Get entities in radius
center := pathweaver.NewVector2D(5, 5)
entities := engine.GetEntitiesInRadius(center, 10.0)

// Get nearest entity
nearest := engine.GetNearestEntity(position, 50.0)

// Get entities by type
players := engine.GetEntitiesByType(core.EntityTypePlayer)
```

### Pathfinding

```go
// Find path
path, err := engine.FindPath(start, goal)

// Set custom heuristic
engine.SetPathfindingHeuristic(pathweaver.ManhattanDistance)

// Optimize path
optimizedPath := pathweaver.OptimizePath(path, 0.5)
smoothPath := pathweaver.SmoothPath(path, 3)
```

### Collision Detection

```go
// Check if movement is possible
canMove, collisions := engine.CheckMovement(entity, newPosition)

// Perform raycast
hit := engine.Raycast(start, direction, maxDistance)
if hit != nil {
    fmt.Printf("Hit entity %d at %v\n", hit.Entity.ID, hit.ContactPoint)
}

// Check circle overlap
entities := engine.OverlapCircle(center, radius)
```

## Entity Types

PathWeaver supports different entity types with specific behaviors:

- **EntityTypePlayer**: Player characters with collision
- **EntityTypeNPC**: Non-player characters with collision  
- **EntityTypeObstacle**: Static obstacles that block movement
- **EntityTypePickup**: Items that don't block movement
- **EntityTypeProjectile**: Fast-moving objects with minimal collision

## Performance Characteristics

### Spatial Operations
- **Insertion**: O(log n) average case
- **Removal**: O(log n) average case  
- **Range Query**: O(log n + k) where k is result count
- **Nearest Neighbor**: O(log n) average case

### Pathfinding
- **A\***: O(b^d) where b is branching factor, d is depth
- **JPS**: 10-40x faster than A* on open maps
- **Memory Usage**: ~100 bytes per explored node

### Collision Detection
- **AABB vs AABB**: O(1)
- **Raycast**: O(log n + k) where k is intersecting entities
- **Circle Overlap**: O(log n + k)

## Configuration Options

```go
type Config struct {
    SceneBounds         core.AABB       // World boundaries
    PathfindingGrid     float64         // Grid cell size
    PathfindingType     PathfindingType // Algorithm choice
    MaxPathfindingNodes int            // Search limit
}
```

### Pathfinding Types
- `PathfindingAStar`: Classic A* algorithm
- `PathfindingJPS`: Jump Point Search optimization

## Advanced Usage

### Custom Heuristics

```go
// Define custom heuristic function
customHeuristic := func(a, b core.Vector2D) float64 {
    // Your custom distance calculation
    return math.Abs(a.X - b.X) + math.Abs(a.Y - b.Y) * 1.1
}

engine.SetPathfindingHeuristic(customHeuristic)
```

### Batch Operations

```go
// Batch entity updates for better performance
var entities []*core.Entity
// ... populate entities with updated positions
err := engine.BatchUpdate(entities)

// Batch entity creation
var newEntities []*core.Entity
// ... create entities
err := engine.BatchAddEntities(newEntities)
```

### Performance Monitoring

```go
stats := engine.GetStats()
fmt.Printf("Scene has %d entities\n", stats.EntityCount)
fmt.Printf("Obstacles: %d\n", stats.ObstacleCount)
```

## Examples

Check the `examples/` directory for complete usage examples:

- **Basic Demo** (`examples/basic/main.go`): Comprehensive feature demonstration
- **Performance Test**: Large-scale entity management
- **Game Server Integration**: Real-world usage patterns

## Testing

Run the test suite:

```bash
go test ./...
```

Run benchmarks:

```bash
go test -bench=. ./...
```

## Architecture

PathWeaver is designed with modularity and performance in mind:

```
pkg/pathweaver/          # Public API
├── engine.go            # Main engine interface
└── utils.go             # Utility functions

internal/
├── core/                # Core data types
├── spatial/             # Spatial data structures
├── pathfinding/         # Pathfinding algorithms
├── collision/           # Collision detection
└── scene/              # Scene management
```

## Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

## Performance Tips

1. **Choose appropriate grid size**: Smaller grids = more accuracy but slower pathfinding
2. **Use JPS for open maps**: 10-40x faster than A* on sparse obstacle maps
3. **Batch operations**: Group entity updates for better performance
4. **Limit entity count**: Consider level-of-detail systems for large scenes
5. **Pre-allocate**: Use object pools for frequently created/destroyed entities

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Acknowledgments

- A* algorithm implementation inspired by academic research
- Jump Point Search based on the work of Daniel Harabor and Alban Grastien
- Spatial indexing concepts from computational geometry literature
