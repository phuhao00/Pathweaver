package core

import "sync"

// Vector2D represents a 2D coordinate/vector
type Vector2D struct {
	X, Y float64
}

// Vector3D represents a 3D coordinate/vector
type Vector3D struct {
	X, Y, Z float64
}

// AABB (Axis-Aligned Bounding Box) represents a rectangular boundary
type AABB struct {
	Min, Max Vector2D
}

// AABB3D (Axis-Aligned Bounding Box 3D) represents a 3D box boundary
type AABB3D struct {
	Min, Max Vector3D
}

// OBB (Oriented Bounding Box) represents a 3D oriented bounding box
type OBB struct {
	Center      Vector3D
	Extents     Vector3D    // Half-widths along each axis
	Orientation [3]Vector3D // Local coordinate axes (must be orthonormal)
}

// Sphere represents a spherical boundary
type Sphere struct {
	Center Vector3D
	Radius float64
}

// Plane represents a 3D plane
type Plane struct {
	Normal   Vector3D
	Distance float64 // Distance from origin along normal
}

// Entity represents any object in the 2D game scene
type Entity struct {
	ID       uint64
	Position Vector2D
	Bounds   AABB
	Type     EntityType
	Static   bool        // Whether this entity can move
	Data     interface{} // Custom data for the entity
}

// Entity3D represents any object in the 3D game scene
type Entity3D struct {
	ID       uint64
	Position Vector3D
	Bounds   interface{} // Can be AABB3D, OBB, or Sphere
	Type     EntityType
	Static   bool        // Whether this entity can move
	Layer    int         // For layered visibility (useful for MMOs)
	Data     interface{} // Custom data for the entity
}

// EntityType represents different types of entities
type EntityType uint8

const (
	EntityTypeUnknown EntityType = iota
	EntityTypePlayer
	EntityTypeNPC
	EntityTypeObstacle
	EntityTypePickup
	EntityTypeProjectile
	EntityTypeBuilding   // For RTS/MOBA buildings
	EntityTypeUnit       // For RTS units
	EntityTypeTerrain    // For terrain elements
	EntityTypeTrigger    // For trigger volumes
	EntityTypeDecoration // For decorative objects
)

// Scene represents the 2D game world/scene
type Scene struct {
	mu       sync.RWMutex
	entities map[uint64]*Entity
	spatial  SpatialIndex
	bounds   AABB
}

// Scene3D represents the 3D game world/scene
type Scene3D struct {
	mu       sync.RWMutex
	entities map[uint64]*Entity3D
	spatial  SpatialIndex3D
	bounds   AABB3D
	layers   map[int]map[uint64]*Entity3D // Layer-based entity management
}

// SpatialIndex interface for 2D spatial data structures
type SpatialIndex interface {
	Insert(entity *Entity) error
	Remove(id uint64) error
	Update(entity *Entity) error
	Query(bounds AABB) []*Entity
	QueryRadius(center Vector2D, radius float64) []*Entity
	GetNearest(point Vector2D, maxDistance float64) *Entity
	Clear()
}

// SpatialIndex3D interface for 3D spatial data structures
type SpatialIndex3D interface {
	Insert(entity *Entity3D) error
	Remove(id uint64) error
	Update(entity *Entity3D) error
	Query(bounds AABB3D) []*Entity3D
	QuerySphere(center Vector3D, radius float64) []*Entity3D
	QueryFrustum(frustum *ViewFrustum) []*Entity3D
	GetNearest(point Vector3D, maxDistance float64) *Entity3D
	Clear()
}

// ViewFrustum represents a camera's view frustum for culling
type ViewFrustum struct {
	Planes [6]Plane // Near, Far, Left, Right, Top, Bottom
}

// PathNode represents a node in 2D pathfinding
type PathNode struct {
	Position Vector2D
	G, H, F  float64 // G: distance from start, H: heuristic, F: G+H
	Parent   *PathNode
	Index    int // Index in priority queue
}

// PathNode3D represents a node in 3D pathfinding
type PathNode3D struct {
	Position Vector3D
	G, H, F  float64 // G: distance from start, H: heuristic, F: G+H
	Parent   *PathNode3D
	Index    int // Index in priority queue
}

// Pathfinder interface for 2D pathfinding algorithms
type Pathfinder interface {
	FindPath(start, goal Vector2D, obstacles []AABB) ([]Vector2D, error)
	SetHeuristic(heuristic HeuristicFunc)
}

// Pathfinder3D interface for 3D pathfinding algorithms
type Pathfinder3D interface {
	FindPath(start, goal Vector3D, obstacles []AABB3D) ([]Vector3D, error)
	SetHeuristic(heuristic HeuristicFunc3D)
}

// HeuristicFunc defines heuristic function for 2D pathfinding
type HeuristicFunc func(a, b Vector2D) float64

// HeuristicFunc3D defines heuristic function for 3D pathfinding
type HeuristicFunc3D func(a, b Vector3D) float64

// GameType represents different types of games for specialized optimizations
type GameType int

const (
	GameTypeGeneric GameType = iota
	GameTypeMMO              // Massively Multiplayer Online
	GameTypeMOBA             // Multiplayer Online Battle Arena
	GameTypeFPS              // First Person Shooter
	GameTypeRTS              // Real Time Strategy
)
