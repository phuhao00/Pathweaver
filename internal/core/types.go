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

// Entity represents any object in the game scene
type Entity struct {
	ID       uint64
	Position Vector2D
	Bounds   AABB
	Type     EntityType
	Static   bool        // Whether this entity can move
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
)

// Scene represents the game world/scene
type Scene struct {
	mu       sync.RWMutex
	entities map[uint64]*Entity
	spatial  SpatialIndex
	bounds   AABB
}

// SpatialIndex interface for spatial data structures
type SpatialIndex interface {
	Insert(entity *Entity) error
	Remove(id uint64) error
	Update(entity *Entity) error
	Query(bounds AABB) []*Entity
	QueryRadius(center Vector2D, radius float64) []*Entity
	GetNearest(point Vector2D, maxDistance float64) *Entity
	Clear()
}

// PathNode represents a node in pathfinding
type PathNode struct {
	Position Vector2D
	G, H, F  float64 // G: distance from start, H: heuristic, F: G+H
	Parent   *PathNode
	Index    int // Index in priority queue
}

// Pathfinder interface for different pathfinding algorithms
type Pathfinder interface {
	FindPath(start, goal Vector2D, obstacles []AABB) ([]Vector2D, error)
	SetHeuristic(heuristic HeuristicFunc)
}

// HeuristicFunc defines heuristic function for pathfinding
type HeuristicFunc func(a, b Vector2D) float64
