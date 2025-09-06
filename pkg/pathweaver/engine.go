package pathweaver

import (
	"fmt"
	"pathweaver/internal/collision"
	"pathweaver/internal/core"
	"pathweaver/internal/pathfinding"
	"pathweaver/internal/scene"
	"pathweaver/internal/spatial"
)

// Engine is the main game scene management engine
type Engine struct {
	sceneManager      *scene.Manager
	collisionDetector *collision.Detector
	pathfinder        core.Pathfinder
	config            *Config
}

// Config holds configuration for the engine
type Config struct {
	SceneBounds         core.AABB
	PathfindingGrid     float64
	PathfindingType     PathfindingType
	MaxPathfindingNodes int
}

// PathfindingType defines the type of pathfinding algorithm to use
type PathfindingType int

const (
	PathfindingAStar PathfindingType = iota
	PathfindingJPS
)

// NewEngine creates a new pathweaver engine
func NewEngine(config *Config) *Engine {
	if config == nil {
		config = DefaultConfig()
	}

	// Create scene manager with quadtree spatial indexing
	sceneManager := scene.NewManager(config.SceneBounds)

	// Create collision detector
	spatialIndex := spatial.NewQuadTree(config.SceneBounds)
	collisionDetector := collision.NewDetector(spatialIndex)

	// Create pathfinder based on configuration
	var pathfinder core.Pathfinder
	switch config.PathfindingType {
	case PathfindingJPS:
		jps := pathfinding.NewJPSPathfinder(config.PathfindingGrid)
		pathfinder = jps
	default: // PathfindingAStar
		astar := pathfinding.NewAStarPathfinder(config.PathfindingGrid)
		astar.SetMaxNodes(config.MaxPathfindingNodes)
		pathfinder = astar
	}

	return &Engine{
		sceneManager:      sceneManager,
		collisionDetector: collisionDetector,
		pathfinder:        pathfinder,
		config:            config,
	}
}

// DefaultConfig returns a default configuration
func DefaultConfig() *Config {
	return &Config{
		SceneBounds: core.AABB{
			Min: core.Vector2D{X: -1000, Y: -1000},
			Max: core.Vector2D{X: 1000, Y: 1000},
		},
		PathfindingGrid:     1.0,
		PathfindingType:     PathfindingAStar,
		MaxPathfindingNodes: 10000,
	}
}

// Entity Management

// AddEntity adds a new entity to the scene
func (e *Engine) AddEntity(entity *core.Entity) error {
	return e.sceneManager.AddEntity(entity)
}

// RemoveEntity removes an entity from the scene
func (e *Engine) RemoveEntity(entityID uint64) error {
	return e.sceneManager.RemoveEntity(entityID)
}

// UpdateEntity updates an entity's position and properties
func (e *Engine) UpdateEntity(entity *core.Entity) error {
	return e.sceneManager.UpdateEntity(entity)
}

// GetEntity retrieves an entity by ID
func (e *Engine) GetEntity(entityID uint64) (*core.Entity, error) {
	return e.sceneManager.GetEntity(entityID)
}

// MoveEntity attempts to move an entity to a new position, checking for collisions
func (e *Engine) MoveEntity(entityID uint64, newPosition core.Vector2D) error {
	entity, err := e.sceneManager.GetEntity(entityID)
	if err != nil {
		return err
	}

	// Check if movement is valid
	canMove, collisions := e.collisionDetector.CheckMovement(entity, newPosition)
	if !canMove {
		return fmt.Errorf("movement blocked by %d collision(s)", len(collisions))
	}

	// Update entity position
	entity.Position = newPosition
	// Update bounds based on new position
	offset := core.Vector2D{
		X: newPosition.X - entity.Position.X,
		Y: newPosition.Y - entity.Position.Y,
	}
	entity.Bounds.Min.X += offset.X
	entity.Bounds.Min.Y += offset.Y
	entity.Bounds.Max.X += offset.X
	entity.Bounds.Max.Y += offset.Y

	return e.sceneManager.UpdateEntity(entity)
}

// Spatial Queries

// GetEntitiesInArea returns all entities within the specified area
func (e *Engine) GetEntitiesInArea(bounds core.AABB) []*core.Entity {
	return e.sceneManager.GetEntitiesInArea(bounds)
}

// GetEntitiesInRadius returns all entities within the specified radius
func (e *Engine) GetEntitiesInRadius(center core.Vector2D, radius float64) []*core.Entity {
	return e.sceneManager.GetEntitiesInRadius(center, radius)
}

// GetNearestEntity returns the nearest entity to the given point
func (e *Engine) GetNearestEntity(point core.Vector2D, maxDistance float64) *core.Entity {
	return e.sceneManager.GetNearestEntity(point, maxDistance)
}

// GetEntitiesByType returns all entities of a specific type
func (e *Engine) GetEntitiesByType(entityType core.EntityType) []*core.Entity {
	return e.sceneManager.GetEntitiesByType(entityType)
}

// Pathfinding

// FindPath finds a path from start to goal using the configured pathfinding algorithm
func (e *Engine) FindPath(start, goal core.Vector2D) ([]core.Vector2D, error) {
	obstacles := e.sceneManager.GetObstacles()
	return e.pathfinder.FindPath(start, goal, obstacles)
}

// SetPathfindingHeuristic sets the heuristic function for pathfinding
func (e *Engine) SetPathfindingHeuristic(heuristic core.HeuristicFunc) {
	e.pathfinder.SetHeuristic(heuristic)
}

// Collision Detection

// CheckCollision checks if an entity would collide at a given position
func (e *Engine) CheckCollision(entity *core.Entity, newPosition core.Vector2D) []collision.CollisionResult {
	return e.collisionDetector.CheckCollision(entity, newPosition)
}

// CheckMovement checks if an entity can move to a new position
func (e *Engine) CheckMovement(entity *core.Entity, newPosition core.Vector2D) (bool, []collision.CollisionResult) {
	return e.collisionDetector.CheckMovement(entity, newPosition)
}

// Raycast performs a raycast and returns the first hit
func (e *Engine) Raycast(start, direction core.Vector2D, maxDistance float64) *collision.CollisionResult {
	return e.collisionDetector.RaycastFirst(start, direction, maxDistance)
}

// RaycastAll performs a raycast and returns all hits
func (e *Engine) RaycastAll(start, direction core.Vector2D, maxDistance float64) []collision.CollisionResult {
	return e.collisionDetector.RaycastAll(start, direction, maxDistance)
}

// OverlapCircle checks for entities overlapping with a circle
func (e *Engine) OverlapCircle(center core.Vector2D, radius float64) []*core.Entity {
	return e.collisionDetector.OverlapCircle(center, radius)
}

// Scene Management

// GetEntityCount returns the total number of entities in the scene
func (e *Engine) GetEntityCount() int {
	return e.sceneManager.GetEntityCount()
}

// GetEntityCountByType returns the number of entities of a specific type
func (e *Engine) GetEntityCountByType(entityType core.EntityType) int {
	return e.sceneManager.GetEntityCountByType(entityType)
}

// GetSceneBounds returns the scene boundaries
func (e *Engine) GetSceneBounds() core.AABB {
	return e.sceneManager.GetBounds()
}

// ClearScene removes all entities from the scene
func (e *Engine) ClearScene() {
	e.sceneManager.Clear()
}

// Batch Operations

// BatchUpdate performs multiple entity updates efficiently
func (e *Engine) BatchUpdate(entities []*core.Entity) error {
	return e.sceneManager.BatchUpdate(entities)
}

// BatchAddEntities adds multiple entities efficiently
func (e *Engine) BatchAddEntities(entities []*core.Entity) error {
	for _, entity := range entities {
		if err := e.sceneManager.AddEntity(entity); err != nil {
			return fmt.Errorf("failed to add entity %d: %w", entity.ID, err)
		}
	}
	return nil
}

// Performance and Debugging

// GetConfig returns the current engine configuration
func (e *Engine) GetConfig() *Config {
	return e.config
}

// GetStats returns performance statistics
func (e *Engine) GetStats() Stats {
	return Stats{
		EntityCount:     e.sceneManager.GetEntityCount(),
		PlayerCount:     e.sceneManager.GetEntityCountByType(core.EntityTypePlayer),
		NPCCount:        e.sceneManager.GetEntityCountByType(core.EntityTypeNPC),
		ObstacleCount:   e.sceneManager.GetEntityCountByType(core.EntityTypeObstacle),
		PickupCount:     e.sceneManager.GetEntityCountByType(core.EntityTypePickup),
		ProjectileCount: e.sceneManager.GetEntityCountByType(core.EntityTypeProjectile),
		SceneBounds:     e.sceneManager.GetBounds(),
	}
}

// Stats represents engine performance statistics
type Stats struct {
	EntityCount     int
	PlayerCount     int
	NPCCount        int
	ObstacleCount   int
	PickupCount     int
	ProjectileCount int
	SceneBounds     core.AABB
}
