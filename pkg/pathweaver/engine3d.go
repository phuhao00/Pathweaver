package pathweaver

import (
	"fmt"
	"pathweaver/internal/ai"
	"pathweaver/internal/collision"
	"pathweaver/internal/core"
	"pathweaver/internal/fps"
	"pathweaver/internal/mmo"
	"pathweaver/internal/moba"
	"pathweaver/internal/pathfinding"
	"pathweaver/internal/spatial"
)

// Engine3D is the main 3D game scene management engine
type Engine3D struct {
	spatialIndex      core.SpatialIndex3D
	collisionDetector *collision.Detector3D
	pathfinder3D      core.Pathfinder3D
	config            *Config3D

	// Game-specific optimizations
	zoneManager  *mmo.ZoneManager       // For MMO games
	bspTree      *fps.BSPTree           // For FPS games
	influenceMap *moba.InfluenceMap     // For MOBA games
	flowField    *pathfinding.FlowField // For RTS games

	// AI systems
	goapAgents map[uint64]*ai.GOAPAgent
}

// Config3D holds configuration for the 3D engine
type Config3D struct {
	SceneBounds         core.AABB3D
	PathfindingGrid     float64
	PathfindingType     PathfindingType3D
	MovementType        pathfinding.MovementType
	MaxPathfindingNodes int
	GameType            core.GameType

	// Game-specific settings
	MMOZoneSize          float64 // For MMO zone management
	InfluenceMapCellSize float64 // For MOBA influence maps
	FlowFieldCellSize    float64 // For RTS flow fields
	UseOctree            bool    // Use Octree vs other spatial structures
	UseBSP               bool    // Use BSP tree for FPS games
}

// PathfindingType3D defines the type of 3D pathfinding algorithm to use
type PathfindingType3D int

const (
	Pathfinding3DAStar PathfindingType3D = iota
	Pathfinding3DJPS
	Pathfinding3DHierarchical
)

// NewEngine3D creates a new 3D PathWeaver engine
func NewEngine3D(config *Config3D) *Engine3D {
	if config == nil {
		config = DefaultConfig3D()
	}

	var spatialIndex core.SpatialIndex3D
	if config.UseOctree {
		spatialIndex = spatial.NewOctree(config.SceneBounds, false) // Regular octree
	} else {
		// Could add other spatial structures here (loose octree, etc.)
		spatialIndex = spatial.NewOctree(config.SceneBounds, true) // Loose octree
	}

	// Create collision detector
	collisionDetector := collision.NewDetector3D(spatialIndex)

	// Create 3D pathfinder based on configuration
	var pathfinder3D core.Pathfinder3D
	switch config.PathfindingType {
	case Pathfinding3DJPS:
		// JPS 3D would be implemented here
		pathfinder3D = pathfinding.NewAStar3DPathfinder(config.PathfindingGrid, config.MovementType)
	default: // Pathfinding3DAStar
		pathfinder3D = pathfinding.NewAStar3DPathfinder(config.PathfindingGrid, config.MovementType)
	}

	engine := &Engine3D{
		spatialIndex:      spatialIndex,
		collisionDetector: collisionDetector,
		pathfinder3D:      pathfinder3D,
		config:            config,
		goapAgents:        make(map[uint64]*ai.GOAPAgent),
	}

	// Initialize game-specific systems
	engine.initializeGameSpecificSystems()

	return engine
}

// DefaultConfig3D returns a default 3D configuration
func DefaultConfig3D() *Config3D {
	return &Config3D{
		SceneBounds: core.AABB3D{
			Min: core.Vector3D{X: -1000, Y: -1000, Z: -100},
			Max: core.Vector3D{X: 1000, Y: 1000, Z: 100},
		},
		PathfindingGrid:      1.0,
		PathfindingType:      Pathfinding3DAStar,
		MovementType:         pathfinding.MovementGround,
		MaxPathfindingNodes:  50000,
		GameType:             core.GameTypeGeneric,
		MMOZoneSize:          100.0,
		InfluenceMapCellSize: 2.0,
		FlowFieldCellSize:    1.0,
		UseOctree:            true,
		UseBSP:               false,
	}
}

func (e *Engine3D) initializeGameSpecificSystems() {
	switch e.config.GameType {
	case core.GameTypeMMO:
		// Initialize MMO systems
		e.zoneManager = mmo.NewZoneManager(e.config.SceneBounds, e.config.MMOZoneSize)

	case core.GameTypeMOBA:
		// Initialize MOBA systems
		bounds2D := core.AABB{
			Min: core.Vector2D{X: e.config.SceneBounds.Min.X, Y: e.config.SceneBounds.Min.Y},
			Max: core.Vector2D{X: e.config.SceneBounds.Max.X, Y: e.config.SceneBounds.Max.Y},
		}
		e.influenceMap = moba.NewInfluenceMap(bounds2D, e.config.InfluenceMapCellSize)

	case core.GameTypeRTS:
		// Initialize RTS systems (Flow Fields)
		bounds2D := core.AABB{
			Min: core.Vector2D{X: e.config.SceneBounds.Min.X, Y: e.config.SceneBounds.Min.Y},
			Max: core.Vector2D{X: e.config.SceneBounds.Max.X, Y: e.config.SceneBounds.Max.Y},
		}
		e.flowField = pathfinding.NewFlowField(bounds2D, e.config.FlowFieldCellSize)

	case core.GameTypeFPS:
		// BSP tree would be initialized from level geometry
		if e.config.UseBSP {
			// e.bspTree = fps.NewBSPTree(levelPolygons)
			// BSP tree creation requires actual level geometry
		}
	}
}

// 3D Entity Management

// AddEntity3D adds a 3D entity to the scene
func (e *Engine3D) AddEntity3D(entity *core.Entity3D) error {
	if err := e.spatialIndex.Insert(entity); err != nil {
		return err
	}

	// Add to game-specific systems
	if e.zoneManager != nil {
		e.zoneManager.AddEntity(entity)
	}

	return nil
}

// RemoveEntity3D removes a 3D entity from the scene
func (e *Engine3D) RemoveEntity3D(entityID uint64) error {
	if err := e.spatialIndex.Remove(entityID); err != nil {
		return err
	}

	// Remove from game-specific systems
	if e.zoneManager != nil {
		e.zoneManager.RemoveEntity(entityID)
	}

	// Remove from AI systems
	delete(e.goapAgents, entityID)

	return nil
}

// UpdateEntity3D updates a 3D entity's position and properties
func (e *Engine3D) UpdateEntity3D(entity *core.Entity3D) error {
	if err := e.spatialIndex.Update(entity); err != nil {
		return err
	}

	// Update in game-specific systems
	if e.zoneManager != nil {
		e.zoneManager.UpdateEntity(entity)
	}

	return nil
}

// 3D Spatial Queries

// GetEntitiesInBox3D returns all entities within the specified 3D box
func (e *Engine3D) GetEntitiesInBox3D(bounds core.AABB3D) []*core.Entity3D {
	return e.spatialIndex.Query(bounds)
}

// GetEntitiesInSphere3D returns all entities within the specified sphere
func (e *Engine3D) GetEntitiesInSphere3D(center core.Vector3D, radius float64) []*core.Entity3D {
	return e.spatialIndex.QuerySphere(center, radius)
}

// GetEntitiesInFrustum returns all entities within the view frustum (for rendering)
func (e *Engine3D) GetEntitiesInFrustum(frustum *core.ViewFrustum) []*core.Entity3D {
	return e.spatialIndex.QueryFrustum(frustum)
}

// GetNearestEntity3D returns the nearest 3D entity to the given point
func (e *Engine3D) GetNearestEntity3D(point core.Vector3D, maxDistance float64) *core.Entity3D {
	return e.spatialIndex.GetNearest(point, maxDistance)
}

// 3D Pathfinding

// FindPath3D finds a 3D path from start to goal
func (e *Engine3D) FindPath3D(start, goal core.Vector3D, obstacles []core.AABB3D) ([]core.Vector3D, error) {
	return e.pathfinder3D.FindPath(start, goal, obstacles)
}

// SetPathfindingHeuristic3D sets the heuristic function for 3D pathfinding
func (e *Engine3D) SetPathfindingHeuristic3D(heuristic core.HeuristicFunc3D) {
	e.pathfinder3D.SetHeuristic(heuristic)
}

// SetMovementType sets the movement type for 3D pathfinding
func (e *Engine3D) SetMovementType(movementType pathfinding.MovementType) {
	if astar, ok := e.pathfinder3D.(*pathfinding.AStar3DPathfinder); ok {
		astar.SetMovementType(movementType)
	}
}

// 3D Collision Detection

// CheckCollision3D checks if an entity would collide at a given 3D position
func (e *Engine3D) CheckCollision3D(entity *core.Entity3D, newPosition core.Vector3D) []collision.CollisionResult3D {
	return e.collisionDetector.CheckCollision(entity, newPosition)
}

// Raycast3D performs a 3D raycast and returns the first hit
func (e *Engine3D) Raycast3D(start, direction core.Vector3D, maxDistance float64) *collision.CollisionResult3D {
	return e.collisionDetector.Raycast3D(start, direction, maxDistance)
}

// Game-Specific Features

// MMO Features

// GetPlayerInterestArea returns entities a player should be aware of (MMO)
func (e *Engine3D) GetPlayerInterestArea(playerID uint64, position core.Vector3D) *mmo.InterestArea {
	if e.zoneManager == nil {
		return nil
	}

	return e.zoneManager.GetInterestArea(playerID, position)
}

// GetZonePopulation returns population statistics for MMO zones
func (e *Engine3D) GetZonePopulation(position core.Vector3D) (int, int) {
	if e.zoneManager == nil {
		return 0, 0
	}

	zoneID := e.zoneManager.GetZoneID(position)
	return e.zoneManager.GetZonePopulation(zoneID)
}

// MOBA Features

// AddInfluenceSource adds an influence source for MOBA tactical AI
func (e *Engine3D) AddInfluenceSource(source moba.InfluenceSource) {
	if e.influenceMap != nil {
		e.influenceMap.AddInfluenceSource(source)
	}
}

// GetTeamAdvantage calculates team advantage at a position (MOBA)
func (e *Engine3D) GetTeamAdvantage(position core.Vector2D, team moba.TeamID) float64 {
	if e.influenceMap == nil {
		return 0
	}

	return e.influenceMap.GetTeamAdvantage(position, team)
}

// FindSafePath finds the safest path considering team influence (MOBA)
func (e *Engine3D) FindSafePath(start, goal core.Vector2D, team moba.TeamID) []core.Vector2D {
	if e.influenceMap == nil {
		return []core.Vector2D{start, goal}
	}

	return e.influenceMap.FindSafePath(start, goal, team)
}

// RTS Features

// GenerateFlowField generates a flow field for RTS unit movement
func (e *Engine3D) GenerateFlowField(goals []core.Vector2D) {
	if e.flowField != nil {
		e.flowField.Generate(goals)
	}
}

// GetFlowDirection returns the flow direction at a position (RTS)
func (e *Engine3D) GetFlowDirection(position core.Vector2D) core.Vector2D {
	if e.flowField == nil {
		return core.Vector2D{X: 0, Y: 0}
	}

	return e.flowField.GetFlowAt(position)
}

// FPS Features

// CreateBSPTree creates a BSP tree for FPS level geometry
func (e *Engine3D) CreateBSPTree(polygons []fps.Polygon) {
	if e.config.UseBSP {
		e.bspTree = fps.NewBSPTree(polygons)
	}
}

// GetVisiblePolygons returns visible polygons for FPS rendering
func (e *Engine3D) GetVisiblePolygons(viewpoint, viewDirection core.Vector3D) []fps.Polygon {
	if e.bspTree == nil {
		return nil
	}

	return e.bspTree.VisibilityQuery(viewpoint, viewDirection)
}

// RaycastBSP performs fast raycast using BSP tree (FPS hit detection)
func (e *Engine3D) RaycastBSP(origin, direction core.Vector3D, maxDistance float64) *fps.RaycastHit {
	if e.bspTree == nil {
		return nil
	}

	return e.bspTree.RaycastBSP(origin, direction, maxDistance)
}

// AI Features

// AddGOAPAgent adds a GOAP AI agent
func (e *Engine3D) AddGOAPAgent(entityID uint64, agent *ai.GOAPAgent) {
	e.goapAgents[entityID] = agent
}

// UpdateGOAPAgent updates a GOAP agent's AI
func (e *Engine3D) UpdateGOAPAgent(entityID uint64, performer ai.ActionPerformer) error {
	agent, exists := e.goapAgents[entityID]
	if !exists {
		return fmt.Errorf("GOAP agent not found for entity %d", entityID)
	}

	return agent.Update(performer)
}

// CreateCombatAgent creates a specialized combat AI agent
func (e *Engine3D) CreateCombatAgent(entityID uint64) *ai.CombatAgent {
	agent := ai.NewCombatAgent()
	e.goapAgents[entityID] = agent.GOAPAgent
	return agent
}

// Update Systems

// Update performs one update cycle for all systems
func (e *Engine3D) Update(deltaTime float64) {
	// Update influence map for MOBA games
	if e.influenceMap != nil {
		e.influenceMap.Update(deltaTime)
	}

	// Update AI agents
	// Note: In a real implementation, you'd pass the appropriate ActionPerformer
	// for each agent based on the entity it controls
}

// Performance and Statistics

// GetConfig3D returns the current engine configuration
func (e *Engine3D) GetConfig3D() *Config3D {
	return e.config
}

// GetStats3D returns 3D performance statistics
func (e *Engine3D) GetStats3D() Stats3D {
	stats := Stats3D{
		SceneBounds: e.config.SceneBounds,
		GameType:    e.config.GameType,
	}

	// Add game-specific stats
	if e.zoneManager != nil {
		// Add MMO zone statistics
		stats.ActiveZones = len(e.zoneManager.GetHottestZones(100))
	}

	if e.influenceMap != nil {
		// Add MOBA influence map statistics
		stats.InfluenceMapActive = true
	}

	stats.ActiveGOAPAgents = len(e.goapAgents)

	return stats
}

// Stats3D represents 3D engine performance statistics
type Stats3D struct {
	SceneBounds        core.AABB3D
	GameType           core.GameType
	ActiveZones        int  // For MMO
	InfluenceMapActive bool // For MOBA
	FlowFieldActive    bool // For RTS
	BSPTreeActive      bool // For FPS
	ActiveGOAPAgents   int  // AI agents count
}

// Utility Functions

// CreateEntity3D creates a new 3D entity with the given parameters
func CreateEntity3D(id uint64, entityType core.EntityType, position core.Vector3D, bounds interface{}) *core.Entity3D {
	return &core.Entity3D{
		ID:       id,
		Type:     entityType,
		Position: position,
		Bounds:   bounds,
		Static:   entityType == core.EntityTypeObstacle || entityType == core.EntityTypeBuilding,
		Layer:    0,
	}
}

// CreatePlayer3D creates a new 3D player entity
func CreatePlayer3D(id uint64, position core.Vector3D, radius float64) *core.Entity3D {
	bounds := core.Sphere{
		Center: position,
		Radius: radius,
	}
	return CreateEntity3D(id, core.EntityTypePlayer, position, bounds)
}

// CreateBuilding3D creates a new 3D building entity with AABB bounds
func CreateBuilding3D(id uint64, position core.Vector3D, width, height, depth float64) *core.Entity3D {
	bounds := core.AABB3D{
		Min: core.Vector3D{
			X: position.X - width/2,
			Y: position.Y - height/2,
			Z: position.Z - depth/2,
		},
		Max: core.Vector3D{
			X: position.X + width/2,
			Y: position.Y + height/2,
			Z: position.Z + depth/2,
		},
	}
	return CreateEntity3D(id, core.EntityTypeBuilding, position, bounds)
}

// CreateViewFrustum creates a view frustum for rendering culling
func CreateViewFrustum(position, forward, up core.Vector3D, fov, aspect, near, far float64) *core.ViewFrustum {
	// This is a simplified implementation
	// In practice, you'd calculate the 6 frustum planes properly

	// For now, create a simple frustum approximation
	frustum := &core.ViewFrustum{}

	// Near plane
	frustum.Planes[0] = core.Plane{
		Normal:   forward,
		Distance: -near,
	}

	// Far plane
	frustum.Planes[1] = core.Plane{
		Normal:   core.Vector3D{X: -forward.X, Y: -forward.Y, Z: -forward.Z},
		Distance: far,
	}

	// For simplicity, set other planes to reasonable defaults
	// In a real implementation, you'd calculate left, right, top, bottom planes
	// based on FOV and aspect ratio

	return frustum
}
