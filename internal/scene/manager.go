package scene

import (
	"fmt"
	"pathweaver/internal/core"
	"pathweaver/internal/spatial"
	"sync"
	"sync/atomic"
)

// Manager handles the game scene and all entities within it
type Manager struct {
	mu           sync.RWMutex
	entities     map[uint64]*core.Entity
	spatialIndex core.SpatialIndex
	bounds       core.AABB
	nextEntityID uint64

	// Entity type indices for fast queries
	playerEntities     map[uint64]*core.Entity
	npcEntities        map[uint64]*core.Entity
	obstacleEntities   map[uint64]*core.Entity
	pickupEntities     map[uint64]*core.Entity
	projectileEntities map[uint64]*core.Entity
}

// NewManager creates a new scene manager
func NewManager(bounds core.AABB) *Manager {
	return &Manager{
		entities:           make(map[uint64]*core.Entity),
		spatialIndex:       spatial.NewQuadTree(bounds),
		bounds:             bounds,
		nextEntityID:       1,
		playerEntities:     make(map[uint64]*core.Entity),
		npcEntities:        make(map[uint64]*core.Entity),
		obstacleEntities:   make(map[uint64]*core.Entity),
		pickupEntities:     make(map[uint64]*core.Entity),
		projectileEntities: make(map[uint64]*core.Entity),
	}
}

// AddEntity adds a new entity to the scene
func (m *Manager) AddEntity(entity *core.Entity) error {
	if entity == nil {
		return fmt.Errorf("entity cannot be nil")
	}

	m.mu.Lock()
	defer m.mu.Unlock()

	// Assign ID if not set
	if entity.ID == 0 {
		entity.ID = atomic.AddUint64(&m.nextEntityID, 1)
	}

	// Check if entity already exists
	if _, exists := m.entities[entity.ID]; exists {
		return fmt.Errorf("entity with ID %d already exists", entity.ID)
	}

	// Add to spatial index
	if err := m.spatialIndex.Insert(entity); err != nil {
		return fmt.Errorf("failed to add entity to spatial index: %w", err)
	}

	// Add to main entity map
	m.entities[entity.ID] = entity

	// Add to type-specific index
	m.addToTypeIndex(entity)

	return nil
}

// RemoveEntity removes an entity from the scene
func (m *Manager) RemoveEntity(entityID uint64) error {
	m.mu.Lock()
	defer m.mu.Unlock()

	entity, exists := m.entities[entityID]
	if !exists {
		return fmt.Errorf("entity with ID %d not found", entityID)
	}

	// Remove from spatial index
	if err := m.spatialIndex.Remove(entityID); err != nil {
		return fmt.Errorf("failed to remove entity from spatial index: %w", err)
	}

	// Remove from type-specific index
	m.removeFromTypeIndex(entity)

	// Remove from main entity map
	delete(m.entities, entityID)

	return nil
}

// UpdateEntity updates an entity's position and properties
func (m *Manager) UpdateEntity(entity *core.Entity) error {
	if entity == nil {
		return fmt.Errorf("entity cannot be nil")
	}

	m.mu.Lock()
	defer m.mu.Unlock()

	// Check if entity exists
	existing, exists := m.entities[entity.ID]
	if !exists {
		return fmt.Errorf("entity with ID %d not found", entity.ID)
	}

	// Update spatial index if position changed
	if existing.Position != entity.Position || existing.Bounds != entity.Bounds {
		if err := m.spatialIndex.Update(entity); err != nil {
			return fmt.Errorf("failed to update entity in spatial index: %w", err)
		}
	}

	// Update type index if type changed
	if existing.Type != entity.Type {
		m.removeFromTypeIndex(existing)
		m.addToTypeIndex(entity)
	}

	// Update entity
	m.entities[entity.ID] = entity

	return nil
}

// GetEntity retrieves an entity by ID
func (m *Manager) GetEntity(entityID uint64) (*core.Entity, error) {
	m.mu.RLock()
	defer m.mu.RUnlock()

	entity, exists := m.entities[entityID]
	if !exists {
		return nil, fmt.Errorf("entity with ID %d not found", entityID)
	}

	// Return a copy to prevent external modifications
	entityCopy := *entity
	return &entityCopy, nil
}

// GetEntitiesInArea returns all entities within the specified area
func (m *Manager) GetEntitiesInArea(bounds core.AABB) []*core.Entity {
	m.mu.RLock()
	defer m.mu.RUnlock()

	return m.spatialIndex.Query(bounds)
}

// GetEntitiesInRadius returns all entities within the specified radius
func (m *Manager) GetEntitiesInRadius(center core.Vector2D, radius float64) []*core.Entity {
	m.mu.RLock()
	defer m.mu.RUnlock()

	return m.spatialIndex.QueryRadius(center, radius)
}

// GetNearestEntity returns the nearest entity to the given point
func (m *Manager) GetNearestEntity(point core.Vector2D, maxDistance float64) *core.Entity {
	m.mu.RLock()
	defer m.mu.RUnlock()

	return m.spatialIndex.GetNearest(point, maxDistance)
}

// GetEntitiesByType returns all entities of a specific type
func (m *Manager) GetEntitiesByType(entityType core.EntityType) []*core.Entity {
	m.mu.RLock()
	defer m.mu.RUnlock()

	var typeMap map[uint64]*core.Entity

	switch entityType {
	case core.EntityTypePlayer:
		typeMap = m.playerEntities
	case core.EntityTypeNPC:
		typeMap = m.npcEntities
	case core.EntityTypeObstacle:
		typeMap = m.obstacleEntities
	case core.EntityTypePickup:
		typeMap = m.pickupEntities
	case core.EntityTypeProjectile:
		typeMap = m.projectileEntities
	default:
		return nil
	}

	entities := make([]*core.Entity, 0, len(typeMap))
	for _, entity := range typeMap {
		entities = append(entities, entity)
	}

	return entities
}

// GetObstacles returns all obstacle entities as AABB slices for pathfinding
func (m *Manager) GetObstacles() []core.AABB {
	m.mu.RLock()
	defer m.mu.RUnlock()

	obstacles := make([]core.AABB, 0, len(m.obstacleEntities))
	for _, entity := range m.obstacleEntities {
		obstacles = append(obstacles, entity.Bounds)
	}

	return obstacles
}

// GetEntityCount returns the total number of entities
func (m *Manager) GetEntityCount() int {
	m.mu.RLock()
	defer m.mu.RUnlock()

	return len(m.entities)
}

// GetEntityCountByType returns the number of entities of a specific type
func (m *Manager) GetEntityCountByType(entityType core.EntityType) int {
	m.mu.RLock()
	defer m.mu.RUnlock()

	switch entityType {
	case core.EntityTypePlayer:
		return len(m.playerEntities)
	case core.EntityTypeNPC:
		return len(m.npcEntities)
	case core.EntityTypeObstacle:
		return len(m.obstacleEntities)
	case core.EntityTypePickup:
		return len(m.pickupEntities)
	case core.EntityTypeProjectile:
		return len(m.projectileEntities)
	default:
		return 0
	}
}

// Clear removes all entities from the scene
func (m *Manager) Clear() {
	m.mu.Lock()
	defer m.mu.Unlock()

	m.entities = make(map[uint64]*core.Entity)
	m.spatialIndex.Clear()

	// Clear type indices
	m.playerEntities = make(map[uint64]*core.Entity)
	m.npcEntities = make(map[uint64]*core.Entity)
	m.obstacleEntities = make(map[uint64]*core.Entity)
	m.pickupEntities = make(map[uint64]*core.Entity)
	m.projectileEntities = make(map[uint64]*core.Entity)
}

// GetBounds returns the scene boundaries
func (m *Manager) GetBounds() core.AABB {
	m.mu.RLock()
	defer m.mu.RUnlock()

	return m.bounds
}

// SetBounds updates the scene boundaries
func (m *Manager) SetBounds(bounds core.AABB) {
	m.mu.Lock()
	defer m.mu.Unlock()

	m.bounds = bounds
	// Note: In a production system, you might want to rebuild the spatial index
	// when bounds change significantly
}

// BatchUpdate performs multiple entity updates efficiently
func (m *Manager) BatchUpdate(entities []*core.Entity) error {
	m.mu.Lock()
	defer m.mu.Unlock()

	// Validate all entities first
	for _, entity := range entities {
		if entity == nil {
			return fmt.Errorf("entity cannot be nil")
		}
		if _, exists := m.entities[entity.ID]; !exists {
			return fmt.Errorf("entity with ID %d not found", entity.ID)
		}
	}

	// Update all entities
	for _, entity := range entities {
		existing := m.entities[entity.ID]

		// Update spatial index if position changed
		if existing.Position != entity.Position || existing.Bounds != entity.Bounds {
			if err := m.spatialIndex.Update(entity); err != nil {
				return fmt.Errorf("failed to update entity %d in spatial index: %w", entity.ID, err)
			}
		}

		// Update type index if type changed
		if existing.Type != entity.Type {
			m.removeFromTypeIndex(existing)
			m.addToTypeIndex(entity)
		}

		// Update entity
		m.entities[entity.ID] = entity
	}

	return nil
}

// addToTypeIndex adds an entity to the appropriate type index
func (m *Manager) addToTypeIndex(entity *core.Entity) {
	switch entity.Type {
	case core.EntityTypePlayer:
		m.playerEntities[entity.ID] = entity
	case core.EntityTypeNPC:
		m.npcEntities[entity.ID] = entity
	case core.EntityTypeObstacle:
		m.obstacleEntities[entity.ID] = entity
	case core.EntityTypePickup:
		m.pickupEntities[entity.ID] = entity
	case core.EntityTypeProjectile:
		m.projectileEntities[entity.ID] = entity
	}
}

// removeFromTypeIndex removes an entity from the appropriate type index
func (m *Manager) removeFromTypeIndex(entity *core.Entity) {
	switch entity.Type {
	case core.EntityTypePlayer:
		delete(m.playerEntities, entity.ID)
	case core.EntityTypeNPC:
		delete(m.npcEntities, entity.ID)
	case core.EntityTypeObstacle:
		delete(m.obstacleEntities, entity.ID)
	case core.EntityTypePickup:
		delete(m.pickupEntities, entity.ID)
	case core.EntityTypeProjectile:
		delete(m.projectileEntities, entity.ID)
	}
}
