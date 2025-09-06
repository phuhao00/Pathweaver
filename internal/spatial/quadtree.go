package spatial

import (
	"fmt"
	"math"
	"pathweaver/internal/core"
)

const (
	// MaxEntitiesPerNode defines when to split a quadtree node
	MaxEntitiesPerNode = 10
	// MaxDepth defines maximum depth of the quadtree
	MaxDepth = 8
)

// QuadTree implements a spatial index using quadtree data structure
type QuadTree struct {
	bounds   core.AABB
	entities map[uint64]*core.Entity
	root     *quadNode
}

// quadNode represents a node in the quadtree
type quadNode struct {
	bounds   core.AABB
	entities map[uint64]*core.Entity
	children [4]*quadNode // NW, NE, SW, SE
	depth    int
}

// NewQuadTree creates a new quadtree with the given bounds
func NewQuadTree(bounds core.AABB) *QuadTree {
	return &QuadTree{
		bounds:   bounds,
		entities: make(map[uint64]*core.Entity),
		root: &quadNode{
			bounds:   bounds,
			entities: make(map[uint64]*core.Entity),
			depth:    0,
		},
	}
}

// Insert adds an entity to the quadtree
func (qt *QuadTree) Insert(entity *core.Entity) error {
	if entity == nil {
		return fmt.Errorf("entity cannot be nil")
	}

	if !qt.contains(qt.bounds, entity.Bounds) {
		return fmt.Errorf("entity bounds %+v outside quadtree bounds %+v", entity.Bounds, qt.bounds)
	}

	qt.entities[entity.ID] = entity
	qt.root.insert(entity)
	return nil
}

// Remove removes an entity from the quadtree
func (qt *QuadTree) Remove(id uint64) error {
	entity, exists := qt.entities[id]
	if !exists {
		return fmt.Errorf("entity with id %d not found", id)
	}

	delete(qt.entities, id)
	qt.root.remove(entity)
	return nil
}

// Update updates an entity's position in the quadtree
func (qt *QuadTree) Update(entity *core.Entity) error {
	if entity == nil {
		return fmt.Errorf("entity cannot be nil")
	}

	// Remove and re-insert (simple but effective)
	if _, exists := qt.entities[entity.ID]; exists {
		qt.root.remove(entity)
	}

	qt.entities[entity.ID] = entity
	return qt.Insert(entity)
}

// Query returns all entities within the given bounds
func (qt *QuadTree) Query(bounds core.AABB) []*core.Entity {
	var results []*core.Entity
	qt.root.query(bounds, &results)
	return results
}

// QueryRadius returns all entities within the given radius from center
func (qt *QuadTree) QueryRadius(center core.Vector2D, radius float64) []*core.Entity {
	// Create bounding box for the radius
	bounds := core.AABB{
		Min: core.Vector2D{X: center.X - radius, Y: center.Y - radius},
		Max: core.Vector2D{X: center.X + radius, Y: center.Y + radius},
	}

	candidates := qt.Query(bounds)
	var results []*core.Entity

	// Filter by actual distance
	for _, entity := range candidates {
		if qt.distanceToAABB(center, entity.Bounds) <= radius {
			results = append(results, entity)
		}
	}

	return results
}

// GetNearest returns the nearest entity to the given point
func (qt *QuadTree) GetNearest(point core.Vector2D, maxDistance float64) *core.Entity {
	var nearest *core.Entity
	minDistance := maxDistance

	// Query a reasonable area around the point
	queryBounds := core.AABB{
		Min: core.Vector2D{X: point.X - maxDistance, Y: point.Y - maxDistance},
		Max: core.Vector2D{X: point.X + maxDistance, Y: point.Y + maxDistance},
	}

	candidates := qt.Query(queryBounds)
	for _, entity := range candidates {
		distance := qt.distanceToAABB(point, entity.Bounds)
		if distance < minDistance {
			minDistance = distance
			nearest = entity
		}
	}

	return nearest
}

// Clear removes all entities from the quadtree
func (qt *QuadTree) Clear() {
	qt.entities = make(map[uint64]*core.Entity)
	qt.root = &quadNode{
		bounds:   qt.bounds,
		entities: make(map[uint64]*core.Entity),
		depth:    0,
	}
}

// insert adds an entity to this node or its children
func (qn *quadNode) insert(entity *core.Entity) {
	// If this node has children, try to insert into appropriate child
	if qn.children[0] != nil {
		childIndex := qn.getChildIndex(entity.Bounds)
		if childIndex != -1 {
			qn.children[childIndex].insert(entity)
			return
		}
	}

	// Add to this node
	qn.entities[entity.ID] = entity

	// Split if necessary
	if len(qn.entities) > MaxEntitiesPerNode && qn.depth < MaxDepth {
		qn.split()
	}
}

// remove removes an entity from this node or its children
func (qn *quadNode) remove(entity *core.Entity) {
	delete(qn.entities, entity.ID)

	// Also try to remove from children
	if qn.children[0] != nil {
		for _, child := range qn.children {
			if child != nil {
				child.remove(entity)
			}
		}
	}
}

// query finds all entities within the given bounds
func (qn *quadNode) query(bounds core.AABB, results *[]*core.Entity) {
	// Add entities from this node that intersect
	for _, entity := range qn.entities {
		if intersects(bounds, entity.Bounds) {
			*results = append(*results, entity)
		}
	}

	// Query children if they exist and intersect
	if qn.children[0] != nil {
		for _, child := range qn.children {
			if child != nil && intersects(bounds, child.bounds) {
				child.query(bounds, results)
			}
		}
	}
}

// split divides this node into four children
func (qn *quadNode) split() {
	midX := (qn.bounds.Min.X + qn.bounds.Max.X) / 2
	midY := (qn.bounds.Min.Y + qn.bounds.Max.Y) / 2

	// Create child bounds (NW, NE, SW, SE)
	childBounds := [4]core.AABB{
		{Min: core.Vector2D{X: qn.bounds.Min.X, Y: midY}, Max: core.Vector2D{X: midX, Y: qn.bounds.Max.Y}}, // NW
		{Min: core.Vector2D{X: midX, Y: midY}, Max: qn.bounds.Max},                                         // NE
		{Min: qn.bounds.Min, Max: core.Vector2D{X: midX, Y: midY}},                                         // SW
		{Min: core.Vector2D{X: midX, Y: qn.bounds.Min.Y}, Max: core.Vector2D{X: qn.bounds.Max.X, Y: midY}}, // SE
	}

	// Create children
	for i := 0; i < 4; i++ {
		qn.children[i] = &quadNode{
			bounds:   childBounds[i],
			entities: make(map[uint64]*core.Entity),
			depth:    qn.depth + 1,
		}
	}

	// Redistribute entities
	for _, entity := range qn.entities {
		childIndex := qn.getChildIndex(entity.Bounds)
		if childIndex != -1 {
			qn.children[childIndex].insert(entity)
			delete(qn.entities, entity.ID)
		}
	}
}

// getChildIndex returns which child quadrant the bounds belong to
func (qn *quadNode) getChildIndex(bounds core.AABB) int {
	if qn.children[0] == nil {
		return -1
	}

	for i, child := range qn.children {
		if child != nil && qn.contains(child.bounds, bounds) {
			return i
		}
	}

	return -1 // Doesn't fit completely in any child
}

// Helper functions

func (qt *QuadTree) contains(container, bounds core.AABB) bool {
	return bounds.Min.X >= container.Min.X && bounds.Max.X <= container.Max.X &&
		bounds.Min.Y >= container.Min.Y && bounds.Max.Y <= container.Max.Y
}

func (qn *quadNode) contains(container, bounds core.AABB) bool {
	return bounds.Min.X >= container.Min.X && bounds.Max.X <= container.Max.X &&
		bounds.Min.Y >= container.Min.Y && bounds.Max.Y <= container.Max.Y
}

func intersects(a, b core.AABB) bool {
	return a.Min.X < b.Max.X && a.Max.X > b.Min.X &&
		a.Min.Y < b.Max.Y && a.Max.Y > b.Min.Y
}

func (qt *QuadTree) distanceToAABB(point core.Vector2D, bounds core.AABB) float64 {
	dx := math.Max(0, math.Max(bounds.Min.X-point.X, point.X-bounds.Max.X))
	dy := math.Max(0, math.Max(bounds.Min.Y-point.Y, point.Y-bounds.Max.Y))
	return math.Sqrt(dx*dx + dy*dy)
}
