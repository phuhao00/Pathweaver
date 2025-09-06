package spatial

import (
	"fmt"
	"math"
	"pathweaver/internal/core"
)

const (
	// MaxEntitiesPerOctNode defines when to split an octree node
	MaxEntitiesPerOctNode = 8
	// MaxOctreeDepth defines maximum depth of the octree
	MaxOctreeDepth = 10
	// LooseFactor defines how much larger loose octree bounds are (typically 2.0)
	LooseFactor = 2.0
)

// Octree implements a 3D spatial index using octree data structure
// Based on "Real-Time Collision Detection" by Christer Ericson and
// "Introduction to Game Development" spatial partitioning techniques
type Octree struct {
	bounds   core.AABB3D
	entities map[uint64]*core.Entity3D
	root     *octNode
	loose    bool // Whether to use loose octree for better dynamic object handling
}

// octNode represents a node in the octree
type octNode struct {
	bounds   core.AABB3D
	entities map[uint64]*core.Entity3D
	children [8]*octNode // Octants: [NWU, NEU, SWU, SEU, NWD, NED, SWD, SED]
	depth    int
	parent   *octNode
}

// OctantIndices for readability
const (
	NWU = 0 // North-West-Up
	NEU = 1 // North-East-Up
	SWU = 2 // South-West-Up
	SEU = 3 // South-East-Up
	NWD = 4 // North-West-Down
	NED = 5 // North-East-Down
	SWD = 6 // South-West-Down
	SED = 7 // South-East-Down
)

// NewOctree creates a new octree with the given bounds
func NewOctree(bounds core.AABB3D, loose bool) *Octree {
	return &Octree{
		bounds:   bounds,
		entities: make(map[uint64]*core.Entity3D),
		root: &octNode{
			bounds:   bounds,
			entities: make(map[uint64]*core.Entity3D),
			depth:    0,
		},
		loose: loose,
	}
}

// Insert adds an entity to the octree
func (ot *Octree) Insert(entity *core.Entity3D) error {
	if entity == nil {
		return fmt.Errorf("entity cannot be nil")
	}

	entityBounds, err := ot.getEntityBounds(entity)
	if err != nil {
		return err
	}

	if !ot.contains(ot.bounds, entityBounds) {
		return fmt.Errorf("entity bounds %+v outside octree bounds %+v", entityBounds, ot.bounds)
	}

	ot.entities[entity.ID] = entity
	ot.root.insert(entity, entityBounds)
	return nil
}

// Remove removes an entity from the octree
func (ot *Octree) Remove(id uint64) error {
	entity, exists := ot.entities[id]
	if !exists {
		return fmt.Errorf("entity with id %d not found", id)
	}

	delete(ot.entities, id)
	ot.root.remove(entity)
	return nil
}

// Update updates an entity's position in the octree
func (ot *Octree) Update(entity *core.Entity3D) error {
	if entity == nil {
		return fmt.Errorf("entity cannot be nil")
	}

	// Remove and re-insert (simple but effective for dynamic objects)
	if _, exists := ot.entities[entity.ID]; exists {
		ot.root.remove(entity)
	}

	ot.entities[entity.ID] = entity
	return ot.Insert(entity)
}

// Query returns all entities within the given bounds
func (ot *Octree) Query(bounds core.AABB3D) []*core.Entity3D {
	var results []*core.Entity3D
	ot.root.query(bounds, &results)
	return results
}

// QuerySphere returns all entities within the given sphere
func (ot *Octree) QuerySphere(center core.Vector3D, radius float64) []*core.Entity3D {
	// Create bounding box for the sphere
	bounds := core.AABB3D{
		Min: core.Vector3D{X: center.X - radius, Y: center.Y - radius, Z: center.Z - radius},
		Max: core.Vector3D{X: center.X + radius, Y: center.Y + radius, Z: center.Z + radius},
	}

	candidates := ot.Query(bounds)
	var results []*core.Entity3D

	// Filter by actual distance to sphere
	for _, entity := range candidates {
		entityBounds, err := ot.getEntityBounds(entity)
		if err != nil {
			continue
		}

		if ot.sphereIntersectsAABB(center, radius, entityBounds) {
			results = append(results, entity)
		}
	}

	return results
}

// QueryFrustum returns all entities within the view frustum (for rendering culling)
func (ot *Octree) QueryFrustum(frustum *core.ViewFrustum) []*core.Entity3D {
	var results []*core.Entity3D
	ot.root.queryFrustum(frustum, &results)
	return results
}

// GetNearest returns the nearest entity to the given point
func (ot *Octree) GetNearest(point core.Vector3D, maxDistance float64) *core.Entity3D {
	var nearest *core.Entity3D
	minDistance := maxDistance

	// Query a reasonable area around the point
	queryBounds := core.AABB3D{
		Min: core.Vector3D{X: point.X - maxDistance, Y: point.Y - maxDistance, Z: point.Z - maxDistance},
		Max: core.Vector3D{X: point.X + maxDistance, Y: point.Y + maxDistance, Z: point.Z + maxDistance},
	}

	candidates := ot.Query(queryBounds)
	for _, entity := range candidates {
		distance := ot.distanceToEntity(point, entity)
		if distance < minDistance {
			minDistance = distance
			nearest = entity
		}
	}

	return nearest
}

// Clear removes all entities from the octree
func (ot *Octree) Clear() {
	ot.entities = make(map[uint64]*core.Entity3D)
	ot.root = &octNode{
		bounds:   ot.bounds,
		entities: make(map[uint64]*core.Entity3D),
		depth:    0,
	}
}

// insert adds an entity to this node or its children
func (on *octNode) insert(entity *core.Entity3D, entityBounds core.AABB3D) {
	// If this node has children, try to insert into appropriate child
	if on.children[0] != nil {
		childIndex := on.getChildIndex(entityBounds)
		if childIndex != -1 {
			on.children[childIndex].insert(entity, entityBounds)
			return
		}
	}

	// Add to this node
	on.entities[entity.ID] = entity

	// Split if necessary
	if len(on.entities) > MaxEntitiesPerOctNode && on.depth < MaxOctreeDepth {
		on.split()
	}
}

// remove removes an entity from this node or its children
func (on *octNode) remove(entity *core.Entity3D) {
	delete(on.entities, entity.ID)

	// Also try to remove from children
	if on.children[0] != nil {
		for _, child := range on.children {
			if child != nil {
				child.remove(entity)
			}
		}
	}
}

// query finds all entities within the given bounds
func (on *octNode) query(bounds core.AABB3D, results *[]*core.Entity3D) {
	// Add entities from this node that intersect
	for _, entity := range on.entities {
		entityBounds, err := getEntityBounds3D(entity)
		if err == nil && intersects3D(bounds, entityBounds) {
			*results = append(*results, entity)
		}
	}

	// Query children if they exist and intersect
	if on.children[0] != nil {
		for _, child := range on.children {
			if child != nil && intersects3D(bounds, child.bounds) {
				child.query(bounds, results)
			}
		}
	}
}

// queryFrustum finds all entities within the view frustum
func (on *octNode) queryFrustum(frustum *core.ViewFrustum, results *[]*core.Entity3D) {
	// Check if node bounds intersect with frustum
	if !on.frustumIntersects(frustum, on.bounds) {
		return
	}

	// Add entities from this node that are in frustum
	for _, entity := range on.entities {
		entityBounds, err := getEntityBounds3D(entity)
		if err == nil && on.frustumContains(frustum, entityBounds) {
			*results = append(*results, entity)
		}
	}

	// Query children
	if on.children[0] != nil {
		for _, child := range on.children {
			if child != nil {
				child.queryFrustum(frustum, results)
			}
		}
	}
}

// split divides this node into eight children
func (on *octNode) split() {
	midX := (on.bounds.Min.X + on.bounds.Max.X) / 2
	midY := (on.bounds.Min.Y + on.bounds.Max.Y) / 2
	midZ := (on.bounds.Min.Z + on.bounds.Max.Z) / 2

	// Create child bounds for all 8 octants
	childBounds := [8]core.AABB3D{
		// Upper octants (Z > midZ)
		{Min: core.Vector3D{X: on.bounds.Min.X, Y: midY, Z: midZ}, Max: core.Vector3D{X: midX, Y: on.bounds.Max.Y, Z: on.bounds.Max.Z}}, // NWU
		{Min: core.Vector3D{X: midX, Y: midY, Z: midZ}, Max: on.bounds.Max},                                                             // NEU
		{Min: core.Vector3D{X: on.bounds.Min.X, Y: on.bounds.Min.Y, Z: midZ}, Max: core.Vector3D{X: midX, Y: midY, Z: on.bounds.Max.Z}}, // SWU
		{Min: core.Vector3D{X: midX, Y: on.bounds.Min.Y, Z: midZ}, Max: core.Vector3D{X: on.bounds.Max.X, Y: midY, Z: on.bounds.Max.Z}}, // SEU

		// Lower octants (Z < midZ)
		{Min: core.Vector3D{X: on.bounds.Min.X, Y: midY, Z: on.bounds.Min.Z}, Max: core.Vector3D{X: midX, Y: on.bounds.Max.Y, Z: midZ}}, // NWD
		{Min: core.Vector3D{X: midX, Y: midY, Z: on.bounds.Min.Z}, Max: core.Vector3D{X: on.bounds.Max.X, Y: on.bounds.Max.Y, Z: midZ}}, // NED
		{Min: on.bounds.Min, Max: core.Vector3D{X: midX, Y: midY, Z: midZ}},                                                             // SWD
		{Min: core.Vector3D{X: midX, Y: on.bounds.Min.Y, Z: on.bounds.Min.Z}, Max: core.Vector3D{X: on.bounds.Max.X, Y: midY, Z: midZ}}, // SED
	}

	// Create children
	for i := 0; i < 8; i++ {
		on.children[i] = &octNode{
			bounds:   childBounds[i],
			entities: make(map[uint64]*core.Entity3D),
			depth:    on.depth + 1,
			parent:   on,
		}
	}

	// Redistribute entities
	for _, entity := range on.entities {
		entityBounds, err := getEntityBounds3D(entity)
		if err != nil {
			continue
		}

		childIndex := on.getChildIndex(entityBounds)
		if childIndex != -1 {
			on.children[childIndex].insert(entity, entityBounds)
			delete(on.entities, entity.ID)
		}
	}
}

// getChildIndex returns which child octant the bounds belong to
func (on *octNode) getChildIndex(bounds core.AABB3D) int {
	if on.children[0] == nil {
		return -1
	}

	for i, child := range on.children {
		if child != nil && contains3D(child.bounds, bounds) {
			return i
		}
	}

	return -1 // Doesn't fit completely in any child
}

// Helper functions

func (ot *Octree) getEntityBounds(entity *core.Entity3D) (core.AABB3D, error) {
	return getEntityBounds3D(entity)
}

func getEntityBounds3D(entity *core.Entity3D) (core.AABB3D, error) {
	switch bounds := entity.Bounds.(type) {
	case core.AABB3D:
		return bounds, nil
	case core.Sphere:
		return core.AABB3D{
			Min: core.Vector3D{X: bounds.Center.X - bounds.Radius, Y: bounds.Center.Y - bounds.Radius, Z: bounds.Center.Z - bounds.Radius},
			Max: core.Vector3D{X: bounds.Center.X + bounds.Radius, Y: bounds.Center.Y + bounds.Radius, Z: bounds.Center.Z + bounds.Radius},
		}, nil
	case core.OBB:
		// Convert OBB to AABB (conservative approximation)
		halfExtents := core.Vector3D{
			X: math.Abs(bounds.Extents.X*bounds.Orientation[0].X) + math.Abs(bounds.Extents.Y*bounds.Orientation[1].X) + math.Abs(bounds.Extents.Z*bounds.Orientation[2].X),
			Y: math.Abs(bounds.Extents.X*bounds.Orientation[0].Y) + math.Abs(bounds.Extents.Y*bounds.Orientation[1].Y) + math.Abs(bounds.Extents.Z*bounds.Orientation[2].Y),
			Z: math.Abs(bounds.Extents.X*bounds.Orientation[0].Z) + math.Abs(bounds.Extents.Y*bounds.Orientation[1].Z) + math.Abs(bounds.Extents.Z*bounds.Orientation[2].Z),
		}
		return core.AABB3D{
			Min: core.Vector3D{X: bounds.Center.X - halfExtents.X, Y: bounds.Center.Y - halfExtents.Y, Z: bounds.Center.Z - halfExtents.Z},
			Max: core.Vector3D{X: bounds.Center.X + halfExtents.X, Y: bounds.Center.Y + halfExtents.Y, Z: bounds.Center.Z + halfExtents.Z},
		}, nil
	default:
		return core.AABB3D{}, fmt.Errorf("unsupported bounds type: %T", bounds)
	}
}

func (ot *Octree) contains(container, bounds core.AABB3D) bool {
	return bounds.Min.X >= container.Min.X && bounds.Max.X <= container.Max.X &&
		bounds.Min.Y >= container.Min.Y && bounds.Max.Y <= container.Max.Y &&
		bounds.Min.Z >= container.Min.Z && bounds.Max.Z <= container.Max.Z
}

func contains3D(container, bounds core.AABB3D) bool {
	return bounds.Min.X >= container.Min.X && bounds.Max.X <= container.Max.X &&
		bounds.Min.Y >= container.Min.Y && bounds.Max.Y <= container.Max.Y &&
		bounds.Min.Z >= container.Min.Z && bounds.Max.Z <= container.Max.Z
}

func intersects3D(a, b core.AABB3D) bool {
	return a.Min.X < b.Max.X && a.Max.X > b.Min.X &&
		a.Min.Y < b.Max.Y && a.Max.Y > b.Min.Y &&
		a.Min.Z < b.Max.Z && a.Max.Z > b.Min.Z
}

func (ot *Octree) sphereIntersectsAABB(center core.Vector3D, radius float64, aabb core.AABB3D) bool {
	// Find closest point on AABB to sphere center
	closestX := math.Max(aabb.Min.X, math.Min(center.X, aabb.Max.X))
	closestY := math.Max(aabb.Min.Y, math.Min(center.Y, aabb.Max.Y))
	closestZ := math.Max(aabb.Min.Z, math.Min(center.Z, aabb.Max.Z))

	// Calculate distance from sphere center to closest point
	dx := center.X - closestX
	dy := center.Y - closestY
	dz := center.Z - closestZ

	return (dx*dx + dy*dy + dz*dz) <= (radius * radius)
}

func (ot *Octree) distanceToEntity(point core.Vector3D, entity *core.Entity3D) float64 {
	entityBounds, err := ot.getEntityBounds(entity)
	if err != nil {
		// Fallback to position distance
		dx := point.X - entity.Position.X
		dy := point.Y - entity.Position.Y
		dz := point.Z - entity.Position.Z
		return math.Sqrt(dx*dx + dy*dy + dz*dz)
	}

	// Distance to AABB
	dx := math.Max(0, math.Max(entityBounds.Min.X-point.X, point.X-entityBounds.Max.X))
	dy := math.Max(0, math.Max(entityBounds.Min.Y-point.Y, point.Y-entityBounds.Max.Y))
	dz := math.Max(0, math.Max(entityBounds.Min.Z-point.Z, point.Z-entityBounds.Max.Z))

	return math.Sqrt(dx*dx + dy*dy + dz*dz)
}

// frustumIntersects checks if the node bounds intersect with the view frustum
func (on *octNode) frustumIntersects(frustum *core.ViewFrustum, bounds core.AABB3D) bool {
	// Check against all 6 frustum planes
	for _, plane := range frustum.Planes {
		// Find the positive vertex (vertex most in the direction of plane normal)
		positiveVertex := core.Vector3D{
			X: bounds.Min.X,
			Y: bounds.Min.Y,
			Z: bounds.Min.Z,
		}

		if plane.Normal.X >= 0 {
			positiveVertex.X = bounds.Max.X
		}
		if plane.Normal.Y >= 0 {
			positiveVertex.Y = bounds.Max.Y
		}
		if plane.Normal.Z >= 0 {
			positiveVertex.Z = bounds.Max.Z
		}

		// If positive vertex is outside plane, the box is completely outside
		if dotProduct3D(plane.Normal, positiveVertex)+plane.Distance < 0 {
			return false
		}
	}

	return true
}

// frustumContains checks if bounds are completely within the frustum
func (on *octNode) frustumContains(frustum *core.ViewFrustum, bounds core.AABB3D) bool {
	// For simplicity, use intersection test (conservative)
	return on.frustumIntersects(frustum, bounds)
}

func dotProduct3D(a, b core.Vector3D) float64 {
	return a.X*b.X + a.Y*b.Y + a.Z*b.Z
}
