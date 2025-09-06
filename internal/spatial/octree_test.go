package spatial

import (
	"pathweaver/internal/core"
	"testing"
)

func TestOctreeBasicOperations(t *testing.T) {
	bounds := core.AABB3D{
		Min: core.Vector3D{X: -50, Y: -50, Z: -50},
		Max: core.Vector3D{X: 50, Y: 50, Z: 50},
	}
	octree := NewOctree(bounds, false)
	
	// Test insert
	entity := &core.Entity3D{
		ID:       1,
		Position: core.Vector3D{X: 10, Y: 10, Z: 10},
		Bounds: core.AABB3D{
			Min: core.Vector3D{X: 9, Y: 9, Z: 9},
			Max: core.Vector3D{X: 11, Y: 11, Z: 11},
		},
		Type: core.EntityTypePlayer,
	}
	
	err := octree.Insert(entity)
	if err != nil {
		t.Fatalf("Failed to insert entity: %v", err)
	}
	
	// Test query
	queryBounds := core.AABB3D{
		Min: core.Vector3D{X: 5, Y: 5, Z: 5},
		Max: core.Vector3D{X: 15, Y: 15, Z: 15},
	}
	
	results := octree.Query(queryBounds)
	if len(results) != 1 {
		t.Fatalf("Expected 1 result, got %d", len(results))
	}
	
	if results[0].ID != entity.ID {
		t.Fatalf("Expected entity ID %d, got %d", entity.ID, results[0].ID)
	}
	
	// Test remove
	err = octree.Remove(entity.ID)
	if err != nil {
		t.Fatalf("Failed to remove entity: %v", err)
	}
	
	results = octree.Query(queryBounds)
	if len(results) != 0 {
		t.Fatalf("Expected 0 results after removal, got %d", len(results))
	}
}

func TestOctreeSphereQuery(t *testing.T) {
	bounds := core.AABB3D{
		Min: core.Vector3D{X: -50, Y: -50, Z: -50},
		Max: core.Vector3D{X: 50, Y: 50, Z: 50},
	}
	octree := NewOctree(bounds, false)
	
	// Insert entities at different positions
	entities := []*core.Entity3D{
		{
			ID:       1,
			Position: core.Vector3D{X: 0, Y: 0, Z: 0},
			Bounds:   core.Sphere{Center: core.Vector3D{X: 0, Y: 0, Z: 0}, Radius: 1},
			Type:     core.EntityTypePlayer,
		},
		{
			ID:       2,
			Position: core.Vector3D{X: 5, Y: 0, Z: 0},
			Bounds:   core.Sphere{Center: core.Vector3D{X: 5, Y: 0, Z: 0}, Radius: 1},
			Type:     core.EntityTypeNPC,
		},
		{
			ID:       3,
			Position: core.Vector3D{X: 20, Y: 0, Z: 0},
			Bounds:   core.Sphere{Center: core.Vector3D{X: 20, Y: 0, Z: 0}, Radius: 1},
			Type:     core.EntityTypeNPC,
		},
	}
	
	for _, entity := range entities {
		if err := octree.Insert(entity); err != nil {
			t.Fatalf("Failed to insert entity %d: %v", entity.ID, err)
		}
	}
	
	// Query within radius of 10 from origin
	results := octree.QuerySphere(core.Vector3D{X: 0, Y: 0, Z: 0}, 10)
	
	// Should find entities 1 and 2, but not 3
	if len(results) != 2 {
		t.Fatalf("Expected 2 results within sphere, got %d", len(results))
	}
	
	foundIDs := make(map[uint64]bool)
	for _, result := range results {
		foundIDs[result.ID] = true
	}
	
	if !foundIDs[1] || !foundIDs[2] {
		t.Fatalf("Expected to find entities 1 and 2, found: %v", foundIDs)
	}
	
	if foundIDs[3] {
		t.Fatalf("Entity 3 should not be within sphere")
	}
}

func TestOctreeUpdate(t *testing.T) {
	bounds := core.AABB3D{
		Min: core.Vector3D{X: -50, Y: -50, Z: -50},
		Max: core.Vector3D{X: 50, Y: 50, Z: 50},
	}
	octree := NewOctree(bounds, false)
	
	entity := &core.Entity3D{
		ID:       1,
		Position: core.Vector3D{X: 0, Y: 0, Z: 0},
		Bounds:   core.Sphere{Center: core.Vector3D{X: 0, Y: 0, Z: 0}, Radius: 1},
		Type:     core.EntityTypePlayer,
	}
	
	// Insert entity
	if err := octree.Insert(entity); err != nil {
		t.Fatalf("Failed to insert entity: %v", err)
	}
	
	// Update entity position
	entity.Position = core.Vector3D{X: 20, Y: 20, Z: 20}
	entity.Bounds = core.Sphere{Center: core.Vector3D{X: 20, Y: 20, Z: 20}, Radius: 1}
	
	if err := octree.Update(entity); err != nil {
		t.Fatalf("Failed to update entity: %v", err)
	}
	
	// Query old position - should not find entity
	oldQuery := core.AABB3D{
		Min: core.Vector3D{X: -5, Y: -5, Z: -5},
		Max: core.Vector3D{X: 5, Y: 5, Z: 5},
	}
	results := octree.Query(oldQuery)
	if len(results) != 0 {
		t.Fatalf("Expected 0 results at old position, got %d", len(results))
	}
	
	// Query new position - should find entity
	newQuery := core.AABB3D{
		Min: core.Vector3D{X: 15, Y: 15, Z: 15},
		Max: core.Vector3D{X: 25, Y: 25, Z: 25},
	}
	results = octree.Query(newQuery)
	if len(results) != 1 {
		t.Fatalf("Expected 1 result at new position, got %d", len(results))
	}
	
	if results[0].ID != entity.ID {
		t.Fatalf("Expected entity ID %d, got %d", entity.ID, results[0].ID)
	}
}

func TestOctreeLooseVsTight(t *testing.T) {
	bounds := core.AABB3D{
		Min: core.Vector3D{X: -50, Y: -50, Z: -50},
		Max: core.Vector3D{X: 50, Y: 50, Z: 50},
	}
	
	tightOctree := NewOctree(bounds, false)
	looseOctree := NewOctree(bounds, true)
	
	// Create an entity that spans multiple cells
	entity := &core.Entity3D{
		ID:       1,
		Position: core.Vector3D{X: 0, Y: 0, Z: 0},
		Bounds: core.AABB3D{
			Min: core.Vector3D{X: -15, Y: -15, Z: -15},
			Max: core.Vector3D{X: 15, Y: 15, Z: 15},
		},
		Type: core.EntityTypeObstacle,
	}
	
	// Insert into both octrees
	err1 := tightOctree.Insert(entity)
	err2 := looseOctree.Insert(entity)
	
	if err1 != nil {
		t.Fatalf("Failed to insert into tight octree: %v", err1)
	}
	if err2 != nil {
		t.Fatalf("Failed to insert into loose octree: %v", err2)
	}
	
	// Query a small area that overlaps with the entity
	smallQuery := core.AABB3D{
		Min: core.Vector3D{X: 10, Y: 10, Z: 10},
		Max: core.Vector3D{X: 12, Y: 12, Z: 12},
	}
	
	tightResults := tightOctree.Query(smallQuery)
	looseResults := looseOctree.Query(smallQuery)
	
	// Both should find the entity
	if len(tightResults) != 1 {
		t.Fatalf("Tight octree expected 1 result, got %d", len(tightResults))
	}
	if len(looseResults) != 1 {
		t.Fatalf("Loose octree expected 1 result, got %d", len(looseResults))
	}
}

func TestOctreeFrustumQuery(t *testing.T) {
	bounds := core.AABB3D{
		Min: core.Vector3D{X: -50, Y: -50, Z: -50},
		Max: core.Vector3D{X: 50, Y: 50, Z: 50},
	}
	octree := NewOctree(bounds, false)
	
	// Add some entities
	entities := []*core.Entity3D{
		{
			ID:       1,
			Position: core.Vector3D{X: 0, Y: 0, Z: 10},
			Bounds:   core.Sphere{Center: core.Vector3D{X: 0, Y: 0, Z: 10}, Radius: 1},
			Type:     core.EntityTypeNPC,
		},
		{
			ID:       2,
			Position: core.Vector3D{X: 0, Y: 0, Z: -10},
			Bounds:   core.Sphere{Center: core.Vector3D{X: 0, Y: 0, Z: -10}, Radius: 1},
			Type:     core.EntityTypeNPC,
		},
	}
	
	for _, entity := range entities {
		if err := octree.Insert(entity); err != nil {
			t.Fatalf("Failed to insert entity %d: %v", entity.ID, err)
		}
	}
	
	// Create a simple frustum pointing forward (positive Z)
	frustum := &core.ViewFrustum{
		Planes: [6]core.Plane{
			{Normal: core.Vector3D{X: 0, Y: 0, Z: 1}, Distance: -5},   // Near
			{Normal: core.Vector3D{X: 0, Y: 0, Z: -1}, Distance: 20},  // Far
			{Normal: core.Vector3D{X: 1, Y: 0, Z: 0}, Distance: 10},   // Left
			{Normal: core.Vector3D{X: -1, Y: 0, Z: 0}, Distance: 10},  // Right
			{Normal: core.Vector3D{X: 0, Y: 1, Z: 0}, Distance: 10},   // Bottom
			{Normal: core.Vector3D{X: 0, Y: -1, Z: 0}, Distance: 10},  // Top
		},
	}
	
	results := octree.QueryFrustum(frustum)
	
	// Should find entity 1 (in front) but not entity 2 (behind)
	if len(results) != 1 {
		t.Fatalf("Expected 1 result in frustum, got %d", len(results))
	}
	
	if results[0].ID != 1 {
		t.Fatalf("Expected to find entity 1, got entity %d", results[0].ID)
	}
}
