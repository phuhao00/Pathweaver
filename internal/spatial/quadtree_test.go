package spatial

import (
	"pathweaver/internal/core"
	"testing"
)

func TestQuadTreeBasicOperations(t *testing.T) {
	bounds := core.AABB{
		Min: core.Vector2D{X: -50, Y: -50},
		Max: core.Vector2D{X: 50, Y: 50},
	}
	qt := NewQuadTree(bounds)

	// Test insert
	entity := &core.Entity{
		ID:       1,
		Position: core.Vector2D{X: 10, Y: 10},
		Bounds: core.AABB{
			Min: core.Vector2D{X: 9, Y: 9},
			Max: core.Vector2D{X: 11, Y: 11},
		},
		Type: core.EntityTypePlayer,
	}

	err := qt.Insert(entity)
	if err != nil {
		t.Fatalf("Failed to insert entity: %v", err)
	}

	// Test query
	queryBounds := core.AABB{
		Min: core.Vector2D{X: 5, Y: 5},
		Max: core.Vector2D{X: 15, Y: 15},
	}

	results := qt.Query(queryBounds)
	if len(results) != 1 {
		t.Fatalf("Expected 1 result, got %d", len(results))
	}

	if results[0].ID != entity.ID {
		t.Fatalf("Expected entity ID %d, got %d", entity.ID, results[0].ID)
	}

	// Test remove
	err = qt.Remove(entity.ID)
	if err != nil {
		t.Fatalf("Failed to remove entity: %v", err)
	}

	results = qt.Query(queryBounds)
	if len(results) != 0 {
		t.Fatalf("Expected 0 results after removal, got %d", len(results))
	}
}

func TestQuadTreeRadiusQuery(t *testing.T) {
	bounds := core.AABB{
		Min: core.Vector2D{X: -50, Y: -50},
		Max: core.Vector2D{X: 50, Y: 50},
	}
	qt := NewQuadTree(bounds)

	// Insert multiple entities
	entities := []*core.Entity{
		{
			ID:       1,
			Position: core.Vector2D{X: 0, Y: 0},
			Bounds:   core.AABB{Min: core.Vector2D{X: -1, Y: -1}, Max: core.Vector2D{X: 1, Y: 1}},
			Type:     core.EntityTypePlayer,
		},
		{
			ID:       2,
			Position: core.Vector2D{X: 5, Y: 0},
			Bounds:   core.AABB{Min: core.Vector2D{X: 4, Y: -1}, Max: core.Vector2D{X: 6, Y: 1}},
			Type:     core.EntityTypeNPC,
		},
		{
			ID:       3,
			Position: core.Vector2D{X: 15, Y: 0},
			Bounds:   core.AABB{Min: core.Vector2D{X: 14, Y: -1}, Max: core.Vector2D{X: 16, Y: 1}},
			Type:     core.EntityTypeNPC,
		},
	}

	for _, entity := range entities {
		if err := qt.Insert(entity); err != nil {
			t.Fatalf("Failed to insert entity %d: %v", entity.ID, err)
		}
	}

	// Query within radius of 10 from origin
	results := qt.QueryRadius(core.Vector2D{X: 0, Y: 0}, 10)

	// Should find entities 1 and 2, but not 3
	if len(results) != 2 {
		t.Fatalf("Expected 2 results within radius, got %d", len(results))
	}

	foundIDs := make(map[uint64]bool)
	for _, result := range results {
		foundIDs[result.ID] = true
	}

	if !foundIDs[1] || !foundIDs[2] {
		t.Fatalf("Expected to find entities 1 and 2, found: %v", foundIDs)
	}

	if foundIDs[3] {
		t.Fatalf("Entity 3 should not be within radius")
	}
}

func TestQuadTreeUpdate(t *testing.T) {
	bounds := core.AABB{
		Min: core.Vector2D{X: -50, Y: -50},
		Max: core.Vector2D{X: 50, Y: 50},
	}
	qt := NewQuadTree(bounds)

	entity := &core.Entity{
		ID:       1,
		Position: core.Vector2D{X: 0, Y: 0},
		Bounds:   core.AABB{Min: core.Vector2D{X: -1, Y: -1}, Max: core.Vector2D{X: 1, Y: 1}},
		Type:     core.EntityTypePlayer,
	}

	// Insert entity
	if err := qt.Insert(entity); err != nil {
		t.Fatalf("Failed to insert entity: %v", err)
	}

	// Update entity position
	entity.Position = core.Vector2D{X: 20, Y: 20}
	entity.Bounds = core.AABB{Min: core.Vector2D{X: 19, Y: 19}, Max: core.Vector2D{X: 21, Y: 21}}

	if err := qt.Update(entity); err != nil {
		t.Fatalf("Failed to update entity: %v", err)
	}

	// Query old position - should not find entity
	oldQuery := core.AABB{Min: core.Vector2D{X: -5, Y: -5}, Max: core.Vector2D{X: 5, Y: 5}}
	results := qt.Query(oldQuery)
	if len(results) != 0 {
		t.Fatalf("Expected 0 results at old position, got %d", len(results))
	}

	// Query new position - should find entity
	newQuery := core.AABB{Min: core.Vector2D{X: 15, Y: 15}, Max: core.Vector2D{X: 25, Y: 25}}
	results = qt.Query(newQuery)
	if len(results) != 1 {
		t.Fatalf("Expected 1 result at new position, got %d", len(results))
	}

	if results[0].ID != entity.ID {
		t.Fatalf("Expected entity ID %d, got %d", entity.ID, results[0].ID)
	}
}

func TestQuadTreeGetNearest(t *testing.T) {
	bounds := core.AABB{
		Min: core.Vector2D{X: -50, Y: -50},
		Max: core.Vector2D{X: 50, Y: 50},
	}
	qt := NewQuadTree(bounds)

	// Insert entities at different distances
	entities := []*core.Entity{
		{
			ID:       1,
			Position: core.Vector2D{X: 10, Y: 0},
			Bounds:   core.AABB{Min: core.Vector2D{X: 9, Y: -1}, Max: core.Vector2D{X: 11, Y: 1}},
			Type:     core.EntityTypeNPC,
		},
		{
			ID:       2,
			Position: core.Vector2D{X: 0, Y: 5},
			Bounds:   core.AABB{Min: core.Vector2D{X: -1, Y: 4}, Max: core.Vector2D{X: 1, Y: 6}},
			Type:     core.EntityTypeNPC,
		},
		{
			ID:       3,
			Position: core.Vector2D{X: 20, Y: 0},
			Bounds:   core.AABB{Min: core.Vector2D{X: 19, Y: -1}, Max: core.Vector2D{X: 21, Y: 1}},
			Type:     core.EntityTypeNPC,
		},
	}

	for _, entity := range entities {
		if err := qt.Insert(entity); err != nil {
			t.Fatalf("Failed to insert entity %d: %v", entity.ID, err)
		}
	}

	// Find nearest to origin
	nearest := qt.GetNearest(core.Vector2D{X: 0, Y: 0}, 50)
	if nearest == nil {
		t.Fatalf("Expected to find nearest entity")
	}

	// Should be entity 2 (distance 5) since it's closer than entity 1 (distance 10)
	if nearest.ID != 2 {
		t.Fatalf("Expected nearest entity to be ID 2, got ID %d", nearest.ID)
	}

	// Test with limited range
	nearest = qt.GetNearest(core.Vector2D{X: 0, Y: 0}, 3)
	if nearest != nil {
		t.Fatalf("Expected no entities within range 3, but found entity %d", nearest.ID)
	}
}
