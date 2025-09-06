package pathfinding

import (
	"math"
	"pathweaver/internal/core"
	"testing"
)

func TestAStarBasicPath(t *testing.T) {
	pathfinder := NewAStarPathfinder(1.0)
	
	start := core.Vector2D{X: 0, Y: 0}
	goal := core.Vector2D{X: 5, Y: 0}
	
	// No obstacles - should find straight path
	path, err := pathfinder.FindPath(start, goal, []core.AABB{})
	if err != nil {
		t.Fatalf("Failed to find path: %v", err)
	}
	
	if len(path) < 2 {
		t.Fatalf("Path should have at least 2 points, got %d", len(path))
	}
	
	// First point should be near start
	if distance(path[0], start) > 1.0 {
		t.Fatalf("First point too far from start: %v vs %v", path[0], start)
	}
	
	// Last point should be near goal
	lastPoint := path[len(path)-1]
	if distance(lastPoint, goal) > 1.0 {
		t.Fatalf("Last point too far from goal: %v vs %v", lastPoint, goal)
	}
}

func TestAStarWithObstacles(t *testing.T) {
	pathfinder := NewAStarPathfinder(1.0)
	
	start := core.Vector2D{X: 0, Y: 0}
	goal := core.Vector2D{X: 10, Y: 0}
	
	// Create obstacle blocking direct path
	obstacle := core.AABB{
		Min: core.Vector2D{X: 4, Y: -2},
		Max: core.Vector2D{X: 6, Y: 2},
	}
	
	path, err := pathfinder.FindPath(start, goal, []core.AABB{obstacle})
	if err != nil {
		t.Fatalf("Failed to find path around obstacle: %v", err)
	}
	
	if len(path) < 3 {
		t.Fatalf("Path around obstacle should have at least 3 points, got %d", len(path))
	}
	
	// Verify path doesn't go through obstacle
	for i, point := range path {
		if pointInAABB(point, obstacle) {
			t.Fatalf("Path point %d (%v) is inside obstacle", i, point)
		}
	}
}

func TestAStarImpossiblePath(t *testing.T) {
	pathfinder := NewAStarPathfinder(1.0)
	
	start := core.Vector2D{X: 0, Y: 0}
	goal := core.Vector2D{X: 10, Y: 0}
	
	// Create obstacles that completely surround the start position
	obstacles := []core.AABB{
		{Min: core.Vector2D{X: -5, Y: -5}, Max: core.Vector2D{X: 5, Y: -0.5}}, // Below
		{Min: core.Vector2D{X: -5, Y: 0.5}, Max: core.Vector2D{X: 5, Y: 5}},   // Above  
		{Min: core.Vector2D{X: -5, Y: -5}, Max: core.Vector2D{X: -0.5, Y: 5}}, // Left
		{Min: core.Vector2D{X: 0.5, Y: -5}, Max: core.Vector2D{X: 5, Y: 5}},   // Right
	}
	
	_, err := pathfinder.FindPath(start, goal, obstacles)
	if err == nil {
		t.Fatalf("Expected pathfinding to fail for impossible path")
	}
}

func TestAStarHeuristics(t *testing.T) {
	pathfinder := NewAStarPathfinder(1.0)
	
	start := core.Vector2D{X: 0, Y: 0}
	goal := core.Vector2D{X: 5, Y: 5}
	
	// Test different heuristics
	heuristics := map[string]core.HeuristicFunc{
		"Euclidean":  EuclideanDistance,
		"Manhattan":  ManhattanDistance,
		"Diagonal":   DiagonalDistance,
		"Octile":     OctileDistance,
	}
	
	for name, heuristic := range heuristics {
		t.Run(name, func(t *testing.T) {
			pathfinder.SetHeuristic(heuristic)
			
			path, err := pathfinder.FindPath(start, goal, []core.AABB{})
			if err != nil {
				t.Fatalf("Failed to find path with %s heuristic: %v", name, err)
			}
			
			if len(path) < 2 {
				t.Fatalf("Path should have at least 2 points with %s heuristic", name)
			}
			
			// Verify path connects start to goal
			if distance(path[0], start) > 1.5 {
				t.Fatalf("Path start too far from actual start with %s heuristic", name)
			}
			
			if distance(path[len(path)-1], goal) > 1.5 {
				t.Fatalf("Path end too far from actual goal with %s heuristic", name)
			}
		})
	}
}

func TestAStarDiagonalMovement(t *testing.T) {
	pathfinder := NewAStarPathfinder(1.0)
	
	// Test diagonal path
	start := core.Vector2D{X: 0, Y: 0}
	goal := core.Vector2D{X: 3, Y: 3}
	
	pathfinder.SetAllowDiagonal(true)
	path, err := pathfinder.FindPath(start, goal, []core.AABB{})
	if err != nil {
		t.Fatalf("Failed to find diagonal path: %v", err)
	}
	
	// With diagonal movement allowed, path should be shorter
	diagonalLength := calculatePathLength(path)
	
	pathfinder.SetAllowDiagonal(false)
	pathOrthogonal, err := pathfinder.FindPath(start, goal, []core.AABB{})
	if err != nil {
		t.Fatalf("Failed to find orthogonal path: %v", err)
	}
	
	orthogonalLength := calculatePathLength(pathOrthogonal)
	
	if diagonalLength >= orthogonalLength {
		t.Fatalf("Diagonal path should be shorter than orthogonal path: %.2f vs %.2f", 
			diagonalLength, orthogonalLength)
	}
}

// Helper functions

func distance(a, b core.Vector2D) float64 {
	dx := a.X - b.X
	dy := a.Y - b.Y
	return math.Sqrt(dx*dx + dy*dy)
}

func pointInAABB(point core.Vector2D, aabb core.AABB) bool {
	return point.X >= aabb.Min.X && point.X <= aabb.Max.X &&
		   point.Y >= aabb.Min.Y && point.Y <= aabb.Max.Y
}

func calculatePathLength(path []core.Vector2D) float64 {
	if len(path) < 2 {
		return 0
	}
	
	total := 0.0
	for i := 1; i < len(path); i++ {
		dx := path[i].X - path[i-1].X
		dy := path[i].Y - path[i-1].Y
		total += math.Sqrt(dx*dx + dy*dy)
	}
	return total
}
