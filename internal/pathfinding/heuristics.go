package pathfinding

import (
	"math"
	"pathweaver/internal/core"
)

// ManhattanDistance calculates Manhattan distance between two points
func ManhattanDistance(a, b core.Vector2D) float64 {
	return math.Abs(a.X-b.X) + math.Abs(a.Y-b.Y)
}

// EuclideanDistance calculates Euclidean distance between two points
func EuclideanDistance(a, b core.Vector2D) float64 {
	dx := a.X - b.X
	dy := a.Y - b.Y
	return math.Sqrt(dx*dx + dy*dy)
}

// DiagonalDistance calculates diagonal distance (Chebyshev distance)
func DiagonalDistance(a, b core.Vector2D) float64 {
	dx := math.Abs(a.X - b.X)
	dy := math.Abs(a.Y - b.Y)
	return math.Max(dx, dy)
}

// OctileDistance calculates octile distance (8-directional movement)
func OctileDistance(a, b core.Vector2D) float64 {
	dx := math.Abs(a.X - b.X)
	dy := math.Abs(a.Y - b.Y)
	return (dx + dy) + (math.Sqrt2-2)*math.Min(dx, dy)
}
