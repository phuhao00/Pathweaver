package pathfinding

import (
	"math"
	"pathweaver/internal/core"
)

// 3D Heuristic Functions for pathfinding algorithms
// Based on research in 3D pathfinding and navigation mesh techniques

// EuclideanDistance3D calculates Euclidean distance between two 3D points
func EuclideanDistance3D(a, b core.Vector3D) float64 {
	dx := a.X - b.X
	dy := a.Y - b.Y
	dz := a.Z - b.Z
	return math.Sqrt(dx*dx + dy*dy + dz*dz)
}

// ManhattanDistance3D calculates Manhattan distance between two 3D points
func ManhattanDistance3D(a, b core.Vector3D) float64 {
	return math.Abs(a.X-b.X) + math.Abs(a.Y-b.Y) + math.Abs(a.Z-b.Z)
}

// ChebyshevDistance3D calculates Chebyshev distance (max of axis distances)
func ChebyshevDistance3D(a, b core.Vector3D) float64 {
	dx := math.Abs(a.X - b.X)
	dy := math.Abs(a.Y - b.Y)
	dz := math.Abs(a.Z - b.Z)
	return math.Max(dx, math.Max(dy, dz))
}

// OctileDistance3D calculates octile distance for 3D movement
// Allows diagonal movement in all 3 dimensions
func OctileDistance3D(a, b core.Vector3D) float64 {
	dx := math.Abs(a.X - b.X)
	dy := math.Abs(a.Y - b.Y)
	dz := math.Abs(a.Z - b.Z)

	// Sort distances to find min, mid, max
	distances := []float64{dx, dy, dz}

	// Simple bubble sort for 3 elements
	if distances[0] > distances[1] {
		distances[0], distances[1] = distances[1], distances[0]
	}
	if distances[1] > distances[2] {
		distances[1], distances[2] = distances[2], distances[1]
	}
	if distances[0] > distances[1] {
		distances[0], distances[1] = distances[1], distances[0]
	}

	min, mid, max := distances[0], distances[1], distances[2]

	// Cost calculation for 3D octile distance
	// 3D diagonal: sqrt(3), 2D diagonal: sqrt(2), orthogonal: 1
	return (math.Sqrt(3)-math.Sqrt(2))*min + (math.Sqrt(2)-1)*mid + max
}

// WeightedDistance3D allows custom weighting of different axes
// Useful for games where vertical movement has different cost than horizontal
func WeightedDistance3D(a, b core.Vector3D, weightX, weightY, weightZ float64) float64 {
	dx := math.Abs(a.X-b.X) * weightX
	dy := math.Abs(a.Y-b.Y) * weightY
	dz := math.Abs(a.Z-b.Z) * weightZ
	return math.Sqrt(dx*dx + dy*dy + dz*dz)
}

// LayeredDistance3D gives higher cost to vertical movement
// Particularly useful for games where vertical movement is restricted or expensive
func LayeredDistance3D(a, b core.Vector3D) float64 {
	dx := a.X - b.X
	dy := a.Y - b.Y
	dz := a.Z - b.Z

	// Give vertical movement higher cost
	verticalCost := math.Abs(dz) * 1.5
	horizontalCost := math.Sqrt(dx*dx + dy*dy)

	return horizontalCost + verticalCost
}

// FlightDistance3D optimized for flying/swimming entities
// Treats all directions equally with no vertical penalty
func FlightDistance3D(a, b core.Vector3D) float64 {
	return EuclideanDistance3D(a, b)
}

// GroundDistance3D for ground-based entities that prefer 2D movement
// Heavily penalizes vertical movement
func GroundDistance3D(a, b core.Vector3D) float64 {
	dx := a.X - b.X
	dy := a.Y - b.Y
	dz := a.Z - b.Z

	// Heavy penalty for vertical movement
	verticalPenalty := math.Abs(dz) * 10.0
	horizontalDistance := math.Sqrt(dx*dx + dy*dy)

	return horizontalDistance + verticalPenalty
}

// AdaptiveDistance3D chooses heuristic based on entity type
func AdaptiveDistance3D(a, b core.Vector3D, entityType core.EntityType) float64 {
	switch entityType {
	case core.EntityTypeProjectile:
		return FlightDistance3D(a, b)
	case core.EntityTypePlayer, core.EntityTypeNPC:
		return LayeredDistance3D(a, b)
	case core.EntityTypeUnit:
		return GroundDistance3D(a, b)
	default:
		return EuclideanDistance3D(a, b)
	}
}

// Vector3D utility functions for heuristics

// Magnitude3D returns the length of a 3D vector
func Magnitude3D(v core.Vector3D) float64 {
	return math.Sqrt(v.X*v.X + v.Y*v.Y + v.Z*v.Z)
}

// Normalize3D normalizes a 3D vector to unit length
func Normalize3D(v core.Vector3D) core.Vector3D {
	length := Magnitude3D(v)
	if length == 0 {
		return core.Vector3D{X: 0, Y: 0, Z: 0}
	}
	return core.Vector3D{X: v.X / length, Y: v.Y / length, Z: v.Z / length}
}

// DotProduct3D calculates the dot product of two 3D vectors
func DotProduct3D(a, b core.Vector3D) float64 {
	return a.X*b.X + a.Y*b.Y + a.Z*b.Z
}

// CrossProduct3D calculates the cross product of two 3D vectors
func CrossProduct3D(a, b core.Vector3D) core.Vector3D {
	return core.Vector3D{
		X: a.Y*b.Z - a.Z*b.Y,
		Y: a.Z*b.X - a.X*b.Z,
		Z: a.X*b.Y - a.Y*b.X,
	}
}

// Lerp3D linearly interpolates between two 3D vectors
func Lerp3D(a, b core.Vector3D, t float64) core.Vector3D {
	return core.Vector3D{
		X: a.X + (b.X-a.X)*t,
		Y: a.Y + (b.Y-a.Y)*t,
		Z: a.Z + (b.Z-a.Z)*t,
	}
}
