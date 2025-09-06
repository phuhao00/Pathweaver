package pathweaver

import (
	"math"
	"math/rand"
	"pathweaver/internal/core"
	"pathweaver/internal/pathfinding"
	"time"
)

// Vector2D utility functions

// NewVector2D creates a new 2D vector
func NewVector2D(x, y float64) core.Vector2D {
	return core.Vector2D{X: x, Y: y}
}

// Distance calculates the Euclidean distance between two points
func Distance(a, b core.Vector2D) float64 {
	return pathfinding.EuclideanDistance(a, b)
}

// ManhattanDistance calculates the Manhattan distance between two points
func ManhattanDistance(a, b core.Vector2D) float64 {
	return pathfinding.ManhattanDistance(a, b)
}

// Normalize normalizes a vector to unit length
func Normalize(v core.Vector2D) core.Vector2D {
	length := math.Sqrt(v.X*v.X + v.Y*v.Y)
	if length == 0 {
		return core.Vector2D{X: 0, Y: 0}
	}
	return core.Vector2D{X: v.X / length, Y: v.Y / length}
}

// Magnitude returns the length of a vector
func Magnitude(v core.Vector2D) float64 {
	return math.Sqrt(v.X*v.X + v.Y*v.Y)
}

// DotProduct calculates the dot product of two vectors
func DotProduct(a, b core.Vector2D) float64 {
	return a.X*b.X + a.Y*b.Y
}

// Lerp linearly interpolates between two vectors
func Lerp(a, b core.Vector2D, t float64) core.Vector2D {
	return core.Vector2D{
		X: a.X + (b.X-a.X)*t,
		Y: a.Y + (b.Y-a.Y)*t,
	}
}

// RotateVector rotates a vector by the given angle (in radians)
func RotateVector(v core.Vector2D, angle float64) core.Vector2D {
	cos := math.Cos(angle)
	sin := math.Sin(angle)
	return core.Vector2D{
		X: v.X*cos - v.Y*sin,
		Y: v.X*sin + v.Y*cos,
	}
}

// AABB utility functions

// NewAABB creates a new axis-aligned bounding box
func NewAABB(minX, minY, maxX, maxY float64) core.AABB {
	return core.AABB{
		Min: core.Vector2D{X: minX, Y: minY},
		Max: core.Vector2D{X: maxX, Y: maxY},
	}
}

// AABBFromCenterSize creates an AABB from center point and size
func AABBFromCenterSize(center core.Vector2D, width, height float64) core.AABB {
	halfWidth := width / 2
	halfHeight := height / 2
	return core.AABB{
		Min: core.Vector2D{X: center.X - halfWidth, Y: center.Y - halfHeight},
		Max: core.Vector2D{X: center.X + halfWidth, Y: center.Y + halfHeight},
	}
}

// AABBCenter returns the center point of an AABB
func AABBCenter(bounds core.AABB) core.Vector2D {
	return core.Vector2D{
		X: (bounds.Min.X + bounds.Max.X) / 2,
		Y: (bounds.Min.Y + bounds.Max.Y) / 2,
	}
}

// AABBSize returns the size of an AABB
func AABBSize(bounds core.AABB) core.Vector2D {
	return core.Vector2D{
		X: bounds.Max.X - bounds.Min.X,
		Y: bounds.Max.Y - bounds.Min.Y,
	}
}

// AABBContains checks if an AABB contains a point
func AABBContains(bounds core.AABB, point core.Vector2D) bool {
	return point.X >= bounds.Min.X && point.X <= bounds.Max.X &&
		point.Y >= bounds.Min.Y && point.Y <= bounds.Max.Y
}

// AABBIntersects checks if two AABBs intersect
func AABBIntersects(a, b core.AABB) bool {
	return a.Min.X < b.Max.X && a.Max.X > b.Min.X &&
		a.Min.Y < b.Max.Y && a.Max.Y > b.Min.Y
}

// AABBExpand expands an AABB by the given amount
func AABBExpand(bounds core.AABB, amount float64) core.AABB {
	return core.AABB{
		Min: core.Vector2D{X: bounds.Min.X - amount, Y: bounds.Min.Y - amount},
		Max: core.Vector2D{X: bounds.Max.X + amount, Y: bounds.Max.Y + amount},
	}
}

// Entity utility functions

// NewEntity creates a new entity with the given parameters
func NewEntity(id uint64, entityType core.EntityType, position core.Vector2D, bounds core.AABB) *core.Entity {
	return &core.Entity{
		ID:       id,
		Type:     entityType,
		Position: position,
		Bounds:   bounds,
		Static:   entityType == core.EntityTypeObstacle,
	}
}

// NewPlayerEntity creates a new player entity
func NewPlayerEntity(id uint64, position core.Vector2D, size float64) *core.Entity {
	bounds := AABBFromCenterSize(position, size, size)
	return NewEntity(id, core.EntityTypePlayer, position, bounds)
}

// NewNPCEntity creates a new NPC entity
func NewNPCEntity(id uint64, position core.Vector2D, size float64) *core.Entity {
	bounds := AABBFromCenterSize(position, size, size)
	return NewEntity(id, core.EntityTypeNPC, position, bounds)
}

// NewObstacleEntity creates a new obstacle entity
func NewObstacleEntity(id uint64, position core.Vector2D, width, height float64) *core.Entity {
	bounds := AABBFromCenterSize(position, width, height)
	entity := NewEntity(id, core.EntityTypeObstacle, position, bounds)
	entity.Static = true
	return entity
}

// NewPickupEntity creates a new pickup entity
func NewPickupEntity(id uint64, position core.Vector2D, size float64) *core.Entity {
	bounds := AABBFromCenterSize(position, size, size)
	return NewEntity(id, core.EntityTypePickup, position, bounds)
}

// NewProjectileEntity creates a new projectile entity
func NewProjectileEntity(id uint64, position core.Vector2D, size float64) *core.Entity {
	bounds := AABBFromCenterSize(position, size, size)
	return NewEntity(id, core.EntityTypeProjectile, position, bounds)
}

// Path utility functions

// SmoothPath applies smoothing to a path using simple averaging
func SmoothPath(path []core.Vector2D, iterations int) []core.Vector2D {
	if len(path) < 3 || iterations <= 0 {
		return path
	}

	smoothed := make([]core.Vector2D, len(path))
	copy(smoothed, path)

	for iter := 0; iter < iterations; iter++ {
		// Keep first and last points unchanged
		for i := 1; i < len(smoothed)-1; i++ {
			// Average with neighbors
			prev := smoothed[i-1]
			next := smoothed[i+1]
			smoothed[i] = core.Vector2D{
				X: (prev.X + smoothed[i].X + next.X) / 3,
				Y: (prev.Y + smoothed[i].Y + next.Y) / 3,
			}
		}
	}

	return smoothed
}

// OptimizePath removes unnecessary waypoints from a path
func OptimizePath(path []core.Vector2D, tolerance float64) []core.Vector2D {
	if len(path) < 3 {
		return path
	}

	optimized := []core.Vector2D{path[0]} // Always keep first point

	for i := 1; i < len(path)-1; i++ {
		// Check if current point can be skipped
		prev := optimized[len(optimized)-1]
		current := path[i]
		next := path[i+1]

		// Calculate distance from current point to line prev->next
		distance := pointToLineDistance(current, prev, next)

		if distance > tolerance {
			optimized = append(optimized, current)
		}
	}

	optimized = append(optimized, path[len(path)-1]) // Always keep last point
	return optimized
}

// pointToLineDistance calculates the distance from a point to a line
func pointToLineDistance(point, lineStart, lineEnd core.Vector2D) float64 {
	// Vector from lineStart to lineEnd
	line := core.Vector2D{X: lineEnd.X - lineStart.X, Y: lineEnd.Y - lineStart.Y}
	lineLength := Magnitude(line)

	if lineLength == 0 {
		return Distance(point, lineStart)
	}

	// Normalize line vector
	line = core.Vector2D{X: line.X / lineLength, Y: line.Y / lineLength}

	// Vector from lineStart to point
	toPoint := core.Vector2D{X: point.X - lineStart.X, Y: point.Y - lineStart.Y}

	// Project toPoint onto line
	projectedLength := DotProduct(toPoint, line)

	// Clamp to line segment
	if projectedLength < 0 {
		projectedLength = 0
	} else if projectedLength > lineLength {
		projectedLength = lineLength
	}

	// Find closest point on line
	closestPoint := core.Vector2D{
		X: lineStart.X + line.X*projectedLength,
		Y: lineStart.Y + line.Y*projectedLength,
	}

	return Distance(point, closestPoint)
}

// Random utility functions

var rng = rand.New(rand.NewSource(time.Now().UnixNano()))

// RandomPosition generates a random position within the given bounds
func RandomPosition(bounds core.AABB) core.Vector2D {
	return core.Vector2D{
		X: bounds.Min.X + rng.Float64()*(bounds.Max.X-bounds.Min.X),
		Y: bounds.Min.Y + rng.Float64()*(bounds.Max.Y-bounds.Min.Y),
	}
}

// RandomPositionInCircle generates a random position within a circle
func RandomPositionInCircle(center core.Vector2D, radius float64) core.Vector2D {
	angle := rng.Float64() * 2 * math.Pi
	distance := rng.Float64() * radius

	return core.Vector2D{
		X: center.X + math.Cos(angle)*distance,
		Y: center.Y + math.Sin(angle)*distance,
	}
}

// Performance utility functions

// SimplifyPath reduces path complexity while maintaining accuracy
func SimplifyPath(path []core.Vector2D, epsilon float64) []core.Vector2D {
	if len(path) < 3 {
		return path
	}

	return douglasPeucker(path, epsilon)
}

// douglasPeucker implements the Douglas-Peucker algorithm for path simplification
func douglasPeucker(path []core.Vector2D, epsilon float64) []core.Vector2D {
	if len(path) < 3 {
		return path
	}

	// Find the point with maximum distance from line
	maxDistance := 0.0
	maxIndex := 0

	for i := 1; i < len(path)-1; i++ {
		distance := pointToLineDistance(path[i], path[0], path[len(path)-1])
		if distance > maxDistance {
			maxDistance = distance
			maxIndex = i
		}
	}

	// If maximum distance is greater than epsilon, recursively simplify
	if maxDistance > epsilon {
		// Recursive call on both parts
		left := douglasPeucker(path[:maxIndex+1], epsilon)
		right := douglasPeucker(path[maxIndex:], epsilon)

		// Combine results (remove duplicate point)
		result := append(left[:len(left)-1], right...)
		return result
	} else {
		// Return simplified path with just start and end points
		return []core.Vector2D{path[0], path[len(path)-1]}
	}
}
