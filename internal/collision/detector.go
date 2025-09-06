package collision

import (
	"math"
	"pathweaver/internal/core"
)

// CollisionResult represents the result of a collision check
type CollisionResult struct {
	Colliding    bool
	Entity       *core.Entity
	Penetration  float64       // How much objects are overlapping
	Normal       core.Vector2D // Collision normal (direction to resolve collision)
	ContactPoint core.Vector2D // Point of contact
}

// Detector handles collision detection between entities
type Detector struct {
	spatialIndex core.SpatialIndex
}

// NewDetector creates a new collision detector
func NewDetector(spatialIndex core.SpatialIndex) *Detector {
	return &Detector{
		spatialIndex: spatialIndex,
	}
}

// CheckCollision checks if an entity would collide at a given position
func (d *Detector) CheckCollision(entity *core.Entity, newPosition core.Vector2D) []CollisionResult {
	// Calculate new bounds based on the new position
	offset := core.Vector2D{
		X: newPosition.X - entity.Position.X,
		Y: newPosition.Y - entity.Position.Y,
	}

	newBounds := core.AABB{
		Min: core.Vector2D{
			X: entity.Bounds.Min.X + offset.X,
			Y: entity.Bounds.Min.Y + offset.Y,
		},
		Max: core.Vector2D{
			X: entity.Bounds.Max.X + offset.X,
			Y: entity.Bounds.Max.Y + offset.Y,
		},
	}

	return d.CheckAABBCollision(newBounds, entity.ID)
}

// CheckAABBCollision checks for collisions with a bounding box
func (d *Detector) CheckAABBCollision(bounds core.AABB, excludeID uint64) []CollisionResult {
	var results []CollisionResult

	// Query spatial index for potential collisions
	candidates := d.spatialIndex.Query(bounds)

	for _, candidate := range candidates {
		// Skip self
		if candidate.ID == excludeID {
			continue
		}

		// Check for actual collision
		if AABBIntersects(bounds, candidate.Bounds) {
			result := CollisionResult{
				Colliding: true,
				Entity:    candidate,
			}

			// Calculate collision details
			d.calculateCollisionDetails(&result, bounds, candidate.Bounds)
			results = append(results, result)
		}
	}

	return results
}

// CheckMovement checks if an entity can move from one position to another
func (d *Detector) CheckMovement(entity *core.Entity, newPosition core.Vector2D) (bool, []CollisionResult) {
	collisions := d.CheckCollision(entity, newPosition)

	// Filter out non-blocking collisions (e.g., pickups, projectiles hitting enemies)
	var blockingCollisions []CollisionResult

	for _, collision := range collisions {
		if d.isBlockingCollision(entity, collision.Entity) {
			blockingCollisions = append(blockingCollisions, collision)
		}
	}

	return len(blockingCollisions) == 0, blockingCollisions
}

// RaycastFirst performs a raycast and returns the first hit
func (d *Detector) RaycastFirst(start, direction core.Vector2D, maxDistance float64) *CollisionResult {
	// Normalize direction
	length := math.Sqrt(direction.X*direction.X + direction.Y*direction.Y)
	if length == 0 {
		return nil
	}

	normalizedDir := core.Vector2D{
		X: direction.X / length,
		Y: direction.Y / length,
	}

	// Calculate end point
	end := core.Vector2D{
		X: start.X + normalizedDir.X*maxDistance,
		Y: start.Y + normalizedDir.Y*maxDistance,
	}

	// Create bounding box for the ray
	rayBounds := core.AABB{
		Min: core.Vector2D{
			X: math.Min(start.X, end.X) - 1, // Small padding
			Y: math.Min(start.Y, end.Y) - 1,
		},
		Max: core.Vector2D{
			X: math.Max(start.X, end.X) + 1,
			Y: math.Max(start.Y, end.Y) + 1,
		},
	}

	// Get potential collision candidates
	candidates := d.spatialIndex.Query(rayBounds)

	var closestResult *CollisionResult
	closestDistance := maxDistance + 1

	for _, candidate := range candidates {
		// Check ray-AABB intersection
		t := d.rayAABBIntersection(start, normalizedDir, candidate.Bounds)
		if t >= 0 && t <= maxDistance && t < closestDistance {
			closestDistance = t

			hitPoint := core.Vector2D{
				X: start.X + normalizedDir.X*t,
				Y: start.Y + normalizedDir.Y*t,
			}

			closestResult = &CollisionResult{
				Colliding:    true,
				Entity:       candidate,
				ContactPoint: hitPoint,
				Penetration:  0, // No penetration in raycast
			}

			// Calculate normal
			center := core.Vector2D{
				X: (candidate.Bounds.Min.X + candidate.Bounds.Max.X) / 2,
				Y: (candidate.Bounds.Min.Y + candidate.Bounds.Max.Y) / 2,
			}

			toCenter := core.Vector2D{
				X: center.X - hitPoint.X,
				Y: center.Y - hitPoint.Y,
			}

			// Normalize
			length := math.Sqrt(toCenter.X*toCenter.X + toCenter.Y*toCenter.Y)
			if length > 0 {
				closestResult.Normal = core.Vector2D{
					X: -toCenter.X / length,
					Y: -toCenter.Y / length,
				}
			}
		}
	}

	return closestResult
}

// RaycastAll performs a raycast and returns all hits
func (d *Detector) RaycastAll(start, direction core.Vector2D, maxDistance float64) []CollisionResult {
	var results []CollisionResult

	// Normalize direction
	length := math.Sqrt(direction.X*direction.X + direction.Y*direction.Y)
	if length == 0 {
		return results
	}

	normalizedDir := core.Vector2D{
		X: direction.X / length,
		Y: direction.Y / length,
	}

	// Calculate end point
	end := core.Vector2D{
		X: start.X + normalizedDir.X*maxDistance,
		Y: start.Y + normalizedDir.Y*maxDistance,
	}

	// Create bounding box for the ray
	rayBounds := core.AABB{
		Min: core.Vector2D{
			X: math.Min(start.X, end.X) - 1,
			Y: math.Min(start.Y, end.Y) - 1,
		},
		Max: core.Vector2D{
			X: math.Max(start.X, end.X) + 1,
			Y: math.Max(start.Y, end.Y) + 1,
		},
	}

	// Get potential collision candidates
	candidates := d.spatialIndex.Query(rayBounds)

	for _, candidate := range candidates {
		// Check ray-AABB intersection
		t := d.rayAABBIntersection(start, normalizedDir, candidate.Bounds)
		if t >= 0 && t <= maxDistance {
			hitPoint := core.Vector2D{
				X: start.X + normalizedDir.X*t,
				Y: start.Y + normalizedDir.Y*t,
			}

			result := CollisionResult{
				Colliding:    true,
				Entity:       candidate,
				ContactPoint: hitPoint,
				Penetration:  0,
			}

			// Calculate normal
			center := core.Vector2D{
				X: (candidate.Bounds.Min.X + candidate.Bounds.Max.X) / 2,
				Y: (candidate.Bounds.Min.Y + candidate.Bounds.Max.Y) / 2,
			}

			toCenter := core.Vector2D{
				X: center.X - hitPoint.X,
				Y: center.Y - hitPoint.Y,
			}

			// Normalize
			length := math.Sqrt(toCenter.X*toCenter.X + toCenter.Y*toCenter.Y)
			if length > 0 {
				result.Normal = core.Vector2D{
					X: -toCenter.X / length,
					Y: -toCenter.Y / length,
				}
			}

			results = append(results, result)
		}
	}

	return results
}

// OverlapCircle checks for entities overlapping with a circle
func (d *Detector) OverlapCircle(center core.Vector2D, radius float64) []*core.Entity {
	return d.spatialIndex.QueryRadius(center, radius)
}

// Helper methods

// AABBIntersects checks if two AABBs intersect
func AABBIntersects(a, b core.AABB) bool {
	return a.Min.X < b.Max.X && a.Max.X > b.Min.X &&
		a.Min.Y < b.Max.Y && a.Max.Y > b.Min.Y
}

// calculateCollisionDetails calculates penetration depth and normal
func (d *Detector) calculateCollisionDetails(result *CollisionResult, bounds1, bounds2 core.AABB) {
	// Calculate overlap on each axis
	overlapX := math.Min(bounds1.Max.X, bounds2.Max.X) - math.Max(bounds1.Min.X, bounds2.Min.X)
	overlapY := math.Min(bounds1.Max.Y, bounds2.Max.Y) - math.Max(bounds1.Min.Y, bounds2.Min.Y)

	// Find the axis with minimum overlap (this is the separation axis)
	if overlapX < overlapY {
		result.Penetration = overlapX

		// Determine normal direction
		center1X := (bounds1.Min.X + bounds1.Max.X) / 2
		center2X := (bounds2.Min.X + bounds2.Max.X) / 2

		if center1X < center2X {
			result.Normal = core.Vector2D{X: -1, Y: 0}
		} else {
			result.Normal = core.Vector2D{X: 1, Y: 0}
		}

		// Calculate contact point
		result.ContactPoint = core.Vector2D{
			X: math.Max(bounds1.Min.X, bounds2.Min.X) + overlapX/2,
			Y: (math.Max(bounds1.Min.Y, bounds2.Min.Y) + math.Min(bounds1.Max.Y, bounds2.Max.Y)) / 2,
		}
	} else {
		result.Penetration = overlapY

		// Determine normal direction
		center1Y := (bounds1.Min.Y + bounds1.Max.Y) / 2
		center2Y := (bounds2.Min.Y + bounds2.Max.Y) / 2

		if center1Y < center2Y {
			result.Normal = core.Vector2D{X: 0, Y: -1}
		} else {
			result.Normal = core.Vector2D{X: 0, Y: 1}
		}

		// Calculate contact point
		result.ContactPoint = core.Vector2D{
			X: (math.Max(bounds1.Min.X, bounds2.Min.X) + math.Min(bounds1.Max.X, bounds2.Max.X)) / 2,
			Y: math.Max(bounds1.Min.Y, bounds2.Min.Y) + overlapY/2,
		}
	}
}

// isBlockingCollision determines if a collision should block movement
func (d *Detector) isBlockingCollision(entity1, entity2 *core.Entity) bool {
	// Define collision rules
	switch entity1.Type {
	case core.EntityTypePlayer, core.EntityTypeNPC:
		// Players and NPCs are blocked by obstacles and other players/NPCs
		return entity2.Type == core.EntityTypeObstacle ||
			entity2.Type == core.EntityTypePlayer ||
			entity2.Type == core.EntityTypeNPC

	case core.EntityTypeProjectile:
		// Projectiles are blocked by obstacles but can pass through pickups
		return entity2.Type == core.EntityTypeObstacle

	case core.EntityTypePickup:
		// Pickups don't block anything
		return false

	case core.EntityTypeObstacle:
		// Obstacles block everything except other obstacles
		return entity2.Type != core.EntityTypeObstacle

	default:
		// Default behavior - block solid entities
		return entity2.Type == core.EntityTypeObstacle ||
			entity2.Type == core.EntityTypePlayer ||
			entity2.Type == core.EntityTypeNPC
	}
}

// rayAABBIntersection calculates ray-AABB intersection
func (d *Detector) rayAABBIntersection(rayStart, rayDir core.Vector2D, bounds core.AABB) float64 {
	// Calculate t values for each axis
	var tMin, tMax float64

	if rayDir.X != 0 {
		t1 := (bounds.Min.X - rayStart.X) / rayDir.X
		t2 := (bounds.Max.X - rayStart.X) / rayDir.X
		tMin = math.Min(t1, t2)
		tMax = math.Max(t1, t2)
	} else {
		// Ray is parallel to YZ plane
		if rayStart.X < bounds.Min.X || rayStart.X > bounds.Max.X {
			return -1 // No intersection
		}
		tMin = math.Inf(-1)
		tMax = math.Inf(1)
	}

	if rayDir.Y != 0 {
		t1 := (bounds.Min.Y - rayStart.Y) / rayDir.Y
		t2 := (bounds.Max.Y - rayStart.Y) / rayDir.Y
		tyMin := math.Min(t1, t2)
		tyMax := math.Max(t1, t2)

		tMin = math.Max(tMin, tyMin)
		tMax = math.Min(tMax, tyMax)
	} else {
		// Ray is parallel to XZ plane
		if rayStart.Y < bounds.Min.Y || rayStart.Y > bounds.Max.Y {
			return -1 // No intersection
		}
	}

	// Check if there's a valid intersection
	if tMax < 0 || tMin > tMax {
		return -1 // No intersection
	}

	// Return the closest intersection point
	if tMin > 0 {
		return tMin
	} else {
		return tMax
	}
}
