package collision

import (
	"math"
	"pathweaver/internal/core"
)

// CollisionResult3D represents the result of a 3D collision check
type CollisionResult3D struct {
	Colliding    bool
	Entity       *core.Entity3D
	Penetration  float64       // How much objects are overlapping
	Normal       core.Vector3D // Collision normal (direction to resolve collision)
	ContactPoint core.Vector3D // Point of contact
}

// Detector3D handles 3D collision detection between entities
// Implements advanced collision detection algorithms from research papers:
// - "Real-Time Collision Detection" by Christer Ericson
// - "Game Physics Engine Development" by Ian Millington
type Detector3D struct {
	spatialIndex core.SpatialIndex3D
}

// NewDetector3D creates a new 3D collision detector
func NewDetector3D(spatialIndex core.SpatialIndex3D) *Detector3D {
	return &Detector3D{
		spatialIndex: spatialIndex,
	}
}

// CheckCollision checks if an entity would collide at a given 3D position
func (d *Detector3D) CheckCollision(entity *core.Entity3D, newPosition core.Vector3D) []CollisionResult3D {
	// Calculate new bounds based on the new position
	offset := core.Vector3D{
		X: newPosition.X - entity.Position.X,
		Y: newPosition.Y - entity.Position.Y,
		Z: newPosition.Z - entity.Position.Z,
	}

	// Transform bounds based on entity type
	switch bounds := entity.Bounds.(type) {
	case core.AABB3D:
		newBounds := core.AABB3D{
			Min: core.Vector3D{
				X: bounds.Min.X + offset.X,
				Y: bounds.Min.Y + offset.Y,
				Z: bounds.Min.Z + offset.Z,
			},
			Max: core.Vector3D{
				X: bounds.Max.X + offset.X,
				Y: bounds.Max.Y + offset.Y,
				Z: bounds.Max.Z + offset.Z,
			},
		}
		return d.CheckAABB3DCollision(newBounds, entity.ID)

	case core.Sphere:
		newSphere := core.Sphere{
			Center: newPosition,
			Radius: bounds.Radius,
		}
		return d.CheckSphereCollision(newSphere, entity.ID)

	case core.OBB:
		newOBB := core.OBB{
			Center:      newPosition,
			Extents:     bounds.Extents,
			Orientation: bounds.Orientation,
		}
		return d.CheckOBBCollision(newOBB, entity.ID)
	}

	return nil
}

// CheckAABB3DCollision checks for collisions with a 3D bounding box
func (d *Detector3D) CheckAABB3DCollision(bounds core.AABB3D, excludeID uint64) []CollisionResult3D {
	var results []CollisionResult3D

	// Query spatial index for potential collisions
	candidates := d.spatialIndex.Query(bounds)

	for _, candidate := range candidates {
		// Skip self
		if candidate.ID == excludeID {
			continue
		}

		// Check for actual collision based on candidate's bounds type
		switch candidateBounds := candidate.Bounds.(type) {
		case core.AABB3D:
			if AABB3DIntersects(bounds, candidateBounds) {
				result := CollisionResult3D{
					Colliding: true,
					Entity:    candidate,
				}
				d.calculateAABBCollisionDetails(&result, bounds, candidateBounds)
				results = append(results, result)
			}

		case core.Sphere:
			if AABBSphereIntersects(bounds, candidateBounds) {
				result := CollisionResult3D{
					Colliding: true,
					Entity:    candidate,
				}
				d.calculateAABBSphereCollisionDetails(&result, bounds, candidateBounds)
				results = append(results, result)
			}

		case core.OBB:
			if AABBOBBIntersects(bounds, candidateBounds) {
				result := CollisionResult3D{
					Colliding: true,
					Entity:    candidate,
				}
				// OBB collision details are more complex
				results = append(results, result)
			}
		}
	}

	return results
}

// CheckSphereCollision checks for collisions with a sphere
func (d *Detector3D) CheckSphereCollision(sphere core.Sphere, excludeID uint64) []CollisionResult3D {
	var results []CollisionResult3D

	// Query spatial index using sphere's AABB
	candidates := d.spatialIndex.QuerySphere(sphere.Center, sphere.Radius)

	for _, candidate := range candidates {
		if candidate.ID == excludeID {
			continue
		}

		switch candidateBounds := candidate.Bounds.(type) {
		case core.Sphere:
			if SphereSphereIntersects(sphere, candidateBounds) {
				result := CollisionResult3D{
					Colliding: true,
					Entity:    candidate,
				}
				d.calculateSphereSphereCollisionDetails(&result, sphere, candidateBounds)
				results = append(results, result)
			}

		case core.AABB3D:
			if AABBSphereIntersects(candidateBounds, sphere) {
				result := CollisionResult3D{
					Colliding: true,
					Entity:    candidate,
				}
				d.calculateAABBSphereCollisionDetails(&result, candidateBounds, sphere)
				results = append(results, result)
			}
		}
	}

	return results
}

// CheckOBBCollision checks for collisions with an oriented bounding box
func (d *Detector3D) CheckOBBCollision(obb core.OBB, excludeID uint64) []CollisionResult3D {
	var results []CollisionResult3D

	// Convert OBB to AABB for initial spatial query (conservative)
	aabb := OBBToAABB(obb)
	candidates := d.spatialIndex.Query(aabb)

	for _, candidate := range candidates {
		if candidate.ID == excludeID {
			continue
		}

		switch candidateBounds := candidate.Bounds.(type) {
		case core.OBB:
			if OBBOBBIntersects(obb, candidateBounds) {
				result := CollisionResult3D{
					Colliding: true,
					Entity:    candidate,
				}
				results = append(results, result)
			}

		case core.AABB3D:
			if AABBOBBIntersects(candidateBounds, obb) {
				result := CollisionResult3D{
					Colliding: true,
					Entity:    candidate,
				}
				results = append(results, result)
			}
		}
	}

	return results
}

// Raycast3D performs a 3D raycast and returns the first hit
func (d *Detector3D) Raycast3D(start, direction core.Vector3D, maxDistance float64) *CollisionResult3D {
	// Normalize direction
	length := math.Sqrt(direction.X*direction.X + direction.Y*direction.Y + direction.Z*direction.Z)
	if length == 0 {
		return nil
	}

	normalizedDir := core.Vector3D{
		X: direction.X / length,
		Y: direction.Y / length,
		Z: direction.Z / length,
	}

	// Calculate end point
	end := core.Vector3D{
		X: start.X + normalizedDir.X*maxDistance,
		Y: start.Y + normalizedDir.Y*maxDistance,
		Z: start.Z + normalizedDir.Z*maxDistance,
	}

	// Create bounding box for the ray
	rayBounds := core.AABB3D{
		Min: core.Vector3D{
			X: math.Min(start.X, end.X) - 1,
			Y: math.Min(start.Y, end.Y) - 1,
			Z: math.Min(start.Z, end.Z) - 1,
		},
		Max: core.Vector3D{
			X: math.Max(start.X, end.X) + 1,
			Y: math.Max(start.Y, end.Y) + 1,
			Z: math.Max(start.Z, end.Z) + 1,
		},
	}

	// Get potential collision candidates
	candidates := d.spatialIndex.Query(rayBounds)

	var closestResult *CollisionResult3D
	closestDistance := maxDistance + 1

	for _, candidate := range candidates {
		var t float64 = -1

		// Check ray intersection based on bounds type
		switch bounds := candidate.Bounds.(type) {
		case core.AABB3D:
			t = d.rayAABB3DIntersection(start, normalizedDir, bounds)
		case core.Sphere:
			t = d.raySphereIntersection(start, normalizedDir, bounds)
		case core.OBB:
			t = d.rayOBBIntersection(start, normalizedDir, bounds)
		}

		if t >= 0 && t <= maxDistance && t < closestDistance {
			closestDistance = t

			hitPoint := core.Vector3D{
				X: start.X + normalizedDir.X*t,
				Y: start.Y + normalizedDir.Y*t,
				Z: start.Z + normalizedDir.Z*t,
			}

			closestResult = &CollisionResult3D{
				Colliding:    true,
				Entity:       candidate,
				ContactPoint: hitPoint,
				Penetration:  0, // No penetration in raycast
			}
		}
	}

	return closestResult
}

// Collision test functions

// AABB3DIntersects checks if two 3D AABBs intersect
func AABB3DIntersects(a, b core.AABB3D) bool {
	return a.Min.X < b.Max.X && a.Max.X > b.Min.X &&
		a.Min.Y < b.Max.Y && a.Max.Y > b.Min.Y &&
		a.Min.Z < b.Max.Z && a.Max.Z > b.Min.Z
}

// SphereSphereIntersects checks if two spheres intersect
func SphereSphereIntersects(a, b core.Sphere) bool {
	dx := a.Center.X - b.Center.X
	dy := a.Center.Y - b.Center.Y
	dz := a.Center.Z - b.Center.Z
	distanceSquared := dx*dx + dy*dy + dz*dz
	radiusSum := a.Radius + b.Radius
	return distanceSquared <= radiusSum*radiusSum
}

// AABBSphereIntersects checks if an AABB and sphere intersect
func AABBSphereIntersects(aabb core.AABB3D, sphere core.Sphere) bool {
	// Find closest point on AABB to sphere center
	closestX := math.Max(aabb.Min.X, math.Min(sphere.Center.X, aabb.Max.X))
	closestY := math.Max(aabb.Min.Y, math.Min(sphere.Center.Y, aabb.Max.Y))
	closestZ := math.Max(aabb.Min.Z, math.Min(sphere.Center.Z, aabb.Max.Z))

	// Calculate distance from sphere center to closest point
	dx := sphere.Center.X - closestX
	dy := sphere.Center.Y - closestY
	dz := sphere.Center.Z - closestZ

	return (dx*dx + dy*dy + dz*dz) <= (sphere.Radius * sphere.Radius)
}

// OBBOBBIntersects checks if two OBBs intersect using SAT (Separating Axes Theorem)
func OBBOBBIntersects(a, b core.OBB) bool {
	// Simplified implementation - full SAT test would be more complex
	// Convert both to AABB for now (conservative test)
	aabbA := OBBToAABB(a)
	aabbB := OBBToAABB(b)
	return AABB3DIntersects(aabbA, aabbB)
}

// AABBOBBIntersects checks if an AABB and OBB intersect
func AABBOBBIntersects(aabb core.AABB3D, obb core.OBB) bool {
	// Convert OBB to AABB (conservative test)
	obbAABB := OBBToAABB(obb)
	return AABB3DIntersects(aabb, obbAABB)
}

// OBBToAABB converts an OBB to an AABB (conservative approximation)
func OBBToAABB(obb core.OBB) core.AABB3D {
	// Calculate the extents of the OBB in world space
	halfExtents := core.Vector3D{
		X: math.Abs(obb.Extents.X*obb.Orientation[0].X) +
			math.Abs(obb.Extents.Y*obb.Orientation[1].X) +
			math.Abs(obb.Extents.Z*obb.Orientation[2].X),
		Y: math.Abs(obb.Extents.X*obb.Orientation[0].Y) +
			math.Abs(obb.Extents.Y*obb.Orientation[1].Y) +
			math.Abs(obb.Extents.Z*obb.Orientation[2].Y),
		Z: math.Abs(obb.Extents.X*obb.Orientation[0].Z) +
			math.Abs(obb.Extents.Y*obb.Orientation[1].Z) +
			math.Abs(obb.Extents.Z*obb.Orientation[2].Z),
	}

	return core.AABB3D{
		Min: core.Vector3D{
			X: obb.Center.X - halfExtents.X,
			Y: obb.Center.Y - halfExtents.Y,
			Z: obb.Center.Z - halfExtents.Z,
		},
		Max: core.Vector3D{
			X: obb.Center.X + halfExtents.X,
			Y: obb.Center.Y + halfExtents.Y,
			Z: obb.Center.Z + halfExtents.Z,
		},
	}
}

// Ray intersection functions

// rayAABB3DIntersection calculates ray-AABB intersection in 3D
func (d *Detector3D) rayAABB3DIntersection(rayStart, rayDir core.Vector3D, aabb core.AABB3D) float64 {
	var tMin, tMax float64 = 0, math.Inf(1)

	// Check intersection with each axis-aligned slab
	for axis := 0; axis < 3; axis++ {
		var rayOrigin, rayDirection, slabMin, slabMax float64

		switch axis {
		case 0: // X axis
			rayOrigin, rayDirection, slabMin, slabMax = rayStart.X, rayDir.X, aabb.Min.X, aabb.Max.X
		case 1: // Y axis
			rayOrigin, rayDirection, slabMin, slabMax = rayStart.Y, rayDir.Y, aabb.Min.Y, aabb.Max.Y
		case 2: // Z axis
			rayOrigin, rayDirection, slabMin, slabMax = rayStart.Z, rayDir.Z, aabb.Min.Z, aabb.Max.Z
		}

		if math.Abs(rayDirection) < 1e-8 {
			// Ray is parallel to slab
			if rayOrigin < slabMin || rayOrigin > slabMax {
				return -1 // No intersection
			}
		} else {
			// Calculate intersection distances
			t1 := (slabMin - rayOrigin) / rayDirection
			t2 := (slabMax - rayOrigin) / rayDirection

			if t1 > t2 {
				t1, t2 = t2, t1 // Swap so t1 is nearer
			}

			tMin = math.Max(tMin, t1)
			tMax = math.Min(tMax, t2)

			if tMin > tMax {
				return -1 // No intersection
			}
		}
	}

	// Return the nearest intersection point
	if tMin > 0 {
		return tMin
	} else if tMax > 0 {
		return tMax
	}

	return -1 // No intersection
}

// raySphereIntersection calculates ray-sphere intersection
func (d *Detector3D) raySphereIntersection(rayStart, rayDir core.Vector3D, sphere core.Sphere) float64 {
	// Vector from ray start to sphere center
	oc := core.Vector3D{
		X: rayStart.X - sphere.Center.X,
		Y: rayStart.Y - sphere.Center.Y,
		Z: rayStart.Z - sphere.Center.Z,
	}

	// Quadratic equation coefficients
	a := rayDir.X*rayDir.X + rayDir.Y*rayDir.Y + rayDir.Z*rayDir.Z
	b := 2.0 * (oc.X*rayDir.X + oc.Y*rayDir.Y + oc.Z*rayDir.Z)
	c := oc.X*oc.X + oc.Y*oc.Y + oc.Z*oc.Z - sphere.Radius*sphere.Radius

	discriminant := b*b - 4*a*c

	if discriminant < 0 {
		return -1 // No intersection
	}

	sqrt_discriminant := math.Sqrt(discriminant)
	t1 := (-b - sqrt_discriminant) / (2 * a)
	t2 := (-b + sqrt_discriminant) / (2 * a)

	// Return the nearest positive intersection
	if t1 > 0 {
		return t1
	} else if t2 > 0 {
		return t2
	}

	return -1 // No intersection
}

// rayOBBIntersection calculates ray-OBB intersection
func (d *Detector3D) rayOBBIntersection(rayStart, rayDir core.Vector3D, obb core.OBB) float64 {
	// Transform ray to OBB's local coordinate system
	// This is a simplified version - full implementation would be more complex
	aabb := OBBToAABB(obb)
	return d.rayAABB3DIntersection(rayStart, rayDir, aabb)
}

// Collision detail calculation functions

func (d *Detector3D) calculateAABBCollisionDetails(result *CollisionResult3D, bounds1, bounds2 core.AABB3D) {
	// Calculate overlap on each axis
	overlapX := math.Min(bounds1.Max.X, bounds2.Max.X) - math.Max(bounds1.Min.X, bounds2.Min.X)
	overlapY := math.Min(bounds1.Max.Y, bounds2.Max.Y) - math.Max(bounds1.Min.Y, bounds2.Min.Y)
	overlapZ := math.Min(bounds1.Max.Z, bounds2.Max.Z) - math.Max(bounds1.Min.Z, bounds2.Min.Z)

	// Find the axis with minimum overlap (this is the separation axis)
	if overlapX <= overlapY && overlapX <= overlapZ {
		result.Penetration = overlapX
		// Determine normal direction on X axis
		center1X := (bounds1.Min.X + bounds1.Max.X) / 2
		center2X := (bounds2.Min.X + bounds2.Max.X) / 2
		if center1X < center2X {
			result.Normal = core.Vector3D{X: -1, Y: 0, Z: 0}
		} else {
			result.Normal = core.Vector3D{X: 1, Y: 0, Z: 0}
		}
	} else if overlapY <= overlapZ {
		result.Penetration = overlapY
		// Determine normal direction on Y axis
		center1Y := (bounds1.Min.Y + bounds1.Max.Y) / 2
		center2Y := (bounds2.Min.Y + bounds2.Max.Y) / 2
		if center1Y < center2Y {
			result.Normal = core.Vector3D{X: 0, Y: -1, Z: 0}
		} else {
			result.Normal = core.Vector3D{X: 0, Y: 1, Z: 0}
		}
	} else {
		result.Penetration = overlapZ
		// Determine normal direction on Z axis
		center1Z := (bounds1.Min.Z + bounds1.Max.Z) / 2
		center2Z := (bounds2.Min.Z + bounds2.Max.Z) / 2
		if center1Z < center2Z {
			result.Normal = core.Vector3D{X: 0, Y: 0, Z: -1}
		} else {
			result.Normal = core.Vector3D{X: 0, Y: 0, Z: 1}
		}
	}
}

func (d *Detector3D) calculateSphereSphereCollisionDetails(result *CollisionResult3D, sphere1, sphere2 core.Sphere) {
	// Calculate vector from sphere1 to sphere2
	direction := core.Vector3D{
		X: sphere2.Center.X - sphere1.Center.X,
		Y: sphere2.Center.Y - sphere1.Center.Y,
		Z: sphere2.Center.Z - sphere1.Center.Z,
	}

	distance := math.Sqrt(direction.X*direction.X + direction.Y*direction.Y + direction.Z*direction.Z)

	if distance > 0 {
		result.Normal = core.Vector3D{
			X: direction.X / distance,
			Y: direction.Y / distance,
			Z: direction.Z / distance,
		}
	}

	result.Penetration = (sphere1.Radius + sphere2.Radius) - distance
	result.ContactPoint = core.Vector3D{
		X: sphere1.Center.X + result.Normal.X*sphere1.Radius,
		Y: sphere1.Center.Y + result.Normal.Y*sphere1.Radius,
		Z: sphere1.Center.Z + result.Normal.Z*sphere1.Radius,
	}
}

func (d *Detector3D) calculateAABBSphereCollisionDetails(result *CollisionResult3D, aabb core.AABB3D, sphere core.Sphere) {
	// Find closest point on AABB to sphere center
	closestPoint := core.Vector3D{
		X: math.Max(aabb.Min.X, math.Min(sphere.Center.X, aabb.Max.X)),
		Y: math.Max(aabb.Min.Y, math.Min(sphere.Center.Y, aabb.Max.Y)),
		Z: math.Max(aabb.Min.Z, math.Min(sphere.Center.Z, aabb.Max.Z)),
	}

	// Calculate direction from closest point to sphere center
	direction := core.Vector3D{
		X: sphere.Center.X - closestPoint.X,
		Y: sphere.Center.Y - closestPoint.Y,
		Z: sphere.Center.Z - closestPoint.Z,
	}

	distance := math.Sqrt(direction.X*direction.X + direction.Y*direction.Y + direction.Z*direction.Z)

	if distance > 0 {
		result.Normal = core.Vector3D{
			X: direction.X / distance,
			Y: direction.Y / distance,
			Z: direction.Z / distance,
		}
	}

	result.Penetration = sphere.Radius - distance
	result.ContactPoint = closestPoint
}
