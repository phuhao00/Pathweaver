package fps

import (
	"math"
	"pathweaver/internal/core"
)

// BSPTree implements a Binary Space Partitioning tree for FPS games
// Based on the classic BSP algorithm used in games like Quake and Doom
// Provides fast visibility determination and rendering order
type BSPTree struct {
	root     *BSPNode
	polygons []Polygon
}

// BSPNode represents a node in the BSP tree
type BSPNode struct {
	plane    core.Plane
	front    *BSPNode
	back     *BSPNode
	polygons []Polygon // Polygons that lie exactly on the plane
	isLeaf   bool
	leafData *LeafData // Only valid if isLeaf is true
}

// Polygon represents a convex polygon in 3D space
type Polygon struct {
	vertices []core.Vector3D
	normal   core.Vector3D
	plane    core.Plane
	material MaterialType
}

// LeafData contains information about BSP leaf nodes
type LeafData struct {
	cluster     int              // PVS cluster ID
	entities    []*core.Entity3D // Entities in this leaf
	volume      float64          // Volume of this leaf
	surfaceArea float64          // Surface area for acoustics
}

// MaterialType defines different surface materials for FPS games
type MaterialType int

const (
	MaterialWall MaterialType = iota
	MaterialFloor
	MaterialCeiling
	MaterialWater
	MaterialGlass
	MaterialMetal
	MaterialWood
	MaterialFabric
)

// PVSData represents Potentially Visible Set data
type PVSData struct {
	clusters      int
	visibilityMap [][]bool // clusters x clusters visibility matrix
	compressed    []byte   // Compressed visibility data
}

// NewBSPTree creates a new BSP tree from a list of polygons
func NewBSPTree(polygons []Polygon) *BSPTree {
	tree := &BSPTree{
		polygons: polygons,
	}

	if len(polygons) > 0 {
		tree.root = tree.buildBSP(polygons, 0)
	}

	return tree
}

// buildBSP recursively builds the BSP tree
func (bsp *BSPTree) buildBSP(polygons []Polygon, depth int) *BSPNode {
	const maxDepth = 20
	const minPolygons = 1

	if len(polygons) <= minPolygons || depth >= maxDepth {
		// Create leaf node
		return &BSPNode{
			isLeaf:   true,
			polygons: polygons,
			leafData: &LeafData{
				cluster: depth, // Simple cluster assignment
			},
		}
	}

	// Select splitting plane (use first polygon's plane for simplicity)
	// In production, you'd use more sophisticated heuristics
	splitter := polygons[0]

	node := &BSPNode{
		plane:    splitter.plane,
		isLeaf:   false,
		polygons: []Polygon{splitter},
	}

	var frontPolygons, backPolygons []Polygon

	// Classify remaining polygons
	for i := 1; i < len(polygons); i++ {
		poly := polygons[i]
		classification := bsp.classifyPolygon(poly, node.plane)

		switch classification {
		case PlaneClassificationFront:
			frontPolygons = append(frontPolygons, poly)
		case PlaneClassificationBack:
			backPolygons = append(backPolygons, poly)
		case PlaneClassificationCoplanar:
			node.polygons = append(node.polygons, poly)
		case PlaneClassificationSpanning:
			// Split polygon (simplified - in practice this is more complex)
			front, back := bsp.splitPolygon(poly, node.plane)
			if front != nil {
				frontPolygons = append(frontPolygons, *front)
			}
			if back != nil {
				backPolygons = append(backPolygons, *back)
			}
		}
	}

	// Recursively build child nodes
	if len(frontPolygons) > 0 {
		node.front = bsp.buildBSP(frontPolygons, depth+1)
	}
	if len(backPolygons) > 0 {
		node.back = bsp.buildBSP(backPolygons, depth+1)
	}

	return node
}

// PlaneClassification represents polygon classification relative to a plane
type PlaneClassification int

const (
	PlaneClassificationCoplanar PlaneClassification = iota
	PlaneClassificationFront
	PlaneClassificationBack
	PlaneClassificationSpanning
)

// classifyPolygon determines which side of a plane a polygon is on
func (bsp *BSPTree) classifyPolygon(poly Polygon, plane core.Plane) PlaneClassification {
	const epsilon = 1e-6

	frontVertices := 0
	backVertices := 0

	for _, vertex := range poly.vertices {
		distance := bsp.pointToPlaneDistance(vertex, plane)

		if distance > epsilon {
			frontVertices++
		} else if distance < -epsilon {
			backVertices++
		}
	}

	if frontVertices > 0 && backVertices > 0 {
		return PlaneClassificationSpanning
	} else if frontVertices > 0 {
		return PlaneClassificationFront
	} else if backVertices > 0 {
		return PlaneClassificationBack
	} else {
		return PlaneClassificationCoplanar
	}
}

// splitPolygon splits a polygon by a plane (simplified implementation)
func (bsp *BSPTree) splitPolygon(poly Polygon, plane core.Plane) (*Polygon, *Polygon) {
	// This is a simplified implementation
	// In practice, you'd properly handle polygon splitting with intersection points

	var frontVertices, backVertices []core.Vector3D

	for _, vertex := range poly.vertices {
		distance := bsp.pointToPlaneDistance(vertex, plane)

		if distance >= 0 {
			frontVertices = append(frontVertices, vertex)
		} else {
			backVertices = append(backVertices, vertex)
		}
	}

	var front, back *Polygon

	if len(frontVertices) >= 3 {
		front = &Polygon{
			vertices: frontVertices,
			normal:   poly.normal,
			material: poly.material,
		}
		front.plane = bsp.polygonToPlane(*front)
	}

	if len(backVertices) >= 3 {
		back = &Polygon{
			vertices: backVertices,
			normal:   poly.normal,
			material: poly.material,
		}
		back.plane = bsp.polygonToPlane(*back)
	}

	return front, back
}

// pointToPlaneDistance calculates distance from point to plane
func (bsp *BSPTree) pointToPlaneDistance(point core.Vector3D, plane core.Plane) float64 {
	return plane.Normal.X*point.X + plane.Normal.Y*point.Y + plane.Normal.Z*point.Z + plane.Distance
}

// polygonToPlane converts a polygon to a plane
func (bsp *BSPTree) polygonToPlane(poly Polygon) core.Plane {
	// Calculate plane from first three vertices
	if len(poly.vertices) < 3 {
		return core.Plane{}
	}

	v1 := poly.vertices[0]
	v2 := poly.vertices[1]
	v3 := poly.vertices[2]

	// Calculate normal using cross product
	edge1 := core.Vector3D{X: v2.X - v1.X, Y: v2.Y - v1.Y, Z: v2.Z - v1.Z}
	edge2 := core.Vector3D{X: v3.X - v1.X, Y: v3.Y - v1.Y, Z: v3.Z - v1.Z}

	normal := core.Vector3D{
		X: edge1.Y*edge2.Z - edge1.Z*edge2.Y,
		Y: edge1.Z*edge2.X - edge1.X*edge2.Z,
		Z: edge1.X*edge2.Y - edge1.Y*edge2.X,
	}

	// Normalize
	length := math.Sqrt(normal.X*normal.X + normal.Y*normal.Y + normal.Z*normal.Z)
	if length > 0 {
		normal.X /= length
		normal.Y /= length
		normal.Z /= length
	}

	// Calculate distance
	distance := -(normal.X*v1.X + normal.Y*v1.Y + normal.Z*v1.Z)

	return core.Plane{Normal: normal, Distance: distance}
}

// VisibilityQuery performs visibility determination from a viewpoint
func (bsp *BSPTree) VisibilityQuery(viewpoint core.Vector3D, viewDirection core.Vector3D) []Polygon {
	if bsp.root == nil {
		return nil
	}

	var visiblePolygons []Polygon
	bsp.collectVisiblePolygons(bsp.root, viewpoint, viewDirection, &visiblePolygons)

	return visiblePolygons
}

// collectVisiblePolygons traverses BSP tree and collects visible polygons
func (bsp *BSPTree) collectVisiblePolygons(node *BSPNode, viewpoint, viewDirection core.Vector3D, visible *[]Polygon) {
	if node == nil {
		return
	}

	if node.isLeaf {
		// Add all polygons in leaf (with potential frustum culling)
		for _, poly := range node.polygons {
			if bsp.isPolygonVisible(poly, viewpoint, viewDirection) {
				*visible = append(*visible, poly)
			}
		}
		return
	}

	// Determine which side of the plane the viewpoint is on
	distance := bsp.pointToPlaneDistance(viewpoint, node.plane)

	if distance > 0 {
		// Viewpoint is in front of plane
		// First traverse front side, then back side
		bsp.collectVisiblePolygons(node.front, viewpoint, viewDirection, visible)

		// Add polygons on the plane
		for _, poly := range node.polygons {
			if bsp.isPolygonVisible(poly, viewpoint, viewDirection) {
				*visible = append(*visible, poly)
			}
		}

		bsp.collectVisiblePolygons(node.back, viewpoint, viewDirection, visible)
	} else {
		// Viewpoint is behind plane
		// First traverse back side, then front side (back-to-front for transparency)
		bsp.collectVisiblePolygons(node.back, viewpoint, viewDirection, visible)

		// Check if plane faces the viewer
		toViewer := core.Vector3D{
			X: viewpoint.X - node.polygons[0].vertices[0].X,
			Y: viewpoint.Y - node.polygons[0].vertices[0].Y,
			Z: viewpoint.Z - node.polygons[0].vertices[0].Z,
		}

		// Dot product with plane normal
		dot := node.plane.Normal.X*toViewer.X + node.plane.Normal.Y*toViewer.Y + node.plane.Normal.Z*toViewer.Z

		if dot > 0 {
			// Plane faces viewer, add its polygons
			for _, poly := range node.polygons {
				if bsp.isPolygonVisible(poly, viewpoint, viewDirection) {
					*visible = append(*visible, poly)
				}
			}
		}

		bsp.collectVisiblePolygons(node.front, viewpoint, viewDirection, visible)
	}
}

// isPolygonVisible performs basic visibility culling
func (bsp *BSPTree) isPolygonVisible(poly Polygon, viewpoint, viewDirection core.Vector3D) bool {
	// Simple frustum culling based on view direction
	// In practice, you'd use a proper view frustum

	if len(poly.vertices) == 0 {
		return false
	}

	// Calculate polygon center
	center := core.Vector3D{}
	for _, vertex := range poly.vertices {
		center.X += vertex.X
		center.Y += vertex.Y
		center.Z += vertex.Z
	}
	center.X /= float64(len(poly.vertices))
	center.Y /= float64(len(poly.vertices))
	center.Z /= float64(len(poly.vertices))

	// Vector from viewpoint to polygon center
	toPolygon := core.Vector3D{
		X: center.X - viewpoint.X,
		Y: center.Y - viewpoint.Y,
		Z: center.Z - viewpoint.Z,
	}

	// Normalize view direction and toPolygon
	viewLength := math.Sqrt(viewDirection.X*viewDirection.X + viewDirection.Y*viewDirection.Y + viewDirection.Z*viewDirection.Z)
	polygonLength := math.Sqrt(toPolygon.X*toPolygon.X + toPolygon.Y*toPolygon.Y + toPolygon.Z*toPolygon.Z)

	if viewLength == 0 || polygonLength == 0 {
		return false
	}

	// Calculate angle between view direction and polygon direction
	dot := (viewDirection.X*toPolygon.X + viewDirection.Y*toPolygon.Y + viewDirection.Z*toPolygon.Z) / (viewLength * polygonLength)

	// Only show polygons in front of the viewer (90-degree field of view)
	return dot > 0
}

// RaycastBSP performs fast raycasting using the BSP tree
// This is optimized for FPS hit detection
func (bsp *BSPTree) RaycastBSP(origin, direction core.Vector3D, maxDistance float64) *RaycastHit {
	if bsp.root == nil {
		return nil
	}

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

	hit := bsp.raycastNode(bsp.root, origin, normalizedDir, 0, maxDistance)
	return hit
}

// RaycastHit represents the result of a BSP raycast
type RaycastHit struct {
	Hit          bool
	Distance     float64
	Point        core.Vector3D
	Normal       core.Vector3D
	Polygon      *Polygon
	MaterialType MaterialType
}

// raycastNode performs recursive raycast against BSP node
func (bsp *BSPTree) raycastNode(node *BSPNode, origin, direction core.Vector3D, minT, maxT float64) *RaycastHit {
	if node == nil || minT > maxT {
		return nil
	}

	if node.isLeaf {
		// Test ray against all polygons in leaf
		var closestHit *RaycastHit

		for _, poly := range node.polygons {
			hit := bsp.rayPolygonIntersection(origin, direction, poly, minT, maxT)
			if hit != nil && hit.Hit {
				if closestHit == nil || hit.Distance < closestHit.Distance {
					closestHit = hit
				}
			}
		}

		return closestHit
	}

	// Calculate intersection with splitting plane
	denom := node.plane.Normal.X*direction.X + node.plane.Normal.Y*direction.Y + node.plane.Normal.Z*direction.Z

	if math.Abs(denom) < 1e-8 {
		// Ray is parallel to plane
		distance := bsp.pointToPlaneDistance(origin, node.plane)
		if distance > 0 {
			return bsp.raycastNode(node.front, origin, direction, minT, maxT)
		} else {
			return bsp.raycastNode(node.back, origin, direction, minT, maxT)
		}
	}

	t := -(node.plane.Normal.X*origin.X + node.plane.Normal.Y*origin.Y + node.plane.Normal.Z*origin.Z + node.plane.Distance) / denom

	var near, far *BSPNode
	originDistance := bsp.pointToPlaneDistance(origin, node.plane)

	if originDistance > 0 {
		near = node.front
		far = node.back
	} else {
		near = node.back
		far = node.front
	}

	if t > maxT || t < 0 {
		// Intersection outside ray segment
		return bsp.raycastNode(near, origin, direction, minT, maxT)
	} else if t < minT {
		// Intersection behind ray start
		return bsp.raycastNode(far, origin, direction, minT, maxT)
	} else {
		// Ray intersects plane within segment
		nearHit := bsp.raycastNode(near, origin, direction, minT, t)
		if nearHit != nil && nearHit.Hit {
			return nearHit
		}

		// Check intersection with plane polygons
		for _, poly := range node.polygons {
			hit := bsp.rayPolygonIntersection(origin, direction, poly, minT, maxT)
			if hit != nil && hit.Hit && hit.Distance <= t {
				return hit
			}
		}

		return bsp.raycastNode(far, origin, direction, t, maxT)
	}
}

// rayPolygonIntersection tests ray intersection with a polygon
func (bsp *BSPTree) rayPolygonIntersection(origin, direction core.Vector3D, poly Polygon, minT, maxT float64) *RaycastHit {
	if len(poly.vertices) < 3 {
		return nil
	}

	// Ray-plane intersection
	denom := poly.plane.Normal.X*direction.X + poly.plane.Normal.Y*direction.Y + poly.plane.Normal.Z*direction.Z

	if math.Abs(denom) < 1e-8 {
		return nil // Ray parallel to plane
	}

	t := -(poly.plane.Normal.X*origin.X + poly.plane.Normal.Y*origin.Y + poly.plane.Normal.Z*origin.Z + poly.plane.Distance) / denom

	if t < minT || t > maxT {
		return nil // Intersection outside ray segment
	}

	// Calculate intersection point
	intersection := core.Vector3D{
		X: origin.X + direction.X*t,
		Y: origin.Y + direction.Y*t,
		Z: origin.Z + direction.Z*t,
	}

	// Check if intersection point is inside polygon (simplified point-in-polygon test)
	if bsp.pointInPolygon(intersection, poly) {
		return &RaycastHit{
			Hit:          true,
			Distance:     t,
			Point:        intersection,
			Normal:       poly.plane.Normal,
			Polygon:      &poly,
			MaterialType: poly.material,
		}
	}

	return nil
}

// pointInPolygon simplified point-in-polygon test for 3D
func (bsp *BSPTree) pointInPolygon(point core.Vector3D, poly Polygon) bool {
	// This is a simplified implementation
	// In practice, you'd project to 2D and use a proper point-in-polygon algorithm

	// For now, just check if the point is reasonably close to the polygon centroid
	if len(poly.vertices) == 0 {
		return false
	}

	center := core.Vector3D{}
	for _, vertex := range poly.vertices {
		center.X += vertex.X
		center.Y += vertex.Y
		center.Z += vertex.Z
	}
	center.X /= float64(len(poly.vertices))
	center.Y /= float64(len(poly.vertices))
	center.Z /= float64(len(poly.vertices))

	// Calculate average distance from center to vertices (rough polygon "radius")
	avgRadius := 0.0
	for _, vertex := range poly.vertices {
		dx := vertex.X - center.X
		dy := vertex.Y - center.Y
		dz := vertex.Z - center.Z
		avgRadius += math.Sqrt(dx*dx + dy*dy + dz*dz)
	}
	avgRadius /= float64(len(poly.vertices))

	// Check if point is within this radius
	dx := point.X - center.X
	dy := point.Y - center.Y
	dz := point.Z - center.Z
	distance := math.Sqrt(dx*dx + dy*dy + dz*dz)

	return distance <= avgRadius
}
