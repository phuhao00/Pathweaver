package pathfinding

import (
	"container/heap"
	"fmt"
	"math"
	"pathweaver/internal/core"
)

// AStar3DPathfinder implements the A* pathfinding algorithm for 3D space
// Based on the classic A* algorithm extended to 3D with optimizations for
// different movement types (ground, flight, swimming)
type AStar3DPathfinder struct {
	heuristic       core.HeuristicFunc3D
	gridSize        float64
	allowDiagonal   bool
	allow3DDiagonal bool // Allow true 3D diagonal movement
	movementType    MovementType
	maxNodes        int
	verticalCost    float64 // Cost modifier for vertical movement
}

// MovementType defines different types of 3D movement
type MovementType int

const (
	MovementGround MovementType = iota // Ground-based, prefers 2D movement
	MovementFlight                     // Flying, all directions equal
	MovementSwim                       // Swimming, slight preference for horizontal
	MovementClimb                      // Climbing, can move vertically with cost
)

// Priority queue for 3D path nodes
type NodePriorityQueue3D []*core.PathNode3D

func (pq NodePriorityQueue3D) Len() int { return len(pq) }

func (pq NodePriorityQueue3D) Less(i, j int) bool {
	return pq[i].F < pq[j].F
}

func (pq NodePriorityQueue3D) Swap(i, j int) {
	pq[i], pq[j] = pq[j], pq[i]
	pq[i].Index = i
	pq[j].Index = j
}

func (pq *NodePriorityQueue3D) Push(x interface{}) {
	n := len(*pq)
	node := x.(*core.PathNode3D)
	node.Index = n
	*pq = append(*pq, node)
}

func (pq *NodePriorityQueue3D) Pop() interface{} {
	old := *pq
	n := len(old)
	node := old[n-1]
	old[n-1] = nil
	node.Index = -1
	*pq = old[0 : n-1]
	return node
}

// NewAStar3DPathfinder creates a new 3D A* pathfinder
func NewAStar3DPathfinder(gridSize float64, movementType MovementType) *AStar3DPathfinder {
	pathfinder := &AStar3DPathfinder{
		heuristic:       EuclideanDistance3D,
		gridSize:        gridSize,
		allowDiagonal:   true,
		allow3DDiagonal: true,
		movementType:    movementType,
		maxNodes:        50000,
		verticalCost:    1.0,
	}

	// Set movement-specific parameters
	switch movementType {
	case MovementGround:
		pathfinder.heuristic = GroundDistance3D
		pathfinder.verticalCost = 3.0 // Higher cost for vertical movement
		pathfinder.allow3DDiagonal = false
	case MovementFlight:
		pathfinder.heuristic = FlightDistance3D
		pathfinder.verticalCost = 1.0 // No vertical penalty
	case MovementSwim:
		pathfinder.heuristic = EuclideanDistance3D
		pathfinder.verticalCost = 1.2 // Slight vertical penalty
	case MovementClimb:
		pathfinder.heuristic = LayeredDistance3D
		pathfinder.verticalCost = 2.0 // Moderate vertical cost
	}

	return pathfinder
}

// SetHeuristic sets the heuristic function
func (a *AStar3DPathfinder) SetHeuristic(heuristic core.HeuristicFunc3D) {
	a.heuristic = heuristic
}

// FindPath finds a 3D path from start to goal avoiding obstacles
func (a *AStar3DPathfinder) FindPath(start, goal core.Vector3D, obstacles []core.AABB3D) ([]core.Vector3D, error) {
	// Discretize start and goal to grid
	startGrid := a.worldToGrid(start)
	goalGrid := a.worldToGrid(goal)

	// Create obstacle map for quick lookup
	obstacleMap := a.createObstacleMap(obstacles)

	// Initialize data structures
	openSet := &NodePriorityQueue3D{}
	heap.Init(openSet)

	openMap := make(map[string]*core.PathNode3D)
	closedMap := make(map[string]*core.PathNode3D)

	// Create start node
	startNode := &core.PathNode3D{
		Position: startGrid,
		G:        0,
		H:        a.heuristic(startGrid, goalGrid),
	}
	startNode.F = startNode.G + startNode.H

	heap.Push(openSet, startNode)
	openMap[a.positionKey3D(startGrid)] = startNode

	nodesExplored := 0

	for openSet.Len() > 0 && nodesExplored < a.maxNodes {
		current := heap.Pop(openSet).(*core.PathNode3D)
		currentKey := a.positionKey3D(current.Position)

		delete(openMap, currentKey)
		closedMap[currentKey] = current
		nodesExplored++

		// Check if we've reached the goal
		if a.isEqual3D(current.Position, goalGrid) {
			return a.reconstructPath3D(current), nil
		}

		// Explore neighbors
		neighbors := a.getNeighbors3D(current.Position)
		for _, neighborPos := range neighbors {
			neighborKey := a.positionKey3D(neighborPos)

			// Skip if already in closed set
			if _, inClosed := closedMap[neighborKey]; inClosed {
				continue
			}

			// Skip if position is blocked
			if a.isBlocked3D(neighborPos, obstacleMap) {
				continue
			}

			// Calculate tentative G score
			moveCost := a.getMoveCost3D(current.Position, neighborPos)
			tentativeG := current.G + moveCost

			// Check if this path to neighbor is better
			neighborNode, inOpen := openMap[neighborKey]

			if !inOpen {
				// Create new neighbor node
				neighborNode = &core.PathNode3D{
					Position: neighborPos,
					G:        tentativeG,
					H:        a.heuristic(neighborPos, goalGrid),
					Parent:   current,
				}
				neighborNode.F = neighborNode.G + neighborNode.H

				heap.Push(openSet, neighborNode)
				openMap[neighborKey] = neighborNode
			} else if tentativeG < neighborNode.G {
				// Update existing neighbor node
				neighborNode.G = tentativeG
				neighborNode.F = neighborNode.G + neighborNode.H
				neighborNode.Parent = current

				// Update position in heap
				heap.Fix(openSet, neighborNode.Index)
			}
		}
	}

	return nil, fmt.Errorf("no 3D path found from %v to %v (explored %d nodes)", start, goal, nodesExplored)
}

// worldToGrid converts world coordinates to grid coordinates
func (a *AStar3DPathfinder) worldToGrid(pos core.Vector3D) core.Vector3D {
	return core.Vector3D{
		X: math.Floor(pos.X/a.gridSize) * a.gridSize,
		Y: math.Floor(pos.Y/a.gridSize) * a.gridSize,
		Z: math.Floor(pos.Z/a.gridSize) * a.gridSize,
	}
}

// gridToWorld converts grid coordinates to world coordinates
func (a *AStar3DPathfinder) gridToWorld(pos core.Vector3D) core.Vector3D {
	return core.Vector3D{
		X: pos.X + a.gridSize/2,
		Y: pos.Y + a.gridSize/2,
		Z: pos.Z + a.gridSize/2,
	}
}

// createObstacleMap creates a map for quick 3D obstacle lookup
func (a *AStar3DPathfinder) createObstacleMap(obstacles []core.AABB3D) map[string]bool {
	obstacleMap := make(map[string]bool)

	for _, obstacle := range obstacles {
		// Discretize obstacle bounds to grid
		minX := math.Floor(obstacle.Min.X/a.gridSize) * a.gridSize
		maxX := math.Ceil(obstacle.Max.X/a.gridSize) * a.gridSize
		minY := math.Floor(obstacle.Min.Y/a.gridSize) * a.gridSize
		maxY := math.Ceil(obstacle.Max.Y/a.gridSize) * a.gridSize
		minZ := math.Floor(obstacle.Min.Z/a.gridSize) * a.gridSize
		maxZ := math.Ceil(obstacle.Max.Z/a.gridSize) * a.gridSize

		// Mark all grid cells within obstacle as blocked
		for x := minX; x < maxX; x += a.gridSize {
			for y := minY; y < maxY; y += a.gridSize {
				for z := minZ; z < maxZ; z += a.gridSize {
					key := a.positionKey3D(core.Vector3D{X: x, Y: y, Z: z})
					obstacleMap[key] = true
				}
			}
		}
	}

	return obstacleMap
}

// isBlocked3D checks if a 3D position is blocked by obstacles
func (a *AStar3DPathfinder) isBlocked3D(pos core.Vector3D, obstacleMap map[string]bool) bool {
	key := a.positionKey3D(pos)
	return obstacleMap[key]
}

// getNeighbors3D returns valid 3D neighbor positions
func (a *AStar3DPathfinder) getNeighbors3D(pos core.Vector3D) []core.Vector3D {
	var neighbors []core.Vector3D

	// 6-directional movement (face neighbors)
	faceDirections := []core.Vector3D{
		{X: a.gridSize, Y: 0, Z: 0},  // +X
		{X: -a.gridSize, Y: 0, Z: 0}, // -X
		{X: 0, Y: a.gridSize, Z: 0},  // +Y
		{X: 0, Y: -a.gridSize, Z: 0}, // -Y
		{X: 0, Y: 0, Z: a.gridSize},  // +Z
		{X: 0, Y: 0, Z: -a.gridSize}, // -Z
	}

	for _, dir := range faceDirections {
		neighbor := core.Vector3D{
			X: pos.X + dir.X,
			Y: pos.Y + dir.Y,
			Z: pos.Z + dir.Z,
		}
		neighbors = append(neighbors, neighbor)
	}

	// Add edge neighbors (12 edges of a cube) if diagonal movement allowed
	if a.allowDiagonal {
		edgeDirections := []core.Vector3D{
			// XY plane diagonals
			{X: a.gridSize, Y: a.gridSize, Z: 0},
			{X: a.gridSize, Y: -a.gridSize, Z: 0},
			{X: -a.gridSize, Y: a.gridSize, Z: 0},
			{X: -a.gridSize, Y: -a.gridSize, Z: 0},
			// XZ plane diagonals
			{X: a.gridSize, Y: 0, Z: a.gridSize},
			{X: a.gridSize, Y: 0, Z: -a.gridSize},
			{X: -a.gridSize, Y: 0, Z: a.gridSize},
			{X: -a.gridSize, Y: 0, Z: -a.gridSize},
			// YZ plane diagonals
			{X: 0, Y: a.gridSize, Z: a.gridSize},
			{X: 0, Y: a.gridSize, Z: -a.gridSize},
			{X: 0, Y: -a.gridSize, Z: a.gridSize},
			{X: 0, Y: -a.gridSize, Z: -a.gridSize},
		}

		for _, dir := range edgeDirections {
			neighbor := core.Vector3D{
				X: pos.X + dir.X,
				Y: pos.Y + dir.Y,
				Z: pos.Z + dir.Z,
			}
			neighbors = append(neighbors, neighbor)
		}
	}

	// Add corner neighbors (8 corners of a cube) if 3D diagonal movement allowed
	if a.allow3DDiagonal {
		cornerDirections := []core.Vector3D{
			{X: a.gridSize, Y: a.gridSize, Z: a.gridSize},
			{X: a.gridSize, Y: a.gridSize, Z: -a.gridSize},
			{X: a.gridSize, Y: -a.gridSize, Z: a.gridSize},
			{X: a.gridSize, Y: -a.gridSize, Z: -a.gridSize},
			{X: -a.gridSize, Y: a.gridSize, Z: a.gridSize},
			{X: -a.gridSize, Y: a.gridSize, Z: -a.gridSize},
			{X: -a.gridSize, Y: -a.gridSize, Z: a.gridSize},
			{X: -a.gridSize, Y: -a.gridSize, Z: -a.gridSize},
		}

		for _, dir := range cornerDirections {
			neighbor := core.Vector3D{
				X: pos.X + dir.X,
				Y: pos.Y + dir.Y,
				Z: pos.Z + dir.Z,
			}
			neighbors = append(neighbors, neighbor)
		}
	}

	return neighbors
}

// getMoveCost3D calculates the cost to move from one 3D position to another
func (a *AStar3DPathfinder) getMoveCost3D(from, to core.Vector3D) float64 {
	dx := math.Abs(to.X - from.X)
	dy := math.Abs(to.Y - from.Y)
	dz := math.Abs(to.Z - from.Z)

	// Count number of dimensions involved in movement
	dimensions := 0
	if dx > 0 {
		dimensions++
	}
	if dy > 0 {
		dimensions++
	}
	if dz > 0 {
		dimensions++
	}

	var baseCost float64

	switch dimensions {
	case 1: // Face neighbor (orthogonal)
		baseCost = a.gridSize
	case 2: // Edge neighbor (2D diagonal)
		baseCost = math.Sqrt(2) * a.gridSize
	case 3: // Corner neighbor (3D diagonal)
		baseCost = math.Sqrt(3) * a.gridSize
	default:
		baseCost = a.gridSize
	}

	// Apply vertical cost modifier
	if dz > 0 {
		baseCost *= a.verticalCost
	}

	return baseCost
}

// positionKey3D creates a unique string key for a 3D position
func (a *AStar3DPathfinder) positionKey3D(pos core.Vector3D) string {
	return fmt.Sprintf("%.2f,%.2f,%.2f", pos.X, pos.Y, pos.Z)
}

// isEqual3D checks if two 3D positions are equal (within tolerance)
func (a *AStar3DPathfinder) isEqual3D(a1, b core.Vector3D) bool {
	const epsilon = 0.001
	return math.Abs(a1.X-b.X) < epsilon &&
		math.Abs(a1.Y-b.Y) < epsilon &&
		math.Abs(a1.Z-b.Z) < epsilon
}

// reconstructPath3D builds the final 3D path from goal to start
func (a *AStar3DPathfinder) reconstructPath3D(goalNode *core.PathNode3D) []core.Vector3D {
	var path []core.Vector3D
	current := goalNode

	for current != nil {
		// Convert grid position back to world position
		worldPos := a.gridToWorld(current.Position)
		path = append(path, worldPos)
		current = current.Parent
	}

	// Reverse path to go from start to goal
	for i, j := 0, len(path)-1; i < j; i, j = i+1, j-1 {
		path[i], path[j] = path[j], path[i]
	}

	return path
}

// SetMovementType changes the movement type and adjusts parameters accordingly
func (a *AStar3DPathfinder) SetMovementType(movementType MovementType) {
	a.movementType = movementType

	switch movementType {
	case MovementGround:
		a.heuristic = GroundDistance3D
		a.verticalCost = 3.0
		a.allow3DDiagonal = false
	case MovementFlight:
		a.heuristic = FlightDistance3D
		a.verticalCost = 1.0
		a.allow3DDiagonal = true
	case MovementSwim:
		a.heuristic = EuclideanDistance3D
		a.verticalCost = 1.2
		a.allow3DDiagonal = true
	case MovementClimb:
		a.heuristic = LayeredDistance3D
		a.verticalCost = 2.0
		a.allow3DDiagonal = false
	}
}

// SetVerticalCost sets custom vertical movement cost
func (a *AStar3DPathfinder) SetVerticalCost(cost float64) {
	a.verticalCost = cost
}

// SetMaxNodes sets the maximum number of nodes to explore
func (a *AStar3DPathfinder) SetMaxNodes(maxNodes int) {
	a.maxNodes = maxNodes
}
