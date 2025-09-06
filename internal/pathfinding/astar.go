package pathfinding

import (
	"container/heap"
	"fmt"
	"math"
	"pathweaver/internal/core"
)

// AStarPathfinder implements the A* pathfinding algorithm
type AStarPathfinder struct {
	heuristic     core.HeuristicFunc
	gridSize      float64
	allowDiagonal bool
	maxNodes      int // Prevent infinite loops
}

// NewAStarPathfinder creates a new A* pathfinder
func NewAStarPathfinder(gridSize float64) *AStarPathfinder {
	return &AStarPathfinder{
		heuristic:     EuclideanDistance,
		gridSize:      gridSize,
		allowDiagonal: true,
		maxNodes:      10000, // Reasonable limit
	}
}

// SetHeuristic sets the heuristic function
func (a *AStarPathfinder) SetHeuristic(heuristic core.HeuristicFunc) {
	a.heuristic = heuristic
}

// SetAllowDiagonal sets whether diagonal movement is allowed
func (a *AStarPathfinder) SetAllowDiagonal(allow bool) {
	a.allowDiagonal = allow
}

// SetMaxNodes sets the maximum number of nodes to explore
func (a *AStarPathfinder) SetMaxNodes(maxNodes int) {
	a.maxNodes = maxNodes
}

// FindPath finds a path from start to goal avoiding obstacles
func (a *AStarPathfinder) FindPath(start, goal core.Vector2D, obstacles []core.AABB) ([]core.Vector2D, error) {
	// Discretize start and goal to grid
	startGrid := a.worldToGrid(start)
	goalGrid := a.worldToGrid(goal)

	// Create obstacle map for quick lookup
	obstacleMap := a.createObstacleMap(obstacles)

	// Initialize data structures
	openSet := &NodePriorityQueue{}
	heap.Init(openSet)

	openMap := make(map[string]*core.PathNode)   // For quick lookup in open set
	closedMap := make(map[string]*core.PathNode) // Nodes already processed

	// Create start node
	startNode := &core.PathNode{
		Position: startGrid,
		G:        0,
		H:        a.heuristic(startGrid, goalGrid),
	}
	startNode.F = startNode.G + startNode.H

	heap.Push(openSet, startNode)
	openMap[a.positionKey(startGrid)] = startNode

	nodesExplored := 0

	for openSet.Len() > 0 && nodesExplored < a.maxNodes {
		// Get the node with lowest F score
		current := heap.Pop(openSet).(*core.PathNode)
		currentKey := a.positionKey(current.Position)

		delete(openMap, currentKey)
		closedMap[currentKey] = current
		nodesExplored++

		// Check if we've reached the goal
		if a.isEqual(current.Position, goalGrid) {
			return a.reconstructPath(current), nil
		}

		// Explore neighbors
		neighbors := a.getNeighbors(current.Position)
		for _, neighborPos := range neighbors {
			neighborKey := a.positionKey(neighborPos)

			// Skip if already in closed set
			if _, inClosed := closedMap[neighborKey]; inClosed {
				continue
			}

			// Skip if position is blocked
			if a.isBlocked(neighborPos, obstacleMap) {
				continue
			}

			// Calculate tentative G score
			moveCost := a.getMoveCost(current.Position, neighborPos)
			tentativeG := current.G + moveCost

			// Check if this path to neighbor is better
			neighborNode, inOpen := openMap[neighborKey]

			if !inOpen {
				// Create new neighbor node
				neighborNode = &core.PathNode{
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

	return nil, fmt.Errorf("no path found from %v to %v (explored %d nodes)", start, goal, nodesExplored)
}

// worldToGrid converts world coordinates to grid coordinates
func (a *AStarPathfinder) worldToGrid(pos core.Vector2D) core.Vector2D {
	return core.Vector2D{
		X: math.Floor(pos.X/a.gridSize) * a.gridSize,
		Y: math.Floor(pos.Y/a.gridSize) * a.gridSize,
	}
}

// gridToWorld converts grid coordinates to world coordinates (center of grid cell)
func (a *AStarPathfinder) gridToWorld(pos core.Vector2D) core.Vector2D {
	return core.Vector2D{
		X: pos.X + a.gridSize/2,
		Y: pos.Y + a.gridSize/2,
	}
}

// createObstacleMap creates a map for quick obstacle lookup
func (a *AStarPathfinder) createObstacleMap(obstacles []core.AABB) map[string]bool {
	obstacleMap := make(map[string]bool)

	for _, obstacle := range obstacles {
		// Discretize obstacle bounds to grid
		minX := math.Floor(obstacle.Min.X/a.gridSize) * a.gridSize
		maxX := math.Ceil(obstacle.Max.X/a.gridSize) * a.gridSize
		minY := math.Floor(obstacle.Min.Y/a.gridSize) * a.gridSize
		maxY := math.Ceil(obstacle.Max.Y/a.gridSize) * a.gridSize

		// Mark all grid cells within obstacle as blocked
		for x := minX; x < maxX; x += a.gridSize {
			for y := minY; y < maxY; y += a.gridSize {
				key := a.positionKey(core.Vector2D{X: x, Y: y})
				obstacleMap[key] = true
			}
		}
	}

	return obstacleMap
}

// isBlocked checks if a position is blocked by obstacles
func (a *AStarPathfinder) isBlocked(pos core.Vector2D, obstacleMap map[string]bool) bool {
	key := a.positionKey(pos)
	return obstacleMap[key]
}

// getNeighbors returns valid neighbor positions
func (a *AStarPathfinder) getNeighbors(pos core.Vector2D) []core.Vector2D {
	var neighbors []core.Vector2D

	// 4-directional movement (orthogonal)
	directions := []core.Vector2D{
		{X: 0, Y: a.gridSize},  // North
		{X: a.gridSize, Y: 0},  // East
		{X: 0, Y: -a.gridSize}, // South
		{X: -a.gridSize, Y: 0}, // West
	}

	// Add diagonal directions if allowed
	if a.allowDiagonal {
		diagonals := []core.Vector2D{
			{X: a.gridSize, Y: a.gridSize},   // Northeast
			{X: a.gridSize, Y: -a.gridSize},  // Southeast
			{X: -a.gridSize, Y: -a.gridSize}, // Southwest
			{X: -a.gridSize, Y: a.gridSize},  // Northwest
		}
		directions = append(directions, diagonals...)
	}

	for _, dir := range directions {
		neighbor := core.Vector2D{
			X: pos.X + dir.X,
			Y: pos.Y + dir.Y,
		}
		neighbors = append(neighbors, neighbor)
	}

	return neighbors
}

// getMoveCost calculates the cost to move from one position to another
func (a *AStarPathfinder) getMoveCost(from, to core.Vector2D) float64 {
	dx := math.Abs(to.X - from.X)
	dy := math.Abs(to.Y - from.Y)

	// Diagonal movement
	if dx > 0 && dy > 0 {
		return math.Sqrt2 * a.gridSize
	}

	// Orthogonal movement
	return a.gridSize
}

// positionKey creates a unique string key for a position
func (a *AStarPathfinder) positionKey(pos core.Vector2D) string {
	return fmt.Sprintf("%.2f,%.2f", pos.X, pos.Y)
}

// isEqual checks if two positions are equal (within tolerance)
func (a *AStarPathfinder) isEqual(a1, b core.Vector2D) bool {
	const epsilon = 0.001
	return math.Abs(a1.X-b.X) < epsilon && math.Abs(a1.Y-b.Y) < epsilon
}

// reconstructPath builds the final path from goal to start
func (a *AStarPathfinder) reconstructPath(goalNode *core.PathNode) []core.Vector2D {
	var path []core.Vector2D
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
