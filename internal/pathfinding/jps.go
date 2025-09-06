package pathfinding

import (
	"container/heap"
	"fmt"
	"math"
	"pathweaver/internal/core"
)

// JPSPathfinder implements Jump Point Search algorithm
// JPS is an optimization of A* that can be significantly faster on grid-based maps
type JPSPathfinder struct {
	heuristic   core.HeuristicFunc
	gridSize    float64
	maxNodes    int
	obstacleMap map[string]bool
}

// NewJPSPathfinder creates a new JPS pathfinder
func NewJPSPathfinder(gridSize float64) *JPSPathfinder {
	return &JPSPathfinder{
		heuristic: EuclideanDistance,
		gridSize:  gridSize,
		maxNodes:  10000,
	}
}

// SetHeuristic sets the heuristic function
func (j *JPSPathfinder) SetHeuristic(heuristic core.HeuristicFunc) {
	j.heuristic = heuristic
}

// FindPath finds a path using Jump Point Search
func (j *JPSPathfinder) FindPath(start, goal core.Vector2D, obstacles []core.AABB) ([]core.Vector2D, error) {
	// Discretize start and goal to grid
	startGrid := j.worldToGrid(start)
	goalGrid := j.worldToGrid(goal)

	// Create obstacle map
	j.obstacleMap = j.createObstacleMap(obstacles)

	// Initialize data structures
	openSet := &NodePriorityQueue{}
	heap.Init(openSet)

	openMap := make(map[string]*core.PathNode)
	closedMap := make(map[string]*core.PathNode)

	// Create start node
	startNode := &core.PathNode{
		Position: startGrid,
		G:        0,
		H:        j.heuristic(startGrid, goalGrid),
	}
	startNode.F = startNode.G + startNode.H

	heap.Push(openSet, startNode)
	openMap[j.positionKey(startGrid)] = startNode

	nodesExplored := 0

	for openSet.Len() > 0 && nodesExplored < j.maxNodes {
		current := heap.Pop(openSet).(*core.PathNode)
		currentKey := j.positionKey(current.Position)

		delete(openMap, currentKey)
		closedMap[currentKey] = current
		nodesExplored++

		// Check if we've reached the goal
		if j.isEqual(current.Position, goalGrid) {
			return j.reconstructPath(current), nil
		}

		// Get jump points as successors
		successors := j.getSuccessors(current, goalGrid)

		for _, successor := range successors {
			successorKey := j.positionKey(successor)

			if _, inClosed := closedMap[successorKey]; inClosed {
				continue
			}

			// Calculate G score (distance from start)
			tentativeG := current.G + j.heuristic(current.Position, successor)

			successorNode, inOpen := openMap[successorKey]

			if !inOpen {
				successorNode = &core.PathNode{
					Position: successor,
					G:        tentativeG,
					H:        j.heuristic(successor, goalGrid),
					Parent:   current,
				}
				successorNode.F = successorNode.G + successorNode.H

				heap.Push(openSet, successorNode)
				openMap[successorKey] = successorNode
			} else if tentativeG < successorNode.G {
				successorNode.G = tentativeG
				successorNode.F = successorNode.G + successorNode.H
				successorNode.Parent = current
				heap.Fix(openSet, successorNode.Index)
			}
		}
	}

	return nil, fmt.Errorf("no path found using JPS from %v to %v (explored %d nodes)", start, goal, nodesExplored)
}

// getSuccessors finds jump points from the current node
func (j *JPSPathfinder) getSuccessors(current *core.PathNode, goal core.Vector2D) []core.Vector2D {
	var successors []core.Vector2D

	// Get pruned neighbors based on parent direction
	neighbors := j.getPrunedNeighbors(current)

	for _, neighbor := range neighbors {
		// Calculate direction from current to neighbor
		direction := core.Vector2D{
			X: neighbor.X - current.Position.X,
			Y: neighbor.Y - current.Position.Y,
		}

		// Normalize direction to unit grid steps
		if direction.X != 0 {
			direction.X = direction.X / math.Abs(direction.X) * j.gridSize
		}
		if direction.Y != 0 {
			direction.Y = direction.Y / math.Abs(direction.Y) * j.gridSize
		}

		// Jump in this direction
		jumpPoint := j.jump(current.Position, direction, goal)
		if jumpPoint != nil {
			successors = append(successors, *jumpPoint)
		}
	}

	return successors
}

// getPrunedNeighbors returns neighbors with pruning rules applied
func (j *JPSPathfinder) getPrunedNeighbors(current *core.PathNode) []core.Vector2D {
	var neighbors []core.Vector2D

	if current.Parent == nil {
		// First node - return all valid neighbors
		return j.getAllNeighbors(current.Position)
	}

	// Calculate parent direction
	parentDir := core.Vector2D{
		X: current.Position.X - current.Parent.Position.X,
		Y: current.Position.Y - current.Parent.Position.Y,
	}

	// Normalize direction
	if parentDir.X != 0 {
		parentDir.X = parentDir.X / math.Abs(parentDir.X)
	}
	if parentDir.Y != 0 {
		parentDir.Y = parentDir.Y / math.Abs(parentDir.Y)
	}

	// Apply pruning rules based on direction
	if parentDir.X != 0 && parentDir.Y != 0 {
		// Diagonal movement
		neighbors = j.getDiagonalNeighbors(current.Position, parentDir)
	} else {
		// Orthogonal movement
		neighbors = j.getOrthogonalNeighbors(current.Position, parentDir)
	}

	return neighbors
}

// getAllNeighbors returns all 8 neighbors (if not blocked)
func (j *JPSPathfinder) getAllNeighbors(pos core.Vector2D) []core.Vector2D {
	var neighbors []core.Vector2D

	directions := []core.Vector2D{
		{X: 0, Y: j.gridSize}, {X: j.gridSize, Y: 0}, {X: 0, Y: -j.gridSize}, {X: -j.gridSize, Y: 0},
		{X: j.gridSize, Y: j.gridSize}, {X: j.gridSize, Y: -j.gridSize},
		{X: -j.gridSize, Y: -j.gridSize}, {X: -j.gridSize, Y: j.gridSize},
	}

	for _, dir := range directions {
		neighbor := core.Vector2D{X: pos.X + dir.X, Y: pos.Y + dir.Y}
		if !j.isBlocked(neighbor) {
			neighbors = append(neighbors, neighbor)
		}
	}

	return neighbors
}

// getDiagonalNeighbors returns pruned neighbors for diagonal movement
func (j *JPSPathfinder) getDiagonalNeighbors(pos, parentDir core.Vector2D) []core.Vector2D {
	var neighbors []core.Vector2D

	// Natural neighbors (continue diagonal and its components)
	diagonal := core.Vector2D{X: pos.X + parentDir.X*j.gridSize, Y: pos.Y + parentDir.Y*j.gridSize}
	horizontal := core.Vector2D{X: pos.X + parentDir.X*j.gridSize, Y: pos.Y}
	vertical := core.Vector2D{X: pos.X, Y: pos.Y + parentDir.Y*j.gridSize}

	if !j.isBlocked(diagonal) {
		neighbors = append(neighbors, diagonal)
	}
	if !j.isBlocked(horizontal) {
		neighbors = append(neighbors, horizontal)
	}
	if !j.isBlocked(vertical) {
		neighbors = append(neighbors, vertical)
	}

	// Forced neighbors (when there's an obstacle)
	// Check for forced neighbors due to blocked cells
	leftBlocked := j.isBlocked(core.Vector2D{X: pos.X - parentDir.X*j.gridSize, Y: pos.Y})
	rightBlocked := j.isBlocked(core.Vector2D{X: pos.X, Y: pos.Y - parentDir.Y*j.gridSize})

	if leftBlocked {
		forced := core.Vector2D{X: pos.X - parentDir.X*j.gridSize, Y: pos.Y + parentDir.Y*j.gridSize}
		if !j.isBlocked(forced) {
			neighbors = append(neighbors, forced)
		}
	}

	if rightBlocked {
		forced := core.Vector2D{X: pos.X + parentDir.X*j.gridSize, Y: pos.Y - parentDir.Y*j.gridSize}
		if !j.isBlocked(forced) {
			neighbors = append(neighbors, forced)
		}
	}

	return neighbors
}

// getOrthogonalNeighbors returns pruned neighbors for orthogonal movement
func (j *JPSPathfinder) getOrthogonalNeighbors(pos, parentDir core.Vector2D) []core.Vector2D {
	var neighbors []core.Vector2D

	// Natural neighbor (continue in same direction)
	natural := core.Vector2D{X: pos.X + parentDir.X*j.gridSize, Y: pos.Y + parentDir.Y*j.gridSize}
	if !j.isBlocked(natural) {
		neighbors = append(neighbors, natural)
	}

	// Check for forced neighbors
	if parentDir.X != 0 { // Horizontal movement
		// Check above and below
		if j.isBlocked(core.Vector2D{X: pos.X, Y: pos.Y + j.gridSize}) {
			forced := core.Vector2D{X: pos.X + parentDir.X*j.gridSize, Y: pos.Y + j.gridSize}
			if !j.isBlocked(forced) {
				neighbors = append(neighbors, forced)
			}
		}
		if j.isBlocked(core.Vector2D{X: pos.X, Y: pos.Y - j.gridSize}) {
			forced := core.Vector2D{X: pos.X + parentDir.X*j.gridSize, Y: pos.Y - j.gridSize}
			if !j.isBlocked(forced) {
				neighbors = append(neighbors, forced)
			}
		}
	} else { // Vertical movement
		// Check left and right
		if j.isBlocked(core.Vector2D{X: pos.X + j.gridSize, Y: pos.Y}) {
			forced := core.Vector2D{X: pos.X + j.gridSize, Y: pos.Y + parentDir.Y*j.gridSize}
			if !j.isBlocked(forced) {
				neighbors = append(neighbors, forced)
			}
		}
		if j.isBlocked(core.Vector2D{X: pos.X - j.gridSize, Y: pos.Y}) {
			forced := core.Vector2D{X: pos.X - j.gridSize, Y: pos.Y + parentDir.Y*j.gridSize}
			if !j.isBlocked(forced) {
				neighbors = append(neighbors, forced)
			}
		}
	}

	return neighbors
}

// jump performs the jumping operation in JPS
func (j *JPSPathfinder) jump(pos, direction, goal core.Vector2D) *core.Vector2D {
	next := core.Vector2D{X: pos.X + direction.X, Y: pos.Y + direction.Y}

	// Check if next position is blocked
	if j.isBlocked(next) {
		return nil
	}

	// Check if we reached the goal
	if j.isEqual(next, goal) {
		return &next
	}

	// Check for forced neighbors
	if j.hasForced(next, direction) {
		return &next
	}

	// Diagonal movement: check horizontal and vertical jumps
	if direction.X != 0 && direction.Y != 0 {
		horizontalDir := core.Vector2D{X: direction.X, Y: 0}
		verticalDir := core.Vector2D{X: 0, Y: direction.Y}

		if j.jump(next, horizontalDir, goal) != nil || j.jump(next, verticalDir, goal) != nil {
			return &next
		}
	}

	// Continue jumping in the same direction
	return j.jump(next, direction, goal)
}

// hasForced checks if a position has forced neighbors
func (j *JPSPathfinder) hasForced(pos, direction core.Vector2D) bool {
	if direction.X != 0 && direction.Y != 0 {
		// Diagonal movement - check for forced neighbors
		return j.isBlocked(core.Vector2D{X: pos.X - direction.X, Y: pos.Y}) ||
			j.isBlocked(core.Vector2D{X: pos.X, Y: pos.Y - direction.Y})
	} else if direction.X != 0 {
		// Horizontal movement
		return j.isBlocked(core.Vector2D{X: pos.X, Y: pos.Y + j.gridSize}) ||
			j.isBlocked(core.Vector2D{X: pos.X, Y: pos.Y - j.gridSize})
	} else {
		// Vertical movement
		return j.isBlocked(core.Vector2D{X: pos.X + j.gridSize, Y: pos.Y}) ||
			j.isBlocked(core.Vector2D{X: pos.X - j.gridSize, Y: pos.Y})
	}
}

// Helper methods (shared with A*)

func (j *JPSPathfinder) worldToGrid(pos core.Vector2D) core.Vector2D {
	return core.Vector2D{
		X: math.Floor(pos.X/j.gridSize) * j.gridSize,
		Y: math.Floor(pos.Y/j.gridSize) * j.gridSize,
	}
}

func (j *JPSPathfinder) gridToWorld(pos core.Vector2D) core.Vector2D {
	return core.Vector2D{
		X: pos.X + j.gridSize/2,
		Y: pos.Y + j.gridSize/2,
	}
}

func (j *JPSPathfinder) createObstacleMap(obstacles []core.AABB) map[string]bool {
	obstacleMap := make(map[string]bool)

	for _, obstacle := range obstacles {
		minX := math.Floor(obstacle.Min.X/j.gridSize) * j.gridSize
		maxX := math.Ceil(obstacle.Max.X/j.gridSize) * j.gridSize
		minY := math.Floor(obstacle.Min.Y/j.gridSize) * j.gridSize
		maxY := math.Ceil(obstacle.Max.Y/j.gridSize) * j.gridSize

		for x := minX; x < maxX; x += j.gridSize {
			for y := minY; y < maxY; y += j.gridSize {
				key := j.positionKey(core.Vector2D{X: x, Y: y})
				obstacleMap[key] = true
			}
		}
	}

	return obstacleMap
}

func (j *JPSPathfinder) isBlocked(pos core.Vector2D) bool {
	key := j.positionKey(pos)
	return j.obstacleMap[key]
}

func (j *JPSPathfinder) positionKey(pos core.Vector2D) string {
	return fmt.Sprintf("%.2f,%.2f", pos.X, pos.Y)
}

func (j *JPSPathfinder) isEqual(a, b core.Vector2D) bool {
	const epsilon = 0.001
	return math.Abs(a.X-b.X) < epsilon && math.Abs(a.Y-b.Y) < epsilon
}

func (j *JPSPathfinder) reconstructPath(goalNode *core.PathNode) []core.Vector2D {
	var path []core.Vector2D
	current := goalNode

	for current != nil {
		worldPos := j.gridToWorld(current.Position)
		path = append(path, worldPos)
		current = current.Parent
	}

	// Reverse path
	for i, j := 0, len(path)-1; i < j; i, j = i+1, j-1 {
		path[i], path[j] = path[j], path[i]
	}

	return path
}
