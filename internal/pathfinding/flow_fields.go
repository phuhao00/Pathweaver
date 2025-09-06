package pathfinding

import (
	"math"
	"pathweaver/internal/core"
)

// FlowField represents a flow field for efficient pathfinding of multiple agents
// Based on research papers on flow fields and crowd simulation
// Used in games like Supreme Commander and Company of Heroes for large unit groups
type FlowField struct {
	width       int
	height      int
	cellSize    float64
	bounds      core.AABB
	costField   [][]float64    // Cost to traverse each cell
	integrationField [][]float64 // Integration field (distance to goal)
	flowField   [][]core.Vector2D // Flow vectors
	goals       []core.Vector2D   // Multiple goals supported
}

// FlowFieldCell represents a single cell in the flow field
type FlowFieldCell struct {
	Cost        float64
	Integration float64
	Flow        core.Vector2D
	Blocked     bool
}

// NewFlowField creates a new flow field
func NewFlowField(bounds core.AABB, cellSize float64) *FlowField {
	width := int(math.Ceil((bounds.Max.X - bounds.Min.X) / cellSize))
	height := int(math.Ceil((bounds.Max.Y - bounds.Min.Y) / cellSize))
	
	// Initialize cost field
	costField := make([][]float64, height)
	for i := range costField {
		costField[i] = make([]float64, width)
		for j := range costField[i] {
			costField[i][j] = 1.0 // Default cost
		}
	}
	
	// Initialize integration field
	integrationField := make([][]float64, height)
	for i := range integrationField {
		integrationField[i] = make([]float64, width)
		for j := range integrationField[i] {
			integrationField[i][j] = math.MaxFloat64
		}
	}
	
	// Initialize flow field
	flowField := make([][]core.Vector2D, height)
	for i := range flowField {
		flowField[i] = make([]core.Vector2D, width)
	}
	
	return &FlowField{
		width:            width,
		height:           height,
		cellSize:         cellSize,
		bounds:           bounds,
		costField:        costField,
		integrationField: integrationField,
		flowField:        flowField,
	}
}

// SetCost sets the movement cost for a cell
func (ff *FlowField) SetCost(x, y int, cost float64) {
	if x >= 0 && x < ff.width && y >= 0 && y < ff.height {
		ff.costField[y][x] = cost
	}
}

// SetCostAtWorldPos sets the movement cost at a world position
func (ff *FlowField) SetCostAtWorldPos(pos core.Vector2D, cost float64) {
	x, y := ff.worldToGrid(pos)
	ff.SetCost(x, y, cost)
}

// AddObstacle adds an obstacle (infinite cost) to the field
func (ff *FlowField) AddObstacle(bounds core.AABB) {
	// Convert world bounds to grid bounds
	minX, minY := ff.worldToGrid(bounds.Min)
	maxX, maxY := ff.worldToGrid(bounds.Max)
	
	// Clamp to field bounds
	minX = int(math.Max(0, float64(minX)))
	minY = int(math.Max(0, float64(minY)))
	maxX = int(math.Min(float64(ff.width-1), float64(maxX)))
	maxY = int(math.Min(float64(ff.height-1), float64(maxY)))
	
	// Set high cost for obstacle cells
	for y := minY; y <= maxY; y++ {
		for x := minX; x <= maxX; x++ {
			ff.costField[y][x] = math.MaxFloat64
		}
	}
}

// Generate generates the flow field for the given goals
func (ff *FlowField) Generate(goals []core.Vector2D) {
	ff.goals = goals
	
	// Reset integration field
	for y := 0; y < ff.height; y++ {
		for x := 0; x < ff.width; x++ {
			ff.integrationField[y][x] = math.MaxFloat64
		}
	}
	
	// Set goals with zero integration cost
	goalQueue := []gridPoint{}
	for _, goal := range goals {
		gx, gy := ff.worldToGrid(goal)
		if gx >= 0 && gx < ff.width && gy >= 0 && gy < ff.height {
			ff.integrationField[gy][gx] = 0
			goalQueue = append(goalQueue, gridPoint{x: gx, y: gy})
		}
	}
	
	// Generate integration field using Dijkstra-like algorithm
	ff.generateIntegrationField(goalQueue)
	
	// Generate flow field from integration field
	ff.generateFlowField()
}

// gridPoint represents a point in grid coordinates
type gridPoint struct {
	x, y int
}

// generateIntegrationField uses a priority queue to calculate integration values
func (ff *FlowField) generateIntegrationField(startQueue []gridPoint) {
	// Simple queue-based propagation (could be optimized with priority queue)
	queue := make([]gridPoint, len(startQueue))
	copy(queue, startQueue)
	
	visited := make([][]bool, ff.height)
	for i := range visited {
		visited[i] = make([]bool, ff.width)
	}
	
	// Mark start points as visited
	for _, point := range startQueue {
		visited[point.y][point.x] = true
	}
	
	// 8-directional neighbors
	neighbors := []gridPoint{
		{-1, -1}, {0, -1}, {1, -1},
		{-1,  0},          {1,  0},
		{-1,  1}, {0,  1}, {1,  1},
	}
	
	for len(queue) > 0 {
		current := queue[0]
		queue = queue[1:]
		
		currentIntegration := ff.integrationField[current.y][current.x]
		
		// Check all neighbors
		for _, neighbor := range neighbors {
			nx := current.x + neighbor.x
			ny := current.y + neighbor.y
			
			// Check bounds
			if nx < 0 || nx >= ff.width || ny < 0 || ny >= ff.height {
				continue
			}
			
			// Skip if already visited
			if visited[ny][nx] {
				continue
			}
			
			// Skip if blocked
			if ff.costField[ny][nx] == math.MaxFloat64 {
				continue
			}
			
			// Calculate distance cost (diagonal vs orthogonal)
			var moveCost float64
			if neighbor.x != 0 && neighbor.y != 0 {
				moveCost = math.Sqrt2 * ff.costField[ny][nx] // Diagonal
			} else {
				moveCost = ff.costField[ny][nx] // Orthogonal
			}
			
			newIntegration := currentIntegration + moveCost
			
			// Update if we found a better path
			if newIntegration < ff.integrationField[ny][nx] {
				ff.integrationField[ny][nx] = newIntegration
				queue = append(queue, gridPoint{x: nx, y: ny})
				visited[ny][nx] = true
			}
		}
	}
}

// generateFlowField calculates flow vectors from integration field
func (ff *FlowField) generateFlowField() {
	for y := 0; y < ff.height; y++ {
		for x := 0; x < ff.width; x++ {
			// Skip blocked cells
			if ff.costField[y][x] == math.MaxFloat64 {
				ff.flowField[y][x] = core.Vector2D{X: 0, Y: 0}
				continue
			}
			
			// Find direction of steepest descent in integration field
			bestDirection := core.Vector2D{X: 0, Y: 0}
			currentIntegration := ff.integrationField[y][x]
			
			// Check all 8 neighbors
			neighbors := []struct {
				dx, dy int
			}{
				{-1, -1}, {0, -1}, {1, -1},
				{-1,  0},          {1,  0},
				{-1,  1}, {0,  1}, {1,  1},
			}
			
			for _, neighbor := range neighbors {
				nx := x + neighbor.dx
				ny := y + neighbor.dy
				
				// Check bounds
				if nx < 0 || nx >= ff.width || ny < 0 || ny >= ff.height {
					continue
				}
				
				neighborIntegration := ff.integrationField[ny][nx]
				
				// If neighbor has lower integration, it's a good direction
				if neighborIntegration < currentIntegration {
					// Weight by how much lower the integration is
					weight := currentIntegration - neighborIntegration
					direction := core.Vector2D{
						X: float64(neighbor.dx) * weight,
						Y: float64(neighbor.dy) * weight,
					}
					
					bestDirection.X += direction.X
					bestDirection.Y += direction.Y
				}
			}
			
			// Normalize the flow vector
			length := math.Sqrt(bestDirection.X*bestDirection.X + bestDirection.Y*bestDirection.Y)
			if length > 0 {
				ff.flowField[y][x] = core.Vector2D{
					X: bestDirection.X / length,
					Y: bestDirection.Y / length,
				}
			}
		}
	}
}

// GetFlowAt returns the flow vector at a world position
func (ff *FlowField) GetFlowAt(pos core.Vector2D) core.Vector2D {
	x, y := ff.worldToGrid(pos)
	if x < 0 || x >= ff.width || y < 0 || y >= ff.height {
		return core.Vector2D{X: 0, Y: 0}
	}
	
	return ff.flowField[y][x]
}

// GetIntegrationAt returns the integration value at a world position
func (ff *FlowField) GetIntegrationAt(pos core.Vector2D) float64 {
	x, y := ff.worldToGrid(pos)
	if x < 0 || x >= ff.width || y < 0 || y >= ff.height {
		return math.MaxFloat64
	}
	
	return ff.integrationField[y][x]
}

// GetCostAt returns the cost at a world position
func (ff *FlowField) GetCostAt(pos core.Vector2D) float64 {
	x, y := ff.worldToGrid(pos)
	if x < 0 || x >= ff.width || y < 0 || y >= ff.height {
		return math.MaxFloat64
	}
	
	return ff.costField[y][x]
}

// FollowFlow returns the next position for an agent following the flow field
func (ff *FlowField) FollowFlow(currentPos core.Vector2D, stepSize float64) core.Vector2D {
	flow := ff.GetFlowAt(currentPos)
	
	return core.Vector2D{
		X: currentPos.X + flow.X*stepSize,
		Y: currentPos.Y + flow.Y*stepSize,
	}
}

// GeneratePath generates a path following the flow field
func (ff *FlowField) GeneratePath(start core.Vector2D, maxSteps int, stepSize float64) []core.Vector2D {
	path := []core.Vector2D{start}
	current := start
	
	for i := 0; i < maxSteps; i++ {
		flow := ff.GetFlowAt(current)
		
		// Stop if no flow (reached goal or stuck)
		if flow.X == 0 && flow.Y == 0 {
			break
		}
		
		// Take a step in flow direction
		next := core.Vector2D{
			X: current.X + flow.X*stepSize,
			Y: current.Y + flow.Y*stepSize,
		}
		
		path = append(path, next)
		current = next
		
		// Stop if we've reached a goal (low integration value)
		if ff.GetIntegrationAt(current) < 1.0 {
			break
		}
	}
	
	return path
}

// SmoothPath applies smoothing to reduce jagged flow-field paths
func (ff *FlowField) SmoothPath(path []core.Vector2D, iterations int) []core.Vector2D {
	if len(path) < 3 || iterations <= 0 {
		return path
	}
	
	smoothed := make([]core.Vector2D, len(path))
	copy(smoothed, path)
	
	for iter := 0; iter < iterations; iter++ {
		// Keep first and last points unchanged
		for i := 1; i < len(smoothed)-1; i++ {
			// Weighted average with neighbors
			prev := smoothed[i-1]
			next := smoothed[i+1]
			
			smoothed[i] = core.Vector2D{
				X: (prev.X + 2*smoothed[i].X + next.X) / 4,
				Y: (prev.Y + 2*smoothed[i].Y + next.Y) / 4,
			}
		}
	}
	
	return smoothed
}

// GetFlowFieldData returns raw flow field data for debugging/visualization
func (ff *FlowField) GetFlowFieldData() [][]core.Vector2D {
	// Return copy of flow field
	result := make([][]core.Vector2D, ff.height)
	for i := range result {
		result[i] = make([]core.Vector2D, ff.width)
		copy(result[i], ff.flowField[i])
	}
	return result
}

// GetIntegrationFieldData returns raw integration field data
func (ff *FlowField) GetIntegrationFieldData() [][]float64 {
	// Return copy of integration field
	result := make([][]float64, ff.height)
	for i := range result {
		result[i] = make([]float64, ff.width)
		copy(result[i], ff.integrationField[i])
	}
	return result
}

// Helper methods

func (ff *FlowField) worldToGrid(pos core.Vector2D) (int, int) {
	x := int(math.Floor((pos.X - ff.bounds.Min.X) / ff.cellSize))
	y := int(math.Floor((pos.Y - ff.bounds.Min.Y) / ff.cellSize))
	return x, y
}

func (ff *FlowField) gridToWorld(x, y int) core.Vector2D {
	worldX := ff.bounds.Min.X + float64(x)*ff.cellSize + ff.cellSize/2
	worldY := ff.bounds.Min.Y + float64(y)*ff.cellSize + ff.cellSize/2
	return core.Vector2D{X: worldX, Y: worldY}
}

// MultiGoalFlowField handles multiple goals with different priorities
type MultiGoalFlowField struct {
	*FlowField
	goalPriorities map[int]float64 // Goal index to priority mapping
}

// NewMultiGoalFlowField creates a flow field that can handle multiple prioritized goals
func NewMultiGoalFlowField(bounds core.AABB, cellSize float64) *MultiGoalFlowField {
	return &MultiGoalFlowField{
		FlowField:      NewFlowField(bounds, cellSize),
		goalPriorities: make(map[int]float64),
	}
}

// GenerateWithPriorities generates flow field with goal priorities
func (mgff *MultiGoalFlowField) GenerateWithPriorities(goals []core.Vector2D, priorities []float64) {
	mgff.goals = goals
	
	// Reset integration field
	for y := 0; y < mgff.height; y++ {
		for x := 0; x < mgff.width; x++ {
			mgff.integrationField[y][x] = math.MaxFloat64
		}
	}
	
	// Set goals with priority-weighted integration cost
	goalQueue := []gridPoint{}
	for i, goal := range goals {
		gx, gy := mgff.worldToGrid(goal)
		if gx >= 0 && gx < mgff.width && gy >= 0 && gy < mgff.height {
			priority := 1.0
			if i < len(priorities) {
				priority = priorities[i]
			}
			
			mgff.integrationField[gy][gx] = 1.0 / priority // Higher priority = lower cost
			goalQueue = append(goalQueue, gridPoint{x: gx, y: gy})
		}
	}
	
	// Generate fields
	mgff.generateIntegrationField(goalQueue)
	mgff.generateFlowField()
}
