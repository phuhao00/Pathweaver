package moba

import (
	"fmt"
	"math"
	"pathweaver/internal/core"
	"sync"
)

// InfluenceMap represents tactical influence for MOBA AI decision making
// Based on research papers on influence maps in RTS/MOBA games and
// techniques used in games like League of Legends and Dota 2
type InfluenceMap struct {
	mu         sync.RWMutex
	width      int
	height     int
	cellSize   float64
	bounds     core.AABB
	cells      [][]InfluenceCell
	layers     map[InfluenceLayer][][]float64
	decayRate  float64
	updateTime int64
}

// InfluenceCell represents a single cell in the influence map
type InfluenceCell struct {
	TotalInfluence float64
	TeamInfluences map[TeamID]float64
	DangerLevel    float64
	SafetyLevel    float64
	ControlLevel   float64
	LastUpdate     int64
}

// InfluenceLayer represents different types of influence
type InfluenceLayer int

const (
	LayerDamage    InfluenceLayer = iota // Raw damage potential
	LayerSafety                          // Safety for movement
	LayerControl                         // Map control (towers, objectives)
	LayerVision                          // Vision coverage
	LayerMobility                        // Movement speed influence
	LayerHealing                         // Healing/support influence
	LayerObjective                       // Objective importance
)

// TeamID represents team identifier
type TeamID int

const (
	TeamNeutral TeamID = iota
	TeamBlue
	TeamRed
)

// InfluenceSource represents an entity that generates influence
type InfluenceSource struct {
	Position  core.Vector2D
	Team      TeamID
	Influence float64
	Range     float64
	Layer     InfluenceLayer
	DecayType DecayType
	Duration  float64 // For temporary influences
	IsActive  bool
}

// DecayType defines how influence decays with distance
type DecayType int

const (
	DecayLinear DecayType = iota
	DecayQuadratic
	DecayExponential
	DecayConstant
)

// NewInfluenceMap creates a new influence map
func NewInfluenceMap(bounds core.AABB, cellSize float64) *InfluenceMap {
	width := int(math.Ceil((bounds.Max.X - bounds.Min.X) / cellSize))
	height := int(math.Ceil((bounds.Max.Y - bounds.Min.Y) / cellSize))

	cells := make([][]InfluenceCell, height)
	for i := range cells {
		cells[i] = make([]InfluenceCell, width)
		for j := range cells[i] {
			cells[i][j] = InfluenceCell{
				TeamInfluences: make(map[TeamID]float64),
			}
		}
	}

	layers := make(map[InfluenceLayer][][]float64)
	for layer := LayerDamage; layer <= LayerObjective; layer++ {
		layerData := make([][]float64, height)
		for i := range layerData {
			layerData[i] = make([]float64, width)
		}
		layers[layer] = layerData
	}

	return &InfluenceMap{
		width:     width,
		height:    height,
		cellSize:  cellSize,
		bounds:    bounds,
		cells:     cells,
		layers:    layers,
		decayRate: 0.95, // 5% decay per update
	}
}

// AddInfluenceSource adds a source of influence to the map
func (im *InfluenceMap) AddInfluenceSource(source InfluenceSource) {
	im.mu.Lock()
	defer im.mu.Unlock()

	if !source.IsActive {
		return
	}

	// Convert world position to grid coordinates
	centerX, centerY := im.worldToGrid(source.Position)
	if centerX < 0 || centerX >= im.width || centerY < 0 || centerY >= im.height {
		return
	}

	// Calculate affected area
	rangeCells := int(math.Ceil(source.Range / im.cellSize))

	// Apply influence in a radius around the source
	for dy := -rangeCells; dy <= rangeCells; dy++ {
		for dx := -rangeCells; dx <= rangeCells; dx++ {
			x := centerX + dx
			y := centerY + dy

			if x < 0 || x >= im.width || y < 0 || y >= im.height {
				continue
			}

			// Calculate distance from source
			worldX, worldY := im.gridToWorld(x, y)
			distance := math.Sqrt(
				math.Pow(worldX-source.Position.X, 2) +
					math.Pow(worldY-source.Position.Y, 2),
			)

			if distance <= source.Range {
				// Calculate influence value based on decay type
				influence := im.calculateInfluence(source.Influence, distance, source.Range, source.DecayType)

				// Apply to appropriate layer
				im.layers[source.Layer][y][x] += influence

				// Update cell data
				cell := &im.cells[y][x]
				cell.TeamInfluences[source.Team] += influence
				cell.TotalInfluence += influence
				cell.LastUpdate = im.updateTime

				// Update derived values
				im.updateCellDerivedValues(cell, x, y)
			}
		}
	}
}

// RemoveInfluenceSource removes influence from a source
func (im *InfluenceMap) RemoveInfluenceSource(source InfluenceSource) {
	source.Influence = -source.Influence // Negative influence to subtract
	im.AddInfluenceSource(source)
}

// GetInfluenceAt returns influence data at a world position
func (im *InfluenceMap) GetInfluenceAt(position core.Vector2D) InfluenceCell {
	im.mu.RLock()
	defer im.mu.RUnlock()

	x, y := im.worldToGrid(position)
	if x < 0 || x >= im.width || y < 0 || y >= im.height {
		return InfluenceCell{TeamInfluences: make(map[TeamID]float64)}
	}

	return im.cells[y][x]
}

// GetLayerInfluenceAt returns influence for a specific layer at position
func (im *InfluenceMap) GetLayerInfluenceAt(position core.Vector2D, layer InfluenceLayer) float64 {
	im.mu.RLock()
	defer im.mu.RUnlock()

	x, y := im.worldToGrid(position)
	if x < 0 || x >= im.width || y < 0 || y >= im.height {
		return 0
	}

	return im.layers[layer][y][x]
}

// FindSafePath finds the safest path between two points
func (im *InfluenceMap) FindSafePath(start, goal core.Vector2D, team TeamID) []core.Vector2D {
	startX, startY := im.worldToGrid(start)
	goalX, goalY := im.worldToGrid(goal)

	// Simple A* implementation with safety as cost function
	type pathNode struct {
		x, y    int
		g, h, f float64
		parent  *pathNode
	}

	openSet := []*pathNode{{x: startX, y: startY, g: 0, h: im.heuristic(startX, startY, goalX, goalY)}}
	openSet[0].f = openSet[0].h

	closedSet := make(map[string]bool)

	for len(openSet) > 0 {
		// Find node with lowest f score
		current := openSet[0]
		currentIndex := 0
		for i, node := range openSet {
			if node.f < current.f {
				current = node
				currentIndex = i
			}
		}

		// Remove current from open set
		openSet = append(openSet[:currentIndex], openSet[currentIndex+1:]...)
		closedSet[im.nodeKey(current.x, current.y)] = true

		// Check if goal reached
		if current.x == goalX && current.y == goalY {
			// Reconstruct path
			var path []core.Vector2D
			for current != nil {
				worldX, worldY := im.gridToWorld(current.x, current.y)
				path = append([]core.Vector2D{{X: worldX, Y: worldY}}, path...)
				current = current.parent
			}
			return path
		}

		// Explore neighbors
		neighbors := [][]int{{-1, 0}, {1, 0}, {0, -1}, {0, 1}, {-1, -1}, {-1, 1}, {1, -1}, {1, 1}}
		for _, neighbor := range neighbors {
			nx, ny := current.x+neighbor[0], current.y+neighbor[1]

			if nx < 0 || nx >= im.width || ny < 0 || ny >= im.height {
				continue
			}

			key := im.nodeKey(nx, ny)
			if closedSet[key] {
				continue
			}

			// Calculate safety cost (inverse of danger)
			danger := im.getDangerAt(nx, ny, team)
			safetyCost := 1.0 + danger*10.0 // Higher danger = higher cost

			tentativeG := current.g + safetyCost

			// Check if in open set
			var neighborNode *pathNode
			for _, node := range openSet {
				if node.x == nx && node.y == ny {
					neighborNode = node
					break
				}
			}

			if neighborNode == nil {
				neighborNode = &pathNode{
					x: nx, y: ny,
					g:      tentativeG,
					h:      im.heuristic(nx, ny, goalX, goalY),
					parent: current,
				}
				neighborNode.f = neighborNode.g + neighborNode.h
				openSet = append(openSet, neighborNode)
			} else if tentativeG < neighborNode.g {
				neighborNode.g = tentativeG
				neighborNode.f = neighborNode.g + neighborNode.h
				neighborNode.parent = current
			}
		}
	}

	// No path found, return direct path
	return []core.Vector2D{start, goal}
}

// GetTeamAdvantage calculates team advantage at a position
func (im *InfluenceMap) GetTeamAdvantage(position core.Vector2D, team TeamID) float64 {
	cell := im.GetInfluenceAt(position)

	teamInfluence := cell.TeamInfluences[team]
	enemyInfluence := 0.0

	for otherTeam, influence := range cell.TeamInfluences {
		if otherTeam != team && otherTeam != TeamNeutral {
			enemyInfluence += influence
		}
	}

	if enemyInfluence == 0 {
		return 1.0 // Complete advantage
	}

	return (teamInfluence - enemyInfluence) / (teamInfluence + enemyInfluence)
}

// FindBestPositionNear finds the best tactical position near a target
func (im *InfluenceMap) FindBestPositionNear(target core.Vector2D, team TeamID, searchRadius float64) core.Vector2D {
	im.mu.RLock()
	defer im.mu.RUnlock()

	bestPosition := target
	bestScore := float64(-math.MaxFloat64)

	centerX, centerY := im.worldToGrid(target)
	rangeCells := int(math.Ceil(searchRadius / im.cellSize))

	for dy := -rangeCells; dy <= rangeCells; dy++ {
		for dx := -rangeCells; dx <= rangeCells; dx++ {
			x := centerX + dx
			y := centerY + dy

			if x < 0 || x >= im.width || y < 0 || y >= im.height {
				continue
			}

			worldX, worldY := im.gridToWorld(x, y)
			position := core.Vector2D{X: worldX, Y: worldY}

			distance := math.Sqrt(math.Pow(worldX-target.X, 2) + math.Pow(worldY-target.Y, 2))
			if distance <= searchRadius {
				score := im.evaluatePosition(position, team)
				if score > bestScore {
					bestScore = score
					bestPosition = position
				}
			}
		}
	}

	return bestPosition
}

// Update applies decay to all influence values
func (im *InfluenceMap) Update(deltaTime float64) {
	im.mu.Lock()
	defer im.mu.Unlock()

	im.updateTime++

	// Decay all influences
	for y := 0; y < im.height; y++ {
		for x := 0; x < im.width; x++ {
			cell := &im.cells[y][x]

			// Decay team influences
			for team, influence := range cell.TeamInfluences {
				cell.TeamInfluences[team] = influence * im.decayRate
				if cell.TeamInfluences[team] < 0.01 {
					delete(cell.TeamInfluences, team)
				}
			}

			// Decay layer influences
			for layer := range im.layers {
				im.layers[layer][y][x] *= im.decayRate
				if im.layers[layer][y][x] < 0.01 {
					im.layers[layer][y][x] = 0
				}
			}

			// Recalculate total influence
			cell.TotalInfluence = 0
			for _, influence := range cell.TeamInfluences {
				cell.TotalInfluence += influence
			}

			// Update derived values
			im.updateCellDerivedValues(cell, x, y)
		}
	}
}

// Helper methods

func (im *InfluenceMap) worldToGrid(position core.Vector2D) (int, int) {
	x := int(math.Floor((position.X - im.bounds.Min.X) / im.cellSize))
	y := int(math.Floor((position.Y - im.bounds.Min.Y) / im.cellSize))
	return x, y
}

func (im *InfluenceMap) gridToWorld(x, y int) (float64, float64) {
	worldX := im.bounds.Min.X + float64(x)*im.cellSize + im.cellSize/2
	worldY := im.bounds.Min.Y + float64(y)*im.cellSize + im.cellSize/2
	return worldX, worldY
}

func (im *InfluenceMap) calculateInfluence(baseInfluence, distance, maxRange float64, decayType DecayType) float64 {
	if distance >= maxRange {
		return 0
	}

	ratio := distance / maxRange

	switch decayType {
	case DecayLinear:
		return baseInfluence * (1.0 - ratio)
	case DecayQuadratic:
		return baseInfluence * (1.0 - ratio*ratio)
	case DecayExponential:
		return baseInfluence * math.Exp(-ratio*3.0) // e^(-3*ratio)
	case DecayConstant:
		return baseInfluence
	default:
		return baseInfluence * (1.0 - ratio)
	}
}

func (im *InfluenceMap) updateCellDerivedValues(cell *InfluenceCell, x, y int) {
	// Calculate danger level (enemy influence)
	cell.DangerLevel = 0
	cell.SafetyLevel = 0
	cell.ControlLevel = 0

	// Sum enemy influences for danger
	for team, influence := range cell.TeamInfluences {
		if team != TeamNeutral {
			if influence > cell.DangerLevel {
				cell.DangerLevel = influence
			}
		}
	}

	// Safety is inverse of danger plus friendly influence
	maxFriendlyInfluence := 0.0
	for _, influence := range cell.TeamInfluences {
		if influence > maxFriendlyInfluence {
			maxFriendlyInfluence = influence
		}
	}

	cell.SafetyLevel = maxFriendlyInfluence / (1.0 + cell.DangerLevel)
	cell.ControlLevel = cell.TotalInfluence
}

func (im *InfluenceMap) getDangerAt(x, y int, team TeamID) float64 {
	cell := &im.cells[y][x]
	danger := 0.0

	for otherTeam, influence := range cell.TeamInfluences {
		if otherTeam != team && otherTeam != TeamNeutral {
			danger += influence
		}
	}

	return danger
}

func (im *InfluenceMap) heuristic(x1, y1, x2, y2 int) float64 {
	dx := float64(x2 - x1)
	dy := float64(y2 - y1)
	return math.Sqrt(dx*dx + dy*dy)
}

func (im *InfluenceMap) nodeKey(x, y int) string {
	return fmt.Sprintf("%d,%d", x, y)
}

func (im *InfluenceMap) evaluatePosition(position core.Vector2D, team TeamID) float64 {
	cell := im.GetInfluenceAt(position)

	// Score based on multiple factors
	teamAdvantage := im.GetTeamAdvantage(position, team)
	safetyScore := cell.SafetyLevel
	controlScore := cell.ControlLevel * 0.1 // Weight control lower than safety

	return teamAdvantage*0.5 + safetyScore*0.4 + controlScore*0.1
}
