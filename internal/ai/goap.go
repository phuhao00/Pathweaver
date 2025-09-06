package ai

import (
	"fmt"
	"sort"
	"pathweaver/internal/core"
)

// GOAP (Goal-Oriented Action Planning) implementation
// Based on the algorithm used in games like F.E.A.R. and research papers on AI planning
// Provides dynamic AI behavior through goal-driven action selection

// WorldState represents the current state of the world using key-value pairs
type WorldState map[string]interface{}

// Clone creates a deep copy of the world state
func (ws WorldState) Clone() WorldState {
	clone := make(WorldState)
	for key, value := range ws {
		clone[key] = value
	}
	return clone
}

// Matches checks if this state matches another state for the given keys
func (ws WorldState) Matches(other WorldState, keys []string) bool {
	for _, key := range keys {
		if ws[key] != other[key] {
			return false
		}
	}
	return true
}

// Action represents a single action that can be performed
type Action struct {
	Name         string
	Cost         float64
	Preconditions WorldState
	Effects       WorldState
	Performer    ActionPerformer
}

// ActionPerformer interface for entities that can perform actions
type ActionPerformer interface {
	CanPerform(action *Action, worldState WorldState) bool
	Perform(action *Action, worldState WorldState) error
	GetPosition() core.Vector2D
}

// Goal represents a desired world state
type Goal struct {
	Name       string
	Priority   float64
	Conditions WorldState
	IsActive   bool
}

// Plan represents a sequence of actions to achieve a goal
type Plan struct {
	Actions    []*Action
	TotalCost  float64
	Goal       *Goal
	StartState WorldState
	EndState   WorldState
}

// PlanNode represents a node in the planning search tree
type PlanNode struct {
	Action     *Action
	State      WorldState
	Cost       float64
	Parent     *PlanNode
	Depth      int
}

// GOAPAgent represents an AI agent using GOAP
type GOAPAgent struct {
	Actions         []*Action
	Goals           []*Goal
	CurrentPlan     *Plan
	CurrentAction   int
	WorldState      WorldState
	Memory          map[string]interface{} // Agent's knowledge/memory
	MaxPlanningDepth int
	MaxPlanningCost  float64
}

// NewGOAPAgent creates a new GOAP agent
func NewGOAPAgent() *GOAPAgent {
	return &GOAPAgent{
		Actions:          make([]*Action, 0),
		Goals:            make([]*Goal, 0),
		WorldState:       make(WorldState),
		Memory:           make(map[string]interface{}),
		MaxPlanningDepth: 10,
		MaxPlanningCost:  1000.0,
	}
}

// AddAction adds an action to the agent's repertoire
func (agent *GOAPAgent) AddAction(action *Action) {
	agent.Actions = append(agent.Actions, action)
}

// AddGoal adds a goal to the agent
func (agent *GOAPAgent) AddGoal(goal *Goal) {
	agent.Goals = append(agent.Goals, goal)
}

// UpdateWorldState updates the agent's perception of the world
func (agent *GOAPAgent) UpdateWorldState(newState WorldState) {
	for key, value := range newState {
		agent.WorldState[key] = value
	}
}

// SelectGoal selects the highest priority active goal
func (agent *GOAPAgent) SelectGoal() *Goal {
	var bestGoal *Goal
	highestPriority := float64(-1)
	
	for _, goal := range agent.Goals {
		if goal.IsActive && goal.Priority > highestPriority {
			// Check if goal is already satisfied
			if !agent.WorldState.Matches(goal.Conditions, agent.getKeys(goal.Conditions)) {
				bestGoal = goal
				highestPriority = goal.Priority
			}
		}
	}
	
	return bestGoal
}

// CreatePlan creates a plan to achieve the given goal
func (agent *GOAPAgent) CreatePlan(goal *Goal) *Plan {
	if goal == nil {
		return nil
	}
	
	// Use A* search to find optimal action sequence
	startNode := &PlanNode{
		Action: nil,
		State:  agent.WorldState.Clone(),
		Cost:   0,
		Parent: nil,
		Depth:  0,
	}
	
	openSet := []*PlanNode{startNode}
	closedSet := make(map[string]*PlanNode)
	
	for len(openSet) > 0 {
		// Find node with lowest cost
		current := openSet[0]
		currentIndex := 0
		for i, node := range openSet {
			if node.Cost < current.Cost {
				current = node
				currentIndex = i
			}
		}
		
		// Remove from open set
		openSet = append(openSet[:currentIndex], openSet[currentIndex+1:]...)
		
		// Add to closed set
		stateKey := agent.stateToKey(current.State)
		closedSet[stateKey] = current
		
		// Check if we've reached the goal
		goalKeys := agent.getKeys(goal.Conditions)
		if current.State.Matches(goal.Conditions, goalKeys) {
			return agent.reconstructPlan(current, goal)
		}
		
		// Don't go too deep or too expensive
		if current.Depth >= agent.MaxPlanningDepth || current.Cost >= agent.MaxPlanningCost {
			continue
		}
		
		// Explore all possible actions
		for _, action := range agent.Actions {
			// Check if action is applicable
			if !agent.canApplyAction(action, current.State) {
				continue
			}
			
			// Apply action to get new state
			newState := agent.applyAction(action, current.State)
			newStateKey := agent.stateToKey(newState)
			
			// Skip if we've already explored this state
			if _, exists := closedSet[newStateKey]; exists {
				continue
			}
			
			// Create new node
			newNode := &PlanNode{
				Action: action,
				State:  newState,
				Cost:   current.Cost + action.Cost,
				Parent: current,
				Depth:  current.Depth + 1,
			}
			
			// Add heuristic cost (estimate of remaining cost to goal)
			newNode.Cost += agent.calculateHeuristic(newState, goal.Conditions)
			
			// Check if we already have this node in open set
			found := false
			for i, existing := range openSet {
				if agent.stateToKey(existing.State) == newStateKey {
					// If new path is better, replace
					if newNode.Cost < existing.Cost {
						openSet[i] = newNode
					}
					found = true
					break
				}
			}
			
			if !found {
				openSet = append(openSet, newNode)
			}
		}
	}
	
	// No plan found
	return nil
}

// ExecutePlan executes the current plan
func (agent *GOAPAgent) ExecutePlan(performer ActionPerformer) error {
	if agent.CurrentPlan == nil || agent.CurrentAction >= len(agent.CurrentPlan.Actions) {
		return fmt.Errorf("no valid plan to execute")
	}
	
	action := agent.CurrentPlan.Actions[agent.CurrentAction]
	
	// Check if we can still perform this action
	if !performer.CanPerform(action, agent.WorldState) {
		// Plan is invalid, need to replan
		agent.CurrentPlan = nil
		agent.CurrentAction = 0
		return fmt.Errorf("action no longer valid: %s", action.Name)
	}
	
	// Perform the action
	err := performer.Perform(action, agent.WorldState)
	if err != nil {
		return fmt.Errorf("failed to perform action %s: %w", action.Name, err)
	}
	
	// Apply action effects to world state
	for key, value := range action.Effects {
		agent.WorldState[key] = value
	}
	
	// Move to next action
	agent.CurrentAction++
	
	// Check if plan is complete
	if agent.CurrentAction >= len(agent.CurrentPlan.Actions) {
		agent.CurrentPlan = nil
		agent.CurrentAction = 0
	}
	
	return nil
}

// Update performs one update cycle of the GOAP agent
func (agent *GOAPAgent) Update(performer ActionPerformer) error {
	// If we have a current plan, continue executing it
	if agent.CurrentPlan != nil {
		return agent.ExecutePlan(performer)
	}
	
	// Select a goal
	goal := agent.SelectGoal()
	if goal == nil {
		return nil // No active goals
	}
	
	// Create a plan for the goal
	plan := agent.CreatePlan(goal)
	if plan == nil {
		return fmt.Errorf("no plan found for goal: %s", goal.Name)
	}
	
	// Set as current plan and start executing
	agent.CurrentPlan = plan
	agent.CurrentAction = 0
	
	return agent.ExecutePlan(performer)
}

// Helper methods

func (agent *GOAPAgent) canApplyAction(action *Action, state WorldState) bool {
	// Check preconditions
	for key, requiredValue := range action.Preconditions {
		if state[key] != requiredValue {
			return false
		}
	}
	return true
}

func (agent *GOAPAgent) applyAction(action *Action, state WorldState) WorldState {
	newState := state.Clone()
	
	// Apply effects
	for key, value := range action.Effects {
		newState[key] = value
	}
	
	return newState
}

func (agent *GOAPAgent) calculateHeuristic(state WorldState, goal WorldState) float64 {
	// Simple heuristic: count number of unsatisfied goal conditions
	unsatisfied := 0.0
	for key, goalValue := range goal {
		if state[key] != goalValue {
			unsatisfied += 1.0
		}
	}
	return unsatisfied
}

func (agent *GOAPAgent) reconstructPlan(goalNode *PlanNode, goal *Goal) *Plan {
	actions := []*Action{}
	totalCost := goalNode.Cost
	
	current := goalNode
	for current.Parent != nil {
		actions = append([]*Action{current.Action}, actions...)
		current = current.Parent
	}
	
	return &Plan{
		Actions:    actions,
		TotalCost:  totalCost,
		Goal:       goal,
		StartState: current.State,
		EndState:   goalNode.State,
	}
}

func (agent *GOAPAgent) stateToKey(state WorldState) string {
	// Create a consistent string representation of the state
	keys := make([]string, 0, len(state))
	for key := range state {
		keys = append(keys, key)
	}
	sort.Strings(keys)
	
	result := ""
	for _, key := range keys {
		result += fmt.Sprintf("%s:%v;", key, state[key])
	}
	return result
}

func (agent *GOAPAgent) getKeys(state WorldState) []string {
	keys := make([]string, 0, len(state))
	for key := range state {
		keys = append(keys, key)
	}
	return keys
}

// Specialized GOAP implementations

// CombatAgent represents a combat-focused GOAP agent
type CombatAgent struct {
	*GOAPAgent
	Health       float64
	Ammo         int
	Position     core.Vector2D
	Weapons      []string
	Enemies      []core.Vector2D
	CoverPoints  []core.Vector2D
}

// NewCombatAgent creates a combat-specialized GOAP agent
func NewCombatAgent() *CombatAgent {
	agent := &CombatAgent{
		GOAPAgent: NewGOAPAgent(),
		Health:    100.0,
		Ammo:      30,
		Weapons:   []string{"rifle"},
	}
	
	// Add combat-specific actions
	agent.setupCombatActions()
	agent.setupCombatGoals()
	
	return agent
}

func (ca *CombatAgent) setupCombatActions() {
	// Attack action
	attack := &Action{
		Name: "Attack",
		Cost: 1.0,
		Preconditions: WorldState{
			"hasAmmo":     true,
			"hasTarget":   true,
			"inRange":     true,
		},
		Effects: WorldState{
			"targetDamaged": true,
			"ammoUsed":      true,
		},
	}
	
	// Reload action
	reload := &Action{
		Name: "Reload",
		Cost: 2.0,
		Preconditions: WorldState{
			"hasAmmo":    false,
			"hasSpareAmmo": true,
		},
		Effects: WorldState{
			"hasAmmo": true,
		},
	}
	
	// Take cover action
	takeCover := &Action{
		Name: "TakeCover",
		Cost: 3.0,
		Preconditions: WorldState{
			"inCover":    false,
			"coverNear":  true,
		},
		Effects: WorldState{
			"inCover": true,
		},
	}
	
	// Move to target action
	moveToTarget := &Action{
		Name: "MoveToTarget",
		Cost: 5.0,
		Preconditions: WorldState{
			"hasTarget": true,
			"inRange":   false,
		},
		Effects: WorldState{
			"inRange": true,
		},
	}
	
	ca.AddAction(attack)
	ca.AddAction(reload)
	ca.AddAction(takeCover)
	ca.AddAction(moveToTarget)
}

func (ca *CombatAgent) setupCombatGoals() {
	// Eliminate enemy goal
	eliminateEnemy := &Goal{
		Name:     "EliminateEnemy",
		Priority: 10.0,
		Conditions: WorldState{
			"targetDamaged": true,
		},
		IsActive: true,
	}
	
	// Survive goal
	survive := &Goal{
		Name:     "Survive",
		Priority: 15.0,
		Conditions: WorldState{
			"inCover": true,
		},
		IsActive: true,
	}
	
	ca.AddGoal(eliminateEnemy)
	ca.AddGoal(survive)
}

// GetPosition implements ActionPerformer interface
func (ca *CombatAgent) GetPosition() core.Vector2D {
	return ca.Position
}

// CanPerform implements ActionPerformer interface
func (ca *CombatAgent) CanPerform(action *Action, worldState WorldState) bool {
	switch action.Name {
	case "Attack":
		return ca.Ammo > 0
	case "Reload":
		return ca.Ammo == 0
	case "TakeCover":
		return len(ca.CoverPoints) > 0
	case "MoveToTarget":
		return len(ca.Enemies) > 0
	default:
		return true
	}
}

// Perform implements ActionPerformer interface
func (ca *CombatAgent) Perform(action *Action, worldState WorldState) error {
	switch action.Name {
	case "Attack":
		if ca.Ammo <= 0 {
			return fmt.Errorf("no ammo to attack")
		}
		ca.Ammo--
		return nil
		
	case "Reload":
		ca.Ammo = 30 // Full reload
		return nil
		
	case "TakeCover":
		if len(ca.CoverPoints) == 0 {
			return fmt.Errorf("no cover available")
		}
		// Move to nearest cover point
		ca.Position = ca.CoverPoints[0]
		return nil
		
	case "MoveToTarget":
		if len(ca.Enemies) == 0 {
			return fmt.Errorf("no target to move to")
		}
		// Move closer to enemy (simplified)
		target := ca.Enemies[0]
		ca.Position = core.Vector2D{
			X: ca.Position.X + (target.X-ca.Position.X)*0.1,
			Y: ca.Position.Y + (target.Y-ca.Position.Y)*0.1,
		}
		return nil
		
	default:
		return fmt.Errorf("unknown action: %s", action.Name)
	}
}
