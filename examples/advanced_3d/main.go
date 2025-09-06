package main

import (
	"fmt"
	"log"
	"pathweaver/internal/ai"
	"pathweaver/internal/core"
	"pathweaver/internal/moba"
	"pathweaver/internal/pathfinding"
	"pathweaver/pkg/pathweaver"
)

func main() {
	fmt.Println("PathWeaver Advanced 3D Game Scene Management Demo")
	fmt.Println("================================================")
	
	// Demo different game types
	demonstrateMMO()
	demonstrateMOBA()
	demonstrateRTS()
	demonstrateFPS()
	demonstrateGOAP()
}

func demonstrateMMO() {
	fmt.Println("\n🌍 MMO (Massively Multiplayer Online) Demo")
	fmt.Println("------------------------------------------")
	
	// Create MMO-optimized 3D engine
	config := &pathweaver.Config3D{
		SceneBounds: core.AABB3D{
			Min: core.Vector3D{X: -500, Y: -500, Z: -50},
			Max: core.Vector3D{X: 500, Y: 500, Z: 50},
		},
		GameType:    core.GameTypeMMO,
		MMOZoneSize: 100.0,
		PathfindingGrid: 2.0, // Larger grid for MMO scale
		MovementType: pathfinding.MovementGround,
	}
	
	engine := pathweaver.NewEngine3D(config)
	
	// Add players across different zones
	players := []struct {
		id       uint64
		position core.Vector3D
		name     string
	}{
		{1, core.Vector3D{X: 50, Y: 50, Z: 0}, "Player_Alpha"},
		{2, core.Vector3D{X: 150, Y: 50, Z: 0}, "Player_Beta"},
		{3, core.Vector3D{X: -200, Y: 100, Z: 0}, "Player_Gamma"},
		{4, core.Vector3D{X: 300, Y: -100, Z: 0}, "Player_Delta"},
	}
	
	for _, p := range players {
		player := pathweaver.CreatePlayer3D(p.id, p.position, 1.0)
		if err := engine.AddEntity3D(player); err != nil {
			log.Printf("Failed to add %s: %v", p.name, err)
		} else {
			fmt.Printf("Added %s at zone with %d total entities\n", 
				p.name, len(engine.GetEntitiesInSphere3D(p.position, 50)))
		}
	}
	
	// Demonstrate interest management
	player1Pos := core.Vector3D{X: 50, Y: 50, Z: 0}
	interestArea := engine.GetPlayerInterestArea(1, player1Pos)
	if interestArea != nil {
		fmt.Printf("Player 1 is interested in %d entities across %d zones\n", 
			len(interestArea.Entities), len(interestArea.Zones))
	}
	
	// Show zone population
	pop, activePlayers := engine.GetZonePopulation(player1Pos)
	fmt.Printf("Zone population: %d entities (%d active players)\n", pop, activePlayers)
	
	// Demonstrate 3D pathfinding with ground movement
	start := core.Vector3D{X: 0, Y: 0, Z: 0}
	goal := core.Vector3D{X: 100, Y: 100, Z: 0}
	
	// Add some 3D obstacles
	obstacles := []core.AABB3D{
		{
			Min: core.Vector3D{X: 40, Y: 40, Z: -5},
			Max: core.Vector3D{X: 60, Y: 60, Z: 15},
		},
	}
	
	path, err := engine.FindPath3D(start, goal, obstacles)
	if err != nil {
		fmt.Printf("Failed to find 3D path: %v\n", err)
	} else {
		fmt.Printf("Found 3D path with %d waypoints\n", len(path))
		
		// Show first few waypoints
		for i, waypoint := range path[:min(3, len(path))] {
			fmt.Printf("  Waypoint %d: (%.1f, %.1f, %.1f)\n", 
				i, waypoint.X, waypoint.Y, waypoint.Z)
		}
	}
}

func demonstrateMOBA() {
	fmt.Println("\n⚔️ MOBA (Multiplayer Online Battle Arena) Demo")
	fmt.Println("----------------------------------------------")
	
	// Create MOBA-optimized engine
	config := &pathweaver.Config3D{
		SceneBounds: core.AABB3D{
			Min: core.Vector3D{X: -100, Y: -100, Z: 0},
			Max: core.Vector3D{X: 100, Y: 100, Z: 10},
		},
		GameType: core.GameTypeMOBA,
		InfluenceMapCellSize: 1.0,
		PathfindingGrid: 0.5, // Finer grid for tactical movement
		MovementType: pathfinding.MovementGround,
	}
	
	engine := pathweaver.NewEngine3D(config)
	
	// Add team structures
	blueBase := pathweaver.CreateBuilding3D(100, 
		core.Vector3D{X: -80, Y: -80, Z: 5}, 10, 10, 10)
	redBase := pathweaver.CreateBuilding3D(101, 
		core.Vector3D{X: 80, Y: 80, Z: 5}, 10, 10, 10)
	
	engine.AddEntity3D(blueBase)
	engine.AddEntity3D(redBase)
	
	// Add influence sources for tactical AI
	blueBaseInfluence := moba.InfluenceSource{
		Position:  core.Vector2D{X: -80, Y: -80},
		Team:      moba.TeamBlue,
		Influence: 100.0,
		Range:     30.0,
		Layer:     moba.LayerControl,
		DecayType: moba.DecayLinear,
		IsActive:  true,
	}
	
	redTowerInfluence := moba.InfluenceSource{
		Position:  core.Vector2D{X: 50, Y: 50},
		Team:      moba.TeamRed,
		Influence: 80.0,
		Range:     25.0,
		Layer:     moba.LayerDamage,
		DecayType: moba.DecayQuadratic,
		IsActive:  true,
	}
	
	engine.AddInfluenceSource(blueBaseInfluence)
	engine.AddInfluenceSource(redTowerInfluence)
	
	// Demonstrate tactical pathfinding
	unitPos := core.Vector2D{X: -50, Y: -50}
	targetPos := core.Vector2D{X: 60, Y: 60}
	
	// Find safe path considering team influence
	safePath := engine.FindSafePath(unitPos, targetPos, moba.TeamBlue)
	fmt.Printf("Safe path for Blue team has %d waypoints\n", len(safePath))
	
	// Show team advantage at different positions
	positions := []core.Vector2D{
		{X: -60, Y: -60}, // Near blue base
		{X: 0, Y: 0},     // Center
		{X: 40, Y: 40},   // Near red tower
	}
	
	for i, pos := range positions {
		advantage := engine.GetTeamAdvantage(pos, moba.TeamBlue)
		fmt.Printf("Position %d: Blue team advantage = %.2f\n", i+1, advantage)
	}
}

func demonstrateRTS() {
	fmt.Println("\n🏗️ RTS (Real-Time Strategy) Demo")
	fmt.Println("--------------------------------")
	
	// Create RTS-optimized engine with flow fields
	config := &pathweaver.Config3D{
		SceneBounds: core.AABB3D{
			Min: core.Vector3D{X: -200, Y: -200, Z: 0},
			Max: core.Vector3D{X: 200, Y: 200, Z: 20},
		},
		GameType: core.GameTypeRTS,
		FlowFieldCellSize: 2.0,
		PathfindingGrid: 1.0,
		MovementType: pathfinding.MovementGround,
	}
	
	engine := pathweaver.NewEngine3D(config)
	
	// Add buildings and resources
	buildings := []struct {
		id       uint64
		pos      core.Vector3D
		size     [3]float64
		name     string
	}{
		{200, core.Vector3D{X: -150, Y: -150, Z: 5}, [3]float64{15, 15, 10}, "Command Center"},
		{201, core.Vector3D{X: 100, Y: 120, Z: 3}, [3]float64{8, 8, 6}, "Resource Depot"},
		{202, core.Vector3D{X: 0, Y: 50, Z: 4}, [3]float64{6, 6, 8}, "Factory"},
	}
	
	for _, b := range buildings {
		building := pathweaver.CreateBuilding3D(b.id, b.pos, b.size[0], b.size[1], b.size[2])
		engine.AddEntity3D(building)
		fmt.Printf("Built %s at (%.0f, %.0f)\n", b.name, b.pos.X, b.pos.Y)
	}
	
	// Generate flow field for multiple rally points
	rallyPoints := []core.Vector2D{
		{X: 100, Y: 120}, // Near resource depot
		{X: 0, Y: 50},    // Near factory
	}
	
	engine.GenerateFlowField(rallyPoints)
	fmt.Printf("Generated flow field for %d rally points\n", len(rallyPoints))
	
	// Demonstrate unit movement following flow field
	unitPositions := []core.Vector2D{
		{X: -120, Y: -120},
		{X: -100, Y: -130},
		{X: -110, Y: -140},
	}
	
	fmt.Println("Unit movement following flow field:")
	for i, pos := range unitPositions {
		flowDir := engine.GetFlowDirection(pos)
		nextPos := core.Vector2D{
			X: pos.X + flowDir.X*5.0,
			Y: pos.Y + flowDir.Y*5.0,
		}
		fmt.Printf("  Unit %d: (%.1f,%.1f) → (%.1f,%.1f)\n", 
			i+1, pos.X, pos.Y, nextPos.X, nextPos.Y)
	}
}

func demonstrateFPS() {
	fmt.Println("\n🔫 FPS (First Person Shooter) Demo")
	fmt.Println("----------------------------------")
	
	// Create FPS-optimized engine
	config := &pathweaver.Config3D{
		SceneBounds: core.AABB3D{
			Min: core.Vector3D{X: -50, Y: -50, Z: -10},
			Max: core.Vector3D{X: 50, Y: 50, Z: 20},
		},
		GameType: core.GameTypeFPS,
		PathfindingGrid: 0.5, // Fine grid for precise movement
		MovementType: pathfinding.MovementFlight, // Allow 3D movement
		UseBSP: true,
	}
	
	engine := pathweaver.NewEngine3D(config)
	
	// Add some entities for hit detection
	targets := []struct {
		id   uint64
		pos  core.Vector3D
		name string
	}{
		{300, core.Vector3D{X: 20, Y: 0, Z: 5}, "Target_A"},
		{301, core.Vector3D{X: 0, Y: 25, Z: 8}, "Target_B"},
		{302, core.Vector3D{X: -15, Y: -20, Z: 3}, "Target_C"},
	}
	
	for _, target := range targets {
		entity := pathweaver.CreatePlayer3D(target.id, target.pos, 1.0)
		engine.AddEntity3D(entity)
		fmt.Printf("Placed %s at (%.1f, %.1f, %.1f)\n", 
			target.name, target.pos.X, target.pos.Y, target.pos.Z)
	}
	
	// Demonstrate 3D raycast for hit detection
	shooterPos := core.Vector3D{X: 0, Y: 0, Z: 5}
	shootDirections := []struct {
		dir  core.Vector3D
		name string
	}{
		{core.Vector3D{X: 1, Y: 0, Z: 0}, "East"},
		{core.Vector3D{X: 0, Y: 1, Z: 0.2}, "North-Up"},
		{core.Vector3D{X: -0.6, Y: -0.8, Z: -0.1}, "Southwest-Down"},
	}
	
	fmt.Println("Raycast hit detection:")
	for _, shot := range shootDirections {
		hit := engine.Raycast3D(shooterPos, shot.dir, 50.0)
		if hit != nil {
			// Calculate distance from shooter to hit point
			dx := hit.ContactPoint.X - shooterPos.X
			dy := hit.ContactPoint.Y - shooterPos.Y
			dz := hit.ContactPoint.Z - shooterPos.Z
			distance := dx*dx + dy*dy + dz*dz // Using squared distance for efficiency
			
			fmt.Printf("  %s shot: HIT entity %d at distance %.2f\n", 
				shot.name, hit.Entity.ID, distance)
		} else {
			fmt.Printf("  %s shot: MISS\n", shot.name)
		}
	}
	
	// Demonstrate view frustum culling
	viewPos := core.Vector3D{X: 0, Y: 0, Z: 5}
	viewForward := core.Vector3D{X: 1, Y: 0, Z: 0}
	viewUp := core.Vector3D{X: 0, Y: 0, Z: 1}
	
	frustum := pathweaver.CreateViewFrustum(viewPos, viewForward, viewUp, 90, 1.6, 1, 100)
	visibleEntities := engine.GetEntitiesInFrustum(frustum)
	
	fmt.Printf("View frustum culling: %d entities visible\n", len(visibleEntities))
	for _, entity := range visibleEntities {
		fmt.Printf("  Visible: Entity %d\n", entity.ID)
	}
}

func demonstrateGOAP() {
	fmt.Println("\n🧠 GOAP (Goal-Oriented Action Planning) AI Demo")
	fmt.Println("-----------------------------------------------")
	
	// Create a combat agent using GOAP
	combatAgent := ai.NewCombatAgent()
	
	// Set up the agent's environment
	combatAgent.Position = core.Vector2D{X: 0, Y: 0}
	combatAgent.Health = 100.0
	combatAgent.Ammo = 10
	combatAgent.Enemies = []core.Vector2D{{X: 20, Y: 5}}
	combatAgent.CoverPoints = []core.Vector2D{{X: -5, Y: 10}}
	
	// Set initial world state
	combatAgent.UpdateWorldState(ai.WorldState{
		"hasAmmo":    true,
		"hasTarget":  true,
		"inRange":    false,
		"inCover":    false,
		"coverNear":  true,
		"hasSpareAmmo": true,
	})
	
	fmt.Println("GOAP Agent initial state:")
	fmt.Printf("  Position: (%.1f, %.1f)\n", combatAgent.Position.X, combatAgent.Position.Y)
	fmt.Printf("  Health: %.0f\n", combatAgent.Health)
	fmt.Printf("  Ammo: %d\n", combatAgent.Ammo)
	fmt.Printf("  Enemies: %d\n", len(combatAgent.Enemies))
	
	// Simulate AI updates
	fmt.Println("\nAI Decision Making:")
	for i := 0; i < 5; i++ {
		err := combatAgent.Update(combatAgent)
		if err != nil {
			fmt.Printf("  Step %d: %v\n", i+1, err)
		} else {
			if combatAgent.CurrentPlan != nil {
				actionName := "Planning"
				if combatAgent.CurrentAction < len(combatAgent.CurrentPlan.Actions) {
					actionName = combatAgent.CurrentPlan.Actions[combatAgent.CurrentAction].Name
				}
				fmt.Printf("  Step %d: Executing action '%s'\n", i+1, actionName)
			} else {
				fmt.Printf("  Step %d: No active plan\n", i+1)
			}
		}
		
		// Show agent state changes
		fmt.Printf("    Ammo: %d, Position: (%.1f, %.1f)\n", 
			combatAgent.Ammo, combatAgent.Position.X, combatAgent.Position.Y)
	}
	
	// Show final state
	fmt.Println("\nFinal GOAP Agent state:")
	fmt.Printf("  Position: (%.1f, %.1f)\n", combatAgent.Position.X, combatAgent.Position.Y)
	fmt.Printf("  Health: %.0f\n", combatAgent.Health)
	fmt.Printf("  Ammo: %d\n", combatAgent.Ammo)
}

func min(a, b int) int {
	if a < b {
		return a
	}
	return b
}
