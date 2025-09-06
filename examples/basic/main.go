package main

import (
	"fmt"
	"log"
	"math"
	"pathweaver/internal/core"
	"pathweaver/pkg/pathweaver"
)

func main() {
	fmt.Println("PathWeaver Game Scene Management Demo")
	fmt.Println("====================================")

	// Create engine configuration
	config := &pathweaver.Config{
		SceneBounds:         pathweaver.NewAABB(-100, -100, 100, 100),
		PathfindingGrid:     1.0,
		PathfindingType:     pathweaver.PathfindingAStar,
		MaxPathfindingNodes: 10000,
	}

	// Create the engine
	engine := pathweaver.NewEngine(config)

	// Demo 1: Entity Management
	fmt.Println("\n1. Entity Management Demo")
	fmt.Println("------------------------")

	// Create some entities
	player := pathweaver.NewPlayerEntity(1, pathweaver.NewVector2D(0, 0), 2.0)
	npc1 := pathweaver.NewNPCEntity(2, pathweaver.NewVector2D(10, 10), 2.0)
	npc2 := pathweaver.NewNPCEntity(3, pathweaver.NewVector2D(-10, 10), 2.0)

	// Add entities to scene
	if err := engine.AddEntity(player); err != nil {
		log.Fatal("Failed to add player:", err)
	}
	if err := engine.AddEntity(npc1); err != nil {
		log.Fatal("Failed to add NPC1:", err)
	}
	if err := engine.AddEntity(npc2); err != nil {
		log.Fatal("Failed to add NPC2:", err)
	}

	fmt.Printf("Added %d entities to scene\n", engine.GetEntityCount())
	fmt.Printf("Players: %d, NPCs: %d\n",
		engine.GetEntityCountByType(core.EntityTypePlayer),
		engine.GetEntityCountByType(core.EntityTypeNPC))

	// Demo 2: Obstacle Creation and Spatial Queries
	fmt.Println("\n2. Obstacle Creation and Spatial Queries")
	fmt.Println("---------------------------------------")

	// Create some obstacles
	obstacles := []*core.Entity{
		pathweaver.NewObstacleEntity(10, pathweaver.NewVector2D(5, 0), 3, 3),
		pathweaver.NewObstacleEntity(11, pathweaver.NewVector2D(-5, 5), 2, 4),
		pathweaver.NewObstacleEntity(12, pathweaver.NewVector2D(0, 15), 6, 2),
		pathweaver.NewObstacleEntity(13, pathweaver.NewVector2D(15, -5), 3, 8),
	}

	// Add obstacles
	for _, obstacle := range obstacles {
		if err := engine.AddEntity(obstacle); err != nil {
			log.Printf("Failed to add obstacle %d: %v", obstacle.ID, err)
		}
	}

	fmt.Printf("Added %d obstacles\n", engine.GetEntityCountByType(core.EntityTypeObstacle))

	// Spatial query - find entities near player
	nearbyEntities := engine.GetEntitiesInRadius(player.Position, 15.0)
	fmt.Printf("Entities within 15 units of player: %d\n", len(nearbyEntities))
	for _, entity := range nearbyEntities {
		if entity.ID != player.ID {
			distance := pathweaver.Distance(player.Position, entity.Position)
			fmt.Printf("  Entity %d (Type: %d) at distance %.2f\n",
				entity.ID, entity.Type, distance)
		}
	}

	// Demo 3: Pathfinding
	fmt.Println("\n3. Pathfinding Demo")
	fmt.Println("------------------")

	startPos := pathweaver.NewVector2D(-20, -20)
	goalPos := pathweaver.NewVector2D(20, 20)

	fmt.Printf("Finding path from (%.1f, %.1f) to (%.1f, %.1f)\n",
		startPos.X, startPos.Y, goalPos.X, goalPos.Y)

	path, err := engine.FindPath(startPos, goalPos)
	if err != nil {
		fmt.Printf("Pathfinding failed: %v\n", err)
	} else {
		fmt.Printf("Path found with %d waypoints:\n", len(path))
		for i, waypoint := range path {
			fmt.Printf("  %d: (%.2f, %.2f)\n", i, waypoint.X, waypoint.Y)
			if i >= 5 { // Limit output for readability
				fmt.Printf("  ... and %d more waypoints\n", len(path)-i-1)
				break
			}
		}

		// Calculate path length
		totalDistance := 0.0
		for i := 1; i < len(path); i++ {
			totalDistance += pathweaver.Distance(path[i-1], path[i])
		}
		fmt.Printf("Total path length: %.2f units\n", totalDistance)

		// Optimize path
		optimizedPath := pathweaver.OptimizePath(path, 0.5)
		fmt.Printf("Optimized path has %d waypoints (reduced by %d)\n",
			len(optimizedPath), len(path)-len(optimizedPath))
	}

	// Demo 4: Collision Detection
	fmt.Println("\n4. Collision Detection Demo")
	fmt.Println("--------------------------")

	// Try to move player to a position that might collide
	newPos := pathweaver.NewVector2D(5, 0) // Near an obstacle

	canMove, collisions := engine.CheckMovement(player, newPos)
	fmt.Printf("Can move player to (%.1f, %.1f): %t\n", newPos.X, newPos.Y, canMove)
	if !canMove {
		fmt.Printf("Movement blocked by %d collision(s):\n", len(collisions))
		for _, collision := range collisions {
			fmt.Printf("  - Collision with entity %d (Type: %d), penetration: %.2f\n",
				collision.Entity.ID, collision.Entity.Type, collision.Penetration)
		}
	}

	// Try a different position
	newPos2 := pathweaver.NewVector2D(8, 8) // Should be clear
	canMove2, _ := engine.CheckMovement(player, newPos2)
	fmt.Printf("Can move player to (%.1f, %.1f): %t\n", newPos2.X, newPos2.Y, canMove2)

	if canMove2 {
		if err := engine.MoveEntity(player.ID, newPos2); err != nil {
			fmt.Printf("Failed to move entity: %v\n", err)
		} else {
			fmt.Printf("Player moved successfully to (%.1f, %.1f)\n", newPos2.X, newPos2.Y)
		}
	}

	// Demo 5: Raycast
	fmt.Println("\n5. Raycast Demo")
	fmt.Println("--------------")

	rayStart := pathweaver.NewVector2D(-10, -10)
	rayDirection := pathweaver.NewVector2D(1, 1) // 45 degree angle
	rayDistance := 30.0

	hit := engine.Raycast(rayStart, rayDirection, rayDistance)
	if hit != nil {
		fmt.Printf("Raycast hit entity %d (Type: %d) at (%.2f, %.2f)\n",
			hit.Entity.ID, hit.Entity.Type, hit.ContactPoint.X, hit.ContactPoint.Y)
		distance := pathweaver.Distance(rayStart, hit.ContactPoint)
		fmt.Printf("Hit distance: %.2f units\n", distance)
	} else {
		fmt.Println("Raycast didn't hit anything")
	}

	// Demo 6: Performance Statistics
	fmt.Println("\n6. Performance Statistics")
	fmt.Println("------------------------")

	stats := engine.GetStats()
	fmt.Printf("Scene Statistics:\n")
	fmt.Printf("  Total entities: %d\n", stats.EntityCount)
	fmt.Printf("  Players: %d\n", stats.PlayerCount)
	fmt.Printf("  NPCs: %d\n", stats.NPCCount)
	fmt.Printf("  Obstacles: %d\n", stats.ObstacleCount)
	fmt.Printf("  Pickups: %d\n", stats.PickupCount)
	fmt.Printf("  Projectiles: %d\n", stats.ProjectileCount)
	fmt.Printf("  Scene bounds: (%.1f,%.1f) to (%.1f,%.1f)\n",
		stats.SceneBounds.Min.X, stats.SceneBounds.Min.Y,
		stats.SceneBounds.Max.X, stats.SceneBounds.Max.Y)

	// Demo 7: Batch Operations
	fmt.Println("\n7. Batch Operations Demo")
	fmt.Println("-----------------------")

	// Create some projectiles
	var projectiles []*core.Entity
	for i := 0; i < 10; i++ {
		angle := float64(i) * 2 * math.Pi / 10 // Distribute in circle
		pos := pathweaver.NewVector2D(
			math.Cos(angle)*5,
			math.Sin(angle)*5,
		)
		projectile := pathweaver.NewProjectileEntity(uint64(100+i), pos, 0.5)
		projectiles = append(projectiles, projectile)
	}

	// Batch add projectiles
	if err := engine.BatchAddEntities(projectiles); err != nil {
		fmt.Printf("Batch add failed: %v\n", err)
	} else {
		fmt.Printf("Successfully added %d projectiles in batch\n", len(projectiles))
	}

	// Update projectile positions (simulate movement)
	for _, projectile := range projectiles {
		// Move towards center
		direction := pathweaver.Normalize(pathweaver.NewVector2D(
			-projectile.Position.X, -projectile.Position.Y))
		newPos := pathweaver.NewVector2D(
			projectile.Position.X+direction.X*2,
			projectile.Position.Y+direction.Y*2,
		)
		projectile.Position = newPos
		// Update bounds
		projectile.Bounds = pathweaver.AABBFromCenterSize(newPos, 0.5, 0.5)
	}

	// Batch update
	if err := engine.BatchUpdate(projectiles); err != nil {
		fmt.Printf("Batch update failed: %v\n", err)
	} else {
		fmt.Println("Batch update successful")
	}

	// Final statistics
	finalStats := engine.GetStats()
	fmt.Printf("\nFinal entity count: %d\n", finalStats.EntityCount)

	fmt.Println("\nDemo completed successfully!")
}
