package mmo

import (
	"fmt"
	"math"
	"pathweaver/internal/core"
	"sync"
)

// ZoneManager handles MMO-specific zone management and interest management
// Based on research from "Massively Multiplayer Online Game Development" papers
// and techniques from modern MMO architectures like WoW's zone system
type ZoneManager struct {
	mu             sync.RWMutex
	zones          map[ZoneID]*Zone
	zoneSize       float64
	overlapSize    float64 // Overlap between zones for smooth transitions
	worldBounds    core.AABB3D
	playerZones    map[uint64]ZoneID // Player to zone mapping
	interestRadius float64           // Radius for interest management
}

// ZoneID represents a unique zone identifier
type ZoneID struct {
	X, Y, Z int
}

// Zone represents a game zone/chunk
type Zone struct {
	ID            ZoneID
	Bounds        core.AABB3D
	Entities      map[uint64]*core.Entity3D
	Players       map[uint64]*core.Entity3D
	Population    int
	LoadLevel     float64 // 0.0 to 1.0, affects LOD decisions
	ActivePlayers int
	mu            sync.RWMutex
}

// InterestArea represents what a player is interested in
type InterestArea struct {
	PlayerID   uint64
	Position   core.Vector3D
	Zones      []ZoneID
	Entities   map[uint64]*core.Entity3D
	LastUpdate int64
}

// NewZoneManager creates a new zone manager for MMO games
func NewZoneManager(worldBounds core.AABB3D, zoneSize float64) *ZoneManager {
	return &ZoneManager{
		zones:          make(map[ZoneID]*Zone),
		zoneSize:       zoneSize,
		overlapSize:    zoneSize * 0.1, // 10% overlap
		worldBounds:    worldBounds,
		playerZones:    make(map[uint64]ZoneID),
		interestRadius: zoneSize * 1.5, // Interest radius covers multiple zones
	}
}

// GetZoneID calculates zone ID from world position
func (zm *ZoneManager) GetZoneID(pos core.Vector3D) ZoneID {
	return ZoneID{
		X: int(math.Floor(pos.X / zm.zoneSize)),
		Y: int(math.Floor(pos.Y / zm.zoneSize)),
		Z: int(math.Floor(pos.Z / zm.zoneSize)),
	}
}

// GetOrCreateZone gets existing zone or creates new one
func (zm *ZoneManager) GetOrCreateZone(zoneID ZoneID) *Zone {
	zm.mu.Lock()
	defer zm.mu.Unlock()

	if zone, exists := zm.zones[zoneID]; exists {
		return zone
	}

	// Create zone bounds
	minX := float64(zoneID.X) * zm.zoneSize
	minY := float64(zoneID.Y) * zm.zoneSize
	minZ := float64(zoneID.Z) * zm.zoneSize

	zone := &Zone{
		ID: zoneID,
		Bounds: core.AABB3D{
			Min: core.Vector3D{X: minX, Y: minY, Z: minZ},
			Max: core.Vector3D{
				X: minX + zm.zoneSize,
				Y: minY + zm.zoneSize,
				Z: minZ + zm.zoneSize,
			},
		},
		Entities:      make(map[uint64]*core.Entity3D),
		Players:       make(map[uint64]*core.Entity3D),
		Population:    0,
		LoadLevel:     0.0,
		ActivePlayers: 0,
	}

	zm.zones[zoneID] = zone
	return zone
}

// AddEntity adds an entity to appropriate zone
func (zm *ZoneManager) AddEntity(entity *core.Entity3D) error {
	zoneID := zm.GetZoneID(entity.Position)
	zone := zm.GetOrCreateZone(zoneID)

	zone.mu.Lock()
	zone.Entities[entity.ID] = entity
	zone.Population++

	// Track players separately for interest management
	if entity.Type == core.EntityTypePlayer {
		zone.Players[entity.ID] = entity
		zone.ActivePlayers++

		zm.mu.Lock()
		zm.playerZones[entity.ID] = zoneID
		zm.mu.Unlock()
	}
	zone.mu.Unlock()

	// Update zone load level
	zm.updateZoneLoad(zone)

	return nil
}

// RemoveEntity removes an entity from its zone
func (zm *ZoneManager) RemoveEntity(entityID uint64) error {
	zm.mu.RLock()
	zoneID, exists := zm.playerZones[entityID]
	zm.mu.RUnlock()

	if !exists {
		// Try to find in all zones (expensive but handles edge cases)
		return zm.removeEntityFromAllZones(entityID)
	}

	zone, exists := zm.zones[zoneID]
	if !exists {
		return fmt.Errorf("zone not found for entity %d", entityID)
	}

	zone.mu.Lock()
	delete(zone.Entities, entityID)
	zone.Population--

	if _, isPlayer := zone.Players[entityID]; isPlayer {
		delete(zone.Players, entityID)
		zone.ActivePlayers--

		zm.mu.Lock()
		delete(zm.playerZones, entityID)
		zm.mu.Unlock()
	}
	zone.mu.Unlock()

	zm.updateZoneLoad(zone)
	return nil
}

// UpdateEntity updates entity position and potentially moves it between zones
func (zm *ZoneManager) UpdateEntity(entity *core.Entity3D) error {
	newZoneID := zm.GetZoneID(entity.Position)

	zm.mu.RLock()
	currentZoneID, exists := zm.playerZones[entity.ID]
	zm.mu.RUnlock()

	if !exists || currentZoneID != newZoneID {
		// Entity moved to different zone
		if exists {
			zm.RemoveEntity(entity.ID)
		}
		zm.AddEntity(entity)
	} else {
		// Update entity in current zone
		if zone, exists := zm.zones[currentZoneID]; exists {
			zone.mu.Lock()
			zone.Entities[entity.ID] = entity
			if entity.Type == core.EntityTypePlayer {
				zone.Players[entity.ID] = entity
			}
			zone.mu.Unlock()
		}
	}

	return nil
}

// GetInterestArea calculates what entities a player should be aware of
// This is crucial for MMO interest management and bandwidth optimization
func (zm *ZoneManager) GetInterestArea(playerID uint64, position core.Vector3D) *InterestArea {
	interestArea := &InterestArea{
		PlayerID: playerID,
		Position: position,
		Entities: make(map[uint64]*core.Entity3D),
		Zones:    []ZoneID{},
	}

	// Get all zones within interest radius
	interestZones := zm.getZonesInRadius(position, zm.interestRadius)

	for _, zoneID := range interestZones {
		if zone, exists := zm.zones[zoneID]; exists {
			interestArea.Zones = append(interestArea.Zones, zoneID)

			zone.mu.RLock()
			// Add entities from this zone with distance filtering
			for _, entity := range zone.Entities {
				if entity.ID != playerID { // Don't include self
					distance := zm.distance3D(position, entity.Position)
					if distance <= zm.interestRadius {
						interestArea.Entities[entity.ID] = entity
					}
				}
			}
			zone.mu.RUnlock()
		}
	}

	return interestArea
}

// GetNearbyPlayers gets players in nearby zones (useful for PvP, chat, etc.)
func (zm *ZoneManager) GetNearbyPlayers(position core.Vector3D, radius float64) []*core.Entity3D {
	var players []*core.Entity3D
	nearbyZones := zm.getZonesInRadius(position, radius)

	for _, zoneID := range nearbyZones {
		if zone, exists := zm.zones[zoneID]; exists {
			zone.mu.RLock()
			for _, player := range zone.Players {
				distance := zm.distance3D(position, player.Position)
				if distance <= radius {
					players = append(players, player)
				}
			}
			zone.mu.RUnlock()
		}
	}

	return players
}

// GetZonePopulation returns current population statistics
func (zm *ZoneManager) GetZonePopulation(zoneID ZoneID) (int, int) {
	zm.mu.RLock()
	zone, exists := zm.zones[zoneID]
	zm.mu.RUnlock()

	if !exists {
		return 0, 0
	}

	zone.mu.RLock()
	defer zone.mu.RUnlock()

	return zone.Population, zone.ActivePlayers
}

// GetHottestZones returns zones with highest activity (for load balancing)
func (zm *ZoneManager) GetHottestZones(limit int) []ZoneID {
	type zoneLoad struct {
		ID   ZoneID
		Load float64
	}

	var loads []zoneLoad

	zm.mu.RLock()
	for id, zone := range zm.zones {
		zone.mu.RLock()
		loads = append(loads, zoneLoad{ID: id, Load: zone.LoadLevel})
		zone.mu.RUnlock()
	}
	zm.mu.RUnlock()

	// Sort by load (simple bubble sort for now)
	for i := 0; i < len(loads)-1; i++ {
		for j := 0; j < len(loads)-i-1; j++ {
			if loads[j].Load < loads[j+1].Load {
				loads[j], loads[j+1] = loads[j+1], loads[j]
			}
		}
	}

	var result []ZoneID
	for i := 0; i < limit && i < len(loads); i++ {
		result = append(result, loads[i].ID)
	}

	return result
}

// ShouldUseDetailedLOD determines if detailed Level of Detail should be used
// Based on zone population and player proximity
func (zm *ZoneManager) ShouldUseDetailedLOD(position core.Vector3D, entityType core.EntityType) bool {
	zoneID := zm.GetZoneID(position)

	zm.mu.RLock()
	zone, exists := zm.zones[zoneID]
	zm.mu.RUnlock()

	if !exists {
		return true // Default to detailed for new zones
	}

	zone.mu.RLock()
	loadLevel := zone.LoadLevel
	playerCount := zone.ActivePlayers
	zone.mu.RUnlock()

	// Use detailed LOD if:
	// - Load level is low (< 0.7)
	// - Player count is reasonable (< 50)
	// - Entity is a player (always high detail)
	if entityType == core.EntityTypePlayer {
		return true
	}

	return loadLevel < 0.7 && playerCount < 50
}

// Helper methods

func (zm *ZoneManager) getZonesInRadius(center core.Vector3D, radius float64) []ZoneID {
	var zones []ZoneID

	// Calculate zone range
	centerZone := zm.GetZoneID(center)
	zoneRadius := int(math.Ceil(radius / zm.zoneSize))

	for x := centerZone.X - zoneRadius; x <= centerZone.X+zoneRadius; x++ {
		for y := centerZone.Y - zoneRadius; y <= centerZone.Y+zoneRadius; y++ {
			for z := centerZone.Z - zoneRadius; z <= centerZone.Z+zoneRadius; z++ {
				zoneID := ZoneID{X: x, Y: y, Z: z}

				// Check if zone center is within radius
				zoneCenterX := float64(x)*zm.zoneSize + zm.zoneSize/2
				zoneCenterY := float64(y)*zm.zoneSize + zm.zoneSize/2
				zoneCenterZ := float64(z)*zm.zoneSize + zm.zoneSize/2

				zoneCenter := core.Vector3D{X: zoneCenterX, Y: zoneCenterY, Z: zoneCenterZ}
				if zm.distance3D(center, zoneCenter) <= radius+zm.zoneSize*0.7 {
					zones = append(zones, zoneID)
				}
			}
		}
	}

	return zones
}

func (zm *ZoneManager) distance3D(a, b core.Vector3D) float64 {
	dx := a.X - b.X
	dy := a.Y - b.Y
	dz := a.Z - b.Z
	return math.Sqrt(dx*dx + dy*dy + dz*dz)
}

func (zm *ZoneManager) updateZoneLoad(zone *Zone) {
	zone.mu.Lock()
	defer zone.mu.Unlock()

	// Calculate load based on population and activity
	populationFactor := float64(zone.Population) / 100.0 // Assume 100 entities per zone is high
	playerFactor := float64(zone.ActivePlayers) / 20.0   // Assume 20 players per zone is high

	zone.LoadLevel = math.Min(1.0, populationFactor*0.6+playerFactor*0.4)
}

func (zm *ZoneManager) removeEntityFromAllZones(entityID uint64) error {
	zm.mu.RLock()
	defer zm.mu.RUnlock()

	for _, zone := range zm.zones {
		zone.mu.Lock()
		if _, exists := zone.Entities[entityID]; exists {
			delete(zone.Entities, entityID)
			zone.Population--

			if _, isPlayer := zone.Players[entityID]; isPlayer {
				delete(zone.Players, entityID)
				zone.ActivePlayers--
			}
			zone.mu.Unlock()
			zm.updateZoneLoad(zone)
			return nil
		}
		zone.mu.Unlock()
	}

	return fmt.Errorf("entity %d not found in any zone", entityID)
}
