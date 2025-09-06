package pathfinding

import (
	"container/heap"
	"pathweaver/internal/core"
)

// NodePriorityQueue implements a min-heap for PathNodes
type NodePriorityQueue []*core.PathNode

func (pq NodePriorityQueue) Len() int { return len(pq) }

func (pq NodePriorityQueue) Less(i, j int) bool {
	// Min-heap based on F value
	return pq[i].F < pq[j].F
}

func (pq NodePriorityQueue) Swap(i, j int) {
	pq[i], pq[j] = pq[j], pq[i]
	pq[i].Index = i
	pq[j].Index = j
}

func (pq *NodePriorityQueue) Push(x interface{}) {
	n := len(*pq)
	node := x.(*core.PathNode)
	node.Index = n
	*pq = append(*pq, node)
}

func (pq *NodePriorityQueue) Pop() interface{} {
	old := *pq
	n := len(old)
	node := old[n-1]
	old[n-1] = nil  // avoid memory leak
	node.Index = -1 // for safety
	*pq = old[0 : n-1]
	return node
}

// Update modifies the F value of a PathNode and re-establishes heap invariant
func (pq *NodePriorityQueue) Update(node *core.PathNode, f float64) {
	node.F = f
	heap.Fix(pq, node.Index)
}
