package control

import (
	"sync"
)

// Signal holds any data passed between blocks.
type Signal struct {
	signal    []float64
	time      []int
	dimension int
	name      string
	blockType controlBlockType
	mu        sync.Mutex
}

func makeSignal(name string, blockType controlBlockType) *Signal {
	var s Signal
	dimension := 1
	s.dimension = dimension
	s.signal = make([]float64, dimension)
	s.time = make([]int, dimension)
	s.name = name
	s.blockType = blockType
	return &s
}

// makeSignals returns a Signal object where the length of its signal[] array is dependent
// on the number of PIDSets from the config.
func makeSignals(name string, blockType controlBlockType, dimension int) *Signal {
	var s Signal
	s.dimension = dimension
	s.signal = make([]float64, dimension)
	s.time = make([]int, dimension)
	s.name = name
	s.blockType = blockType
	return &s
}

// GetSignalValueAt returns the value of the signal at an index, threadsafe.
func (s *Signal) GetSignalValueAt(i int) float64 {
	s.mu.Lock()
	defer s.mu.Unlock()
	if !(i < len(s.signal)) {
		return 0.0
	}
	return s.signal[i]
}

// SetSignalValueAt set the value of a signal at an index, threadsafe.
func (s *Signal) SetSignalValueAt(i int, val float64) {
	s.mu.Lock()
	defer s.mu.Unlock()
	if !(i < len(s.signal)) {
		return
	}
	s.signal[i] = val
}
