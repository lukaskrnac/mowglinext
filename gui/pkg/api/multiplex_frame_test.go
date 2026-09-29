package api

import (
	"encoding/json"
	"testing"

	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
	"github.com/vmihailenco/msgpack/v5"
)

// occupancyGridJSON returns the upstream JSON of an OccupancyGrid with n cells
// (every third one mowed), as produced for the mowProgress topic.
func occupancyGridJSON(t *testing.T, n int) []byte {
	t.Helper()
	cells := make([]int8, n)
	for i := range cells {
		if i%3 == 0 {
			cells[i] = 100
		}
	}
	data, err := json.Marshal(map[string]interface{}{
		"header": map[string]interface{}{"frame_id": "map", "stamp": map[string]interface{}{"sec": 1790000000, "nanosec": 5}},
		"info": map[string]interface{}{
			"resolution": 0.05, "width": 540, "height": n / 540,
			"origin": map[string]interface{}{"position": map[string]interface{}{"x": -13.5, "y": 2.25, "z": 0}},
		},
		"data": cells,
	})
	require.NoError(t, err)
	return data
}

// Regression: json.Unmarshal into interface{} makes every number a float64,
// which default msgpack writes as 9 bytes. A ~300k-cell mowProgress grid became
// a ~2.6 MB frame and saturated the WiFi link (teleop stutter, map lag).
func TestEncodeMultiplexFrameKeepsOccupancyGridsCompact(t *testing.T) {
	const cells = 540 * 540
	frame, err := encodeMultiplexFrame("mowProgress", occupancyGridJSON(t, cells))
	require.NoError(t, err)

	// One byte per cell plus a small envelope; the old encoding was ~9 B/cell.
	assert.Less(t, len(frame), cells+cells/10, "frame is %d bytes for %d cells", len(frame), cells)
}

func TestEncodeMultiplexFrameRoundTripsValues(t *testing.T) {
	src := occupancyGridJSON(t, 540*4)
	frame, err := encodeMultiplexFrame("mowProgress", src)
	require.NoError(t, err)

	var decoded map[string]interface{}
	require.NoError(t, msgpack.Unmarshal(frame, &decoded))
	assert.Equal(t, "mowProgress", decoded["topic"])

	// Compare through JSON so int/float encodings of the same number are equal —
	// the browser (msgpackr) decodes both to a JS number.
	var want, got interface{}
	require.NoError(t, json.Unmarshal(src, &want))
	gotJSON, err := json.Marshal(decoded["data"])
	require.NoError(t, err)
	require.NoError(t, json.Unmarshal(gotJSON, &got))
	assert.Equal(t, want, got)

	info := decoded["data"].(map[string]interface{})["info"].(map[string]interface{})
	assert.InDelta(t, 0.05, info["resolution"], 1e-12, "non-integral floats must survive")
}

func TestEncodeMultiplexFrameRejectsInvalidJSON(t *testing.T) {
	_, err := encodeMultiplexFrame("mowProgress", []byte("{"))
	assert.Error(t, err)
}
