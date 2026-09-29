package api

import (
	"encoding/json"
	"errors"
	"net/http"
	"net/http/httptest"
	"os"
	"path/filepath"
	"testing"

	"github.com/gin-gonic/gin"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"

	"github.com/mowglinext/mowglinext/pkg/types"
)

func withLidarMapCalibrationPath(t *testing.T, path string) {
	t.Helper()
	prev := lidarMapCalibrationPath
	lidarMapCalibrationPath = path
	t.Cleanup(func() { lidarMapCalibrationPath = prev })
}

func TestReadLidarMapCalibrationStatus_Missing(t *testing.T) {
	withLidarMapCalibrationPath(t, filepath.Join(t.TempDir(), "absent.yaml"))
	st := readLidarMapCalibrationStatus()
	assert.False(t, st.Present)
	assert.Empty(t, st.Error)
}

func TestReadLidarMapCalibrationStatus_Parses(t *testing.T) {
	path := filepath.Join(t.TempDir(), "lidar_map_calibration.yaml")
	// Format written by calibrate_lidar_map_node (WriteCalibration).
	content := `# Written by calibrate_lidar_map_node. p_map = R(yaw) * p_lidar_map + (x, y).
lidar_map_calibration:
  x: 1.25
  y: -3.5
  yaw: 1.5707963267948966
  lidar_frame: lidar_map
  datum_lat: 48.123456789
  datum_lon: 19.987654321
  pairs: 412
  inliers: 400
  rms_m: 0.031
  max_residual_m: 0.09
  spread_major_std_m: 4.2
  spread_minor_std_m: 2.1
  calibrated_at: "2026-09-26T10:00:00Z"
`
	require.NoError(t, os.WriteFile(path, []byte(content), 0o644))
	withLidarMapCalibrationPath(t, path)

	st := readLidarMapCalibrationStatus()
	require.True(t, st.Present)
	assert.Empty(t, st.Error)
	assert.InDelta(t, 1.25, st.X, 1e-9)
	assert.InDelta(t, -3.5, st.Y, 1e-9)
	assert.InDelta(t, 90.0, st.YawDeg, 1e-9)
	assert.InDelta(t, 48.123456789, st.DatumLat, 1e-12)
	assert.Equal(t, 412, st.Pairs)
	assert.Equal(t, 400, st.Inliers)
	assert.InDelta(t, 0.031, st.RmsM, 1e-9)
	assert.Equal(t, "2026-09-26T10:00:00Z", st.CalibratedAt)
}

func TestReadLidarMapCalibrationStatus_WrongShape(t *testing.T) {
	path := filepath.Join(t.TempDir(), "lidar_map_calibration.yaml")
	require.NoError(t, os.WriteFile(path, []byte("something_else: 1\n"), 0o644))
	withLidarMapCalibrationPath(t, path)

	st := readLidarMapCalibrationStatus()
	assert.True(t, st.Present)
	assert.NotEmpty(t, st.Error)
}

func newCalibrationRouter(ros types.IRosProvider) *gin.Engine {
	gin.SetMode(gin.TestMode)
	r := gin.New()
	CalibrationRoutes(r.Group("/api"), ros, types.NewMockDBProvider())
	return r
}

func TestLidarMapCalibrationTrigger_CallsServices(t *testing.T) {
	for _, tc := range []struct{ path, service string }{
		{"/api/calibration/lidar-map/start", "/calibrate_lidar_map_node/start"},
		{"/api/calibration/lidar-map/cancel", "/calibrate_lidar_map_node/cancel"},
	} {
		ros := types.NewMockRosProvider()
		ros.ServiceResponder = func(_ string, _ any, res any) {
			b, _ := json.Marshal(map[string]any{"success": true, "message": "ok"})
			_ = json.Unmarshal(b, res)
		}
		r := newCalibrationRouter(ros)

		w := httptest.NewRecorder()
		r.ServeHTTP(w, httptest.NewRequest(http.MethodPost, tc.path, nil))
		require.Equal(t, http.StatusOK, w.Code, tc.path)
		require.Len(t, ros.ServiceCalls, 1)
		assert.Equal(t, tc.service, ros.ServiceCalls[0].Service)

		var body map[string]any
		require.NoError(t, json.Unmarshal(w.Body.Bytes(), &body))
		assert.Equal(t, true, body["success"])
		assert.Equal(t, "ok", body["message"])
	}
}

func TestLidarMapCalibrationTrigger_ServiceError(t *testing.T) {
	ros := types.NewMockRosProvider()
	ros.ServiceErr = errors.New("service not advertised")
	r := newCalibrationRouter(ros)

	w := httptest.NewRecorder()
	r.ServeHTTP(w, httptest.NewRequest(http.MethodPost, "/api/calibration/lidar-map/start", nil))
	assert.Equal(t, http.StatusInternalServerError, w.Code)
	assert.Contains(t, w.Body.String(), "service not advertised")
}
