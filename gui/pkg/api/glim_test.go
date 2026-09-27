package api

import (
	"bytes"
	"encoding/json"
	"errors"
	"net/http"
	"net/http/httptest"
	"testing"

	"github.com/gin-gonic/gin"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"

	"github.com/mowglinext/mowglinext/pkg/types"
)

func newGlimRouter(ros types.IRosProvider) *gin.Engine {
	gin.SetMode(gin.TestMode)
	r := gin.New()
	GlimRoutes(r.Group("/api"), ros)
	return r
}

func respondTrigger(success bool, message string) func(string, any, any) {
	return func(_ string, _ any, res any) {
		b, _ := json.Marshal(map[string]any{"success": success, "message": message})
		_ = json.Unmarshal(b, res)
	}
}

func TestGlimTriggers_CallSupervisorServices(t *testing.T) {
	for path, service := range map[string]string{
		"/api/glim/start-mapping": "/glim_supervisor/start_mapping",
		"/api/glim/stop-mapping":  "/glim_supervisor/stop_mapping",
		"/api/glim/close-viewer":  "/glim_supervisor/close_viewer",
	} {
		ros := types.NewMockRosProvider()
		ros.ServiceResponder = respondTrigger(true, "mapping_2026-09-27_164005")
		w := httptest.NewRecorder()
		newGlimRouter(ros).ServeHTTP(w, httptest.NewRequest(http.MethodPost, path, nil))
		require.Equal(t, http.StatusOK, w.Code, path)
		require.Len(t, ros.ServiceCalls, 1, path)
		assert.Equal(t, service, ros.ServiceCalls[0].Service)
		assert.Contains(t, w.Body.String(), `"success":true`)
	}
}

func postGlimSession(r *gin.Engine, path, session string) *httptest.ResponseRecorder {
	body, _ := json.Marshal(GlimSessionRequest{Session: session})
	w := httptest.NewRecorder()
	r.ServeHTTP(w, httptest.NewRequest(http.MethodPost, path, bytes.NewReader(body)))
	return w
}

func TestGlimSessionTrigger_SetsTargetThenCalls(t *testing.T) {
	for path, service := range map[string]string{
		"/api/glim/open-viewer": "/glim_supervisor/open_viewer",
		"/api/glim/export-map":  "/glim_supervisor/export_map",
	} {
		ros := types.NewMockRosProvider()
		ros.ServiceResponder = respondTrigger(true, "merged_v2")
		w := postGlimSession(newGlimRouter(ros), path, "merged_v2")
		require.Equal(t, http.StatusOK, w.Code, path)
		require.Len(t, ros.SetParams, 1)
		assert.Equal(t, []types.RosParameter{{Name: "glim_supervisor.target_session", Value: "merged_v2"}}, ros.SetParams[0])
		require.Len(t, ros.ServiceCalls, 1)
		assert.Equal(t, service, ros.ServiceCalls[0].Service)
		assert.Contains(t, w.Body.String(), `"success":true`)
	}
}

func TestGlimSessionTrigger_RejectsBadNames(t *testing.T) {
	for _, name := range []string{"", "../etc", "a/b", ".hidden", "a..b"} {
		ros := types.NewMockRosProvider()
		w := postGlimSession(newGlimRouter(ros), "/api/glim/export-map", name)
		assert.Equal(t, http.StatusBadRequest, w.Code, name)
		assert.Empty(t, ros.SetParams, name)
		assert.Empty(t, ros.ServiceCalls, name)
	}
}

func TestGlimSessionTrigger_DetectsStaleSelection(t *testing.T) {
	ros := types.NewMockRosProvider()
	ros.ServiceResponder = respondTrigger(true, "older_session")
	w := postGlimSession(newGlimRouter(ros), "/api/glim/export-map", "wanted")
	require.Equal(t, http.StatusOK, w.Code)
	assert.Contains(t, w.Body.String(), `"success":false`)
	assert.Contains(t, w.Body.String(), "older_session")
}

func TestGlimSessionTrigger_PassesRefusal(t *testing.T) {
	ros := types.NewMockRosProvider()
	ros.ServiceResponder = respondTrigger(false, "Busy: mapping.")
	w := postGlimSession(newGlimRouter(ros), "/api/glim/open-viewer", "a")
	require.Equal(t, http.StatusOK, w.Code)
	assert.Contains(t, w.Body.String(), "Busy: mapping.")
}

func TestGlimTrigger_ServiceError(t *testing.T) {
	ros := types.NewMockRosProvider()
	ros.ServiceErr = errors.New("service not advertised")
	w := httptest.NewRecorder()
	newGlimRouter(ros).ServeHTTP(w, httptest.NewRequest(http.MethodPost, "/api/glim/start-mapping", nil))
	assert.Equal(t, http.StatusInternalServerError, w.Code)
	assert.Contains(t, w.Body.String(), "service not advertised")
}
