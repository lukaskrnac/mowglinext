package api

import (
	"context"
	"net/http"
	"regexp"
	"strings"
	"time"

	"github.com/gin-gonic/gin"

	"github.com/mowglinext/mowglinext/pkg/types"
)

// GLIM mapping is driven through glim_supervisor (3d_mowgli_slam_stack,
// glim/supervisor/glim_supervisor.py): the GLIM container runs permanently
// and the supervisor starts glim_rosnode / offline_viewer on demand. Every
// action is a std_srvs/Trigger service; the session for open_viewer and
// export_map is passed through the supervisor's `target_session` parameter
// (Trigger has no arguments). Status (state, sessions, active map) streams on
// the "glimSupervisorStatus" topic.
const (
	glimSupervisorNode   = "glim_supervisor"
	glimTargetSessionKey = glimSupervisorNode + ".target_session"
)

// Mirrors SESSION_NAME_RE in glim_supervisor.py. Validated here too because
// this API has no authentication and the name ends up in a filesystem path.
var glimSessionNameRe = regexp.MustCompile(`^[A-Za-z0-9][A-Za-z0-9._-]{0,127}$`)

func validGlimSessionName(name string) bool {
	return glimSessionNameRe.MatchString(name) && !strings.Contains(name, "..")
}

// GlimSessionRequest is the body for POST /glim/open-viewer and /glim/export-map.
type GlimSessionRequest struct {
	Session string `json:"session"`
}

// GlimRoutes registers the GLIM mapping endpoints.
//
// @Summary control GLIM mapping through glim_supervisor
// @Description start/stop a mapping run, open a session in the offline viewer, close it, or export a session as the active map
// @Tags glim
// @Accept json
// @Produce json
// @Param body body GlimSessionRequest false "session (open-viewer, export-map only)"
// @Success 200 {object} map[string]interface{}
// @Failure 400 {object} ErrorResponse
// @Failure 500 {object} ErrorResponse
// @Router /glim/start-mapping [post]
// @Router /glim/stop-mapping [post]
// @Router /glim/close-viewer [post]
// @Router /glim/open-viewer [post]
// @Router /glim/export-map [post]
func GlimRoutes(r *gin.RouterGroup, rosProvider types.IRosProvider) {
	group := r.Group("/glim")
	group.POST("/start-mapping", postGlimTrigger(rosProvider, "start_mapping"))
	group.POST("/stop-mapping", postGlimTrigger(rosProvider, "stop_mapping"))
	group.POST("/close-viewer", postGlimTrigger(rosProvider, "close_viewer"))
	group.POST("/open-viewer", postGlimSessionTrigger(rosProvider, "open_viewer"))
	group.POST("/export-map", postGlimSessionTrigger(rosProvider, "export_map"))
}

type triggerResult struct {
	Success bool   `json:"success"`
	Message string `json:"message"`
}

func callGlimTrigger(ctx context.Context, rosProvider types.IRosProvider, service string) (triggerResult, error) {
	var res triggerResult
	err := rosProvider.CallService(ctx, "/"+glimSupervisorNode+"/"+service, &struct{}{}, &res, "std_srvs/srv/Trigger")
	return res, err
}

func postGlimTrigger(rosProvider types.IRosProvider, service string) gin.HandlerFunc {
	return func(c *gin.Context) {
		ctx, cancel := context.WithTimeout(c.Request.Context(), 10*time.Second)
		defer cancel()
		res, err := callGlimTrigger(ctx, rosProvider, service)
		if err != nil {
			c.JSON(http.StatusInternalServerError, ErrorResponse{Error: "glim_supervisor " + service + ": " + err.Error()})
			return
		}
		c.JSON(http.StatusOK, gin.H{"success": res.Success, "message": res.Message})
	}
}

func postGlimSessionTrigger(rosProvider types.IRosProvider, service string) gin.HandlerFunc {
	return func(c *gin.Context) {
		var req GlimSessionRequest
		if err := c.BindJSON(&req); err != nil {
			c.JSON(http.StatusBadRequest, ErrorResponse{Error: "invalid payload: " + err.Error()})
			return
		}
		if !validGlimSessionName(req.Session) {
			c.JSON(http.StatusBadRequest, ErrorResponse{Error: "invalid session name"})
			return
		}
		ctx, cancel := context.WithTimeout(c.Request.Context(), 15*time.Second)
		defer cancel()
		if _, err := rosProvider.SetParameters(ctx, []types.RosParameter{
			{Name: glimTargetSessionKey, Value: req.Session},
		}); err != nil {
			c.JSON(http.StatusInternalServerError, ErrorResponse{Error: "glim_supervisor target_session: " + err.Error()})
			return
		}
		res, err := callGlimTrigger(ctx, rosProvider, service)
		if err != nil {
			c.JSON(http.StatusInternalServerError, ErrorResponse{Error: "glim_supervisor " + service + ": " + err.Error()})
			return
		}
		// The supervisor answers with the session it actually used. The
		// foxglove bridge treats a parameter-set timeout as success, so this is
		// what proves the selection arrived and was not a stale value.
		if res.Success && res.Message != req.Session {
			c.JSON(http.StatusOK, gin.H{
				"success": false,
				"message": "glim_supervisor used session '" + res.Message + "' instead of '" + req.Session + "'",
			})
			return
		}
		c.JSON(http.StatusOK, gin.H{"success": res.Success, "message": res.Message})
	}
}
