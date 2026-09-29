package api

import (
	"bufio"
	"math"
	"net/http"
	"os"
	"strconv"
	"strings"
	"time"

	"github.com/gin-gonic/gin"
	"github.com/sirupsen/logrus"
	"gopkg.in/yaml.v3"

	"github.com/mowglinext/mowglinext/pkg/types"
)

// Paths to the runtime calibration artefacts. The mowgli_robot.yaml
// path is provided at runtime by the DB provider (key
// system.mower.yamlConfigFile) because it differs between containers
// (gui sees /mowgli_config/mowgli_robot.yaml, ros2 sees
// /ros2_ws/config/mowgli_robot.yaml). The earlier hardcoded constant
// caused this endpoint to silently report Present=false on the gui
// container, which then surfaced as the "dock calibration missing"
// banner in the GUI even when the dock pose was correctly persisted.
const (
	imuCalibrationPath = "/ros2_ws/maps/imu_calibration.txt"
	magCalibrationPath = "/ros2_ws/maps/mag_calibration.yaml"
)

// lidarMapCalibrationPath is written by calibrate_lidar_map_node (the
// lidar_map -> map transform for the external LiDAR localizer) and read by
// fusion_graph.launch.py. A var, not a const, so tests can point it at a
// temp file.
var lidarMapCalibrationPath = "/ros2_ws/maps/lidar_map_calibration.yaml"

// ---------------------------------------------------------------------------
// Response types
// ---------------------------------------------------------------------------

// DockCalibrationStatus reports the dock pose currently persisted in
// mowgli_robot.yaml. Present is false when the file is missing or all
// three pose values are zero (an uninitialised configuration).
type DockCalibrationStatus struct {
	Present        bool    `json:"present"`
	DockPoseX      float64 `json:"dock_pose_x,omitempty"`
	DockPoseY      float64 `json:"dock_pose_y,omitempty"`
	DockPoseYawRad float64 `json:"dock_pose_yaw_rad,omitempty"`
	DockPoseYawDeg float64 `json:"dock_pose_yaw_deg,omitempty"`
	Error          string  `json:"error,omitempty"`
}

// ImuCalibrationStatus mirrors the plaintext imu_calibration.txt format
// written by hardware_bridge_node (v1 header).
type ImuCalibrationStatus struct {
	Present         bool    `json:"present"`
	CalibratedAt    string  `json:"calibrated_at,omitempty"`
	SamplesUsed     int     `json:"samples_used,omitempty"`
	AccelBiasX      float64 `json:"accel_bias_x,omitempty"`
	AccelBiasY      float64 `json:"accel_bias_y,omitempty"`
	GyroBiasX       float64 `json:"gyro_bias_x,omitempty"`
	GyroBiasY       float64 `json:"gyro_bias_y,omitempty"`
	GyroBiasZ       float64 `json:"gyro_bias_z,omitempty"`
	ImpliedPitchDeg float64 `json:"implied_pitch_deg,omitempty"`
	ImpliedRollDeg  float64 `json:"implied_roll_deg,omitempty"`
	Error           string  `json:"error,omitempty"`
}

// MagCalibrationStatus mirrors the relevant subset of mag_calibration.yaml.
type MagCalibrationStatus struct {
	Present         bool    `json:"present"`
	CalibratedAt    string  `json:"calibrated_at,omitempty"`
	MagnitudeMeanUT float64 `json:"magnitude_mean_uT,omitempty"`
	MagnitudeStdUT  float64 `json:"magnitude_std_uT,omitempty"`
	SampleCount     int     `json:"sample_count,omitempty"`
	Error           string  `json:"error,omitempty"`
}

// LidarMapCalibrationStatus mirrors lidar_map_calibration.yaml
// (p_map = R(yaw) * p_lidar_map + (x, y)). fusion_graph.launch.py ignores
// the file when its datum differs from the configured one, so DatumLat/Lon
// are reported for the GUI to flag a stale calibration.
type LidarMapCalibrationStatus struct {
	Present      bool    `json:"present"`
	CalibratedAt string  `json:"calibrated_at,omitempty"`
	X            float64 `json:"x,omitempty"`
	Y            float64 `json:"y,omitempty"`
	YawRad       float64 `json:"yaw_rad,omitempty"`
	YawDeg       float64 `json:"yaw_deg,omitempty"`
	DatumLat     float64 `json:"datum_lat,omitempty"`
	DatumLon     float64 `json:"datum_lon,omitempty"`
	Pairs        int     `json:"pairs,omitempty"`
	Inliers      int     `json:"inliers,omitempty"`
	RmsM         float64 `json:"rms_m,omitempty"`
	MaxResidualM float64 `json:"max_residual_m,omitempty"`
	Error        string  `json:"error,omitempty"`
}

// CalibrationStatusResponse is the payload for GET /calibration/status.
type CalibrationStatusResponse struct {
	Dock     DockCalibrationStatus     `json:"dock"`
	Imu      ImuCalibrationStatus      `json:"imu"`
	Mag      MagCalibrationStatus      `json:"mag"`
	LidarMap LidarMapCalibrationStatus `json:"lidar_map"`
}

// ---------------------------------------------------------------------------
// Route registration
// ---------------------------------------------------------------------------

// RegisterCalibrationStatusRoute wires GET /calibration/status into the
// existing calibration group. The DB provider is needed so we can
// resolve the runtime path of mowgli_robot.yaml instead of relying on
// a hardcoded constant (the gui and ros2 containers mount the file at
// different paths).
func registerCalibrationStatusRoute(group *gin.RouterGroup, dbProvider types.IDBProvider) {
	group.GET("/status", getCalibrationStatus(dbProvider))
}

// ---------------------------------------------------------------------------
// GET /calibration/status
// ---------------------------------------------------------------------------

func getCalibrationStatus(dbProvider types.IDBProvider) gin.HandlerFunc {
	return func(c *gin.Context) {
		resp := CalibrationStatusResponse{
			Dock:     readDockCalibrationStatus(dbProvider),
			Imu:      readImuCalibrationStatus(),
			Mag:      readMagCalibrationStatus(),
			LidarMap: readLidarMapCalibrationStatus(),
		}
		c.JSON(http.StatusOK, resp)
	}
}

// readDockCalibrationStatus reads dock_pose_x/y/yaw from
// mowgli_robot.yaml — the single source of truth, written by
// calibrate_imu_yaw_node and /map_server_node/set_docking_point.
// Reports {Present: false} when the file is missing or the dock pose
// is still all-zero (uninitialised configuration).
func readDockCalibrationStatus(dbProvider types.IDBProvider) DockCalibrationStatus {
	mowgliRobotYamlPath, err := dbProvider.Get("system.mower.yamlConfigFile")
	if err != nil {
		logrus.Warnf("calibration_status: cannot resolve yaml path from db: %v", err)
		return DockCalibrationStatus{Present: false, Error: "cannot resolve yaml path"}
	}
	data, err := os.ReadFile(string(mowgliRobotYamlPath))
	if err != nil {
		if os.IsNotExist(err) {
			return DockCalibrationStatus{Present: false}
		}
		logrus.Warnf("calibration_status: cannot read %s: %v", mowgliRobotYamlPath, err)
		return DockCalibrationStatus{Present: true, Error: err.Error()}
	}
	var yamlData map[string]interface{}
	if err := yaml.Unmarshal(data, &yamlData); err != nil {
		return DockCalibrationStatus{Present: true, Error: "parse: " + err.Error()}
	}
	x := extractYAMLFloat(yamlData, "dock_pose_x")
	y := extractYAMLFloat(yamlData, "dock_pose_y")
	yawRad := extractYAMLFloat(yamlData, "dock_pose_yaw")
	if x == 0 && y == 0 && yawRad == 0 {
		return DockCalibrationStatus{Present: false}
	}
	return DockCalibrationStatus{
		Present:        true,
		DockPoseX:      x,
		DockPoseY:      y,
		DockPoseYawRad: yawRad,
		DockPoseYawDeg: yawRad * 180.0 / math.Pi,
	}
}

// readImuCalibrationStatus parses the v1 plaintext format written by
// hardware_bridge_node. Format:
//
//	# mowgli_imu_calibration_v1
//	<ts_unix> <n_samples>
//	<off_ax> <off_ay> <off_gx> <off_gy> <off_gz>
//	<cov_ax> <cov_ay> <cov_gx> <cov_gy> <cov_gz>
//	<implied_pitch_deg> <implied_roll_deg>
func readImuCalibrationStatus() ImuCalibrationStatus {
	f, err := os.Open(imuCalibrationPath)
	if err != nil {
		if os.IsNotExist(err) {
			return ImuCalibrationStatus{Present: false}
		}
		return ImuCalibrationStatus{Present: true, Error: err.Error()}
	}
	defer f.Close()

	scanner := bufio.NewScanner(f)
	lines := []string{}
	for scanner.Scan() {
		lines = append(lines, strings.TrimSpace(scanner.Text()))
	}
	if err := scanner.Err(); err != nil {
		return ImuCalibrationStatus{Present: true, Error: err.Error()}
	}
	if len(lines) < 4 {
		return ImuCalibrationStatus{Present: true, Error: "unexpected line count"}
	}
	if lines[0] != "# mowgli_imu_calibration_v1" {
		return ImuCalibrationStatus{Present: true, Error: "header mismatch: " + lines[0]}
	}
	status := ImuCalibrationStatus{Present: true}

	// Line 1: <ts_unix> <n_samples>
	if parts := strings.Fields(lines[1]); len(parts) >= 2 {
		if ts, err := strconv.ParseInt(parts[0], 10, 64); err == nil {
			status.CalibratedAt = time.Unix(ts, 0).UTC().Format(time.RFC3339)
		}
		if n, err := strconv.Atoi(parts[1]); err == nil {
			status.SamplesUsed = n
		}
	}
	// Line 2: accel biases (X, Y) + gyro biases (X, Y, Z)
	if parts := strings.Fields(lines[2]); len(parts) >= 5 {
		status.AccelBiasX = parseFloat(parts[0])
		status.AccelBiasY = parseFloat(parts[1])
		status.GyroBiasX = parseFloat(parts[2])
		status.GyroBiasY = parseFloat(parts[3])
		status.GyroBiasZ = parseFloat(parts[4])
	}
	// Line 4 (optional): implied pitch/roll
	if len(lines) >= 5 {
		if parts := strings.Fields(lines[4]); len(parts) >= 2 {
			status.ImpliedPitchDeg = parseFloat(parts[0])
			status.ImpliedRollDeg = parseFloat(parts[1])
		}
	}
	return status
}

// readMagCalibrationStatus loads /ros2_ws/maps/mag_calibration.yaml.
func readMagCalibrationStatus() MagCalibrationStatus {
	data, err := os.ReadFile(magCalibrationPath)
	if err != nil {
		if os.IsNotExist(err) {
			return MagCalibrationStatus{Present: false}
		}
		return MagCalibrationStatus{Present: true, Error: err.Error()}
	}
	var parsed struct {
		MagCalibration struct {
			CalibratedAt    string  `yaml:"calibrated_at"`
			MagnitudeMeanUT float64 `yaml:"magnitude_mean_uT"`
			MagnitudeStdUT  float64 `yaml:"magnitude_std_uT"`
			SampleCount     int     `yaml:"sample_count"`
		} `yaml:"mag_calibration"`
	}
	if err := yaml.Unmarshal(data, &parsed); err != nil {
		return MagCalibrationStatus{Present: true, Error: "parse: " + err.Error()}
	}
	return MagCalibrationStatus{
		Present:         true,
		CalibratedAt:    parsed.MagCalibration.CalibratedAt,
		MagnitudeMeanUT: parsed.MagCalibration.MagnitudeMeanUT,
		MagnitudeStdUT:  parsed.MagCalibration.MagnitudeStdUT,
		SampleCount:     parsed.MagCalibration.SampleCount,
	}
}

// readLidarMapCalibrationStatus loads lidar_map_calibration.yaml.
func readLidarMapCalibrationStatus() LidarMapCalibrationStatus {
	data, err := os.ReadFile(lidarMapCalibrationPath)
	if err != nil {
		if os.IsNotExist(err) {
			return LidarMapCalibrationStatus{Present: false}
		}
		return LidarMapCalibrationStatus{Present: true, Error: err.Error()}
	}
	var parsed struct {
		Cal *struct {
			X            float64 `yaml:"x"`
			Y            float64 `yaml:"y"`
			Yaw          float64 `yaml:"yaw"`
			DatumLat     float64 `yaml:"datum_lat"`
			DatumLon     float64 `yaml:"datum_lon"`
			Pairs        int     `yaml:"pairs"`
			Inliers      int     `yaml:"inliers"`
			RmsM         float64 `yaml:"rms_m"`
			MaxResidualM float64 `yaml:"max_residual_m"`
			CalibratedAt string  `yaml:"calibrated_at"`
		} `yaml:"lidar_map_calibration"`
	}
	if err := yaml.Unmarshal(data, &parsed); err != nil {
		return LidarMapCalibrationStatus{Present: true, Error: "parse: " + err.Error()}
	}
	if parsed.Cal == nil {
		return LidarMapCalibrationStatus{Present: true, Error: "missing lidar_map_calibration section"}
	}
	cal := parsed.Cal
	return LidarMapCalibrationStatus{
		Present:      true,
		CalibratedAt: cal.CalibratedAt,
		X:            cal.X,
		Y:            cal.Y,
		YawRad:       cal.Yaw,
		YawDeg:       cal.Yaw * 180.0 / math.Pi,
		DatumLat:     cal.DatumLat,
		DatumLon:     cal.DatumLon,
		Pairs:        cal.Pairs,
		Inliers:      cal.Inliers,
		RmsM:         cal.RmsM,
		MaxResidualM: cal.MaxResidualM,
	}
}

func parseFloat(s string) float64 {
	v, _ := strconv.ParseFloat(s, 64)
	return v
}
