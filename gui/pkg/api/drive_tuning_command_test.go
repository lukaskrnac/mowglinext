package api

import (
	"strings"
	"testing"
)

func TestBuildFeedForwardCommandPassesRuntimeRobotConfigPath(t *testing.T) {
	args, _ := buildFeedForwardCommand(driveFFCalibrationStartRequest{
		DistanceMeters: 3.0,
		TestSpeedMps:   0.3,
		OdomTimeoutS:   4.0,
		Passes:         3,
	})

	script := args[len(args)-1]
	if !strings.Contains(script, "'--hardware-config' '"+driveTuningRobotConfigPath+"'") {
		t.Fatalf("expected feed-forward command to pass --hardware-config %s, got %v", driveTuningRobotConfigPath, args)
	}
}

func TestBuildPIDCommandPassesRuntimeRobotConfigPath(t *testing.T) {
	args, _ := buildPIDCommand(drivePIDTuningStartRequest{
		MaxSpeedMps:      0.3,
		SegmentDurationS: 5.0,
		Passes:           3,
	})

	script := args[len(args)-1]
	if !strings.Contains(script, "'--hardware-config' '"+driveTuningRobotConfigPath+"'") {
		t.Fatalf("expected pid command to pass --hardware-config %s, got %v", driveTuningRobotConfigPath, args)
	}
}
