import {useCallback, useEffect, useState} from "react";
import {useApi} from "./useApi.ts";

export interface DockCalibrationStatus {
    present: boolean;
    dock_pose_x?: number;
    dock_pose_y?: number;
    dock_pose_yaw_rad?: number;
    dock_pose_yaw_deg?: number;
    error?: string;
}

export interface ImuCalibrationStatus {
    present: boolean;
    calibrated_at?: string;
    samples_used?: number;
    accel_bias_x?: number;
    accel_bias_y?: number;
    gyro_bias_x?: number;
    gyro_bias_y?: number;
    gyro_bias_z?: number;
    implied_pitch_deg?: number;
    implied_roll_deg?: number;
    error?: string;
}

export interface MagCalibrationStatus {
    present: boolean;
    calibrated_at?: string;
    magnitude_mean_uT?: number;
    magnitude_std_uT?: number;
    sample_count?: number;
    error?: string;
}

/** lidar_map_calibration.yaml (calibrate_lidar_map_node): p_map = R(yaw)·p_lidar_map + (x, y). */
export interface LidarMapCalibrationFileStatus {
    present: boolean;
    calibrated_at?: string;
    x?: number;
    y?: number;
    yaw_rad?: number;
    yaw_deg?: number;
    datum_lat?: number;
    datum_lon?: number;
    pairs?: number;
    inliers?: number;
    rms_m?: number;
    max_residual_m?: number;
    error?: string;
}

export interface CalibrationStatus {
    dock: DockCalibrationStatus;
    imu: ImuCalibrationStatus;
    mag: MagCalibrationStatus;
    /** Absent on backends older than the LiDAR map calibration. */
    lidar_map?: LidarMapCalibrationFileStatus;
}

/**
 * Polls /api/calibration/status every 5 seconds. The backend reads
 * dock pose from mowgli_robot.yaml plus on-disk artefacts
 * (imu_calibration.txt, mag_calibration.yaml) and returns
 * {present: false} for any that are missing. Safe to call even when
 * the robot has never been calibrated.
 */
export const useCalibrationStatus = () => {
    const guiApi = useApi();
    const [status, setStatus] = useState<CalibrationStatus | null>(null);
    const [error, setError] = useState<string | null>(null);

    const refresh = useCallback(async () => {
        try {
            const response = await guiApi.request<CalibrationStatus>({
                path: "/calibration/status",
                method: "GET",
                format: "json",
            });
            setStatus(response.data);
            setError(null);
        } catch (e) {
            setError(e instanceof Error ? e.message : "Failed to fetch calibration status");
        }
    }, []);

    useEffect(() => {
        refresh();
        const id = setInterval(refresh, 5000);
        return () => clearInterval(id);
    }, [refresh]);

    return {status, error, refresh};
};
