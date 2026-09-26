import {useCallback, useEffect, useState} from "react";
import {useWS} from "./useWS.ts";

/**
 * Live status of calibrate_lidar_map_node (lidar_map → map transform for the
 * external LiDAR localizer). The node publishes a latched std_msgs/String
 * whose `data` is a JSON object — see PublishStatus() in
 * calibrate_lidar_map_node.cpp. The node never drives the robot: the operator
 * drives (manual mode) while it collects RTK-Fixed ↔ /pcl_pose pairs.
 */
export type LidarMapCalibrationState = "idle" | "collecting" | "succeeded" | "failed" | "canceled";

export interface LidarMapCalibrationLiveStatus {
    /** One of LidarMapCalibrationState; kept as string for unknown future states. */
    state: string;
    message: string;
    pairs: number;
    min_pairs: number;
    spread_major_std_m: number;
    spread_minor_std_m: number;
    yaw_deg?: number;
    x_m?: number;
    y_m?: number;
    rms_cm?: number;
    inliers?: number;
    lidar_healthy: boolean;
    diag?: {
        lever_cfg: [number, number];
        lever_est: [number, number];
        scale: number;
        yaw_deg: number;
        rms_cm: number;
    };
    gps_fixes?: number;
    gps_fix_sigma_m?: number;
    gps_rtk_fixed?: boolean;
    gps_accuracy_m?: number | null;
    gps_status?: string;
    warning?: string;
    rejected?: Record<string, number>;
}

/** Parse the std_msgs/String payload; tolerant of either field casing. */
export function parseLidarMapCalibrationStatus(raw: unknown): LidarMapCalibrationLiveStatus | null {
    const msg = raw as {data?: unknown; Data?: unknown} | null;
    const text = msg?.data ?? msg?.Data;
    if (typeof text !== "string" || text.length === 0) return null;
    try {
        const parsed = JSON.parse(text) as unknown;
        return parsed !== null && typeof parsed === "object" &&
            typeof (parsed as {state?: unknown}).state === "string"
            ? (parsed as LidarMapCalibrationLiveStatus)
            : null;
    } catch {
        return null;
    }
}

// Mirrors the node's defaults (calibrate_lidar_map_node: min_major_std_m /
// min_minor_std_m) so the progress bars have a target before the first fit.
export const SPREAD_TARGET_MAJOR_M = 2.0;
export const SPREAD_TARGET_MINOR_M = 1.0;

type TriggerResponse = {success?: boolean; message?: string; error?: string};

async function trigger(verb: "start" | "cancel"): Promise<TriggerResponse> {
    const res = await fetch(`/api/calibration/lidar-map/${verb}`, {method: "POST"});
    const body = (await res.json().catch(() => ({}))) as TriggerResponse;
    if (!res.ok) throw new Error(body.error || `HTTP ${res.status}`);
    return body;
}

export const useLidarMapCalibration = () => {
    const [status, setStatus] = useState<LidarMapCalibrationLiveStatus | null>(null);
    const [busy, setBusy] = useState(false);
    const [actionError, setActionError] = useState<string | null>(null);

    const stream = useWS<string>(
        () => {},
        () => {},
        (e) => {
            const parsed = parseLidarMapCalibrationStatus(e);
            if (parsed) setStatus(parsed);
        },
    );

    useEffect(() => {
        stream.start("/api/mowglinext/subscribe/lidarMapCalibrationStatus");
        return () => stream.stop();
        // eslint-disable-next-line react-hooks/exhaustive-deps
    }, []);

    const run = useCallback(async (verb: "start" | "cancel") => {
        setBusy(true);
        setActionError(null);
        try {
            const body = await trigger(verb);
            if (!body.success) {
                setActionError(body.message || verb);
                return false;
            }
            return true;
        } catch (e: unknown) {
            setActionError(e instanceof Error ? e.message : String(e));
            return false;
        } finally {
            setBusy(false);
        }
    }, []);

    const start = useCallback(() => run("start"), [run]);
    const cancel = useCallback(() => run("cancel"), [run]);
    const collecting = status?.state === "collecting";

    return {status, busy, actionError, start, cancel, collecting};
};
