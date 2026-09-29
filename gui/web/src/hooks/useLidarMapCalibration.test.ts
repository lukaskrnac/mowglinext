import {describe, expect, it} from "vitest";
import {parseLidarMapCalibrationStatus} from "./useLidarMapCalibration.ts";

describe("parseLidarMapCalibrationStatus", () => {
    it("parses the node's JSON status from std_msgs/String.data", () => {
        const data = JSON.stringify({
            state: "collecting", message: "collecting pairs", pairs: 120, min_pairs: 300,
            spread_major_std_m: 1.5, spread_minor_std_m: 0.4, lidar_healthy: true,
            gps_rtk_fixed: true, gps_accuracy_m: 0.014, rejected: {speed: 3},
        });
        const st = parseLidarMapCalibrationStatus({data});
        expect(st?.state).toBe("collecting");
        expect(st?.pairs).toBe(120);
        expect(st?.rejected).toEqual({speed: 3});
    });

    it("accepts the capitalised Data field", () => {
        const st = parseLidarMapCalibrationStatus({Data: JSON.stringify({state: "idle"})});
        expect(st?.state).toBe("idle");
    });

    it("rejects malformed payloads", () => {
        expect(parseLidarMapCalibrationStatus(null)).toBeNull();
        expect(parseLidarMapCalibrationStatus({data: ""})).toBeNull();
        expect(parseLidarMapCalibrationStatus({data: "{not json"})).toBeNull();
        expect(parseLidarMapCalibrationStatus({data: JSON.stringify({pairs: 1})})).toBeNull();
    });
});
