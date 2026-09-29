import {describe, expect, it} from "vitest";
import {formatBytes, formatDuration, parseGlimStatus} from "./useGlimSupervisor.ts";

describe("parseGlimStatus", () => {
    it("parses the supervisor JSON from std_msgs/String", () => {
        const data = JSON.stringify({
            state: "mapping", message: "Mapping into mapping_2026-09-27_164005.",
            session: "mapping_2026-09-27_164005", started_at: "2026-09-27T14:40:05+00:00", elapsed_s: 12.5,
            sessions_dir: "/glim/sessions",
            sessions: [{name: "a", kind: "mapping", complete: true, size_bytes: 1024}],
            active_map: {path: "/glim/active_map/garden_map.ply", exists: true, backups: ["x.ply"]},
            log_tail: ["line"],
        });
        const st = parseGlimStatus({data});
        expect(st?.state).toBe("mapping");
        expect(st?.sessions).toHaveLength(1);
        expect(st?.active_map.backups).toEqual(["x.ply"]);
        expect(st?.elapsed_s).toBe(12.5);
    });

    it("fills defaults for missing fields and rejects garbage", () => {
        const st = parseGlimStatus({Data: JSON.stringify({state: "idle"})});
        expect(st?.sessions).toEqual([]);
        expect(st?.active_map.exists).toBe(false);
        expect(parseGlimStatus({data: "{nope"})).toBeNull();
        expect(parseGlimStatus({data: JSON.stringify({message: "x"})})).toBeNull();
        expect(parseGlimStatus(null)).toBeNull();
    });
});

describe("formatters", () => {
    it("formats sizes and durations", () => {
        expect(formatBytes(512)).toBe("1 kB");
        expect(formatBytes(5 * 1024 ** 2)).toBe("5.0 MB");
        expect(formatBytes(3 * 1024 ** 3)).toBe("3.00 GB");
        expect(formatDuration(65)).toBe("1:05");
        expect(formatDuration(3725)).toBe("1:02:05");
        expect(formatDuration(null)).toBe("—");
    });
});
