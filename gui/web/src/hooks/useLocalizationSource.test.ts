import {describe, expect, it} from "vitest";
import {findParam, localizationSourceErrorText, parseBool, parseSource} from "./useLocalizationSource.ts";

describe("useLocalizationSource helpers", () => {
    it("finds fusion_graph params with or without a leading slash", () => {
        const params = [
            {name: "other_node.primary_localization_source", value: "gps"},
            {name: "/fusion_graph_node.primary_localization_source", value: "lidar"},
            {name: "fusion_graph_node.lidar_pose_map_calibrated", value: true},
        ];
        expect(findParam(params, "primary_localization_source")?.value).toBe("lidar");
        expect(findParam(params, "lidar_pose_map_calibrated")?.value).toBe(true);
        expect(findParam(params, "missing")).toBeUndefined();
    });

    it("parses source and bool values defensively", () => {
        expect(parseSource("gps")).toBe("gps");
        expect(parseSource("lidar")).toBe("lidar");
        expect(parseSource("LIDAR")).toBeNull();
        expect(parseBool(true)).toBe(true);
        expect(parseBool("false")).toBe(false);
        expect(parseBool(1)).toBeNull();
    });

    it("maps error codes to translation keys", () => {
        const t = (key: string, opts?: Record<string, unknown>) => (opts ? `${key}:${JSON.stringify(opts)}` : key);
        expect(localizationSourceErrorText("rejected-lidar", t)).toBe("localizationSource.rejectedLidar");
        expect(localizationSourceErrorText("rejected", t)).toBe("localizationSource.rejected");
        expect(localizationSourceErrorText("persist:HTTP 500", t)).toBe('localizationSource.persistFailed:{"error":"HTTP 500"}');
        expect(localizationSourceErrorText("boom", t)).toBe("boom");
    });
});
