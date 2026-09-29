import React from "react";
import {App, Segmented, Space, Tag, Tooltip, Typography} from "antd";
import {useTranslation} from "react-i18next";
import {
    type LocalizationSource,
    type LocalizationSourceState,
    localizationSourceErrorText,
    useLocalizationSource,
} from "../hooks/useLocalizationSource.ts";

const {Text} = Typography;

type Props = {
    /** Dashboard row: segmented control only, no explanatory text. */
    compact?: boolean;
    /** Injected for tests; defaults to the live hook. */
    state?: LocalizationSourceState;
};

const LiveSwitch: React.FC<{compact?: boolean}> = ({compact}) => {
    const state = useLocalizationSource();
    return <LocalizationSourceSwitchView compact={compact} state={state}/>;
};

export const LocalizationSourceSwitch: React.FC<Props> = ({compact, state}) =>
    state ? <LocalizationSourceSwitchView compact={compact} state={state}/> : <LiveSwitch compact={compact}/>;

/**
 * GPS ↔ LiDAR switch for fusion_graph's primary absolute-position source.
 * Applies live and persists to mowgli_robot.yaml (see useLocalizationSource).
 */
export const LocalizationSourceSwitchView: React.FC<{compact?: boolean; state: LocalizationSourceState}> = ({
    compact,
    state,
}) => {
    const {t} = useTranslation();
    const {modal, notification} = App.useApp();
    const {source, calibrated, loading, switching, error, setSource} = state;

    const unavailable = !loading && source === null;
    const lidarBlocked = calibrated === false;

    const requestSwitch = (next: LocalizationSource) => {
        if (next === source || switching) return;
        modal.confirm({
            title: t(next === "lidar" ? "localizationSource.confirmLidarTitle" : "localizationSource.confirmGpsTitle"),
            content: t("localizationSource.confirmBody"),
            okText: t("localizationSource.confirmOk"),
            cancelText: t("localizationSource.confirmCancel"),
            onOk: async () => {
                const ok = await setSource(next);
                if (ok) {
                    notification.success({
                        message: t("localizationSource.switched", {
                            source: t(next === "lidar" ? "localizationSource.lidar" : "localizationSource.gps"),
                        }),
                        duration: 3,
                    });
                }
            },
        });
    };

    // Errors from a switch attempt surface as a notification once; a polling
    // error (node down) is shown inline as "unavailable" instead.
    const lastShownRef = React.useRef<string | null>(null);
    React.useEffect(() => {
        if (!error || source === null || error === lastShownRef.current) return;
        lastShownRef.current = error;
        notification.error({
            message: t("localizationSource.switchFailed"),
            description: localizationSourceErrorText(error, t),
        });
    }, [error, source, notification, t]);
    React.useEffect(() => {
        if (!error) lastShownRef.current = null;
    }, [error]);

    const control = (
        <Segmented<LocalizationSource>
            size={compact ? "small" : "middle"}
            value={source ?? undefined}
            disabled={loading || switching || unavailable}
            onChange={(v) => requestSwitch(v)}
            options={[
                {label: t("localizationSource.gps"), value: "gps"},
                {
                    label: lidarBlocked ? (
                        <Tooltip title={t("localizationSource.needsCalibration")}>
                            <span>{t("localizationSource.lidar")}</span>
                        </Tooltip>
                    ) : t("localizationSource.lidar"),
                    value: "lidar",
                    disabled: lidarBlocked && source !== "lidar",
                },
            ]}
        />
    );

    if (compact) return control;

    return (
        <Space direction="vertical" size={8} style={{width: "100%"}}>
            <Space wrap>
                {control}
                {unavailable ? (
                    <Tag color="default">{t("localizationSource.unavailable")}</Tag>
                ) : calibrated === true ? (
                    <Tag color="success">{t("localizationSource.calibrated")}</Tag>
                ) : calibrated === false ? (
                    <Tag color="warning">{t("localizationSource.notCalibrated")}</Tag>
                ) : null}
            </Space>
            <Text type="secondary" style={{fontSize: 12}}>
                {t("localizationSource.description")}
            </Text>
            {lidarBlocked && (
                <Text type="secondary" style={{fontSize: 12}}>
                    {t("localizationSource.needsCalibration")}{" "}
                    <Typography.Link href="/diagnostics?tab=calibration">
                        {t("localizationSource.openCalibration")}
                    </Typography.Link>
                </Text>
            )}
            {unavailable && error && (
                <Text type="secondary" style={{fontSize: 11}}>{error}</Text>
            )}
        </Space>
    );
};
