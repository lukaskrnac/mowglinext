import React, { useMemo, useState } from "react";
import { Alert, App, Button, Card, Collapse, Space, Tag, Typography } from "antd";
import { PlayCircleOutlined, ReloadOutlined, SaveOutlined } from "@ant-design/icons";
import { ContentType } from "../../api/Api.ts";
import { useApi } from "../../hooks/useApi.ts";

const { Paragraph, Text } = Typography;

type GnssActionName = "plan" | "apply" | "factory-reset-apply" | "restart";

type GnssCommandExecution = {
    tool?: string;
    command?: string[];
    exit_code?: number;
    stdout?: string;
    stderr?: string;
    success?: boolean;
};

type GnssActionResponse = {
    success?: boolean;
    partial_failure?: boolean;
    action?: string;
    message?: string;
    warnings?: string[];
    receiver_family?: string;
    profile?: string;
    signal_profile?: string;
    profile_rate_hz?: string;
    serial_device?: string;
    runtime_baud?: string;
    config_baud?: string;
    runtime_baud_differs_from_config?: boolean;
    runtime_baud_updated?: boolean;
    gps_container?: string;
    gps_image?: string;
    gps_container_was_running?: boolean;
    stop_attempted?: boolean;
    restart_attempted?: boolean;
    restart_succeeded?: boolean;
    restart_error?: string;
    executions?: GnssCommandExecution[];
};

type Props = {
    isDirty?: boolean;
    saving?: boolean;
    gpsRestarting?: boolean;
    onSave?: () => void | Promise<void>;
    onSaveAndRestartGps?: () => void | Promise<void>;
    onPersistBeforeAction?: () => Promise<boolean>;
    showSaveButtons?: boolean;
};

const GNSS_ACTION_LABELS: Record<GnssActionName, string> = {
    plan: "Plan profile apply",
    apply: "Apply profile to receiver",
    "factory-reset-apply": "Factory reset + apply profile",
    restart: "Restart GPS",
};

const describeApiError = (error: unknown): string => {
    if (error && typeof error === "object") {
        const apiError = (error as any).error?.error;
        if (typeof apiError === "string" && apiError.trim()) {
            return apiError;
        }
        if (typeof (error as any).message === "string" && (error as any).message.trim()) {
            return (error as any).message;
        }
        if (typeof (error as any).statusText === "string" && (error as any).statusText.trim()) {
            return (error as any).statusText;
        }
    }
    return "Unknown GNSS API error";
};

const responseAlertType = (response: GnssActionResponse | null, errorMessage: string | null): "success" | "warning" | "error" | "info" => {
    if (errorMessage) {
        return "error";
    }
    if (!response) {
        return "info";
    }
    if (response.partial_failure) {
        return "warning";
    }
    if (response.success === false) {
        return "error";
    }
    if (response.success) {
        return "success";
    }
    return "info";
};

const formatCommand = (command?: string[]): string => {
    if (!command || command.length === 0) {
        return "";
    }
    return command.join(" ");
};

export const GnssReceiverActionsCard: React.FC<Props> = ({
    isDirty = false,
    saving = false,
    gpsRestarting = false,
    onSave,
    onSaveAndRestartGps,
    onPersistBeforeAction,
    showSaveButtons = false,
}) => {
    const guiApi = useApi();
    const { notification, modal } = App.useApp();
    const [pendingAction, setPendingAction] = useState<GnssActionName | null>(null);
    const [lastResponse, setLastResponse] = useState<GnssActionResponse | null>(null);
    const [transportError, setTransportError] = useState<string | null>(null);

    const actionSummary = useMemo(() => {
        if (transportError) {
            return {
                type: "error" as const,
                message: "GNSS backend request failed",
                description: transportError,
            };
        }
        if (!lastResponse) {
            return null;
        }
        if (lastResponse.partial_failure) {
            return {
                type: "warning" as const,
                message: lastResponse.message || "GNSS action finished with partial failure",
                description: "Review warnings, command output, and restart status below.",
            };
        }
        if (lastResponse.success === false) {
            return {
                type: "error" as const,
                message: lastResponse.message || "GNSS action failed",
                description: "Review command output and warnings below.",
            };
        }
        return {
            type: "success" as const,
            message: lastResponse.message || "GNSS action completed successfully",
            description: "Command output and backend warnings are shown below.",
        };
    }, [lastResponse, transportError]);

    const runAction = async (action: GnssActionName, body?: Record<string, any>) => {
        setPendingAction(action);
        setTransportError(null);

        try {
            if (onPersistBeforeAction) {
                const persisted = await onPersistBeforeAction();
                if (!persisted) {
                    setPendingAction(null);
                    return;
                }
            }

            const response = await guiApi.request<GnssActionResponse, { error?: string }>({
                path: `/settings/gnss/${action}`,
                method: "POST",
                body,
                type: ContentType.Json,
                format: "json",
            });

            const data = response.data ?? {};
            setLastResponse(data);

            if (data.partial_failure) {
                notification.warning({
                    message: data.message || `${GNSS_ACTION_LABELS[action]} completed with partial failure`,
                    description: "Review the backend warnings and restart result in the GNSS action panel.",
                });
            } else if (data.success === false) {
                notification.error({
                    message: data.message || `${GNSS_ACTION_LABELS[action]} failed`,
                    description: "Review the GNSS action panel for stdout/stderr and warnings.",
                });
            } else {
                notification.success({
                    message: data.message || `${GNSS_ACTION_LABELS[action]} completed`,
                });
            }
        } catch (error) {
            const description = describeApiError(error);
            setTransportError(description);
            notification.error({
                message: `${GNSS_ACTION_LABELS[action]} failed`,
                description,
            });
        } finally {
            setPendingAction(null);
        }
    };

    const confirmApply = () => {
        modal.confirm({
            title: "Apply GNSS profile to receiver?",
            content: (
                <Space direction="vertical" size={8}>
                    <Text>
                        The GPS sidecar will be stopped, the receiver profile will be applied in a one-shot GNSS container,
                        and the GPS sidecar will be restarted only if the apply succeeds.
                    </Text>
                    <Text type="warning">
                        Do not disconnect power or USB/serial wiring while the receiver profile is being applied.
                    </Text>
                </Space>
            ),
            okText: "Apply profile",
            cancelText: "Cancel",
            onOk: () => runAction("apply", { confirm: true }),
        });
    };

    const confirmFactoryReset = () => {
        modal.confirm({
            title: "Factory reset receiver and re-apply profile?",
            content: (
                <Space direction="vertical" size={8}>
                    <Text strong type="danger">
                        Factory reset clears receiver settings before rebuilding the selected profile.
                    </Text>
                    <Text>
                        The backend will stop the GPS sidecar, run the persistent factory-reset flow, then re-apply the
                        selected saved profile and restart GPS only if that flow succeeds.
                    </Text>
                    <Text type="warning">
                        Use this only when you intentionally want to rebuild receiver configuration from a clean state.
                    </Text>
                </Space>
            ),
            okText: "Factory reset and apply",
            okType: "danger",
            cancelText: "Cancel",
            maskClosable: false,
            onOk: () => runAction("factory-reset-apply", { confirm_factory_reset: true }),
        });
    };

    const loadingActionLabel = pendingAction ? GNSS_ACTION_LABELS[pendingAction] : "";
    const actionDisabled = Boolean(pendingAction) || saving || gpsRestarting;

    return (
        <Card size="small" title="Receiver Actions" style={{ marginBottom: 16 }}>
            <Space wrap size={[8, 8]}>
                {showSaveButtons && onSave && (
                    <Button
                        type="primary"
                        icon={<SaveOutlined />}
                        onClick={onSave}
                        loading={saving && !gpsRestarting && !pendingAction}
                        disabled={actionDisabled || !isDirty}
                    >
                        Save settings
                    </Button>
                )}
                <Button
                    icon={<PlayCircleOutlined />}
                    onClick={() => runAction("plan")}
                    loading={pendingAction === "plan"}
                    disabled={actionDisabled}
                >
                    Plan profile apply
                </Button>
                <Button
                    onClick={confirmApply}
                    loading={pendingAction === "apply"}
                    disabled={actionDisabled}
                >
                    Apply profile to receiver
                </Button>
                {showSaveButtons && onSaveAndRestartGps && (
                    <Button
                        icon={<ReloadOutlined />}
                        onClick={onSaveAndRestartGps}
                        loading={gpsRestarting && !pendingAction}
                        disabled={Boolean(pendingAction) || saving || gpsRestarting}
                    >
                        Save + restart GPS
                    </Button>
                )}
                <Button
                    icon={<ReloadOutlined />}
                    onClick={() => runAction("restart", {})}
                    loading={pendingAction === "restart"}
                    disabled={actionDisabled}
                >
                    Restart GPS
                </Button>
                <Button
                    danger
                    onClick={confirmFactoryReset}
                    loading={pendingAction === "factory-reset-apply"}
                    disabled={actionDisabled}
                >
                    Factory reset + apply profile
                </Button>
            </Space>

            <Alert
                type={isDirty ? "warning" : "info"}
                showIcon
                style={{ marginTop: 12 }}
                message={isDirty
                    ? "Unsaved GNSS edits will be saved before receiver actions run"
                    : "Receiver actions use the currently saved GNSS configuration"}
                description={isDirty
                    ? "The GUI saves the current receiver-related form values first, then calls the GNSS backend API. Plan does not stop GPS; apply and factory-reset flows release the serial port before touching the receiver."
                    : "Plan previews the saved receiver profile without stopping GPS. Apply and factory-reset flows stop mowgli-gps first so the serial device is released safely."}
            />

            {pendingAction && (
                <Alert
                    type="info"
                    showIcon
                    style={{ marginTop: 12 }}
                    message={`${loadingActionLabel} in progress`}
                    description="Waiting for the GNSS backend API to return command output, warnings, and restart status."
                />
            )}

            {actionSummary && (
                <Alert
                    type={actionSummary.type}
                    showIcon
                    style={{ marginTop: 12 }}
                    message={actionSummary.message}
                    description={actionSummary.description}
                />
            )}

            {(lastResponse || transportError) && (
                <Space direction="vertical" size={12} style={{ width: "100%", marginTop: 12 }}>
                    {lastResponse && (
                        <Card size="small" type="inner" title="Backend Result">
                            <Space wrap size={[8, 8]} style={{ marginBottom: 12 }}>
                                <Tag color={responseAlertType(lastResponse, null) === "success" ? "success" : responseAlertType(lastResponse, null) === "warning" ? "warning" : "error"}>
                                    {lastResponse.partial_failure
                                        ? "Partial failure"
                                        : lastResponse.success
                                            ? "Success"
                                            : "Failed"}
                                </Tag>
                                {lastResponse.restart_attempted && (
                                    <Tag color={lastResponse.restart_succeeded ? "success" : "error"}>
                                        {lastResponse.restart_succeeded ? "GPS restarted" : "GPS restart failed"}
                                    </Tag>
                                )}
                                {lastResponse.runtime_baud_updated && (
                                    <Tag color="processing">Runtime baud updated</Tag>
                                )}
                                {lastResponse.runtime_baud_differs_from_config && (
                                    <Tag color="warning">Baud mismatch</Tag>
                                )}
                            </Space>

                            <Space direction="vertical" size={4} style={{ width: "100%" }}>
                                {lastResponse.receiver_family && (
                                    <Text><Text strong>Receiver family:</Text> {lastResponse.receiver_family}</Text>
                                )}
                                {lastResponse.profile && (
                                    <Text><Text strong>Profile:</Text> {lastResponse.profile}</Text>
                                )}
                                {lastResponse.signal_profile && (
                                    <Text><Text strong>Signal profile:</Text> {lastResponse.signal_profile}</Text>
                                )}
                                {lastResponse.serial_device && (
                                    <Text><Text strong>Serial device:</Text> {lastResponse.serial_device}</Text>
                                )}
                                {(lastResponse.runtime_baud || lastResponse.config_baud) && (
                                    <Text>
                                        <Text strong>Baud:</Text> runtime {lastResponse.runtime_baud ?? "unknown"} / configured {lastResponse.config_baud ?? "unknown"}
                                    </Text>
                                )}
                                {lastResponse.restart_error && (
                                    <Text type="danger"><Text strong>Restart error:</Text> {lastResponse.restart_error}</Text>
                                )}
                            </Space>

                            {lastResponse.warnings && lastResponse.warnings.length > 0 && (
                                <Alert
                                    type="warning"
                                    showIcon
                                    style={{ marginTop: 12 }}
                                    message="Backend warnings"
                                    description={(
                                        <ul style={{ margin: 0, paddingLeft: 18 }}>
                                            {lastResponse.warnings.map((warning) => (
                                                <li key={warning}>{warning}</li>
                                            ))}
                                        </ul>
                                    )}
                                />
                            )}
                        </Card>
                    )}

                    {lastResponse?.executions && lastResponse.executions.length > 0 && (
                        <Collapse
                            size="small"
                            items={lastResponse.executions.map((execution, index) => ({
                                key: `${execution.tool ?? "command"}-${index}`,
                                label: (
                                    <Space wrap size={[8, 8]}>
                                        <Text strong>{execution.tool ?? `Command ${index + 1}`}</Text>
                                        <Tag color={execution.success ? "success" : "error"}>
                                            exit {execution.exit_code ?? "?"}
                                        </Tag>
                                    </Space>
                                ),
                                children: (
                                    <Space direction="vertical" size={8} style={{ width: "100%" }}>
                                        <div>
                                            <Text strong>Command summary</Text>
                                            <Paragraph
                                                code
                                                style={{
                                                    marginBottom: 0,
                                                    marginTop: 4,
                                                    whiteSpace: "pre-wrap",
                                                    wordBreak: "break-word",
                                                }}
                                            >
                                                {formatCommand(execution.command)}
                                            </Paragraph>
                                        </div>
                                        {execution.stdout && (
                                            <details>
                                                <summary>stdout</summary>
                                                <pre style={{ whiteSpace: "pre-wrap", wordBreak: "break-word", marginTop: 8 }}>
                                                    {execution.stdout}
                                                </pre>
                                            </details>
                                        )}
                                        {execution.stderr && (
                                            <details>
                                                <summary>stderr</summary>
                                                <pre style={{ whiteSpace: "pre-wrap", wordBreak: "break-word", marginTop: 8 }}>
                                                    {execution.stderr}
                                                </pre>
                                            </details>
                                        )}
                                    </Space>
                                ),
                            }))}
                        />
                    )}

                    {transportError && (
                        <Card size="small" type="inner" title="Transport Error">
                            <pre style={{ whiteSpace: "pre-wrap", wordBreak: "break-word", margin: 0 }}>
                                {transportError}
                            </pre>
                        </Card>
                    )}
                </Space>
            )}
        </Card>
    );
};
