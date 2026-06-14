import { App } from "antd";
import { render, screen, waitFor } from "@testing-library/react";
import userEvent from "@testing-library/user-event";
import { beforeEach, describe, expect, it, vi } from "vitest";
import { GnssReceiverActionsCard } from "./GnssReceiverActionsCard.tsx";

const requestMock = vi.fn();

vi.mock("../../hooks/useApi.ts", () => ({
    useApi: () => ({
        request: requestMock,
    }),
}));

const renderCard = (props?: Partial<React.ComponentProps<typeof GnssReceiverActionsCard>>) =>
    render(
        <App>
            <GnssReceiverActionsCard
                onPersistBeforeAction={vi.fn().mockResolvedValue(true)}
                {...props}
            />
        </App>,
    );

describe("GnssReceiverActionsCard", () => {
    beforeEach(() => {
        requestMock.mockReset();
        Object.defineProperty(window, "matchMedia", {
            writable: true,
            value: vi.fn().mockImplementation((query: string) => ({
                matches: false,
                media: query,
                onchange: null,
                addListener: vi.fn(),
                removeListener: vi.fn(),
                addEventListener: vi.fn(),
                removeEventListener: vi.fn(),
                dispatchEvent: vi.fn(),
            })),
        });
    });

    it("calls the plan endpoint and shows backend warnings and output", async () => {
        const persistMock = vi.fn().mockResolvedValue(true);
        requestMock.mockResolvedValue({
            data: {
                success: true,
                message: "GNSS profile plan succeeded",
                warnings: [
                    "GNSS_SIGNAL_PROFILE is persisted in the UI, but backend translation to Universal GNSS tool arguments is not implemented yet.",
                ],
                executions: [
                    {
                        tool: "gnss_config_plan",
                        command: ["/opt/gnss_sidecar/bin/gnss_config_plan", "--json", "unicore", "rover_high_precision"],
                        exit_code: 0,
                        stdout: "preview output",
                        success: true,
                    },
                ],
            },
            error: null,
        });

        const user = userEvent.setup();
        renderCard({ onPersistBeforeAction: persistMock });

        await user.click(screen.getByRole("button", { name: /Plan profile apply/i }));

        await waitFor(() => {
            expect(requestMock).toHaveBeenCalledWith(expect.objectContaining({
                path: "/settings/gnss/plan",
                method: "POST",
            }));
        });
        expect(persistMock).toHaveBeenCalledTimes(1);
        const successMessages = await screen.findAllByText("GNSS profile plan succeeded");
        expect(successMessages.length).toBeGreaterThan(0);
        expect(screen.getByText("Backend warnings")).toBeInTheDocument();

        await user.click(screen.getByText("gnss_config_plan"));
        expect(await screen.findByText("Command summary")).toBeInTheDocument();
        expect(screen.getByText("stdout")).toBeInTheDocument();
    });

    it("confirms before apply and sends confirm=true to the backend", async () => {
        const persistMock = vi.fn().mockResolvedValue(true);
        requestMock.mockResolvedValue({
            data: {
                success: false,
                message: "GNSS profile apply failed",
                executions: [
                    {
                        tool: "gnss_config_apply",
                        command: ["/opt/gnss_sidecar/bin/gnss_config_apply", "--confirm"],
                        exit_code: 2,
                        stderr: "device rejected command",
                        success: false,
                    },
                ],
            },
            error: null,
        });

        const user = userEvent.setup();
        renderCard({ onPersistBeforeAction: persistMock });

        await user.click(screen.getByRole("button", { name: "Apply profile to receiver" }));
        await user.click(await screen.findByRole("button", { name: "Apply profile" }));

        await waitFor(() => {
            expect(requestMock).toHaveBeenCalledWith(expect.objectContaining({
                path: "/settings/gnss/apply",
                method: "POST",
                body: { confirm: true },
            }));
        });

        expect(persistMock).toHaveBeenCalledTimes(1);
        const failureMessages = await screen.findAllByText("GNSS profile apply failed");
        expect(failureMessages.length).toBeGreaterThan(0);
    });
});
