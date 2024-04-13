import { useEffect, useState } from "react";
import { laptopSocket } from "../../../connect/socket/subscribe";
import { Flex, HStack, VStack } from "@chakra-ui/react";
import { httpGet } from "../../../connect/http/request";
import { Body3, H2 } from "../../../design/text/textsystem";
import { Color } from "../../../design/color";
import { MonitorIconHover } from "../robotmonitor/RobotMonitor";
import { InfoCard } from "../../../design/other/infocard";
import {
  BatteryCharging,
  BatteryEmpty,
  BatteryFull,
  BatteryHigh,
  BatteryLow,
} from "@phosphor-icons/react";

export interface BatteryMonitorData {
  SoC: number;
  State: string;
}

export const BatteryMonitorIcon = () => {
  const [SOC, setSOC] = useState(0);
  const [charging, setCharging] = useState(false);

  const batteryStyle: React.CSSProperties = {
    border: "2px solid black",
    borderRadius: "4px",
    height: "20px",
    width: "40px",
    position: "relative",
    backgroundColor: "white",
  };

  const batteryLevelStyle = {
    height: "100%",
    width: `${SOC}%`,
    backgroundColor: charging ? Color.Cyan : SOC > 20 ? Color.Green : Color.Red,
    borderRadius: "2px",
    transition: "width 0.5s ease",
    animation: charging
      ? "chargingAnimation 1.5s infinite alternate"
      : "dischargingAnimation 1.5s infinite alternate",
  };

  useEffect(() => {
    httpGet("/laptop/current")
      .onSuccess((data: BatteryMonitorData) => {
        setSOC(data.SoC);
        setCharging(data.State === "Charging");
      })
      .fetch();

    laptopSocket.subscribe("/battery", (data: BatteryMonitorData) => {
      setSOC(data.SoC);
      setCharging(data.State === "Charging" || data.State === "Fully Charged");
    });
  }, []);

  const BatteryIcon = charging
    ? BatteryCharging
    : SOC > 90
    ? BatteryFull
    : SOC > 50
    ? BatteryHigh
    : SOC > 20
    ? BatteryLow
    : BatteryEmpty;

  return (
    <HStack>
      <MonitorIconHover
        style={{
          width: "auto",
        }}
        hoverView={
          <VStack
            style={{
              borderRadius: "0.5rem",
              background: "white",
              boxShadow: "0 0 10px rgba(0,0,0,0.1)",
              overflow: "hidden",
            }}
          >
            <Flex
              style={{
                background:
                  charging || SOC > 60
                    ? Color.Green
                    : SOC > 20
                    ? Color.Yellow
                    : SOC < 1
                    ? "#1c1c1c"
                    : Color.Red,
                width: "10rem",
                height: "8rem",
                alignItems: "center",
                justifyContent: "center",
              }}
            >
              <BatteryIcon
                style={{
                  width: "3rem",
                  height: "3rem",
                }}
              />
            </Flex>
            <Flex
              style={{
                padding: "0.5rem",
                justifyContent: "center",
                alignItems: "center",
                height: "6rem",
                flexDirection: "column",
              }}
            >
              <H2 color="black">{SOC}%</H2>
              <Body3 color="black">
                {charging
                  ? "Charging"
                  : SOC > 20
                  ? "Discharging"
                  : SOC > 1
                  ? "Empty"
                  : "No Monitor"}
              </Body3>
            </Flex>
          </VStack>
        }
      >
        <div style={batteryStyle}>
          <div style={batteryLevelStyle} />
        </div>
      </MonitorIconHover>

      <Body3>{SOC}%</Body3>
      <style>
        {`
          @keyframes chargingAnimation {
            from { background-color: green; }
            to { background-color: lightgreen; }
          }
          @keyframes dischargingAnimation {
            from { background-color: green; }
            to { background-color: darkgreen; }
          }
        `}
      </style>
    </HStack>
  );
};
