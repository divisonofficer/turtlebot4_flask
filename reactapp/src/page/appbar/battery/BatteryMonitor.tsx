import { useEffect, useState } from "react";
import { laptopSocket } from "../../../connect/socket/subscribe";
import { HStack } from "@chakra-ui/react";
import { httpGet } from "../../../connect/http/request";
import { Body3 } from "../../../design/text/textsystem";
import { Color } from "../../../design/color";
import { MonitorIconHover } from "../robotmonitor/RobotMonitor";
import { InfoCard } from "../../../design/other/infocard";

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
      setCharging(data.State === "Charging");
    });
  }, []);

  return (
    <HStack>
      <MonitorIconHover
        style={{
          width: "auto",
        }}
        hoverView={
          <InfoCard
            color={Color.Orange}
            title={"Laptop Battery"}
            value={SOC + "%"}
          />
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
