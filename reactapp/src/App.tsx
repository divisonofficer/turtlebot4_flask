import "./App.css";
import { BrowserRouter, HashRouter, Route, Routes } from "react-router-dom";
import Dashboard from "./page/Dashboard";
import { HStack, useMediaQuery, VStack } from "@chakra-ui/react";
import { Menus, MenuToggle } from "./page/Menu";
import JoysticPage from "./page/Joystick";
import { TopicPage } from "./page/topic/TopicPage";
import ServiceCallPage from "./page/service/ServiceCallPage";
import { LidarPage } from "./page/lidar/LidarPage";
import { DiagnosticPage } from "./page/diagnotics/DiagnoticsPage";
import { DiagnoticsDetailPage } from "./page/diagnotics/DiagnoticsDetailPage";
import { NodeDetailPage, NodePage } from "./page/nodes/NodePage";
import { TopicDetailPage } from "./page/topic/TopicDetailPage";
import { LaunchPage } from "./page/launch/LaunchPage";
import { SlamPage } from "./page/slam/SlamPage";
import { Appbar } from "./page/appbar/Appbar";
import { CapturePage } from "./page/capture/CapturePage";
import { AlertView } from "./page/appbar/AlertView";
import { JaiBridgePage } from "./page/jaibridge/JaiBridgePage";
import { PolarizationCalibrationPage } from "./page/polarization/PolarizationCalibration";

function App() {
  const isMobile = useMediaQuery("(max-width: 600px)")[0];

  const RouteTable = () => {
    return (
      <Routes>
        <Route path="/" element={<Dashboard />} />
        <Route path="/joystick" element={<JoysticPage />} />
        <Route path="/topic" element={<TopicPage />} />
        <Route path="/service_call" element={<ServiceCallPage />} />
        <Route path="/lidar" element={<LidarPage />} />
        <Route path="/diagnostics" element={<DiagnosticPage />} />
        <Route path="/diagnostics/detail" element={<DiagnoticsDetailPage />} />
        <Route path="/nodes" element={<NodePage />} />
        <Route path="/nodes/detail" element={<NodeDetailPage />} />
        <Route path="/topic/detail" element={<TopicDetailPage />} />
        <Route path="/launch" element={<LaunchPage />} />
        <Route path="/slam" element={<SlamPage />} />
        <Route path="/capture" element={<CapturePage />} />
        <Route path="/jaibridge" element={<JaiBridgePage />} />
        <Route path="/polarization" element={<PolarizationCalibrationPage />} />
      </Routes>
    );
  };

  return (
    <div className="App">
      <header className="App-header">
        <HashRouter>
          {isMobile ? (
            <VStack
              style={{
                width: "100%",
                height: "100vh",
              }}
            >
              <MenuToggle />
              <RouteTable />
            </VStack>
          ) : (
            <HStack
              style={{
                width: "100vw",
                height: "100vh",
                overflowX: "hidden",
                left: 0,
                position: "relative",
              }}
            >
              <Menus />
              <AlertView />
              <VStack
                style={{
                  display: "block",

                  flexGrow: 1,
                  height: "100vh",
                  overflowX: "hidden",
                }}
              >
                <Appbar />
                <RouteTable />
              </VStack>
            </HStack>
          )}
        </HashRouter>
      </header>
    </div>
  );
}

export default App;
