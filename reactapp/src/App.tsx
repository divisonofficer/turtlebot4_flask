import "./App.css";
import { BrowserRouter, Route, Routes } from "react-router-dom";
import Dashboard from "./page/Dashboard";
import { HStack, useMediaQuery, VStack } from "@chakra-ui/react";
import { Menus, MenuToggle } from "./page/Menu";
import JoysticPage from "./page/Joystick";
import { TopicPage } from "./page/topic/TopicPage";
import ServiceCallPage from "./page/service/ServiceCallPage";
import { LidarPage } from "./page/lidar/LidarPage";

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
      </Routes>
    );
  };

  return (
    <div className="App">
      <header className="App-header">
        <BrowserRouter>
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
                width: "100%",
                height: "100vh",
              }}
            >
              <Menus />
              <RouteTable />
            </HStack>
          )}
        </BrowserRouter>
      </header>
    </div>
  );
}

export default App;
