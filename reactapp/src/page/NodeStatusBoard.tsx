import { Container, HStack, IconButton, Text, VStack } from "@chakra-ui/react";
import { PkgNodeInfo } from "../data/PkgNode";
import { useCallback, useEffect, useState } from "react";
import { rosSocket } from "../connect/socket/subscribe";
import { httpGet } from "../connect/http/request";
import { TextRibon } from "../design/text/textribon";
import { Body3, H1, H2 } from "../design/text/textsystem";
import { VDivBlock } from "../design/other/blocks";
import { CloseIcon, DeleteIcon } from "@chakra-ui/icons";

const NodeStatusBoard = ({
  node,
  close,
}: {
  node: PkgNodeInfo;
  close: () => void;
}) => {
  const [nodeLogs, setNodeLogs] = useState<string[]>([]);

  const fetchLogs = useCallback(() => {
    httpGet(`/ros/node/${node.pkg}/${node.node}`)
      .onSuccess((data: { logs: string }) => {
        if (data.logs)
          setNodeLogs(data.logs.split("\n").filter((log) => log.length > 0));
        else setNodeLogs([]);
      })
      .onError((e) => {
        setNodeLogs([]);
      })
      .fetch();
  }, [node]);

  useEffect(() => {
    const intervel = setInterval(fetchLogs, 1000);

    return () => {
      clearInterval(intervel);
    };
  }, [node, fetchLogs]);

  const getLogLevel = (log: string) => {
    let message = "debug";
    let color = "#1C1C1C66";
    let textColor = "#eeeeee";
    if (log.includes("INFO")) {
      message = "info";
      color = "#4AA785";
    }
    if (log.includes("WARN")) {
      message = "warn";
      color = "#FFC555";
    }
    if (log.includes("ERROR")) {
      message = "error";
      color = "#FF3333";
    }

    return (
      <TextRibon color={color} size="sm" textColor={textColor}>
        {message}
      </TextRibon>
    );
  };

  return (
    <VStack
      style={{
        flexGrow: 1,

        alignItems: "flex-start",
      }}
    >
      <HStack
        style={{
          width: "100%",
        }}
      >
        <H2>{node.name}</H2>

        <TextRibon color="#4AA785" size="md" textColor="#eeeeee">
          {node.pkg}
        </TextRibon>
        <TextRibon color="#1C1C1C66" size="md" textColor="#eeeeee">
          {node.node}
        </TextRibon>
        <div style={{ flexGrow: 1 }} />
        <IconButton aria-label="close" icon={<CloseIcon />} onClick={close} />
      </HStack>

      <VDivBlock color="#B1E3FF" />
      <Text
        fontSize="lg"
        style={{
          width: "90%",
          textAlign: "left",
        }}
      >
        {node.abstract}
      </Text>

      <VStack
        style={{
          width: "100%",
          flexGrow: 1,
          justifyContent: "flex-start",
          alignItems: "flex-start",
          height: "40rem",
          overflowY: "auto",
          gap: 0,
          background: "#e7e9eB",
          borderRadius: "1rem",
          padding: "2rem",
        }}
      >
        {nodeLogs.length === 0 && (
          <Container
            style={{
              width: "100%",
              height: "100%",
              justifyContent: "center",
              alignItems: "center",
            }}
          >
            <H1
              style={{
                textAlign: "center",
              }}
            >
              No Logs
            </H1>
          </Container>
        )}

        {nodeLogs.map((log, index) => (
          <HStack
            key={index}
            style={{
              height: "2rem",
              minHeight: "2rem",
              justifyContent: "flex-start",
              alignItems: "flex-start",
            }}
          >
            {getLogLevel(log)}
            <Body3>{log}</Body3>
          </HStack>
        ))}
        <div style={{ flexGrow: 1 }}></div>
      </VStack>
    </VStack>
  );
};

export default NodeStatusBoard;
