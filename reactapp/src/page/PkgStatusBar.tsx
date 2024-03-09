import {
  Button,
  Container,
  Grid,
  HStack,
  Switch,
  Text,
  VStack,
} from "@chakra-ui/react";
import { useEffect, useState } from "react";
import { rosSocket } from "../connect/socket/subscribe";
import { httpDel, httpGet, httpPost } from "../connect/http/request";
import { NavLink, useNavigate } from "react-router-dom";
import { PkgNodeInfo } from "../data/PkgNode";
import { TextRibon } from "../design/text/textribon";

const NodeButton = (props: {
  node: PkgNodeInfo;
  checkVisible: boolean;
  checked: boolean;
  onChange: (b: boolean) => void;
  onClick: () => void;
}) => {
  const { node, checkVisible, checked } = props;

  const [isHovered, setIsHovered] = useState(false);

  const defaultStyle = {
    width: "30rem",
    height: "12rem",
    background: "#F7F9FB",
    borderRadius: "1rem",
    transition: "background 0.3s ease", // Smooth transition for the background color
    marginTop: "1rem",
    marginLeft: "1rem",
  };

  const hoveredStyle = {
    ...defaultStyle,
    background: "#c7c9cB", // Change the color to highlight when hovered
  };
  return (
    <div
      style={isHovered ? hoveredStyle : defaultStyle}
      onMouseEnter={() => setIsHovered(true)}
      onMouseLeave={() => setIsHovered(false)}
    >
      <VStack
        style={{
          padding: "1rem",
          alignItems: "flex-start",
          height: "100%",
        }}
        onClick={props.onClick}
      >
        <TextRibon size="sm">{node.pkg}</TextRibon>
        <Text color="#1C1C1C" fontSize="2xl">
          {node.name}
        </Text>

        <div style={{ flexGrow: 1 }} />
        <HStack
          style={{
            width: "100%",
            justifyContent: "flex-end",
            alignItems: "flex-end",
          }}
        >
          <Text
            fontSize="sm"
            color="#1C1C1C66"
            noOfLines={[1, 2, 3]}
            textAlign={"left"}
          >
            {node.abstract}
          </Text>
          {checkVisible && (
            <div onClick={(e) => e.stopPropagation()}>
              <Switch
                size="lg"
                isChecked={checked}
                onChange={(e) => {
                  e.stopPropagation();
                  props.onChange(e.target.checked);
                }}
                onClick={(e) => {
                  e.stopPropagation();
                }}
              />
            </div>
          )}
        </HStack>
      </VStack>
    </div>
  );
};

const PkgStatusBar = ({
  onClickNode,
}: {
  onClickNode?: (node: PkgNodeInfo) => void;
}) => {
  const navigate = useNavigate();

  const [serviceStatus, setServiceStatus] = useState<{
    [key: string]: boolean;
  }>({});

  const [nodeList, setNodeList] = useState<PkgNodeInfo[]>([]);

  useEffect(() => {
    rosSocket.subscribe(
      "/pkg_monitoring",
      (data: { [key: string]: boolean }) => {
        setServiceStatus(data);
      }
    );

    httpGet("/ros/node/list")
      .onSuccess((data: PkgNodeInfo[]) => {
        if (data) setNodeList(data);
      })
      .onError((e: any) => {
        alert(e.message);
      })
      .fetch();
  }, []);

  const launchNode = (
    pkg: string,
    node: string,
    options: string | undefined = undefined
  ) => {
    // ask user yes or no

    window.confirm(`Do you want to launch ${node} in ${pkg} ?`) &&
      httpPost(`/ros/node/${pkg}/${node}`, {
        options: options,
      })
        .onSuccess(() => {})
        .fetch();
  };

  const killNode = (pkg: string, node: string) => {
    // ask user yes or no
    window.confirm(`Do you want to kill ${node} in ${pkg} ?`) &&
      httpDel(`/ros/node/${pkg}/${node}`)
        .onSuccess(() => {})
        .fetch();
  };

  return (
    <Grid
      style={{
        justifyContent: "flex-start",
        alignItems: "flex-start",
        paddingLeft: "2rem",
        paddingBottom: "2rem",
      }}
      templateColumns="repeat(4, 1fr)"
    >
      {nodeList &&
        nodeList.map((node, index) => (
          <NodeButton
            key={index}
            node={node}
            checkVisible={serviceStatus[node.name] !== undefined}
            checked={serviceStatus[node.name]}
            onChange={(b) => {
              if (b) {
                launchNode(node.pkg, node.node, node.launchOption);
              } else {
                killNode(node.pkg, node.node);
              }
            }}
            onClick={() => {
              onClickNode && onClickNode(node);
            }}
          />
        ))}
    </Grid>
  );
};

export default PkgStatusBar;
