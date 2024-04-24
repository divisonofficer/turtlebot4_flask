import {
  Flex,
  Modal,
  ModalBody,
  ModalCloseButton,
  ModalContent,
  ModalHeader,
  ModalOverlay,
  VStack,
  useDisclosure,
} from "@chakra-ui/react";
import { observer } from "mobx-react-lite";
import { useEffect, useState } from "react";
import { httpGet } from "../../connect/http/request";
import { Btn } from "../../design/button/button";

export interface MapData {
  content: string;
  create_time: number;
  create_time_str: string;
  name: string;
  preview: string;
}

export const MapStoreItem = (
  props: MapData & {
    onClick?: () => void;
  }
) => {
  const [hover, setHover] = useState(false);
  return (
    <VStack
      style={{
        background: hover ? "rgba(0,0,0,0.1)" : "rgba(0,0,0,0)",
        borderRadius: "0.5rem",
        overflow: "hidden",
      }}
      onMouseEnter={() => setHover(true)}
      onMouseLeave={() => setHover(false)}
      onClick={props.onClick}
    >
      <img
        src={props.preview}
        alt={props.name}
        style={{
          width: "10rem",
          height: "10rem",
          objectFit: "cover",
        }}
      />
      <div>{props.name}</div>
      <div>{props.create_time_str}</div>
    </VStack>
  );
};

export const MapListView = observer(
  (props: { onClickMap: (mapdata: MapData) => void }) => {
    const [mapList, setMapList] = useState<MapData[]>([]);

    useEffect(() => {
      httpGet("/slam/map/list")
        .onSuccess((data) => {
          setMapList(data);
        })
        .fetch();
    }, []);

    return (
      <Flex wrap="wrap">
        {mapList.map((map) => {
          return (
            <MapStoreItem {...map} onClick={() => props.onClickMap(map)} />
          );
        })}
      </Flex>
    );
  }
);

export const MapListModal = (props: {
  isOpen: boolean;
  onClose: () => void;
  onClickMap: (map: MapData) => void;
}) => {
  return (
    <Modal isOpen={props.isOpen} onClose={props.onClose}>
      <ModalOverlay />
      <ModalContent>
        <ModalHeader>
          <ModalCloseButton />
        </ModalHeader>
        <ModalBody>
          <MapListView onClickMap={props.onClickMap} />
        </ModalBody>
      </ModalContent>
    </Modal>
  );
};

export const MapListModalBtn = (props: {
  onClickMap?: (map: MapData) => void;
  onClickMapWithCallback?: (map: MapData, closeCallback: () => void) => void;
}) => {
  const { isOpen, onOpen, onClose } = useDisclosure();
  return (
    <Flex>
      <Btn onClick={onOpen}>Map List</Btn>
      <MapListModal
        isOpen={isOpen}
        onClose={onClose}
        onClickMap={(map) => {
          if (props.onClickMap) {
            props.onClickMap(map);
          }
          if (props.onClickMapWithCallback) {
            props.onClickMapWithCallback(map, onClose);
          }
        }}
      />
    </Flex>
  );
};
